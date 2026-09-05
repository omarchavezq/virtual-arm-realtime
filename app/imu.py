from __future__ import annotations

import math
import statistics
import threading
import time
from collections import deque
from collections.abc import Sequence

from app.config import ImuConfig

_ACCEL_BITS = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}
_GYRO_BITS = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}

# Lo que contesta el registro WHO_AM_I. El mapa de registros de accel y giro es
# el mismo en toda la familia, así que el driver sirve para las cuatro; se lee
# para saber con qué se está hablando y para el filtro que sólo tiene el 9250.
_WHO_AM_I = 0x75
_CHIP_NAMES = {0x68: "MPU6050", 0x70: "MPU6500", 0x71: "MPU9250", 0x73: "MPU9255"}
# En el MPU6050 el registro 0x1A filtra giróscopo y acelerómetro juntos. En el
# 9250 —el chip del GY-91— sólo filtra el giróscopo: el acelerómetro tiene el
# suyo en 0x1D y arranca en 460 Hz, es decir, prácticamente sin filtrar. Sin
# escribirlo, toda la vibración del motor entra en el ángulo.
_ACCEL_CONFIG_2 = 0x1D
_ACCEL_DLPF_41HZ = 0x03
_SEPARATE_ACCEL_FILTER = frozenset({0x70, 0x71, 0x73})

# Fuera de esta banda el acelerómetro no está midiendo sólo gravedad: la máquina
# acelera, frena o vibra fuerte. El ángulo que saldría no sería inclinación.
_MIN_ACCEL_G = 0.85
_MAX_ACCEL_G = 1.15

# La orientación se declara mala sólo si se sostiene. Un bache no puede dejar al
# servicio sin roll, pero un mapeo cruzado no se arregla solo.
_ORIENTATION_GRACE_S = 2.0

# Ventana sobre la que se mide la dispersión del ángulo del acelerómetro.
_NOISE_WINDOW_S = 1.0

# Ventana que promedia «Poner a cero». Una sola muestra con la vibración del
# motor encima se va grados, y ese error queda grabado en el archivo como offset
# permanente. Se exige al menos un tercio de la ventana llena.
_ZERO_WINDOW_S = 5.0

# Tras este rato integrando sólo el giróscopo, la estimación ya arrastra deriva
# de sesgo: al volver a haber gravedad limpia se vuelve a partir de ella en vez
# de acercarse despacio. Con la máquina parada, el acelerómetro es la verdad.
_RESNAP_AFTER_GATED_S = 1.0

# El sesgo del giróscopo se aprende de lo que el acelerómetro contradice, en vez
# de confiar en `gyro_bias_dps` del archivo. Ese número se mide en el sistema ya
# mapeado, así que cualquier cambio de ejes —«Detectar ejes», sin ir más lejos—
# lo deja aplicado al eje equivocado. Y el error no es inocuo: un filtro
# complementario con constante de tiempo tau arrastra un desvío permanente de
# tau·sesgo, o sea 4° con 2 s y 2 °/s mal restados.
_BIAS_GAIN = 0.02
_MAX_RATE_BIAS_DPS = 10.0


def _signed(high: int, low: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value


def _chip_name(who_am_i: int | None) -> str:
    if who_am_i is None:
        return ""
    return _CHIP_NAMES.get(who_am_i, f"desconocido (0x{who_am_i:02X})")


class _AxisFilter:
    """Filtro complementario de un eje: acelerómetro contra giróscopo.

    Vive aparte porque cabeceo y balanceo necesitan exactamente el mismo
    tratamiento, y duplicarlo sería duplicar también el estimador de sesgo, que
    es la parte delicada.
    """

    def __init__(self, tau_s: float, sample_rate_hz: int) -> None:
        self.tau_s = tau_s
        self.estimate: float | None = None
        self.bias_dps = 0.0
        self.noise_deg: float | None = None
        self.recent: deque[float] = deque(
            maxlen=max(2, int(sample_rate_hz * _NOISE_WINDOW_S))
        )
        self.raw_recent: deque[float] = deque(
            maxlen=max(2, int(sample_rate_hz * _ZERO_WINDOW_S))
        )
        self._gated_since: float | None = None

    def reset(self) -> None:
        self.estimate = None
        self.bias_dps = 0.0
        self.noise_deg = None
        self.recent.clear()
        self.raw_recent.clear()
        self._gated_since = None

    @property
    def zero_samples_required(self) -> int:
        """Un tercio de la ventana llena: suficiente para promediar la vibración
        sin obligar a esperar cinco segundos exactos con el dedo en el botón."""
        return max(2, (self.raw_recent.maxlen or 2) // 3)

    def window(self) -> tuple[float, float, int] | None:
        """Promedio, dispersión y número de muestras del ángulo sin corregir.

        Sólo entran muestras con gravedad limpia y máquina quieta, que son las
        únicas que dicen algo del montaje del sensor.
        """
        if len(self.raw_recent) < 2:
            return None
        values = list(self.raw_recent)
        return statistics.mean(values), statistics.pstdev(values), len(values)

    def update(
        self,
        corrected_deg: float,
        raw_deg: float,
        rate_dps: float,
        dt: float,
        now: float,
        *,
        gated: bool,
        plausible: bool,
    ) -> None:
        drifted = (
            self._gated_since is not None and now - self._gated_since >= _RESNAP_AFTER_GATED_S
        )
        if gated:
            if self._gated_since is None:
                self._gated_since = now
        else:
            self._gated_since = None

        # El sesgo aprendido se descuenta siempre, también con la compuerta
        # cerrada: es ahí donde la deriva no tiene quién la corrija.
        rate = rate_dps - self.bias_dps

        if self.estimate is None:
            # Hay que partir de algún lado, y el giróscopo solo no sabe dónde
            # está el suelo. Sin una lectura limpia no se arranca.
            if not plausible:
                return
            self.estimate = corrected_deg
        elif gated:
            self.estimate += rate * dt
        elif drifted:
            self.estimate = corrected_deg
            self.recent.clear()
            self.noise_deg = None
        else:
            predicted = self.estimate + rate * dt
            alpha = self.tau_s / (self.tau_s + dt)
            self.estimate = alpha * predicted + (1.0 - alpha) * corrected_deg
            # Lo que el acelerómetro desmiente de forma sostenida es sesgo del
            # giróscopo. Corregirlo aquí es lo que evita el desvío permanente.
            self.bias_dps = max(
                -_MAX_RATE_BIAS_DPS,
                min(
                    _MAX_RATE_BIAS_DPS,
                    self.bias_dps - _BIAS_GAIN * (corrected_deg - predicted) * dt,
                ),
            )

        if not gated:
            # Sin corregir: es lo que «Poner a cero» tiene que promediar.
            self.raw_recent.append(raw_deg)
            self.recent.append(corrected_deg)
            if len(self.recent) >= 2:
                self.noise_deg = statistics.pstdev(self.recent)


class RollSensor:
    """Actitud del acelerómetro, con compuerta por movimiento y plausibilidad.

    Sirve para la familia MPU6050 / 6500 / 9250 (el chip del GY-91): comparten
    el mapa de registros de acelerómetro y giróscopo.

    Publica balanceo y cabeceo. El cabeceo existe porque el del UM982, medido
    sobre una línea base de dos metros, vagabundea 1.8° con la máquina inmóvil
    —siete veces más que este acelerómetro— y eso son 12 cm de broca con un
    brazo de 3.9 m.

    Un ángulo equivocado no se nota: corre la broca en silencio y la pantalla
    sigue diciendo PRECISION. Por eso el sensor calla —`roll_deg = None`, que
    invalida la posición en 3D— en vez de publicar un número que no sostiene.
    """

    def __init__(self, config: ImuConfig) -> None:
        self.config = config
        # Valores de fiar, los únicos que entran en el cálculo. None = no
        # publicable.
        self.roll_deg: float | None = None
        self.pitch_deg: float | None = None
        self.received_ms: float | None = None
        self.error = ""
        # Diagnóstico: lo que ve el sensor antes y después del mapeo de ejes, y
        # los ángulos sin corregir. Sin esto no hay forma de calibrarlo en campo.
        self.raw_g: tuple[float, float, float] | None = None
        self.mapped_g: tuple[float, float, float] | None = None
        self.roll_raw_deg: float | None = None
        self.pitch_raw_deg: float | None = None
        self.accel_magnitude_g: float | None = None
        self.tilt_from_vertical_deg: float | None = None
        self.orientation_ok = True
        self.accel_gated = False
        # Con qué chip se está hablando. Cambiar de placa cambia la orientación
        # de los ejes y las taras: verlo publicado evita calibrar a ciegas.
        self.chip_id: int | None = None
        self.chip_name = ""
        # Lo escribe el Runtime en cada época. Con la máquina en marcha el
        # acelerómetro mide fuerza específica: una frenada se lee como ladeo.
        self.moving = False
        self._roll = _AxisFilter(config.filter_tau_s, config.sample_rate_hz)
        self._pitch = _AxisFilter(config.filter_tau_s, config.sample_rate_hz)
        self._bad_orientation_since: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------- diagnóstico

    @property
    def roll_estimate_deg(self) -> float | None:
        return self._roll.estimate

    @property
    def pitch_estimate_deg(self) -> float | None:
        return self._pitch.estimate

    @property
    def roll_noise_deg(self) -> float | None:
        return self._roll.noise_deg

    @property
    def pitch_noise_deg(self) -> float | None:
        return self._pitch.noise_deg

    @property
    def rate_bias_dps(self) -> float:
        return self._roll.bias_dps

    @property
    def pitch_rate_bias_dps(self) -> float:
        return self._pitch.bias_dps

    @property
    def zero_samples_required(self) -> int:
        return self._roll.zero_samples_required

    def raw_roll_window(self) -> tuple[float, float, int] | None:
        return self._roll.window()

    def raw_pitch_window(self) -> tuple[float, float, int] | None:
        return self._pitch.window()

    # ------------------------------------------------------------ hilo

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="roll-imu")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._thread = None

    def _map(self, values: Sequence[float]) -> tuple[float, float, float]:
        return tuple(
            self.config.axis_signs[i] * values[self.config.axis_mapping[i]] for i in range(3)
        )  # type: ignore[return-value]

    def _clear(self) -> None:
        self.roll_deg = self.pitch_deg = None
        self.received_ms = None
        self.raw_g = self.mapped_g = None
        self.roll_raw_deg = self.pitch_raw_deg = None
        self.accel_magnitude_g = self.tilt_from_vertical_deg = None
        self._bad_orientation_since = None
        self._roll.reset()
        self._pitch.reset()

    def _run(self) -> None:
        """Reintenta indefinidamente: un fallo I2C transitorio no puede dejar el
        roll muerto hasta el siguiente reinicio del servicio."""
        backoff = 0.5
        while not self._stop.is_set():
            try:
                from smbus2 import SMBus

                bus_number = int(self.config.bus.rsplit("-", 1)[-1])
                with SMBus(bus_number) as bus:
                    a = self.config.address
                    self.chip_id = bus.read_byte_data(a, _WHO_AM_I)
                    self.chip_name = _chip_name(self.chip_id)
                    bus.write_byte_data(a, 0x6B, 0x01)
                    bus.write_byte_data(
                        a, 0x19, max(0, round(1000 / self.config.sample_rate_hz) - 1)
                    )
                    bus.write_byte_data(a, 0x1A, 0x03)
                    if self.chip_id in _SEPARATE_ACCEL_FILTER:
                        bus.write_byte_data(a, _ACCEL_CONFIG_2, _ACCEL_DLPF_41HZ)
                    bus.write_byte_data(a, 0x1C, _ACCEL_BITS[self.config.accel_range_g])
                    bus.write_byte_data(a, 0x1B, _GYRO_BITS[self.config.gyro_range_dps])
                    self.error = ""
                    backoff = 0.5
                    self._sample_loop(bus)
            except Exception as exc:  # noqa: BLE001 - un fallo I2C no debe tumbar el proceso
                self.error = f"{exc.__class__.__name__}: {exc}"
                self._clear()
                self._stop.wait(backoff)
                backoff = min(10.0, backoff * 2)

    def _sample_loop(self, bus: object) -> None:
        interval = 1.0 / self.config.sample_rate_hz
        previous = time.monotonic()
        next_at = previous
        while not self._stop.is_set():
            raw = bus.read_i2c_block_data(self.config.address, 0x3B, 14)  # type: ignore[attr-defined]
            accel_scale = 32768.0 / self.config.accel_range_g
            gyro_scale = 32768.0 / self.config.gyro_range_dps
            accel_raw = tuple(_signed(raw[i], raw[i + 1]) / accel_scale for i in (0, 2, 4))
            gyro_raw = tuple(_signed(raw[i], raw[i + 1]) / gyro_scale for i in (8, 10, 12))
            now = time.monotonic()
            dt = min(0.1, max(0.0, now - previous))
            previous = now
            self.update(accel_raw, gyro_raw, dt, now)
            next_at += interval
            self._stop.wait(max(0.0, next_at - time.monotonic()))

    def update(
        self,
        accel_raw: Sequence[float],
        gyro_raw: Sequence[float],
        dt: float,
        now: float,
    ) -> None:
        """Un ciclo del filtro. Separado del bus para poder probarlo sin I2C."""
        accel_mapped = self._map(accel_raw)
        gyro_mapped = self._map(gyro_raw)
        accel = tuple(accel_mapped[i] - self.config.accel_bias_g[i] for i in range(3))
        gyro = tuple(gyro_mapped[i] - self.config.gyro_bias_dps[i] for i in range(3))
        self.raw_g = tuple(accel_raw)  # type: ignore[assignment]
        self.mapped_g = accel  # type: ignore[assignment]

        magnitude = math.sqrt(sum(v * v for v in accel))
        self.accel_magnitude_g = magnitude
        # Sólo con ~1 g limpio el vector apunta a donde apunta la gravedad.
        plausible = _MIN_ACCEL_G <= magnitude <= _MAX_ACCEL_G

        # Sistema del cuerpo: x adelante, y izquierda, z arriba. En reposo y a
        # nivel el acelerómetro mide +1 g en z. Balanceo positivo = costado
        # derecho abajo; cabeceo positivo = morro arriba, la misma convención
        # que el pitch del UM982.
        roll_raw = math.degrees(math.atan2(accel[1], accel[2]))
        pitch_raw = math.degrees(math.atan2(accel[0], math.hypot(accel[1], accel[2])))
        self.roll_raw_deg = roll_raw
        self.pitch_raw_deg = pitch_raw

        roll_acc = -roll_raw if self.config.roll_invert else roll_raw
        roll_acc += self.config.roll_offset_deg
        pitch_acc = pitch_raw + self.config.pitch_offset_deg

        if plausible:
            self._check_orientation(accel[2] / magnitude, now)

        # Compuerta: sin gravedad limpia, o con la máquina en marcha, el
        # acelerómetro no corrige. Se integra el giróscopo y nada más.
        gated = self.moving or not plausible
        self.accel_gated = gated

        # Regla de la mano derecha sobre los ejes del cuerpo: girar sobre
        # +adelante levanta el costado izquierdo, o sea balanceo positivo; girar
        # sobre +izquierda hunde el morro, así que el cabeceo lleva signo menos.
        self._roll.update(roll_acc, roll_raw, gyro[0], dt, now, gated=gated, plausible=plausible)
        self._pitch.update(
            pitch_acc, pitch_raw, -gyro[1], dt, now, gated=gated, plausible=plausible
        )

        if self._roll.estimate is None:
            self.roll_deg = self.pitch_deg = None
            return
        self.received_ms = now * 1000.0
        self._publish()

    def _check_orientation(self, cosine: float, now: float) -> None:
        """¿Sostiene la gravedad el eje que el mapeo llama vertical?

        Si no lo sostiene, `atan2` divide ruido entre ruido: sale un ángulo con
        aspecto de medida que en realidad es el vector girando al azar.
        """
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        self.tilt_from_vertical_deg = tilt
        if tilt > self.config.max_tilt_deg:
            if self._bad_orientation_since is None:
                self._bad_orientation_since = now
        else:
            self._bad_orientation_since = None
            self.orientation_ok = True
        if (
            self._bad_orientation_since is not None
            and now - self._bad_orientation_since >= _ORIENTATION_GRACE_S
        ):
            self.orientation_ok = False

    def _publish(self) -> None:
        """Decide si los valores del filtro se pueden usar para mover la broca."""
        if not self.orientation_ok:
            self.roll_deg = self.pitch_deg = None
            self.error = (
                f"El eje vertical del mapeo no sostiene la gravedad: "
                f"{self.tilt_from_vertical_deg:.0f}° de desvío. Nivele la máquina "
                "y pulse «Detectar ejes»"
            )
            return
        limit = self.config.max_roll_noise_deg
        unstable = [
            name
            for name, axis in (("balanceo", self._roll), ("cabeceo", self._pitch))
            if not self.moving and axis.noise_deg is not None and axis.noise_deg > limit
        ]
        self.roll_deg = None if "balanceo" in unstable else self._roll.estimate
        self.pitch_deg = None if "cabeceo" in unstable else self._pitch.estimate
        if unstable:
            peor = max(
                (self._roll.noise_deg or 0.0, self._pitch.noise_deg or 0.0),
            )
            self.error = (
                f"{' y '.join(unstable).capitalize()} inestable: ±{peor:.1f}° con la "
                "máquina detenida. Revise el montaje del sensor y el mapeo de ejes"
            )
            return
        self.error = ""
