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

# Fuera de esta banda el acelerómetro no está midiendo sólo gravedad: la máquina
# acelera, frena o vibra fuerte. El ángulo que saldría no sería inclinación.
_MIN_ACCEL_G = 0.85
_MAX_ACCEL_G = 1.15

# La orientación se declara mala sólo si se sostiene. Un bache no puede dejar al
# servicio sin roll, pero un mapeo cruzado no se arregla solo.
_ORIENTATION_GRACE_S = 2.0

# Ventana sobre la que se mide la dispersión del roll del acelerómetro.
_NOISE_WINDOW_S = 1.0

# Tras este rato integrando sólo el giróscopo, la estimación ya arrastra deriva
# de sesgo: al volver a haber gravedad limpia se vuelve a partir de ella en vez
# de acercarse despacio. Con la máquina parada, el acelerómetro es la verdad.
_RESNAP_AFTER_GATED_S = 1.0


def _signed(high: int, low: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value


class RollSensor:
    """Roll del MPU6050, con compuerta por movimiento y control de plausibilidad.

    Un roll equivocado no se nota: corre la broca en silencio y la pantalla
    sigue diciendo PRECISION. Por eso el sensor calla —`roll_deg = None`, que
    invalida la posición en 3D— en vez de publicar un número que no puede
    sostener.
    """

    def __init__(self, config: ImuConfig) -> None:
        self.config = config
        # Valor de fiar, el único que entra en el cálculo. None = no publicable.
        self.roll_deg: float | None = None
        self.received_ms: float | None = None
        self.error = ""
        # Diagnóstico: lo que ve el sensor antes y después del mapeo de ejes, y
        # el roll sin corregir. Sin esto no hay forma de calibrarlo en campo.
        self.raw_g: tuple[float, float, float] | None = None
        self.mapped_g: tuple[float, float, float] | None = None
        self.roll_raw_deg: float | None = None
        # Salida del filtro se pueda publicar o no: sirve para ver por qué no.
        self.roll_estimate_deg: float | None = None
        self.roll_noise_deg: float | None = None
        self.accel_magnitude_g: float | None = None
        self.tilt_from_vertical_deg: float | None = None
        self.orientation_ok = True
        self.accel_gated = False
        # Lo escribe el Runtime en cada época. Con la máquina en marcha el
        # acelerómetro mide fuerza específica: una frenada se lee como ladeo.
        self.moving = False
        self._estimate: float | None = None
        self._recent: deque[float] = deque(
            maxlen=max(2, int(config.sample_rate_hz * _NOISE_WINDOW_S))
        )
        self._bad_orientation_since: float | None = None
        self._gated_since: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="roll-mpu6050")
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
        self.roll_deg = None
        self.received_ms = None
        self.raw_g = self.mapped_g = self.roll_raw_deg = None
        self.roll_estimate_deg = self.roll_noise_deg = None
        self.accel_magnitude_g = self.tilt_from_vertical_deg = None
        self._estimate = None
        self._bad_orientation_since = None
        self._gated_since = None
        self._recent.clear()

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
                    bus.write_byte_data(a, 0x6B, 0x01)
                    bus.write_byte_data(
                        a, 0x19, max(0, round(1000 / self.config.sample_rate_hz) - 1)
                    )
                    bus.write_byte_data(a, 0x1A, 0x03)
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

        roll_acc = math.degrees(math.atan2(accel[1], accel[2]))
        self.roll_raw_deg = roll_acc
        if self.config.roll_invert:
            roll_acc = -roll_acc
        roll_acc += self.config.roll_offset_deg

        if plausible:
            self._check_orientation(accel[2] / magnitude, now)

        # Compuerta: sin gravedad limpia, o con la máquina en marcha, el
        # acelerómetro no corrige. Se integra el giróscopo y nada más.
        gated = self.moving or not plausible
        self.accel_gated = gated

        drifted = (
            self._gated_since is not None and now - self._gated_since >= _RESNAP_AFTER_GATED_S
        )
        if gated:
            if self._gated_since is None:
                self._gated_since = now
        else:
            self._gated_since = None

        if self._estimate is None:
            # Hay que partir de algún lado, y el giróscopo solo no sabe dónde
            # está el suelo. Sin una lectura limpia no se arranca.
            if not plausible:
                self.roll_deg = None
                return
            self._estimate = roll_acc
        elif gated:
            self._estimate += gyro[0] * dt
        elif drifted:
            self._estimate = roll_acc
            self._recent.clear()
            self.roll_noise_deg = None
        else:
            alpha = self.config.filter_tau_s / (self.config.filter_tau_s + dt)
            self._estimate = alpha * (self._estimate + gyro[0] * dt) + (1.0 - alpha) * roll_acc

        if not gated:
            self._recent.append(roll_acc)
            if len(self._recent) >= 2:
                self.roll_noise_deg = statistics.pstdev(self._recent)

        self.roll_estimate_deg = self._estimate
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
        """Decide si el valor del filtro se puede usar para mover la broca."""
        if not self.orientation_ok:
            self.roll_deg = None
            self.error = (
                f"El eje vertical del mapeo no sostiene la gravedad: "
                f"{self.tilt_from_vertical_deg:.0f}° de desvío. Nivele la máquina "
                "y pulse «Detectar ejes»"
            )
            return
        noise = self.roll_noise_deg
        if not self.moving and noise is not None and noise > self.config.max_roll_noise_deg:
            self.roll_deg = None
            self.error = (
                f"Roll inestable: ±{noise:.1f}° con la máquina detenida. Revise el "
                "montaje del sensor y el mapeo de ejes"
            )
            return
        self.error = ""
        self.roll_deg = self._estimate
