from __future__ import annotations

import asyncio
import contextlib
import math
import time
from collections import deque
from datetime import UTC, datetime

from app.config import LEVER_LIMIT_M, AppConfig
from app.domain import Epoch, Heading
from app.geometry import Geodesy, enu_to_planar_lever, rotate_lever
from app.imu import RollSensor
from app.io import GnssSerial, Ntrip
from app.parser import parse_unicore

_FIXED_HEADING = {"NARROW_INT", "WIDE_INT", "L1_INT"}
_MAX_HEADING_AGE_MS = 500.0
_MAX_EPOCH_AGE_MS = 1000.0
_MAX_IMU_AGE_MS = 200.0

# Sin latido, un GNSS caído deja la última posición publicada para siempre y la
# pantalla no puede distinguirla de una viva.
_HEARTBEAT_MS = 500.0

# La calibración promedia sobre esta ventana. Diez épocas a 10 Hz son un segundo
# de datos, y el error RTK está correlacionado durante minutos: promediar un
# segundo no reduce el sesgo, sólo aparenta rigor.
_CALIBRATION_WINDOW_S = 60.0
_CALIBRATION_MIN_RATIO = 0.6

# Por encima de esto la máquina se movió durante la captura o el rumbo saltó: el
# promedio no representa un brazo rígido.
_CALIBRATION_MAX_SPREAD_M = 0.20
_STATIONARY_SPEED_MPS = 0.05

# Mínimo del PRD. Por debajo el rumbo fija peor y el error angular crece.
_MIN_BASELINE_M = 1.5


class Runtime:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.geodesy = Geodesy(config.projected_crs)
        self.epoch: Epoch | None = None
        self.heading: Heading | None = None
        self.gga: str | None = None
        self.latest: dict | None = None
        self.subscribers: set[asyncio.Queue[dict]] = set()
        window = int(config.gnss.rate_hz * _CALIBRATION_WINDOW_S * 1.5)
        self.calibration_samples: deque[tuple[Epoch, Heading]] = deque(maxlen=window)
        self._config_lock = asyncio.Lock()
        self._last_publish_ms: float | None = None
        self._heartbeat: asyncio.Task | None = None
        # Corre siempre: en 2D no entra en el cálculo, pero permite calibrarla y
        # avisar del desnivel. Si no hay sensor, queda en error y se ve.
        self.imu = RollSensor(config.imu)
        self.gnss = GnssSerial(config.gnss, self.on_line)
        self.ntrip = Ntrip(config.ntrip, self.gnss.write, lambda: self.gga)

    # ------------------------------------------------------------------ ciclo

    async def start(self) -> None:
        self.imu.start()
        await self.gnss.start()
        await self.ntrip.start()
        self._heartbeat = asyncio.create_task(self._heartbeat_loop(), name="heartbeat")

    async def stop(self) -> None:
        if self._heartbeat:
            self._heartbeat.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._heartbeat
            self._heartbeat = None
        await self.ntrip.stop()
        await self.gnss.stop()
        self.imu.stop()

    async def _heartbeat_loop(self) -> None:
        """Republica el estado aunque no lleguen tramas, para que la edad del
        dato crezca a la vista y una pantalla congelada se delate sola."""
        while True:
            await asyncio.sleep(_HEARTBEAT_MS / 1000.0)
            now = time.monotonic() * 1000.0
            if self._last_publish_ms is None or now - self._last_publish_ms >= _HEARTBEAT_MS:
                await self.recompute(now)

    async def on_line(self, line: str) -> None:
        # El UM982 real emite GNGGA aunque se configure con el comando GPGGA.
        if line.startswith("$") and len(line) >= 6 and line[3:6] == "GGA":
            self.gga = line
            return
        parsed = parse_unicore(line)
        if isinstance(parsed, Epoch):
            self.epoch = parsed
            if (
                self.heading is not None
                and self.heading.valid
                and parsed.received_ms - self.heading.received_ms <= _MAX_HEADING_AGE_MS
            ):
                self.calibration_samples.append((parsed, self.heading))
        elif isinstance(parsed, Heading):
            offset = self.config.gnss.heading_offset_deg
            self.heading = Heading(
                heading_deg=(parsed.heading_deg + offset) % 360.0,
                pitch_deg=parsed.pitch_deg,
                baseline_m=parsed.baseline_m,
                sol_status=parsed.sol_status,
                pos_type=parsed.pos_type,
                heading_stddev_deg=parsed.heading_stddev_deg,
                pitch_stddev_deg=parsed.pitch_stddev_deg,
                received_ms=parsed.received_ms,
                crc_ok=parsed.crc_ok,
            )
        else:
            return
        await self.recompute()

    async def apply_config(self, config: AppConfig, ntrip_secret_changed: bool = False) -> None:
        """Aplica configuración operativa sin cerrar el puerto GNSS."""
        async with self._config_lock:
            previous = self.config
            # La contraseña vive fuera del dataclass, así que cambiarla no altera
            # la comparación: hay que avisar aparte o la sesión sigue con la vieja.
            ntrip_changed = config.ntrip != previous.ntrip or ntrip_secret_changed
            imu_changed = config.imu != previous.imu
            if ntrip_changed:
                await self.ntrip.stop()
            if imu_changed:
                self.imu.stop()
            self.config = config
            if config.projected_crs != previous.projected_crs:
                self.geodesy = Geodesy(config.projected_crs)
            if imu_changed:
                self.imu = RollSensor(config.imu)
                self.imu.start()
            if ntrip_changed:
                self.ntrip = Ntrip(config.ntrip, self.gnss.write, lambda: self.gga)
                await self.ntrip.start()
            await self.recompute()

    async def set_port_released(self, released: bool) -> None:
        """Suelta o recupera /dev/serial0 sin reiniciar el servicio.

        Al liberar también se corta el NTRIP: no tiene sentido descargar
        correcciones que no se pueden entregar al receptor, y así no se martillea
        al caster mientras el puerto está fuera.
        """
        async with self._config_lock:
            if released == self.gnss.released:
                return
            if released:
                await self.ntrip.stop()
                await self.gnss.release()
            else:
                await self.gnss.resume()
                self.ntrip = Ntrip(self.config.ntrip, self.gnss.write, lambda: self.gga)
                await self.ntrip.start()
            await self.recompute()

    @property
    def mode(self) -> str:
        if self.config.use_imu:
            return "3D_IMU"
        if self.config.use_pitch:
            return "2D_PITCH"
        return "2D_DIRECT"

    def _lever_enu(self, heading: Heading, roll: float | None) -> tuple[float, float, float]:
        """Brazo en ENU según el modo activo.

        `2D_PITCH` usa el pitch de la baseline ANT1→ANT2 del UM982 y no toca la
        IMU: es la misma rotación 3D con roll = 0.
        """
        lever = self.config.lever
        if self.config.use_imu:
            return rotate_lever(lever.forward_m, lever.left_m, lever.down_m,
                                heading.heading_deg, heading.pitch_deg, roll or 0.0, True)
        if self.config.use_pitch:
            return rotate_lever(lever.forward_m, lever.left_m, lever.down_m,
                                heading.heading_deg, heading.pitch_deg, 0.0, True)
        return rotate_lever(lever.forward_m, lever.left_m, lever.down_m, heading.heading_deg)

    # ----------------------------------------------------------- calibración

    def calibration_2d(self, easting_m: float, northing_m: float) -> dict:
        """Promedia la ventana de épocas FIXED y devuelve adelante/izquierda.

        Rechaza el resultado si el brazo excede el tope físico o si las
        soluciones individuales discrepan demasiado entre sí.
        """
        if self.gnss.released:
            raise ValueError(
                "el puerto del receptor está liberado; vuelva a conectarlo antes de calibrar"
            )
        accepted = self._accepted_calibration_samples()
        required = self._required_samples()
        if len(accepted) < required:
            raise ValueError(
                f"faltan épocas con RTK fijo y máquina quieta: {len(accepted)}/{required}. "
                "Mantenga la broca sobre el punto y la máquina detenida"
            )
        levers: list[tuple[float, float]] = []
        for epoch, heading in accepted:
            east, north = self.geodesy.target_delta_enu(
                epoch.latitude,
                epoch.longitude,
                epoch.ellipsoidal_height_m,
                easting_m,
                northing_m,
            )
            planar_forward, left = enu_to_planar_lever(east, north, heading.heading_deg)
            # El modelo directo en 2D+pitch proyecta el brazo como
            #   x2 = cos(p)·forward + sin(p)·down
            # así que la calibración tiene que deshacer exactamente eso, o el
            # brazo saldría sesgado por la inclinación del momento.
            if self.config.use_pitch and not self.config.use_imu:
                pitch = math.radians(heading.pitch_deg)
                planar_forward = (
                    planar_forward - math.sin(pitch) * self.config.lever.down_m
                ) / math.cos(pitch)
            levers.append((planar_forward, left))

        count = len(levers)
        forward = sum(item[0] for item in levers) / count
        left = sum(item[1] for item in levers) / count

        for name, value in (("adelante", forward), ("izquierda", left)):
            if abs(value) > LEVER_LIMIT_M:
                raise ValueError(
                    f"el brazo calculado ({name} {value:.1f} m) excede el tope de "
                    f"±{LEVER_LIMIT_M:g} m. Revise el sistema de coordenadas y que el "
                    "Este y el Norte del punto estén completos"
                )

        spread = max(math.hypot(f - forward, l - left) for f, l in levers)
        deviation = math.sqrt(
            sum((f - forward) ** 2 + (l - left) ** 2 for f, l in levers) / count
        )
        if spread > _CALIBRATION_MAX_SPREAD_M:
            raise ValueError(
                f"las soluciones discrepan hasta {spread * 100:.0f} cm entre sí. "
                "La máquina se movió o el rumbo saltó durante la captura; repita"
            )
        return {
            "forward_m": forward,
            "left_m": left,
            "horizontal_m": math.hypot(forward, left),
            "samples": count,
            "window_s": self._sample_span_s(accepted),
            "deviation_m": deviation,
            "spread_m": spread,
        }

    def _required_samples(self) -> int:
        expected = self.config.gnss.rate_hz * _CALIBRATION_WINDOW_S
        return max(60, int(expected * _CALIBRATION_MIN_RATIO))

    def calibration_sample_count(self) -> int:
        return len(self._accepted_calibration_samples())

    @staticmethod
    def _sample_span_s(samples: list[tuple[Epoch, Heading]]) -> float:
        if len(samples) < 2:
            return 0.0
        return (samples[-1][0].received_ms - samples[0][0].received_ms) / 1000.0

    def _accepted_calibration_samples(self) -> list[tuple[Epoch, Heading]]:
        now = time.monotonic() * 1000.0
        window_ms = _CALIBRATION_WINDOW_S * 1000.0
        return [
            (epoch, heading)
            for epoch, heading in self.calibration_samples
            if epoch.fix == "FIXED"
            and epoch.crc_ok
            and heading.valid
            and heading.pos_type in _FIXED_HEADING
            # Calibrar en movimiento produce un brazo sesgado que parece válido.
            and epoch.speed_mps is not None
            and epoch.speed_mps <= _STATIONARY_SPEED_MPS
            and now - epoch.received_ms <= window_ms
            and abs(epoch.received_ms - heading.received_ms) <= _MAX_HEADING_AGE_MS
        ]

    # -------------------------------------------------------------- telemetría

    def _tilt_error_m(self, heading: Heading, measured_roll: float | None) -> float | None:
        """Error horizontal que el modo 2D descarta por el pitch del UM982.

        Es una cota inferior: sin IMU el roll es desconocido y aporta lo suyo.
        """
        if self.config.use_imu:
            return None
        lever = self.config.lever
        pitch = 0.0 if self.config.use_pitch else (heading.pitch_deg or 0.0)
        roll = measured_roll or 0.0
        if not pitch and not roll:
            return None
        base = rotate_lever(
            lever.forward_m, lever.left_m, lever.down_m, heading.heading_deg,
            heading.pitch_deg if self.config.use_pitch else 0.0, 0.0, True,
        )
        real = rotate_lever(
            lever.forward_m, lever.left_m, lever.down_m, heading.heading_deg,
            heading.pitch_deg or 0.0, roll, True,
        )
        return math.hypot(real[0] - base[0], real[1] - base[1])

    async def recompute(self, now_ms: float | None = None) -> dict | None:
        # Se publica aunque todavía no haya llegado ninguna época: si no, el
        # stream queda mudo desde el arranque y ni la pantalla ni `gpsevt` pueden
        # distinguir «arrancando» de «servicio muerto».
        epoch, heading = self.epoch, self.heading
        now = now_ms if now_ms is not None else time.monotonic() * 1000.0
        epoch_age = now - epoch.received_ms if epoch else None
        heading_age = now - heading.received_ms if heading else None
        measured_roll = self.imu.roll_deg
        roll = measured_roll if self.config.use_imu else None
        imu_age = now - self.imu.received_ms if self.imu.received_ms else None
        imu_error = self.imu.error

        warnings: list[str] = []
        reason = ""
        valid = (
            epoch is not None
            and epoch.crc_ok
            and epoch_age is not None
            and epoch_age <= _MAX_EPOCH_AGE_MS
            and heading is not None
            and heading.valid
            and heading_age is not None
            and heading_age <= _MAX_HEADING_AGE_MS
        )
        if not valid or self.gnss.released:
            if self.gnss.released:
                valid = False
                reason = (
                    "Puerto del receptor liberado a propósito. No hay posición hasta "
                    "volver a conectarlo desde esta pantalla"
                )
            elif epoch is None:
                reason = (
                    "Esperando la primera posición del receptor. Puede tardar unos "
                    "segundos tras arrancar el servicio"
                )
            elif heading is None:
                reason = "Sin rumbo: el receptor aún no entrega la orientación de las antenas"
            elif epoch_age > _MAX_EPOCH_AGE_MS:
                reason = f"Sin datos del GNSS desde hace {epoch_age / 1000.0:.0f} s"
            else:
                reason = "Posición o rumbo inválido o antiguo"
        elif self.config.use_imu and (
            roll is None or imu_age is None or imu_age > _MAX_IMU_AGE_MS
        ):
            valid = False
            reason = (
                f"IMU sin responder: {imu_error}" if imu_error else "IMU no disponible o antigua"
            )
        if reason:
            warnings.append(reason)

        # Avisos que no invalidan la posición pero cambian cuánto vale.
        if epoch is not None and epoch.fix != "FIXED":
            warnings.append(
                f"Sin RTK fijo ({epoch.fix}): la posición puede tener metros de error"
            )
        if heading is not None and heading.valid and heading.pos_type not in _FIXED_HEADING:
            warnings.append(f"Rumbo sin fijar ({heading.pos_type}): la broca puede desviarse")
        tilt_error = self._tilt_error_m(heading, measured_roll) if heading else None
        if tilt_error is not None and tilt_error >= 0.05:
            warnings.append(
                f"Máquina inclinada {abs(heading.pitch_deg):.1f}°: el modo 2D descarta "
                f"al menos {tilt_error * 100:.0f} cm de corrección. Nivele antes de perforar"
            )

        antenna_e = antenna_n = None
        if epoch is not None:
            antenna_e, antenna_n = self.geodesy.projected(
                epoch.latitude, epoch.longitude, epoch.ellipsoidal_height_m
            )
        virtual = {
            "valid": False,
            "latitude": None,
            "longitude": None,
            "easting_m": None,
            "northing_m": None,
            "ellipsoidal_height_m": None,
            "reason": reason,
        }
        if valid and epoch is not None and heading is not None:
            enu = self._lever_enu(heading, roll)
            lat, lon, height, easting, northing = self.geodesy.offset(
                epoch.latitude, epoch.longitude, epoch.ellipsoidal_height_m, enu
            )
            virtual.update(
                valid=True,
                latitude=lat,
                longitude=lon,
                easting_m=easting,
                northing_m=northing,
                ellipsoidal_height_m=height,
                reason="",
            )

        precise = (
            valid
            and epoch is not None
            and heading is not None
            and epoch.fix == "FIXED"
            and heading.pos_type in _FIXED_HEADING
        )
        speed = epoch.speed_mps if epoch else None
        stationary = speed is not None and speed <= _STATIONARY_SPEED_MPS
        timestamp = (epoch.utc if epoch else None) or datetime.now(UTC)
        accepted = 0 if self.gnss.released else self.calibration_sample_count()
        required = self._required_samples()
        baseline = heading.baseline_m if heading else None
        payload = {
            "timestamp": timestamp.isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "time_source": "GNSS" if epoch and epoch.utc else "SYSTEM",
            "data_age_ms": int(max(epoch_age or 0.0, heading_age or 0.0, imu_age or 0.0)),
            "calculation_mode": self.mode,
            "calibration": {
                "fixed_samples": accepted,
                "required_samples": required,
                "ready": accepted >= required,
                "window_s": _CALIBRATION_WINDOW_S,
                "stationary": stationary,
            },
            "gnss": {
                "connected": self.gnss.connected,
                "port": self.config.gnss.port,
                "port_released": self.gnss.released,
                "fix": epoch.fix if epoch else "NONE",
                "pos_type": epoch.pos_type if epoch else "UNKNOWN",
                "satellites_used": epoch.satellites_used if epoch else 0,
                "latitude": epoch.latitude if epoch else None,
                "longitude": epoch.longitude if epoch else None,
                "ellipsoidal_height_m": epoch.ellipsoidal_height_m if epoch else None,
                "orthometric_height_m": epoch.orthometric_height_m if epoch else None,
                "undulation_m": epoch.undulation_m if epoch else None,
                "easting_m": antenna_e,
                "northing_m": antenna_n,
                "speed_mps": speed,
                "speed_kmh": speed * 3.6 if speed is not None else None,
                "sigma_horizontal_m": epoch.sigma_horizontal_m if epoch else None,
                "epoch_age_ms": int(epoch_age) if epoch_age is not None else None,
            },
            "attitude": {
                "heading_deg": heading.heading_deg if heading else None,
                "pitch_deg": heading.pitch_deg if heading else None,
                "roll_deg": roll,
                "heading_valid": bool(heading and heading.valid),
                "tilt_valid": not self.config.use_imu or roll is not None,
                "tilt_degraded": self.config.use_imu and not valid,
                "imu_stable": self.config.use_imu and roll is not None,
                "imu_error": imu_error,
                "roll_measured_deg": measured_roll,
                "roll_used": self.config.use_imu,
                "baseline_m": baseline,
                "baseline_ok": baseline is None or baseline >= _MIN_BASELINE_M,
                "baseline_min_m": _MIN_BASELINE_M,
                "tilt_error_m": tilt_error,
                "heading_stddev_deg": heading.heading_stddev_deg if heading else None,
                "pitch_stddev_deg": heading.pitch_stddev_deg if heading else None,
                "motion_state": "STATIONARY" if stationary else "MOVING",
            },
            "imu": {
                "ok": measured_roll is not None,
                "error": imu_error,
                "age_ms": int(imu_age) if imu_age is not None else None,
                "roll_deg": measured_roll,
                "roll_raw_deg": self.imu.roll_raw_deg,
                "raw_g": list(self.imu.raw_g) if self.imu.raw_g else None,
                "mapped_g": list(self.imu.mapped_g) if self.imu.mapped_g else None,
                "axis_mapping": list(self.config.imu.axis_mapping),
                "axis_signs": list(self.config.imu.axis_signs),
                "roll_offset_deg": self.config.imu.roll_offset_deg,
                "roll_invert": self.config.imu.roll_invert,
            },
            "virtual_gps": virtual,
            "ntrip": {"connected": self.ntrip.state == "CONNECTED", "state": self.ntrip.state},
            "quality": {
                "mode": "PRECISION" if precise else ("APPROACH" if valid else "INVALID"),
                "calibration_valid": True,
                "position_valid": bool(valid),
                "warnings": warnings,
            },
        }
        self.latest = payload
        self._last_publish_ms = now
        for queue in tuple(self.subscribers):
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(payload)
        return payload

    def subscribe(self) -> asyncio.Queue[dict]:
        queue: asyncio.Queue[dict] = asyncio.Queue(maxsize=1)
        self.subscribers.add(queue)
        if self.latest:
            queue.put_nowait(self.latest)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict]) -> None:
        self.subscribers.discard(queue)
