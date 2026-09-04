from __future__ import annotations

import math
import threading
import time
from collections.abc import Sequence

from app.config import ImuConfig

_ACCEL_BITS = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}
_GYRO_BITS = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}


def _signed(high: int, low: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value >= 32768 else value


class RollSensor:
    """Roll inmediato del MPU6050, sin compuerta ni espera de estacionamiento."""

    def __init__(self, config: ImuConfig) -> None:
        self.config = config
        self.roll_deg: float | None = None
        self.received_ms: float | None = None
        self.error = ""
        # Diagnóstico: lo que ve el sensor antes y después del mapeo de ejes, y
        # el roll sin corregir. Sin esto no hay forma de calibrarlo en campo.
        self.raw_g: tuple[float, float, float] | None = None
        self.mapped_g: tuple[float, float, float] | None = None
        self.roll_raw_deg: float | None = None
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
                self.roll_deg = None
                self.received_ms = None
                self.raw_g = self.mapped_g = self.roll_raw_deg = None
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
            accel_mapped = self._map(accel_raw)
            gyro_mapped = self._map(gyro_raw)
            accel = tuple(accel_mapped[i] - self.config.accel_bias_g[i] for i in range(3))
            gyro = tuple(gyro_mapped[i] - self.config.gyro_bias_dps[i] for i in range(3))
            self.raw_g = accel_raw
            self.mapped_g = accel  # type: ignore[assignment]
            roll_acc = math.degrees(math.atan2(accel[1], accel[2]))
            self.roll_raw_deg = roll_acc
            if self.config.roll_invert:
                roll_acc = -roll_acc
            roll_acc += self.config.roll_offset_deg
            now = time.monotonic()
            dt = min(0.1, max(0.0, now - previous))
            previous = now
            if self.roll_deg is None:
                self.roll_deg = roll_acc
            else:
                predicted = self.roll_deg + gyro[0] * dt
                self.roll_deg = 0.96 * predicted + 0.04 * roll_acc
            self.received_ms = now * 1000.0
            next_at += interval
            self._stop.wait(max(0.0, next_at - time.monotonic()))
