from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime


@dataclass(frozen=True, slots=True)
class Epoch:
    latitude: float
    longitude: float
    ellipsoidal_height_m: float
    orthometric_height_m: float
    undulation_m: float
    fix: str
    pos_type: str
    satellites_used: int
    sigma_lat_m: float | None
    sigma_lon_m: float | None
    speed_mps: float | None
    utc: datetime | None
    received_ms: float
    crc_ok: bool

    @property
    def sigma_horizontal_m(self) -> float | None:
        if self.sigma_lat_m is None or self.sigma_lon_m is None:
            return None
        return (self.sigma_lat_m**2 + self.sigma_lon_m**2) ** 0.5


@dataclass(frozen=True, slots=True)
class Heading:
    heading_deg: float
    pitch_deg: float
    baseline_m: float
    sol_status: str
    pos_type: str
    heading_stddev_deg: float | None
    pitch_stddev_deg: float | None
    received_ms: float
    crc_ok: bool

    @property
    def valid(self) -> bool:
        return self.sol_status == "SOL_COMPUTED" and self.crc_ok

