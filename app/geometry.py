from __future__ import annotations

import math
from dataclasses import dataclass, field

from pyproj import Transformer


def rotate_lever(
    forward_m: float,
    left_m: float,
    down_m: float,
    heading_deg: float,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    use_imu: bool = False,
) -> tuple[float, float, float]:
    """Brazo FLU (adelante, izquierda, arriba) a ENU.

    En 2D se ignoran pitch y roll. En 3D se conserva la convención del proyecto
    original: Rz(90-heading) * Ry(-pitch) * Rx(roll).
    """
    x, y, z = forward_m, left_m, -down_m
    if not use_imu:
        h = math.radians(heading_deg)
        return (
            math.sin(h) * x - math.cos(h) * y,
            math.cos(h) * x + math.sin(h) * y,
            z,
        )

    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z
    x2 = cp * x - sp * z1
    z2 = sp * x + cp * z1
    h = math.radians(heading_deg)
    return (
        math.sin(h) * x2 - math.cos(h) * y1,
        math.cos(h) * x2 + math.sin(h) * y1,
        z2,
    )


@dataclass(slots=True)
class Geodesy:
    projected_crs: str
    _to_ecef: Transformer = field(init=False, repr=False)
    _from_ecef: Transformer = field(init=False, repr=False)
    _to_projected: Transformer = field(init=False, repr=False)
    _from_projected: Transformer = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        self._from_ecef = Transformer.from_crs("EPSG:4978", "EPSG:4979", always_xy=True)
        self._to_projected = Transformer.from_crs("EPSG:4979", self.projected_crs, always_xy=True)
        self._from_projected = Transformer.from_crs(self.projected_crs, "EPSG:4979", always_xy=True)

    def projected(self, latitude: float, longitude: float, height_m: float) -> tuple[float, float]:
        e, n, _ = self._to_projected.transform(longitude, latitude, height_m)
        return float(e), float(n)

    def offset(
        self,
        latitude: float,
        longitude: float,
        height_m: float,
        enu: tuple[float, float, float],
    ) -> tuple[float, float, float, float, float]:
        x, y, z = self._to_ecef.transform(longitude, latitude, height_m)
        lat = math.radians(latitude)
        lon = math.radians(longitude)
        east, north, up = enu
        dx = -math.sin(lon) * east - math.sin(lat) * math.cos(lon) * north + math.cos(lat) * math.cos(lon) * up
        dy = math.cos(lon) * east - math.sin(lat) * math.sin(lon) * north + math.cos(lat) * math.sin(lon) * up
        dz = math.cos(lat) * north + math.sin(lat) * up
        out_lon, out_lat, out_h = self._from_ecef.transform(x + dx, y + dy, z + dz)
        out_e, out_n = self.projected(out_lat, out_lon, out_h)
        return float(out_lat), float(out_lon), float(out_h), out_e, out_n

    def target_delta_enu(
        self,
        antenna_latitude: float,
        antenna_longitude: float,
        antenna_height_m: float,
        target_easting_m: float,
        target_northing_m: float,
    ) -> tuple[float, float]:
        """Desplazamiento EN horizontal ANT1→objetivo desde coordenadas de grilla."""
        target_lon, target_lat, _ = self._from_projected.transform(
            target_easting_m, target_northing_m, antenna_height_m
        )
        ax, ay, az = self._to_ecef.transform(
            antenna_longitude, antenna_latitude, antenna_height_m
        )
        tx, ty, tz = self._to_ecef.transform(target_lon, target_lat, antenna_height_m)
        dx, dy, dz = tx - ax, ty - ay, tz - az
        lat = math.radians(antenna_latitude)
        lon = math.radians(antenna_longitude)
        east = -math.sin(lon) * dx + math.cos(lon) * dy
        north = (
            -math.sin(lat) * math.cos(lon) * dx
            - math.sin(lat) * math.sin(lon) * dy
            + math.cos(lat) * dz
        )
        return float(east), float(north)


def enu_to_planar_lever(east_m: float, north_m: float, heading_deg: float) -> tuple[float, float]:
    """ENU horizontal a (adelante, izquierda) para el heading de ANT1→ANT2."""
    h = math.radians(heading_deg)
    forward = math.sin(h) * east_m + math.cos(h) * north_m
    left = -math.cos(h) * east_m + math.sin(h) * north_m
    return forward, left
