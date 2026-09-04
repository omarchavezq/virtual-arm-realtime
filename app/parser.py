from __future__ import annotations

import time
from datetime import UTC, datetime, timedelta

from app.domain import Epoch, Heading

_POLY = 0xEDB88320


def _build_crc_table() -> tuple[int, ...]:
    out: list[int] = []
    for index in range(256):
        crc = index
        for _ in range(8):
            crc = (crc >> 1) ^ _POLY if crc & 1 else crc >> 1
        out.append(crc)
    return tuple(out)


_TABLE = _build_crc_table()


def monotonic_ms() -> float:
    return time.monotonic() * 1000.0


def crc32_unicore(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = ((crc >> 8) & 0x00FFFFFF) ^ _TABLE[(crc ^ byte) & 0xFF]
    return crc


def _float(fields: list[str], index: int) -> float | None:
    try:
        return float(fields[index])
    except (IndexError, ValueError):
        return None


def _int(fields: list[str], index: int) -> int:
    value = _float(fields, index)
    return int(value) if value is not None else 0


def _gps_time(header: list[str]) -> datetime | None:
    for i in range(len(header) - 1):
        try:
            week = int(float(header[i]))
            seconds = float(header[i + 1])
        except ValueError:
            continue
        if not 1500 <= week <= 3000:
            continue
        if seconds > 604800:
            seconds /= 1000.0
        if 0 <= seconds <= 604800:
            return datetime(1980, 1, 6, tzinfo=UTC) + timedelta(weeks=week, seconds=seconds)
    return None


def _fix(pos_type: str) -> str:
    if pos_type in {"NARROW_INT", "WIDE_INT", "L1_INT"}:
        return "FIXED"
    if pos_type in {"NARROW_FLOAT", "IONOFREE_FLOAT", "L1_FLOAT"}:
        return "FLOAT"
    if pos_type in {"PSRDIFF", "WAAS"}:
        return "DGPS"
    if pos_type in {"SINGLE", "FIXEDPOS"}:
        return "SINGLE"
    return "NONE"


def parse_unicore(line: str) -> Epoch | Heading | None:
    text = line.strip()
    if not text.startswith("#") or "*" not in text or ";" not in text:
        return None
    body, crc_text = text[1:].rsplit("*", 1)
    try:
        crc_ok = crc32_unicore(body.encode("ascii", errors="replace")) == int(crc_text[:8], 16)
    except ValueError:
        return None
    header_text, data_text = body.split(";", 1)
    header = header_text.split(",")
    fields = [part.strip() for part in data_text.split(",")]
    name = header[0].upper()
    now = monotonic_ms()

    if name.startswith("BESTNAV"):
        lat, lon, height_msl = _float(fields, 2), _float(fields, 3), _float(fields, 4)
        if lat is None or lon is None or height_msl is None:
            return None
        undulation = _float(fields, 5) or 0.0
        pos_type = fields[1].upper() if len(fields) > 1 else "UNKNOWN"
        return Epoch(
            latitude=lat,
            longitude=lon,
            ellipsoidal_height_m=height_msl + undulation,
            orthometric_height_m=height_msl,
            undulation_m=undulation,
            fix=_fix(pos_type),
            pos_type=pos_type,
            satellites_used=_int(fields, 14),
            sigma_lat_m=_float(fields, 7),
            sigma_lon_m=_float(fields, 8),
            speed_mps=_float(fields, 25),
            utc=_gps_time(header),
            received_ms=now,
            crc_ok=crc_ok,
        )

    if name.startswith("UNIHEADING"):
        baseline, heading, pitch = _float(fields, 2), _float(fields, 3), _float(fields, 4)
        if baseline is None or heading is None or pitch is None:
            return None
        return Heading(
            heading_deg=heading,
            pitch_deg=pitch,
            baseline_m=baseline,
            sol_status=fields[0].upper() if fields else "UNKNOWN",
            pos_type=fields[1].upper() if len(fields) > 1 else "UNKNOWN",
            heading_stddev_deg=_float(fields, 6),
            pitch_stddev_deg=_float(fields, 7),
            received_ms=now,
            crc_ok=crc_ok,
        )
    return None
