from __future__ import annotations

import json
import os
import tomllib
from dataclasses import dataclass
from pathlib import Path

# Tope físico del brazo ANT1→broca. Una máquina de perforación no tiene 20 m
# de brazo: valores mayores sólo salen de un CRS equivocado o de un error de
# tecleo en la calibración, y hay que rechazarlos antes de escribirlos.
LEVER_LIMIT_M = 20.0


@dataclass(frozen=True, slots=True)
class GnssConfig:
    port: str
    baudrate: int
    rate_hz: int
    heading_offset_deg: float


@dataclass(frozen=True, slots=True)
class LeverConfig:
    forward_m: float
    left_m: float
    down_m: float


@dataclass(frozen=True, slots=True)
class NtripConfig:
    host: str
    port: int
    mountpoint: str
    username: str
    password_file: str
    gga_interval_s: int

    def password(self) -> str:
        path = Path(self.password_file)
        data = json.loads(path.read_text(encoding="utf-8"))
        password = str(data.get("ntrip_password", ""))
        if not password:
            raise ValueError(f"falta ntrip_password en {path}")
        return password

    def has_password(self) -> bool:
        try:
            return bool(self.password())
        except (OSError, ValueError, json.JSONDecodeError):
            return False

    def set_password(self, password: str) -> None:
        if not password:
            return
        path = Path(self.password_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        values: dict[str, str] = {}
        if path.exists():
            try:
                loaded = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(loaded, dict):
                    values = {str(k): str(v) for k, v in loaded.items()}
            except (OSError, json.JSONDecodeError):
                values = {}
        values["ntrip_password"] = password
        temporary = path.with_name(f".{path.name}.tmp")
        descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(values, handle, ensure_ascii=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        path.chmod(0o600)


@dataclass(frozen=True, slots=True)
class ImuConfig:
    bus: str
    address: int
    sample_rate_hz: int
    accel_range_g: int
    gyro_range_dps: int
    gyro_bias_dps: tuple[float, float, float]
    accel_bias_g: tuple[float, float, float]
    axis_mapping: tuple[int, int, int]
    axis_signs: tuple[int, int, int]
    roll_offset_deg: float
    # Invierte el signo del roll sin tocar el mapeo de ejes, como el botón
    # «invert roll» de AgOpenGPS: es la corrección que más se necesita en campo.
    roll_invert: bool = False


@dataclass(frozen=True, slots=True)
class AppConfig:
    gnss: GnssConfig
    projected_crs: str
    lever: LeverConfig
    use_imu: bool
    ntrip: NtripConfig
    imu: ImuConfig
    # Corrige el brazo con el pitch del UM982 (dos antenas), sin IMU.
    use_pitch: bool = False


def _triple(values: object, name: str, cast: type = float) -> tuple:
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{name} debe contener exactamente 3 valores")
    return tuple(cast(v) for v in values)


def _exact_keys(
    section: dict, name: str, expected: set[str], optional: frozenset[str] = frozenset()
) -> None:
    missing = expected - set(section)
    unknown = set(section) - expected - optional
    if missing or unknown:
        raise ValueError(
            f"sección {name}: faltan {sorted(missing)}, sobran {sorted(unknown)}"
        )


def load_config(path: str | Path | None = None) -> AppConfig:
    config_path = Path(path or os.getenv("VIRTUAL_ARM_CONFIG", "config.toml"))
    with config_path.open("rb") as handle:
        raw = tomllib.load(handle)

    allowed = {"gnss", "coordinates", "lever", "calculation", "ntrip", "imu"}
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"secciones no admitidas: {sorted(unknown)}")

    g = raw["gnss"]
    c = raw["coordinates"]
    lever = raw["lever"]
    calc = raw["calculation"]
    n = raw["ntrip"]
    i = raw["imu"]

    _exact_keys(g, "gnss", {"port", "baudrate", "rate_hz", "heading_offset_deg"})
    _exact_keys(c, "coordinates", {"projected_crs"})
    _exact_keys(lever, "lever", {"forward_m", "left_m", "down_m"})
    # `use_pitch` es opcional: los config.toml anteriores no lo traen.
    _exact_keys(calc, "calculation", {"use_imu"}, frozenset({"use_pitch"}))
    _exact_keys(
        n,
        "ntrip",
        {"host", "port", "mountpoint", "username", "password_file", "gga_interval_s"},
    )
    _exact_keys(
        i,
        "imu",
        {
            "bus",
            "address",
            "sample_rate_hz",
            "accel_range_g",
            "gyro_range_dps",
            "gyro_bias_dps",
            "accel_bias_g",
            "axis_mapping",
            "axis_signs",
            "roll_offset_deg",
        },
        frozenset({"roll_invert"}),
    )

    config = AppConfig(
        gnss=GnssConfig(
            port=str(g["port"]),
            baudrate=int(g["baudrate"]),
            rate_hz=int(g["rate_hz"]),
            heading_offset_deg=float(g["heading_offset_deg"]),
        ),
        projected_crs=str(c["projected_crs"]),
        lever=LeverConfig(
            forward_m=float(lever["forward_m"]),
            left_m=float(lever["left_m"]),
            down_m=float(lever["down_m"]),
        ),
        use_imu=bool(calc["use_imu"]),
        use_pitch=bool(calc.get("use_pitch", False)),
        ntrip=NtripConfig(
            host=str(n["host"]),
            port=int(n["port"]),
            mountpoint=str(n["mountpoint"]),
            username=str(n["username"]),
            password_file=str(n["password_file"]),
            gga_interval_s=int(n["gga_interval_s"]),
        ),
        imu=ImuConfig(
            bus=str(i["bus"]),
            address=int(i["address"]),
            sample_rate_hz=int(i["sample_rate_hz"]),
            accel_range_g=int(i["accel_range_g"]),
            gyro_range_dps=int(i["gyro_range_dps"]),
            gyro_bias_dps=_triple(i["gyro_bias_dps"], "gyro_bias_dps"),
            accel_bias_g=_triple(i["accel_bias_g"], "accel_bias_g"),
            axis_mapping=_triple(i["axis_mapping"], "axis_mapping", int),
            axis_signs=_triple(i["axis_signs"], "axis_signs", int),
            roll_offset_deg=float(i["roll_offset_deg"]),
            roll_invert=bool(i.get("roll_invert", False)),
        ),
    )
    if config.gnss.rate_hz < 5 or config.gnss.rate_hz > 20:
        raise ValueError("rate_hz debe estar entre 5 y 20")
    for name, value in (
        ("forward_m", config.lever.forward_m),
        ("left_m", config.lever.left_m),
        ("down_m", config.lever.down_m),
    ):
        if abs(value) > LEVER_LIMIT_M:
            raise ValueError(
                f"lever.{name} = {value:.3f} m excede el tope de ±{LEVER_LIMIT_M:g} m. "
                "Revise el CRS y las coordenadas usadas en la calibración"
            )
    if config.lever.down_m < 0:
        raise ValueError("lever.down_m no puede ser negativo")
    if sorted(config.imu.axis_mapping) != [0, 1, 2]:
        raise ValueError("axis_mapping debe ser una permutación de [0,1,2]")
    if any(v not in (-1, 1) for v in config.imu.axis_signs):
        raise ValueError("axis_signs sólo admite -1 o 1")
    return config


def save_config(config: AppConfig, path: str | Path) -> None:
    """Guarda toda la configuración de forma atómica, sin incluir el password."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    def quote(value: object) -> str:
        return json.dumps(str(value), ensure_ascii=False)

    text = f"""[gnss]
port = {quote(config.gnss.port)}
baudrate = {config.gnss.baudrate}
rate_hz = {config.gnss.rate_hz}
heading_offset_deg = {config.gnss.heading_offset_deg}

[coordinates]
projected_crs = {quote(config.projected_crs)}

[lever]
forward_m = {config.lever.forward_m:.6f}
left_m = {config.lever.left_m:.6f}
down_m = {config.lever.down_m:.6f}

[calculation]
use_imu = {str(config.use_imu).lower()}
use_pitch = {str(config.use_pitch).lower()}

[ntrip]
host = {quote(config.ntrip.host)}
port = {config.ntrip.port}
mountpoint = {quote(config.ntrip.mountpoint)}
username = {quote(config.ntrip.username)}
password_file = {quote(config.ntrip.password_file)}
gga_interval_s = {config.ntrip.gga_interval_s}

[imu]
bus = {quote(config.imu.bus)}
address = {config.imu.address}
sample_rate_hz = {config.imu.sample_rate_hz}
accel_range_g = {config.imu.accel_range_g}
gyro_range_dps = {config.imu.gyro_range_dps}
gyro_bias_dps = {list(config.imu.gyro_bias_dps)}
accel_bias_g = {list(config.imu.accel_bias_g)}
axis_mapping = {list(config.imu.axis_mapping)}
axis_signs = {list(config.imu.axis_signs)}
roll_offset_deg = {config.imu.roll_offset_deg}
roll_invert = {str(config.imu.roll_invert).lower()}
"""
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(text, encoding="utf-8", newline="\n")
    os.replace(temporary, target)
