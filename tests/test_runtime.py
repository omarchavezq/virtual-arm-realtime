import asyncio
import time
from datetime import UTC, datetime

import pytest

from app.config import AppConfig, GnssConfig, ImuConfig, LeverConfig, NtripConfig
from app.domain import Epoch, Heading
from app.geometry import rotate_lever
from app.runtime import Runtime

LAT, LON, HEIGHT = -7.89377, -78.13098, 3385.5


def config(**overrides) -> AppConfig:
    base = dict(
        gnss=GnssConfig("test", 115200, 10, 0.0),
        projected_crs="EPSG:32717",
        lever=LeverConfig(2.270004, 1.192078, 3.919),
        use_imu=False,
        ntrip=NtripConfig("host", 2101, "AUTO", "user", "secret", 10),
        imu=ImuConfig("/dev/i2c-1", 104, 100, 4, 500, (0, 0, 0), (0, 0, 0), (0, 1, 2), (1, 1, 1), 0),
    )
    return AppConfig(**{**base, **overrides})


def epoch_at(stamp: float, *, speed: float = 0.0, fix: str = "FIXED", lat: float = LAT) -> Epoch:
    return Epoch(
        lat, LON, HEIGHT, 3363.15, 22.35, fix, "NARROW_INT" if fix == "FIXED" else "SINGLE",
        36, 0.02, 0.02, speed, datetime.now(UTC), stamp, True,
    )


def heading_at(stamp: float, *, deg: float = 41.0, pitch: float = 0.0) -> Heading:
    return Heading(deg, pitch, 1.395, "SOL_COMPUTED", "NARROW_INT", 0.2, 0.3, stamp, True)


def fill_calibration(runtime: Runtime, *, count: int, speed: float = 0.0, heading_deg: float = 41.0):
    now = time.monotonic() * 1000
    for index in range(count):
        stamp = now - (count - index) * 100
        runtime.calibration_samples.append(
            (epoch_at(stamp, speed=speed), heading_at(stamp, deg=heading_deg))
        )
    return now


def target_for(runtime: Runtime, forward: float, left: float, heading_deg: float = 41.0):
    enu = rotate_lever(forward, left, 0.0, heading_deg)
    _, _, _, east, north = runtime.geodesy.offset(LAT, LON, HEIGHT, enu)
    return east, north


# --------------------------------------------------------------- publicación


@pytest.mark.asyncio
async def test_each_epoch_is_published_immediately_without_stationary_wait() -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now, deg=40.0, pitch=2.0)
    runtime.epoch = epoch_at(now, speed=1.0)
    queue = runtime.subscribe()
    payload = await runtime.recompute(now + 1)
    published = await asyncio.wait_for(queue.get(), timeout=0.05)
    assert payload is published
    assert payload["virtual_gps"]["valid"]
    assert payload["quality"]["mode"] == "PRECISION"
    assert payload["calculation_mode"] == "2D_DIRECT"


@pytest.mark.asyncio
async def test_position_changes_while_machine_is_moving() -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now, deg=40.0)
    runtime.epoch = epoch_at(now, speed=2.0)
    first = await runtime.recompute(now + 1)
    runtime.epoch = epoch_at(now + 100, speed=2.0, lat=-7.893769)
    second = await runtime.recompute(now + 101)
    assert first["virtual_gps"]["easting_m"] != second["virtual_gps"]["easting_m"]


# ------------------------------------------------------------------ frescura


@pytest.mark.asyncio
async def test_stale_epoch_invalidates_the_position() -> None:
    """F-03: sin latido, una pantalla congelada no se distinguía de una viva."""
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now)
    fresh = await runtime.recompute(now + 1)
    assert fresh["quality"]["position_valid"]

    stale = await runtime.recompute(now + 6000)
    assert not stale["quality"]["position_valid"]
    assert stale["quality"]["mode"] == "INVALID"
    assert not stale["virtual_gps"]["valid"]
    assert stale["data_age_ms"] >= 6000
    assert any("Sin datos del GNSS" in w for w in stale["quality"]["warnings"])


@pytest.mark.asyncio
async def test_heartbeat_republishes_without_new_frames() -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now)
    await runtime.recompute(now)
    queue = runtime.subscribe()
    await queue.get()  # el estado inicial que reinyecta subscribe()

    runtime._heartbeat = asyncio.create_task(runtime._heartbeat_loop())
    try:
        beat = await asyncio.wait_for(queue.get(), timeout=2.0)
    finally:
        runtime._heartbeat.cancel()
    assert beat["data_age_ms"] >= 500


@pytest.mark.asyncio
async def test_epoch_without_heading_still_reports_instead_of_going_silent() -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.epoch = epoch_at(now)
    payload = await runtime.recompute(now + 1)
    assert payload is not None
    assert payload["attitude"]["heading_deg"] is None
    assert not payload["virtual_gps"]["valid"]
    assert any("Sin rumbo" in w for w in payload["quality"]["warnings"])


# -------------------------------------------------------------------- avisos


@pytest.mark.asyncio
async def test_single_fix_warns_even_though_position_is_computable() -> None:
    """F-06: SINGLE se presentaba igual que RTK fija, sin un solo aviso."""
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now, fix="SINGLE")
    payload = await runtime.recompute(now + 1)
    assert payload["quality"]["mode"] == "APPROACH"
    assert any("Sin RTK fijo" in w for w in payload["quality"]["warnings"])


@pytest.mark.asyncio
async def test_tilt_is_reported_and_warned_in_2d() -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now, pitch=-1.5)
    runtime.epoch = epoch_at(now)
    payload = await runtime.recompute(now + 1)
    tilt = payload["attitude"]["tilt_error_m"]
    assert tilt == pytest.approx(0.1026, abs=0.002)
    assert any("inclinada" in w for w in payload["quality"]["warnings"])


@pytest.mark.asyncio
async def test_short_baseline_is_flagged() -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)  # baseline 1.395 m, bajo el mínimo de 1.5
    runtime.epoch = epoch_at(now)
    payload = await runtime.recompute(now + 1)
    assert payload["attitude"]["baseline_ok"] is False


@pytest.mark.asyncio
async def test_imu_error_reaches_the_payload() -> None:
    """F-11: el error concreto de I2C se guardaba y nadie lo leía."""
    runtime = Runtime(config(use_imu=True))
    runtime.imu.error = "OSError: [Errno 121] Remote I/O error"
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now)
    payload = await runtime.recompute(now + 1)
    assert payload["attitude"]["imu_error"].startswith("OSError")
    assert any("Errno 121" in w for w in payload["quality"]["warnings"])


# --------------------------------------------------------------- calibración


def test_calibration_averages_the_full_window() -> None:
    runtime = Runtime(config())
    fill_calibration(runtime, count=400)
    east, north = target_for(runtime, 2.27, 1.192)
    result = runtime.calibration_2d(east, north)
    assert result["samples"] >= runtime._required_samples()
    assert result["forward_m"] == pytest.approx(2.27, abs=0.002)
    assert result["left_m"] == pytest.approx(1.192, abs=0.002)
    assert result["deviation_m"] == pytest.approx(0.0, abs=0.001)
    assert result["window_s"] > 30


def test_calibration_rejects_a_moving_machine() -> None:
    """F-07: calibrar rodando producía un brazo sesgado que parecía válido."""
    runtime = Runtime(config())
    fill_calibration(runtime, count=400, speed=3.5)
    east, north = target_for(runtime, 2.27, 1.192)
    assert runtime.calibration_sample_count() == 0
    with pytest.raises(ValueError, match="máquina detenida"):
        runtime.calibration_2d(east, north)


def test_calibration_rejects_a_lever_beyond_the_physical_limit() -> None:
    """F-01/F-02: un dígito de menos o un CRS erróneo escribía cientos de km."""
    runtime = Runtime(config())
    fill_calibration(runtime, count=400)
    with pytest.raises(ValueError, match="excede el tope"):
        runtime.calibration_2d(81637.843, 9126361.129)
    with pytest.raises(ValueError, match="excede el tope"):
        runtime.calibration_2d(0.0, 0.0)


def test_calibration_rejects_inconsistent_solutions() -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    for index in range(400):
        stamp = now - (400 - index) * 100
        # El rumbo salta 20° a mitad de la captura: el promedio no vale.
        deg = 41.0 if index < 200 else 61.0
        runtime.calibration_samples.append((epoch_at(stamp), heading_at(stamp, deg=deg)))
    east, north = target_for(runtime, 2.27, 1.192)
    with pytest.raises(ValueError, match="discrepan"):
        runtime.calibration_2d(east, north)


def test_calibration_needs_the_whole_window_not_ten_epochs() -> None:
    runtime = Runtime(config())
    fill_calibration(runtime, count=10)
    east, north = target_for(runtime, 2.27, 1.192)
    with pytest.raises(ValueError, match="10/360"):
        runtime.calibration_2d(east, north)


# ------------------------------------------------------------------- config


@pytest.mark.asyncio
async def test_changing_only_the_password_restarts_the_ntrip_session(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """F-05: la contraseña vive fuera del dataclass, así que nada se reiniciaba."""
    from app.io import Ntrip

    async def _noop(self) -> None:
        return None

    monkeypatch.setattr(Ntrip, "start", _noop)
    monkeypatch.setattr(Ntrip, "stop", _noop)

    runtime = Runtime(config())
    first = id(runtime.ntrip)
    await runtime.apply_config(runtime.config)
    assert id(runtime.ntrip) == first, "sin cambios no debe reconectar"

    await runtime.apply_config(runtime.config, ntrip_secret_changed=True)
    assert id(runtime.ntrip) != first, "cambiar la contraseña debe reconectar"


# ------------------------------------------------------------- puerto serie


@pytest.fixture
def quiet_io(monkeypatch: pytest.MonkeyPatch):
    """Evita abrir el puerto serie y la red al soltar y recuperar el puerto."""
    from app.io import GnssSerial, Ntrip

    async def _noop(self) -> None:
        return None

    for target in (GnssSerial, Ntrip):
        monkeypatch.setattr(target, "start", _noop)
        monkeypatch.setattr(target, "stop", _noop)


@pytest.mark.asyncio
async def test_releasing_the_port_stops_the_position_on_purpose(quiet_io) -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now)
    assert (await runtime.recompute(now + 1))["quality"]["position_valid"]

    await runtime.set_port_released(True)
    payload = await runtime.recompute(now + 2)
    assert runtime.gnss.released is True
    assert payload["gnss"]["port_released"] is True
    assert payload["gnss"]["port"] == "test"
    assert not payload["virtual_gps"]["valid"]
    # El motivo tiene que decir que fue deliberado, no parecer una avería.
    assert "a propósito" in payload["virtual_gps"]["reason"]
    assert payload["calibration"]["fixed_samples"] == 0
    assert payload["calibration"]["ready"] is False


@pytest.mark.asyncio
async def test_reconnecting_the_port_restores_the_position(quiet_io) -> None:
    runtime = Runtime(config())
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now)
    await runtime.set_port_released(True)
    await runtime.set_port_released(False)
    payload = await runtime.recompute(now + 3)
    assert runtime.gnss.released is False
    assert payload["gnss"]["port_released"] is False
    assert payload["virtual_gps"]["valid"]


@pytest.mark.asyncio
async def test_releasing_twice_is_harmless(quiet_io) -> None:
    runtime = Runtime(config())
    await runtime.set_port_released(True)
    first = id(runtime.ntrip)
    await runtime.set_port_released(True)
    assert id(runtime.ntrip) == first


@pytest.mark.asyncio
async def test_cannot_calibrate_with_the_port_released(quiet_io) -> None:
    runtime = Runtime(config())
    fill_calibration(runtime, count=400)
    east, north = target_for(runtime, 2.27, 1.192)
    await runtime.set_port_released(True)
    with pytest.raises(ValueError, match="liberado"):
        runtime.calibration_2d(east, north)


def test_stop_clears_the_writer_so_writes_cannot_leak() -> None:
    """Antes stop() cerraba el writer pero lo dejaba puesto."""
    runtime = Runtime(config())
    runtime.gnss.writer = None
    runtime.gnss.reader = object()
    asyncio.run(runtime.gnss.stop())
    assert runtime.gnss.reader is None
    assert runtime.gnss.writer is None
    assert runtime.gnss.connected is False


# ------------------------------------------------------------------ arranque


@pytest.mark.asyncio
async def test_publishes_before_the_first_epoch_arrives() -> None:
    """Si el stream calla al arrancar, ni la pantalla ni gpsevt distinguen
    «esperando al receptor» de «servicio muerto»."""
    runtime = Runtime(config())
    queue = runtime.subscribe()
    payload = await runtime.recompute(time.monotonic() * 1000)

    assert payload is not None, "debe publicar aunque no haya llegado ninguna época"
    assert await asyncio.wait_for(queue.get(), timeout=0.05) is payload
    assert payload["gnss"]["fix"] == "NONE"
    assert payload["gnss"]["latitude"] is None
    assert payload["gnss"]["epoch_age_ms"] is None
    assert payload["attitude"]["heading_deg"] is None
    assert not payload["virtual_gps"]["valid"]
    assert payload["quality"]["mode"] == "INVALID"
    assert any("Esperando la primera posición" in w for w in payload["quality"]["warnings"])
    assert payload["timestamp"].endswith("Z")


@pytest.mark.asyncio
async def test_first_epoch_replaces_the_waiting_state() -> None:
    runtime = Runtime(config())
    waiting = await runtime.recompute(time.monotonic() * 1000)
    assert waiting["gnss"]["fix"] == "NONE"

    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now)
    live = await runtime.recompute(now + 1)
    assert live["gnss"]["fix"] == "FIXED"
    assert live["virtual_gps"]["valid"]


# ------------------------------------------------------------- modo 2D+pitch


@pytest.mark.asyncio
async def test_pitch_mode_corrects_what_2d_discards() -> None:
    """El pitch sale del UM982 (dos antenas), no de la IMU."""
    now = time.monotonic() * 1000
    plano = Runtime(config())
    con_pitch = Runtime(config(use_pitch=True))
    for r in (plano, con_pitch):
        r.heading = heading_at(now, pitch=2.0)
        r.epoch = epoch_at(now)

    a = await plano.recompute(now + 1)
    b = await con_pitch.recompute(now + 1)
    assert a["calculation_mode"] == "2D_DIRECT"
    assert b["calculation_mode"] == "2D_PITCH"
    # La IMU corre siempre para poder diagnosticarla, pero su roll no entra en
    # el cálculo: eso es lo que distingue 2D_PITCH de 3D_IMU.
    assert b["attitude"]["roll_deg"] is None, "el roll no debe entrar en el cálculo"
    assert b["attitude"]["roll_used"] is False
    assert b["imu"]["roll_used"] is False if "roll_used" in b["imu"] else True

    import math

    d = math.hypot(
        a["virtual_gps"]["easting_m"] - b["virtual_gps"]["easting_m"],
        a["virtual_gps"]["northing_m"] - b["virtual_gps"]["northing_m"],
    )
    # 2° sobre un brazo vertical de 3.919 m: la diferencia es la corrección.
    assert d == pytest.approx(0.1367, abs=0.003)


@pytest.mark.asyncio
async def test_pitch_mode_stops_warning_about_discarded_tilt() -> None:
    now = time.monotonic() * 1000
    runtime = Runtime(config(use_pitch=True))
    runtime.heading = heading_at(now, pitch=-2.5)
    runtime.epoch = epoch_at(now)
    payload = await runtime.recompute(now + 1)
    assert payload["attitude"]["tilt_error_m"] is None
    assert not any("inclinada" in w for w in payload["quality"]["warnings"])


def test_calibration_in_pitch_mode_is_the_exact_inverse() -> None:
    """Con la máquina inclinada, calibrar y volver a calcular debe devolver el
    punto exacto. Si la inversa no deshace el pitch, el brazo sale sesgado."""
    runtime = Runtime(config(use_pitch=True, lever=LeverConfig(2.27, 1.192, 4.0)))
    pitch, heading_deg = 3.0, 41.0
    now = time.monotonic() * 1000
    for index in range(400):
        stamp = now - (400 - index) * 100
        runtime.calibration_samples.append(
            (epoch_at(stamp), heading_at(stamp, deg=heading_deg, pitch=pitch))
        )
    # Punto verdadero según el modelo directo con pitch.
    enu = rotate_lever(2.27, 1.192, 4.0, heading_deg, pitch, 0.0, True)
    _, _, _, east, north = runtime.geodesy.offset(LAT, LON, HEIGHT, enu)

    result = runtime.calibration_2d(east, north)
    assert result["forward_m"] == pytest.approx(2.27, abs=0.003)
    assert result["left_m"] == pytest.approx(1.192, abs=0.003)


def test_calibration_without_pitch_mode_keeps_the_planar_inverse() -> None:
    runtime = Runtime(config())
    fill_calibration(runtime, count=400)
    east, north = target_for(runtime, 2.27, 1.192)
    result = runtime.calibration_2d(east, north)
    assert result["forward_m"] == pytest.approx(2.27, abs=0.002)
