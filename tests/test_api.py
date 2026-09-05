"""Cobertura de las rutas HTTP. Antes no había ninguna."""

import pytest

from app.config import LEVER_LIMIT_M, load_config
from app.main import merged_config, validate_projected_crs


def payload(**overrides) -> dict:
    base = {
        "ntrip": {
            "host": "rtk.geodnet.com",
            "port": 2101,
            "mountpoint": "AUTO",
            "username": "RTKsub_ODRXQ",
            "password": None,
        },
        "coordinates": {"projected_crs": "EPSG:32717"},
        "lever": {"forward_m": 2.270004, "left_m": 1.192078, "down_m": 3.919},
        "calculation": {"use_imu": False},
    }
    for section, values in overrides.items():
        base[section] = {**base[section], **values}
    return base


# ------------------------------------------------------------------ lectura


def test_index_serves_the_interface(client) -> None:
    response = client.get("/")
    assert response.status_code == 200
    assert "Posición de la broca" in response.text


def test_config_never_returns_the_password(client) -> None:
    body = client.get("/api/config").json()
    assert set(body) == {"ntrip", "coordinates", "lever", "calculation"}
    assert "password" not in body["ntrip"]
    assert body["ntrip"]["has_password"] is False


# ------------------------------------------------------------------- escritura


def test_put_config_applies_and_persists(client, config_file) -> None:
    response = client.put("/api/config", json=payload(coordinates={"projected_crs": "EPSG:32718"}))
    assert response.status_code == 200
    assert response.json()["config"]["coordinates"]["projected_crs"] == "EPSG:32718"
    assert load_config(config_file).projected_crs == "EPSG:32718"


def test_put_config_preserves_sections_the_interface_never_sends(client, config_file) -> None:
    before = load_config(config_file)
    client.put("/api/config", json=payload(calculation={"use_imu": True}))
    after = load_config(config_file)
    assert after.gnss == before.gnss
    assert after.imu == before.imu
    assert after.ntrip.password_file == before.ntrip.password_file
    assert after.ntrip.gga_interval_s == before.ntrip.gga_interval_s


@pytest.mark.parametrize("code", ["WGS84", "EPSG:4326", "EPSG:4979"])
def test_geographic_crs_is_rejected(client, code) -> None:
    """F-04: PROJ los acepta y el Este/Norte pasaba a grados sin aviso."""
    response = client.put("/api/config", json=payload(coordinates={"projected_crs": code}))
    assert response.status_code == 400
    assert "grados" in response.json()["detail"]


def test_unknown_crs_is_rejected(client) -> None:
    response = client.put("/api/config", json=payload(coordinates={"projected_crs": "EPSG:99999"}))
    assert response.status_code == 400


def test_projected_crs_in_metres_is_accepted() -> None:
    validate_projected_crs("EPSG:32717")
    with pytest.raises(ValueError, match="grados"):
        validate_projected_crs("EPSG:4326")


def test_validation_errors_are_readable_not_object_object(client) -> None:
    """F-12: el detail de Pydantic llegaba a pantalla como [object Object]."""
    response = client.put("/api/config", json=payload(lever={"forward_m": 500.0}))
    assert response.status_code == 422
    detail = response.json()["detail"]
    assert isinstance(detail, str)
    assert "Adelante desde ANT1" in detail


def test_out_of_range_lever_never_reaches_disk(client, config_file) -> None:
    before = config_file.read_bytes()
    assert client.put("/api/config", json=payload(lever={"left_m": -900.0})).status_code == 422
    assert config_file.read_bytes() == before


def test_unknown_fields_are_rejected(client) -> None:
    body = payload()
    body["calculation"]["reseed_ms"] = 3000
    assert client.put("/api/config", json=body).status_code == 422


# ---------------------------------------------------------------- calibración


def test_calibration_without_samples_returns_a_useful_conflict(client) -> None:
    response = client.post("/api/calibration/2d", json={"easting_m": 816373.843, "northing_m": 9126361.129})
    assert response.status_code == 409
    assert "0/360" in response.json()["detail"]


def test_calibration_with_a_truncated_easting_is_refused(client, config_file) -> None:
    """F-01: el brazo de 733 km se guardaba y dejaba la interfaz bloqueada."""
    import time

    from tests.test_runtime import epoch_at, heading_at

    runtime = client.runtime
    now = time.monotonic() * 1000
    for index in range(400):
        stamp = now - (400 - index) * 100
        runtime.calibration_samples.append((epoch_at(stamp), heading_at(stamp)))

    before = config_file.read_bytes()
    response = client.post(
        "/api/calibration/2d", json={"easting_m": 81637.843, "northing_m": 9126361.129}
    )
    assert response.status_code == 409
    assert "excede el tope" in response.json()["detail"]
    assert config_file.read_bytes() == before, "un brazo imposible no puede llegar al disco"
    assert abs(load_config(config_file).lever.forward_m) <= LEVER_LIMIT_M


def test_calibration_rejects_non_finite_input(client) -> None:
    response = client.post("/api/calibration/2d", json={"easting_m": "inf", "northing_m": 0})
    assert response.status_code == 422


# ------------------------------------------------------------------ telemetría


@pytest.mark.asyncio
async def test_telemetry_stream_sends_the_current_state_as_sse(client) -> None:
    import json
    import time

    from starlette.requests import Request

    import app.main as main
    from tests.test_runtime import epoch_at, heading_at

    runtime = client.runtime
    now = time.monotonic() * 1000
    runtime.heading = heading_at(now)
    runtime.epoch = epoch_at(now)
    await runtime.recompute(now + 1)

    pending = [{"type": "http.request"}]

    async def receive() -> dict:
        return pending.pop(0) if pending else {"type": "http.disconnect"}

    request = Request(
        {"type": "http", "method": "GET", "path": "/api/v1/telemetry/stream", "headers": []},
        receive,
    )
    response = await main.telemetry_stream(request)
    assert response.media_type == "text/event-stream"

    chunk = await anext(response.body_iterator)
    assert chunk.startswith("data: ")
    body = json.loads(chunk[len("data: ") :])
    assert body["quality"]["mode"] == "PRECISION"
    assert body["calibration"]["required_samples"] == 360
    await response.body_iterator.aclose()
    assert not runtime.subscribers, "el cliente debe darse de baja al cerrar"


def test_merged_config_keeps_the_serial_port_untouched(config_file) -> None:
    from app.api_models import ConfigInput

    current = load_config(config_file)
    merged = merged_config(current, ConfigInput(**payload(ntrip={"host": "otro.caster"})))
    assert merged.gnss.port == current.gnss.port
    assert merged.gnss.heading_offset_deg == current.gnss.heading_offset_deg
    assert merged.ntrip.host == "otro.caster"


# ------------------------------------------------------------- puerto serie


def test_port_can_be_released_and_reconnected(client) -> None:
    released = client.post("/api/gnss/port", json={"released": True})
    assert released.status_code == 200
    assert released.json()["released"] is True
    assert released.json()["port"] == "/dev/serial0"
    assert client.runtime.gnss.released is True

    back = client.post("/api/gnss/port", json={"released": False})
    assert back.status_code == 200
    assert back.json()["released"] is False
    assert client.runtime.gnss.released is False


def test_port_state_is_not_persisted_to_disk(client, config_file) -> None:
    """Un reinicio del servicio debe volver siempre al puerto conectado."""
    before = config_file.read_bytes()
    client.post("/api/gnss/port", json={"released": True})
    assert config_file.read_bytes() == before


def test_port_endpoint_rejects_junk(client) -> None:
    assert client.post("/api/gnss/port", json={"released": "quizás"}).status_code == 422
    assert client.post("/api/gnss/port", json={"abierto": True}).status_code == 422


def test_calibration_is_refused_while_the_port_is_released(client) -> None:
    client.post("/api/gnss/port", json={"released": True})
    response = client.post(
        "/api/calibration/2d", json={"easting_m": 816373.843, "northing_m": 9126361.129}
    )
    assert response.status_code == 409
    assert "liberado" in response.json()["detail"]


# ------------------------------------------------------------------ IMU


def test_imu_actions_need_a_live_sensor(client) -> None:
    """Sin lectura no se puede calibrar, y hay que decirlo con claridad."""
    client.runtime.imu.roll_raw_deg = None
    client.runtime.imu.raw_g = None
    assert client.post("/api/imu/zero").status_code == 409
    assert client.post("/api/imu/detect-axes").status_code == 409


def test_imu_zero_averages_the_window_and_makes_it_the_new_zero(client, config_file) -> None:
    """Una muestra suelta grabaría la vibración del momento como offset fijo.

    En drill-001 el acelerómetro dispersaba ±1° con el motor encendido: el cero
    salió 1.4° corrido, que en la broca de la perforadora son 10 cm.
    """
    client.runtime.imu._roll.raw_recent.extend([8.19] * 100 + [9.19] * 100)
    client.runtime.imu._pitch.raw_recent.extend([1.0] * 200)
    body = client.post("/api/imu/zero").json()
    assert body["imu"]["roll_offset_deg"] == pytest.approx(-8.69)
    # El mismo gesto fija los dos ceros: se nivela una vez.
    assert body["imu"]["pitch_offset_deg"] == pytest.approx(-1.0)
    assert body["pitch_window"]["samples"] == 200
    assert body["window"]["samples"] == 200
    assert body["window"]["spread_deg"] == pytest.approx(0.5)
    from app.config import load_config as lc

    assert lc(config_file).imu.roll_offset_deg == pytest.approx(-8.69)


def test_imu_zero_refuses_a_window_that_is_only_noise(client) -> None:
    client.runtime.imu._roll.raw_recent.extend(
        [-6.0 if index % 2 else 6.0 for index in range(200)]
    )
    response = client.post("/api/imu/zero")
    assert response.status_code == 409
    assert "dispersa" in response.json()["detail"]


def test_imu_zero_needs_enough_quiet_samples(client) -> None:
    client.runtime.imu._roll.raw_recent.extend([1.0] * 10)
    response = client.post("/api/imu/zero")
    assert response.status_code == 409
    assert "faltan lecturas quietas" in response.json()["detail"]


def test_imu_invert_toggles_and_persists(client, config_file) -> None:
    from app.config import load_config as lc

    assert client.post("/api/imu/invert").json()["imu"]["roll_invert"] is True
    assert lc(config_file).imu.roll_invert is True
    assert client.post("/api/imu/invert").json()["imu"]["roll_invert"] is False


def test_detect_axes_puts_gravity_on_the_vertical(client, config_file) -> None:
    """El fallo real medido en drill-001: gravedad en Z crudo y mapeo [2,1,0]."""
    client.runtime.imu.raw_g = (-0.0078, 0.1511, 0.9900)
    body = client.post("/api/imu/detect-axes").json()
    assert body["detected"]["vertical_axis"] == "Z"
    assert body["imu"]["axis_mapping"] == [0, 1, 2]
    assert body["imu"]["axis_signs"][2] == 1
    from app.config import load_config as lc

    assert lc(config_file).imu.axis_mapping == (0, 1, 2)


def test_detect_axes_refuses_while_the_machine_moves(client) -> None:
    client.runtime.imu.raw_g = (0.4, 0.9, 0.9)  # 1.34 g: no esta quieta
    response = client.post("/api/imu/detect-axes")
    assert response.status_code == 409
    assert "quieta" in response.json()["detail"]


def test_detect_axes_keeps_which_horizontal_axis_looks_forward(client, config_file) -> None:
    """La gravedad sólo dice cuál eje es el vertical.

    Cuál de los otros dos mira adelante se averigua ladeando la máquina, y en
    drill-001 costó dos días descubrir que el sensor está girado 90°. Volver a
    pulsar el botón no puede deshacerlo.
    """
    from dataclasses import replace

    from app.config import load_config as lc

    girado = replace(client.runtime.config.imu, axis_mapping=(1, 0, 2), axis_signs=(1, -1, 1))
    client.runtime.config = replace(client.runtime.config, imu=girado)
    client.runtime.imu.raw_g = (-0.02, -0.09, 0.94)
    body = client.post("/api/imu/detect-axes").json()
    assert body["detected"]["vertical_axis"] == "Z"
    assert body["imu"]["axis_mapping"] == [1, 0, 2]
    assert body["imu"]["axis_signs"] == [1, -1, 1]
    assert lc(config_file).imu.axis_mapping == (1, 0, 2)
