"""Pruebas del roll: el sensor tiene que callarse cuando no puede medir.

El caso que originó estas pruebas: en drill-001 la IMU publicaba 51.8° con la
máquina a nivel —el mapeo llamaba vertical a un eje que no sostenía la
gravedad— y la broca salía 75 cm corrida sin un solo aviso en pantalla.
"""

import math

import pytest

from app.config import ImuConfig
from app.imu import RollSensor


def sensor(**overrides) -> RollSensor:
    base = dict(
        bus="/dev/i2c-1",
        address=104,
        sample_rate_hz=100,
        accel_range_g=4,
        gyro_range_dps=500,
        gyro_bias_dps=(0.0, 0.0, 0.0),
        accel_bias_g=(0.0, 0.0, 0.0),
        axis_mapping=(0, 1, 2),
        axis_signs=(1, 1, 1),
        roll_offset_deg=0.0,
    )
    return RollSensor(ImuConfig(**{**base, **overrides}))


def gravity(roll_deg: float) -> tuple[float, float, float]:
    """Lo que mide un acelerómetro bien montado con la máquina ladeada."""
    r = math.radians(roll_deg)
    return (0.0, math.sin(r), math.cos(r))


def feed(
    imu: RollSensor,
    accel,
    *,
    seconds: float,
    gyro=(0.0, 0.0, 0.0),
    dt: float = 0.01,
    start: float = 0.0,
) -> float:
    now = start
    for _ in range(int(seconds / dt)):
        now += dt
        imu.update(accel, gyro, dt, now)
    return now


# ------------------------------------------------------------------ nominal


def test_a_healthy_sensor_tracks_the_real_tilt() -> None:
    imu = sensor()
    feed(imu, gravity(7.0), seconds=20.0)
    assert imu.roll_deg == pytest.approx(7.0, abs=0.1)
    assert imu.orientation_ok
    assert imu.error == ""
    assert imu.roll_noise_deg == pytest.approx(0.0, abs=1e-9)


def test_the_filter_does_not_chase_a_fast_wobble() -> None:
    """Con tau = 2 s, dos décimas de sacudida mueven el ángulo un 10%.

    Los 0.24 s implícitos de antes la habrían seguido casi entera, y cada grado
    son 6.8 cm de broca con un brazo de 3.9 m.
    """
    imu = sensor(filter_tau_s=2.0)
    now = feed(imu, gravity(0.0), seconds=10.0)
    feed(imu, gravity(10.0), seconds=0.2, start=now)
    assert imu.roll_estimate_deg < 1.5


def test_the_filter_still_follows_a_sustained_tilt() -> None:
    """No es un exponencial puro: el estimador de sesgo lo hace de segundo orden
    y sobrepasa un 6% antes de asentarse."""
    imu = sensor(filter_tau_s=2.0)
    now = feed(imu, gravity(0.0), seconds=10.0)
    now = feed(imu, gravity(10.0), seconds=2.0, start=now)
    assert imu.roll_estimate_deg == pytest.approx(6.5, abs=1.0)
    feed(imu, gravity(10.0), seconds=60.0, start=now)
    assert imu.roll_estimate_deg == pytest.approx(10.0, abs=0.3)


# -------------------------------------------------- compuerta por movimiento


def test_a_moving_machine_freezes_the_accelerometer_correction() -> None:
    """En marcha, una frenada se lee como ladeo: sólo se integra el giróscopo."""
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=5.0)
    imu.moving = True
    feed(imu, gravity(30.0), seconds=5.0, start=now)
    assert imu.accel_gated
    assert imu.roll_deg == pytest.approx(0.0, abs=0.05)


def test_the_gyroscope_still_carries_the_roll_while_the_machine_moves() -> None:
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=5.0)
    imu.moving = True
    feed(imu, gravity(0.0), seconds=2.0, gyro=(3.0, 0.0, 0.0), start=now)
    assert imu.roll_deg == pytest.approx(6.0, abs=0.1)


def test_a_shove_is_not_read_as_tilt() -> None:
    """1.5 g no es gravedad: el vector no apunta a donde apunta el suelo."""
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=5.0)
    feed(imu, (0.0, 1.06, 1.06), seconds=1.0, start=now)
    assert imu.accel_gated
    assert imu.roll_deg == pytest.approx(0.0, abs=0.05)


# ------------------------------------------------------- ejes mal orientados


def test_gravity_on_the_wrong_axis_invalidates_the_roll() -> None:
    """El fallo de drill-001: el eje vertical del mapeo no sostiene la gravedad.

    `atan2` divide ruido entre ruido y sale un ángulo con aspecto de medida.
    """
    imu = sensor()
    feed(imu, (1.0, 0.0, 0.0), seconds=5.0)
    assert imu.tilt_from_vertical_deg == pytest.approx(90.0, abs=0.1)
    assert not imu.orientation_ok
    assert imu.roll_deg is None
    assert "Detectar ejes" in imu.error


def test_an_inverted_vertical_axis_is_caught_too() -> None:
    imu = sensor()
    feed(imu, (0.0, 0.0, -1.0), seconds=5.0)
    assert imu.roll_deg is None
    assert not imu.orientation_ok


def test_a_pothole_does_not_kill_the_roll() -> None:
    """La orientación se declara mala sólo si se sostiene dos segundos."""
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=5.0)
    now = feed(imu, (1.0, 0.0, 0.0), seconds=1.0, start=now)
    assert imu.orientation_ok
    assert imu.roll_deg is not None
    feed(imu, gravity(0.0), seconds=1.0, start=now)
    assert imu.orientation_ok


def test_the_roll_comes_back_when_the_mapping_is_fixed() -> None:
    imu = sensor()
    now = feed(imu, (1.0, 0.0, 0.0), seconds=5.0)
    assert imu.roll_deg is None
    feed(imu, gravity(2.0), seconds=5.0, start=now)
    assert imu.orientation_ok
    assert imu.roll_deg == pytest.approx(2.0, abs=0.2)


# -------------------------------------------------------------------- ruido


def test_a_jittery_roll_is_not_published() -> None:
    """En drill-001 la dispersión era de 10.9° con la máquina quieta."""
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=5.0)
    for index in range(300):
        now += 0.01
        imu.update(gravity(6.0 if index % 2 else -6.0), (0.0, 0.0, 0.0), 0.01, now)
    assert imu.roll_noise_deg > 2.0
    assert imu.roll_deg is None
    assert "inestable" in imu.error


def test_noise_does_not_gag_the_sensor_while_the_machine_moves() -> None:
    """En marcha la vibración es esperable y el acelerómetro ya no corrige."""
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=5.0)
    for index in range(300):
        now += 0.01
        imu.update(gravity(6.0 if index % 2 else -6.0), (0.0, 0.0, 0.0), 0.01, now)
    assert imu.roll_deg is None
    imu.moving = True
    feed(imu, gravity(0.0), seconds=1.0, start=now)
    assert imu.roll_deg is not None


def test_the_roll_snaps_back_to_the_accelerometer_after_a_long_drive() -> None:
    """El giróscopo deriva mientras la compuerta está cerrada.

    Al parar, esperar a que el filtro se acerque con tau = 2 s dejaría la broca
    corrida varios segundos justo cuando se la está posicionando.
    """
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=5.0)
    imu.moving = True
    now = feed(imu, gravity(0.0), seconds=30.0, gyro=(0.2, 0.0, 0.0), start=now)
    assert imu.roll_estimate_deg == pytest.approx(6.0, abs=0.1)
    imu.moving = False
    now += 0.01
    imu.update(gravity(0.0), (0.0, 0.0, 0.0), 0.01, now)
    assert imu.roll_estimate_deg == pytest.approx(0.0, abs=0.01)


# --------------------------------------------- sesgo aprendido del giróscopo


def test_a_gyroscope_bias_does_not_leave_a_permanent_offset() -> None:
    """Un filtro complementario arrastra tau·sesgo de desvío permanente.

    Con tau = 2 s y 2 °/s mal restados son 4°, que en la broca de la perforadora
    son 27 cm. El sesgo se aprende de lo que el acelerómetro desmiente.
    """
    imu = sensor()
    feed(imu, gravity(0.0), seconds=240.0, gyro=(2.0, 0.0, 0.0))
    assert imu.rate_bias_dps == pytest.approx(2.0, abs=0.1)
    assert imu.roll_deg == pytest.approx(0.0, abs=0.2)


def test_the_learned_bias_also_holds_the_roll_while_the_machine_moves() -> None:
    """Es con la compuerta cerrada donde la deriva no tiene quién la corrija."""
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=240.0, gyro=(2.0, 0.0, 0.0))
    imu.moving = True
    feed(imu, gravity(0.0), seconds=20.0, gyro=(2.0, 0.0, 0.0), start=now)
    assert imu.roll_estimate_deg == pytest.approx(0.0, abs=1.0)


def test_a_real_tilt_is_not_swallowed_by_the_bias_estimator() -> None:
    """El sesgo se aprende despacio; una máquina que se ladea no lo es."""
    imu = sensor()
    now = feed(imu, gravity(0.0), seconds=60.0)
    feed(imu, gravity(8.0), seconds=30.0, start=now)
    assert imu.roll_deg == pytest.approx(8.0, abs=0.3)


# ------------------------------------------------------------- identificación


def test_the_chip_is_named_from_who_am_i() -> None:
    """El GY-91 lleva un MPU9250, no el MPU6050 del montaje original."""
    from app.imu import _chip_name

    assert _chip_name(0x68) == "MPU6050"
    assert _chip_name(0x71) == "MPU9250"
    assert _chip_name(None) == ""
    assert "0x42" in _chip_name(0x42)


def test_only_the_mpu9250_family_gets_the_separate_accelerometer_filter() -> None:
    """En el 6050 el registro 0x1D no es el filtro del acelerómetro."""
    from app.imu import _SEPARATE_ACCEL_FILTER

    assert 0x68 not in _SEPARATE_ACCEL_FILTER
    assert {0x70, 0x71, 0x73} == set(_SEPARATE_ACCEL_FILTER)
