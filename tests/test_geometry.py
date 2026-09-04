import math

import pytest

from app.geometry import Geodesy, enu_to_planar_lever, rotate_lever


def test_planar_heading_north_keeps_forward_and_left() -> None:
    east, north, up = rotate_lever(2.270004, 1.192078, 3.919, 0.0)
    assert east == pytest.approx(-1.192078)
    assert north == pytest.approx(2.270004)
    assert up == pytest.approx(-3.919)


def test_planar_heading_east_rotates_without_changing_length() -> None:
    east, north, _ = rotate_lever(2.270004, 1.192078, 3.919, 90.0)
    assert east == pytest.approx(2.270004)
    assert north == pytest.approx(1.192078)
    assert math.hypot(east, north) == pytest.approx(2.563975, abs=1e-6)


def test_2d_ignores_pitch_and_roll() -> None:
    flat = rotate_lever(2.27, 1.19, 4.8, 40.0, 0.0, 0.0, False)
    tilted = rotate_lever(2.27, 1.19, 4.8, 40.0, 8.0, -6.0, False)
    assert tilted == pytest.approx(flat)


def test_geodesy_applies_meter_offset() -> None:
    geo = Geodesy("EPSG:32717")
    e0, n0 = geo.projected(-7.89377, -78.13098, 3385.5)
    _, _, _, e1, n1 = geo.offset(-7.89377, -78.13098, 3385.5, (1.0, 2.0, 0.0))
    # UTM gira ligeramente respecto de ENU por convergencia de meridianos; se
    # conserva la longitud local, no cada componente de grilla por separado.
    assert math.hypot(e1 - e0, n1 - n0) == pytest.approx(math.sqrt(5), abs=0.001)


def test_planar_calibration_recovers_body_components() -> None:
    heading = 41.0
    east, north, _ = rotate_lever(2.27, 1.192, 0.0, heading)
    forward, left = enu_to_planar_lever(east, north, heading)
    assert forward == pytest.approx(2.27)
    assert left == pytest.approx(1.192)


def test_3d_matches_2d_when_the_machine_is_level() -> None:
    """La rama use_imu=True no tenía ni una prueba."""
    flat = rotate_lever(2.27, 1.192, 3.919, 41.0, 0.0, 0.0, False)
    solved = rotate_lever(2.27, 1.192, 3.919, 41.0, 0.0, 0.0, True)
    assert solved == pytest.approx(flat)


def test_positive_pitch_lifts_the_forward_axis() -> None:
    _, _, up = rotate_lever(1.0, 0.0, 0.0, 0.0, 90.0, 0.0, True)
    assert up == pytest.approx(1.0)


def test_roll_swings_a_vertical_arm_sideways() -> None:
    """Con el brazo colgando 4 m, 90° de roll lo tumban 4 m en horizontal."""
    east, north, up = rotate_lever(0.0, 0.0, 4.0, 0.0, 0.0, 90.0, True)
    assert math.hypot(east, north) == pytest.approx(4.0)
    assert up == pytest.approx(0.0, abs=1e-9)


def test_tilt_costs_about_seven_centimetres_per_degree() -> None:
    """El número que decide si el modo 2D sirve en esta máquina."""
    flat = rotate_lever(2.270004, 1.192078, 3.919, 41.0, 0.0, 0.0, True)
    tilted = rotate_lever(2.270004, 1.192078, 3.919, 41.0, 1.0, 0.0, True)
    assert math.hypot(tilted[0] - flat[0], tilted[1] - flat[1]) == pytest.approx(0.068, abs=0.002)
