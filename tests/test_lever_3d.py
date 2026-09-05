"""Compensación 3D de cuerpo rígido: la broca no se mueve al nivelar.

Es la prueba sintética que pide la especificación. Se fija la broca en una
coordenada conocida, se calcula dónde tendría que estar ANT1 para cada actitud
—`P_ANT1 = P_broca − R·r`— y se alimenta al algoritmo. Lo que reconstruya tiene
que volver a la broca de partida, para toda inclinación y sin ruido.

Si esto pasa y en campo la broca igual se corre al nivelar, el problema está en
los sensores o en el brazo medido, no en la transformación.
"""

import itertools
import math

import pytest

from app.geometry import Geodesy, rotate_lever

# Punto de la broca, en el mismo huso que usa drill-001.
BIT_LAT, BIT_LON, BIT_HEIGHT = -7.173432, -78.495378, 2718.03

# El vector de la especificación, en la convención del proyecto: adelante,
# izquierda y abajo desde ANT1 hasta la broca. Ojo con el orden: el documento
# lo escribe como [1.00, 0.84, -1.454] y el archivo de drill-001 tiene
# forward = 0.83 / left = 1.00, que son los mismos números al revés.
FORWARD, LEFT, DOWN = 1.00, 0.84, 1.454


@pytest.fixture(scope="module")
def geodesy() -> Geodesy:
    return Geodesy("EPSG:32717")


def antenna_for(geodesy: Geodesy, attitude, bit=(BIT_LAT, BIT_LON, BIT_HEIGHT)):
    """Dónde queda ANT1 si la broca está en `bit` y la máquina en `attitude`."""
    east, north, up = rotate_lever(FORWARD, LEFT, DOWN, *attitude, True)
    lat, lon, height, _, _ = geodesy.offset(*bit, (-east, -north, -up))
    return lat, lon, height


def bit_for(geodesy: Geodesy, antenna, attitude):
    """Lo que calcula el algoritmo: ANT1 más el brazo rotado en 3D."""
    enu = rotate_lever(FORWARD, LEFT, DOWN, *attitude, True)
    lat, lon, height, easting, northing = geodesy.offset(*antenna, enu)
    return lat, lon, height, easting, northing


def separation_m(geodesy: Geodesy, first, second) -> float:
    e1, n1 = geodesy.projected(*first[:3])
    e2, n2 = geodesy.projected(*second[:3])
    return math.hypot(e1 - e2, n1 - n2)


def test_the_bit_is_reconstructed_for_every_attitude(geodesy: Geodesy) -> None:
    """Sin ruido, el error de reconstrucción tiene que ser numérico y nada más."""
    worst = 0.0
    for attitude in itertools.product(
        (0.0, 41.0, 137.0, 180.0, 293.0), (-12.0, -5.0, 0.0, 5.0, 12.0),
        (-15.0, -7.0, 0.0, 7.0, 15.0),
    ):
        antenna = antenna_for(geodesy, attitude)
        recovered = bit_for(geodesy, antenna, attitude)
        worst = max(worst, separation_m(geodesy, recovered, (BIT_LAT, BIT_LON, BIT_HEIGHT)))
    assert worst < 0.001


def test_levelling_the_machine_does_not_move_the_computed_bit(geodesy: Geodesy) -> None:
    """El caso obligatorio de la especificación.

    La broca está 3 cm al este del objetivo. La máquina arranca ladeada, el
    operador la nivela con los gatos y ANT1 se desplaza casi medio metro por la
    rotación del cuerpo. La broca no se ha movido, así que el error calculado
    tiene que seguir siendo 3 cm.
    """
    target_e, target_n = geodesy.projected(BIT_LAT, BIT_LON, BIT_HEIGHT)
    bit = geodesy.offset(BIT_LAT, BIT_LON, BIT_HEIGHT, (0.03, 0.0, 0.0))[:3]

    tilted = (180.0, 5.0, 8.0)
    level = (180.0, 0.0, 0.0)
    antenna_tilted = antenna_for(geodesy, tilted, bit)
    antenna_level = antenna_for(geodesy, level, bit)
    # El nivelado mueve ANT1 de verdad: si no, la prueba no probaría nada.
    assert separation_m(geodesy, antenna_tilted, antenna_level) > 0.20

    errors = []
    for antenna, attitude in ((antenna_tilted, tilted), (antenna_level, level)):
        _, _, _, easting, northing = bit_for(geodesy, antenna, attitude)
        errors.append(math.hypot(easting - target_e, northing - target_n))

    assert errors[0] == pytest.approx(0.03, abs=0.002)
    assert errors[1] == pytest.approx(0.03, abs=0.002)
    assert abs(errors[0] - errors[1]) < 0.005


def test_a_bit_that_really_moves_is_not_frozen(geodesy: Geodesy) -> None:
    """La compensación quita el desplazamiento aparente, no el real."""
    attitude = (180.0, 3.0, 6.0)
    moved = geodesy.offset(BIT_LAT, BIT_LON, BIT_HEIGHT, (0.10, 0.0, 0.0))[:3]
    still = bit_for(geodesy, antenna_for(geodesy, attitude), attitude)
    shifted = bit_for(geodesy, antenna_for(geodesy, attitude, moved), attitude)
    assert separation_m(geodesy, still, shifted) == pytest.approx(0.10, abs=0.001)


def test_the_2d_model_is_the_one_that_drifts_when_levelling(geodesy: Geodesy) -> None:
    """Por qué hace falta el 3D: el mismo nivelado, ignorando la inclinación.

    El modo 2D directo trata el brazo como si la máquina estuviera siempre a
    nivel, así que el desplazamiento de ANT1 se traslada entero a la broca.
    """
    tilted, level = (180.0, 5.0, 8.0), (180.0, 0.0, 0.0)
    positions = []
    for attitude in (tilted, level):
        antenna = antenna_for(geodesy, attitude)
        enu = rotate_lever(FORWARD, LEFT, DOWN, attitude[0])
        positions.append(geodesy.offset(*antenna, enu)[:3])
    assert separation_m(geodesy, *positions) > 0.20
