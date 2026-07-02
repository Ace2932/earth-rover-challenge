import math
from geo import haversine_m, bearing_deg, wrap180


def test_bearing_cardinals():
    assert abs(bearing_deg(0, 0, 1, 0) - 0) < 0.5       # north
    assert abs(bearing_deg(0, 0, 0, 1) - 90) < 0.5      # east
    assert abs(bearing_deg(0, 0, -1, 0) - 180) < 0.5    # south
    assert abs(bearing_deg(0, 0, 0, -1) - 270) < 0.5    # west


def test_haversine_one_degree_lon():
    d = haversine_m(0, 0, 0, 1)
    assert abs(d - 111195) / 111195 < 0.01              # ~111.2 km at equator


def test_haversine_zero():
    assert haversine_m(37.5, -122.1, 37.5, -122.1) < 1e-6


def test_wrap180():
    assert abs(wrap180(190) - (-170)) < 1e-9
    assert abs(wrap180(-190) - 170) < 1e-9
    assert abs(wrap180(360)) < 1e-9
    assert abs(wrap180(45) - 45) < 1e-9
