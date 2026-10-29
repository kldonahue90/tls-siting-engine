"""Tests for tls_siting.geometry module."""

import math
import pytest
from tls_siting.geometry import (
    geodetic_direct,
    geodetic_inverse,
    cartesian_to_geodetic,
    geodetic_to_cartesian,
    calculate_touchdown_aiming_point,
)


def test_geodetic_direct_inverse_roundtrip():
    lat0, lon0 = 36.583336, -115.664908
    azimuth = 273.0
    distance = 1000.0

    lat1, lon1 = geodetic_direct(lat0, lon0, azimuth, distance)
    calc_az, calc_dist = geodetic_inverse(lat0, lon0, lat1, lon1)

    assert math.isclose(calc_dist, distance, abs_tol=1e-3)
    assert math.isclose(calc_az % 360, azimuth % 360, abs_tol=1e-3)


def test_cartesian_to_geodetic_and_inverse():
    lat0, lon0, elev0 = 36.583336, -115.664908, 946.16
    hdg_deg = 273.0
    x, y, z = 300.0, 60.0, 0.0

    lat, lon, elev = cartesian_to_geodetic(x, y, z, lat0, lon0, elev0, hdg_deg)
    rx, ry, rz = geodetic_to_cartesian(lat, lon, elev, lat0, lon0, elev0, hdg_deg)

    assert math.isclose(rx, x, abs_tol=1e-3)
    assert math.isclose(ry, y, abs_tol=1e-3)
    assert math.isclose(rz, z, abs_tol=1e-3)


def test_calculate_touchdown_aiming_point():
    # TCH = 50 ft, GPA = 3.0 deg, dz = 0 ft
    # td = (50 * 0.3048) / tan(rad(3.0)) = 15.24 / 0.052407779 = 290.796m
    td_m = calculate_touchdown_aiming_point(50.0, 3.0, 0.0)
    expected = (50.0 * 0.3048) / math.tan(math.radians(3.0))
    assert math.isclose(td_m, expected, rel_tol=1e-6)

    # With slope dz = 2 ft
    td_dz = calculate_touchdown_aiming_point(50.0, 3.0, 2.0)
    expected_dz = (48.0 * 0.3048) / math.tan(math.radians(3.0))
    assert math.isclose(td_dz, expected_dz, rel_tol=1e-6)
