"""Geodetic and Cartesian coordinate transformation utilities for TLS Siting Engine."""

import math
from typing import Tuple
from pyproj import Geod

WGS84_GEOD = Geod(ellps="WGS84")


def geodetic_direct(lat0: float, lon0: float, azimuth_deg: float, distance_m: float) -> Tuple[float, float]:
    """
    Computes direct geodetic calculation on WGS84 ellipsoid.

    Returns (lat, lon) in degrees.
    """
    end_lon, end_lat, _ = WGS84_GEOD.fwd(lon0, lat0, azimuth_deg, distance_m)
    return float(end_lat), float(end_lon)


def geodetic_inverse(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
    """
    Computes inverse geodetic calculation on WGS84 ellipsoid.

    Returns (forward_azimuth_deg, distance_m).
    """
    fwd_az, _, dist = WGS84_GEOD.inv(lon1, lat1, lon2, lat2)
    return float(fwd_az), float(dist)


def cartesian_to_geodetic(
    x: float,
    y: float,
    z: float,
    lat0: float,
    lon0: float,
    elev0: float,
    hdg_deg: float
) -> Tuple[float, float, float]:
    """
    Transforms local runway Cartesian coordinate (x, y, z) to geodetic (lat, lon, elev).

    - x: along centerline downwind (+X in meters)
    - y: lateral offset (+Y right in meters, -Y left in meters)
    - z: orthometric height above threshold elevation in meters
    - lat0, lon0, elev0: Runway Threshold Point T coordinates (lat/lon in deg, elev in m)
    - hdg_deg: Runway true heading in degrees

    Returns (lat, lon, elev).
    """
    hdg_rad = math.radians(hdg_deg)
    dn = x * math.cos(hdg_rad) - y * math.sin(hdg_rad)
    de = x * math.sin(hdg_rad) + y * math.cos(hdg_rad)

    dist = math.hypot(dn, de)
    if dist < 1e-9:
        return float(lat0), float(lon0), float(elev0 + z)

    # Azimuth in degrees clockwise from North
    azimuth_deg = math.degrees(math.atan2(de, dn)) % 360.0
    lat, lon = geodetic_direct(lat0, lon0, azimuth_deg, dist)
    return lat, lon, float(elev0 + z)


def geodetic_to_cartesian(
    lat: float,
    lon: float,
    elev: float,
    lat0: float,
    lon0: float,
    elev0: float,
    hdg_deg: float
) -> Tuple[float, float, float]:
    """
    Transforms geodetic coordinate (lat, lon, elev) to local runway Cartesian (x, y, z).

    Returns (x, y, z) in meters.
    """
    azimuth_deg, dist = geodetic_inverse(lat0, lon0, lat, lon)
    az_rad = math.radians(azimuth_deg)

    dn = dist * math.cos(az_rad)
    de = dist * math.sin(az_rad)

    hdg_rad = math.radians(hdg_deg)
    x = dn * math.cos(hdg_rad) + de * math.sin(hdg_rad)
    y = -dn * math.sin(hdg_rad) + de * math.cos(hdg_rad)
    z = elev - elev0

    return float(x), float(y), float(z)


def calculate_touchdown_aiming_point(
    tch_ft: float,
    gpa_deg: float,
    runway_dz_ft: float = 0.0
) -> float:
    """
    Calculates touchdown aiming point distance downwind from threshold incorporating
    longitudinal slope correction per ANPC Doc #020-00103.

    - tch_ft: Threshold Crossing Height in feet
    - gpa_deg: Glide Path Angle in degrees
    - runway_dz_ft: Elevation difference across touchdown zone in feet (default 0.0)

    Returns touchdown distance in METERS downwind from threshold.
    """
    td_dist_m = ((tch_ft - runway_dz_ft) * 0.3048) / math.tan(math.radians(gpa_deg))
    return float(td_dist_m)
