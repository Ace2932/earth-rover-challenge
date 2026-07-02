"""Geodesy helpers for GPS-waypoint navigation. All angles in degrees.

Heading/bearing convention: 0 = North, 90 = East (clockwise), matching a
standard compass. The mock sim uses the same convention. On a real bot you must
calibrate how the SDK's `orientation` field maps to this (see waypoint_follower).
"""
import math

_R = 6371000.0  # earth radius, meters


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _R * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing from point 1 to point 2, 0=N 90=E, in [0,360)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def wrap180(deg):
    """Wrap an angle error into [-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0
