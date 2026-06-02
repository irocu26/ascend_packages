"""
ascend_mission_control/geo_utils.py

Geographic utilities — Haversine distance, bearing, radius checks.
Pure Python, zero ROS2 dependencies. Importable in tests without rclpy.

NO CHANGES from original — this file is correct.
It is now imported in __init__.py and can be used in fsm_node if needed
(e.g. for GPS-based distance checks when SLAM is unavailable).
"""

import math
from typing import Tuple


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate distance between two GPS coordinates using Haversine formula.

    Args:
        lat1, lon1: Starting point (degrees)
        lat2, lon2: Ending point (degrees)

    Returns:
        Distance in metres
    """
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    R    = 6371000.0
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a    = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    c    = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate initial bearing from point 1 to point 2.

    Returns:
        Bearing in degrees (0° = North, 90° = East, 180° = South, 270° = West)
    """
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon            = lon2 - lon1
    x               = math.sin(dlon) * math.cos(lat2)
    y               = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    initial_bearing = math.degrees(math.atan2(x, y))
    return (initial_bearing + 360) % 360


def is_within_radius(lat1: float, lon1: float, lat2: float, lon2: float, radius: float) -> bool:
    """
    Check if point 2 is within radius metres of point 1.
    """
    return haversine_distance(lat1, lon1, lat2, lon2) <= radius


def lat_lon_offset(lat: float, lon: float, north: float, east: float) -> Tuple[float, float]:
    """
    Offset a GPS coordinate by north/east distances in metres.

    Returns:
        Tuple of (new_latitude, new_longitude)
    """
    lat_offset = north / 111000.0
    lon_offset = east  / (111000.0 * math.cos(math.radians(lat)))
    return lat + lat_offset, lon + lon_offset