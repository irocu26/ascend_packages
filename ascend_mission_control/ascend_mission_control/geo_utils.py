"""
Geographic utilities - Haversine distance, bearing, radius checks.
Pure Python, no ROS2 dependencies.
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
    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Earth radius in metres
    R = 6371000.0
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    
    return distance


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate initial bearing from point 1 to point 2.
    
    Args:
        lat1, lon1: Starting point (degrees)
        lat2, lon2: Ending point (degrees)
    
    Returns:
        Bearing in degrees (0° = North, 90° = East, 180° = South, 270° = West)
    """
    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    initial_bearing = math.atan2(x, y)
    
    # Convert radians to degrees
    initial_bearing = math.degrees(initial_bearing)
    
    # Normalize to 0° ... 360°
    compass_bearing = (initial_bearing + 360) % 360
    
    return compass_bearing


def is_within_radius(lat1: float, lon1: float, lat2: float, lon2: float, radius: float) -> bool:
    """
    Check if point 2 is within radius of point 1.
    
    Args:
        lat1, lon1: Center point (degrees)
        lat2, lon2: Test point (degrees)
        radius: Radius in metres
    
    Returns:
        True if point 2 is within radius
    """
    distance = haversine_distance(lat1, lon1, lat2, lon2)
    return distance <= radius


def lat_lon_offset(lat: float, lon: float, north: float, east: float) -> Tuple[float, float]:
    """
    Offset a GPS coordinate by north/east distances.
    
    Args:
        lat, lon: Starting point (degrees)
        north: Northward offset (metres)
        east: Eastward offset (metres)
    
    Returns:
        Tuple of (new_latitude, new_longitude)
    """
    lat_offset = north / 111000.0
    lon_offset = east / (111000.0 * math.cos(math.radians(lat)))
    
    return lat + lat_offset, lon + lon_offset