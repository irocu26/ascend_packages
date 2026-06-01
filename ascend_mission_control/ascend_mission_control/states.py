"""
State definitions, events, and mission planning data structures.
Pure data structures with no dependencies.
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional, Dict


class DroneState(Enum):
    """Flight states for the drone."""
    IDLE = "idle"
    ARMED = "armed"
    TAKEOFF = "takeoff"
    HOVER = "hover"
    MISSION = "mission"
    LAND = "land"
    DISARMED = "disarmed"
    ERROR = "error"


class MissionEvent(Enum):
    """Events that trigger FSM transitions."""
    EV_ARM = "arm"
    EV_DISARM = "disarm"
    EV_TAKEOFF_START = "takeoff_start"
    EV_ALTITUDE_REACHED = "altitude_reached"
    EV_WAYPOINT_REACHED = "waypoint_reached"
    EV_MISSION_START = "mission_start"
    EV_MISSION_COMPLETE = "mission_complete"
    EV_HOVER_COMPLETE = "hover_complete"
    EV_LAND_START = "land_start"
    EV_LANDED = "landed"
    EV_ERROR = "error"
    EV_EMERGENCY_LAND = "emergency_land"


@dataclass
class GPSPoint:
    """GPS coordinate representation."""
    latitude: float
    longitude: float
    altitude: float  # AGL in metres

    def __str__(self) -> str:
        return f"GPS({self.latitude:.6f}, {self.longitude:.6f}, {self.altitude:.1f}m)"

    def __eq__(self, other) -> bool:
        if not isinstance(other, GPSPoint):
            return False
        return (abs(self.latitude - other.latitude) < 1e-7 and
                abs(self.longitude - other.longitude) < 1e-7 and
                abs(self.altitude - other.altitude) < 0.1)


@dataclass
class MissionWaypoint:
    """Single mission waypoint."""
    name: str
    position: GPSPoint
    hover_duration: float = 0.0  # Seconds (0 = no hover)
    mission_type: str = "waypoint"  # 'waypoint', 'hover', 'land'
    speed: float = 5.0  # m/s

    def __str__(self) -> str:
        return f"WP({self.name}: {self.position}, hover={self.hover_duration}s)"


@dataclass
class MissionPlan:
    """Complete mission plan."""
    name: str
    home: GPSPoint
    takeoff_altitude: float  # AGL in metres
    waypoints: List[MissionWaypoint] = field(default_factory=list)
    mission_speed: float = 5.0
    waypoint_reach_radius: float = 2.0  # metres

    def add_waypoint(self, waypoint: MissionWaypoint):
        """Add waypoint to mission."""
        self.waypoints.append(waypoint)

    def __str__(self) -> str:
        return (f"Mission({self.name}: home={self.home}, "
                f"alt={self.takeoff_altitude}m, "
                f"waypoints={len(self.waypoints)})")


# FSM State Transition Table
# Format: (current_state, event) -> next_state
TRANSITIONS: Dict[tuple, DroneState] = {
    (DroneState.IDLE, MissionEvent.EV_ARM): DroneState.ARMED,
    (DroneState.ARMED, MissionEvent.EV_TAKEOFF_START): DroneState.TAKEOFF,
    (DroneState.TAKEOFF, MissionEvent.EV_ALTITUDE_REACHED): DroneState.HOVER,
    (DroneState.HOVER, MissionEvent.EV_MISSION_START): DroneState.MISSION,
    (DroneState.MISSION, MissionEvent.EV_WAYPOINT_REACHED): DroneState.HOVER,
    (DroneState.HOVER, MissionEvent.EV_HOVER_COMPLETE): DroneState.MISSION,
    (DroneState.MISSION, MissionEvent.EV_MISSION_COMPLETE): DroneState.LAND,
    (DroneState.LAND, MissionEvent.EV_LANDED): DroneState.DISARMED,
    (DroneState.DISARMED, MissionEvent.EV_DISARM): DroneState.IDLE,
    # Emergency transitions
    (DroneState.ARMED, MissionEvent.EV_EMERGENCY_LAND): DroneState.LAND,
    (DroneState.TAKEOFF, MissionEvent.EV_EMERGENCY_LAND): DroneState.LAND,
    (DroneState.MISSION, MissionEvent.EV_EMERGENCY_LAND): DroneState.LAND,
    (DroneState.HOVER, MissionEvent.EV_EMERGENCY_LAND): DroneState.LAND,
    # Error transitions
    (DroneState.ARMED, MissionEvent.EV_ERROR): DroneState.ERROR,
    (DroneState.TAKEOFF, MissionEvent.EV_ERROR): DroneState.ERROR,
    (DroneState.MISSION, MissionEvent.EV_ERROR): DroneState.ERROR,
    (DroneState.HOVER, MissionEvent.EV_ERROR): DroneState.ERROR,
}