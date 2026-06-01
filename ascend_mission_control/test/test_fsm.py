"""
Unit tests for FSM - pure Python, no ROS2 required.
Run: pytest test/test_fsm.py -v
"""

import pytest
import sys
import os

# Add parent to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ascend_mission_control.states import DroneState, MissionEvent, GPSPoint, MissionWaypoint, MissionPlan
from ascend_mission_control.fsm import FlightFSM
from ascend_mission_control.geo_utils import (
    haversine_distance, calculate_bearing, is_within_radius, lat_lon_offset
)


class TestStates:
    """Test state and event definitions."""

    def test_drone_states_exist(self):
        assert DroneState.IDLE.value == "idle"
        assert DroneState.ARMED.value == "armed"
        assert DroneState.TAKEOFF.value == "takeoff"
        assert DroneState.LAND.value == "land"

    def test_mission_events_exist(self):
        assert MissionEvent.EV_ARM.value == "arm"
        assert MissionEvent.EV_ALTITUDE_REACHED.value == "altitude_reached"
        assert MissionEvent.EV_WAYPOINT_REACHED.value == "waypoint_reached"

    def test_gps_point_creation(self):
        gps = GPSPoint(latitude=40.7128, longitude=-74.0060, altitude=10.5)
        assert gps.latitude == 40.7128
        assert gps.longitude == -74.0060
        assert gps.altitude == 10.5

    def test_gps_point_equality(self):
        gps1 = GPSPoint(40.0, -74.0, 10.0)
        gps2 = GPSPoint(40.0, -74.0, 10.0)
        gps3 = GPSPoint(40.1, -74.0, 10.0)
        
        assert gps1 == gps2
        assert gps1 != gps3

    def test_mission_waypoint_creation(self):
        pos = GPSPoint(40.0, -74.0, 10.0)
        wp = MissionWaypoint(
            name="Test WP",
            position=pos,
            hover_duration=5.0,
            speed=3.0
        )
        assert wp.name == "Test WP"
        assert wp.hover_duration == 5.0

    def test_mission_plan_creation(self):
        home = GPSPoint(40.0, -74.0, 0.0)
        plan = MissionPlan(
            name="Test Mission",
            home=home,
            takeoff_altitude=10.0
        )
        assert plan.name == "Test Mission"
        assert plan.takeoff_altitude == 10.0
        assert len(plan.waypoints) == 0

    def test_mission_plan_add_waypoint(self):
        home = GPSPoint(40.0, -74.0, 0.0)
        plan = MissionPlan("Test", home, 10.0)
        
        wp = MissionWaypoint("WP1", GPSPoint(40.01, -74.0, 10.0))
        plan.add_waypoint(wp)
        
        assert len(plan.waypoints) == 1
        assert plan.waypoints[0].name == "WP1"


class TestFSM:
    """Test FSM state machine logic."""

    def test_fsm_initial_state(self):
        fsm = FlightFSM()
        assert fsm.get_state() == DroneState.IDLE

    def test_fsm_valid_transition(self):
        fsm = FlightFSM()
        result = fsm.process_event(MissionEvent.EV_ARM)
        
        assert result is True
        assert fsm.get_state() == DroneState.ARMED

    def test_fsm_invalid_transition(self):
        fsm = FlightFSM()
        result = fsm.process_event(MissionEvent.EV_LAND_START)
        
        assert result is False
        assert fsm.get_state() == DroneState.IDLE

    def test_fsm_transition_sequence(self):
        fsm = FlightFSM()
        
        assert fsm.process_event(MissionEvent.EV_ARM)
        assert fsm.get_state() == DroneState.ARMED
        
        assert fsm.process_event(MissionEvent.EV_TAKEOFF_START)
        assert fsm.get_state() == DroneState.TAKEOFF
        
        assert fsm.process_event(MissionEvent.EV_ALTITUDE_REACHED)
        assert fsm.get_state() == DroneState.HOVER

    def test_fsm_can_transition(self):
        fsm = FlightFSM()
        
        assert fsm.can_transition(MissionEvent.EV_ARM) is True
        assert fsm.can_transition(MissionEvent.EV_LAND_START) is False
        
        fsm.process_event(MissionEvent.EV_ARM)
        assert fsm.can_transition(MissionEvent.EV_TAKEOFF_START) is True
        assert fsm.can_transition(MissionEvent.EV_ARM) is False

    def test_fsm_callbacks(self):
        fsm = FlightFSM()
        called = []
        
        def on_armed(context):
            called.append("armed")
        
        def on_takeoff(context):
            called.append("takeoff")
        
        fsm.register_enter_callback(DroneState.ARMED, on_armed)
        fsm.register_enter_callback(DroneState.TAKEOFF, on_takeoff)
        
        fsm.process_event(MissionEvent.EV_ARM)
        assert "armed" in called
        
        fsm.process_event(MissionEvent.EV_TAKEOFF_START)
        assert "takeoff" in called

    def test_fsm_previous_state(self):
        fsm = FlightFSM()
        assert fsm.get_previous_state() is None
        
        fsm.process_event(MissionEvent.EV_ARM)
        assert fsm.get_previous_state() == DroneState.IDLE

    def test_fsm_is_in_state(self):
        fsm = FlightFSM()
        assert fsm.is_in_state(DroneState.IDLE)
        
        fsm.process_event(MissionEvent.EV_ARM)
        assert fsm.is_in_state(DroneState.ARMED)
        assert not fsm.is_in_state(DroneState.IDLE)

    def test_fsm_emergency_land_from_mission(self):
        fsm = FlightFSM()
        fsm.process_event(MissionEvent.EV_ARM)
        fsm.process_event(MissionEvent.EV_TAKEOFF_START)
        fsm.process_event(MissionEvent.EV_ALTITUDE_REACHED)
        fsm.process_event(MissionEvent.EV_MISSION_START)
        
        # Emergency land from mission
        result = fsm.process_event(MissionEvent.EV_EMERGENCY_LAND)
        assert result is True
        assert fsm.get_state() == DroneState.LAND

    def test_fsm_full_mission_sequence(self):
        fsm = FlightFSM()
        
        # Arm
        assert fsm.process_event(MissionEvent.EV_ARM)
        assert fsm.get_state() == DroneState.ARMED
        
        # Takeoff
        assert fsm.process_event(MissionEvent.EV_TAKEOFF_START)
        assert fsm.get_state() == DroneState.TAKEOFF
        
        # Altitude reached -> hover
        assert fsm.process_event(MissionEvent.EV_ALTITUDE_REACHED)
        assert fsm.get_state() == DroneState.HOVER
        
        # Start mission
        assert fsm.process_event(MissionEvent.EV_MISSION_START)
        assert fsm.get_state() == DroneState.MISSION
        
        # Waypoint reached -> hover
        assert fsm.process_event(MissionEvent.EV_WAYPOINT_REACHED)
        assert fsm.get_state() == DroneState.HOVER
        
        # Hover complete -> mission
        assert fsm.process_event(MissionEvent.EV_HOVER_COMPLETE)
        assert fsm.get_state() == DroneState.MISSION
        
        # Mission complete -> land
        assert fsm.process_event(MissionEvent.EV_MISSION_COMPLETE)
        assert fsm.get_state() == DroneState.LAND
        
        # Landed -> disarmed
        assert fsm.process_event(MissionEvent.EV_LANDED)
        assert fsm.get_state() == DroneState.DISARMED
        
        # Disarm -> idle
        assert fsm.process_event(MissionEvent.EV_DISARM)
        assert fsm.get_state() == DroneState.IDLE


class TestGeoUtils:
    """Test geographic utilities."""

    def test_haversine_distance_same_point(self):
        dist = haversine_distance(40.0, -74.0, 40.0, -74.0)
        assert abs(dist) < 0.1

    def test_haversine_distance_known(self):
        # NYC to Boston ~300km
        dist = haversine_distance(40.7128, -74.0060, 42.3601, -71.0589)
        assert 250000 < dist < 350000  # metres

    def test_calculate_bearing(self):
        # North bearing
        bearing = calculate_bearing(40.0, -74.0, 41.0, -74.0)
        assert 350 < bearing or bearing < 10  # Close to 0 (north)

    def test_calculate_bearing_east(self):
        # East bearing
        bearing = calculate_bearing(40.0, -74.0, 40.0, -73.0)
        assert 80 < bearing < 100  # Close to 90 (east)

    def test_is_within_radius_true(self):
        result = is_within_radius(40.0, -74.0, 40.0001, -74.0001, 100)
        assert result is True

    def test_is_within_radius_false(self):
        result = is_within_radius(40.0, -74.0, 40.1, -74.0, 100)
        assert result is False

    def test_lat_lon_offset(self):
        lat, lon = lat_lon_offset(40.0, -74.0, 1000, 1000)
        
        # Should be approximately 0.009 degrees north and east
        assert abs(lat - 40.009) < 0.001
        assert abs(lon - (-73.991)) < 0.001


class TestIntegration:
    """Integration tests."""

    def test_full_mission_workflow(self):
        """Simulate full mission workflow."""
        # Create mission plan
        home = GPSPoint(40.0, -74.0, 0.0)
        plan = MissionPlan(
            name="Integration Test",
            home=home,
            takeoff_altitude=10.0,
            mission_speed=5.0
        )
        
        wp1 = MissionWaypoint(
            name="Forward",
            position=GPSPoint(40.01, -74.0, 10.0),
            hover_duration=2.0
        )
        wp2 = MissionWaypoint(
            name="Back",
            position=GPSPoint(40.0, -74.0, 10.0),
            hover_duration=2.0
        )
        
        plan.add_waypoint(wp1)
        plan.add_waypoint(wp2)
        
        # Create FSM and execute
        fsm = FlightFSM()
        
        events = [
            MissionEvent.EV_ARM,
            MissionEvent.EV_TAKEOFF_START,
            MissionEvent.EV_ALTITUDE_REACHED,
            MissionEvent.EV_MISSION_START,
            MissionEvent.EV_WAYPOINT_REACHED,
            MissionEvent.EV_HOVER_COMPLETE,
            MissionEvent.EV_WAYPOINT_REACHED,
            MissionEvent.EV_MISSION_COMPLETE,
            MissionEvent.EV_LANDED,
            MissionEvent.EV_DISARM,
        ]
        
        expected_states = [
            DroneState.ARMED,
            DroneState.TAKEOFF,
            DroneState.HOVER,
            DroneState.MISSION,
            DroneState.HOVER,
            DroneState.MISSION,
            DroneState.HOVER,
            DroneState.LAND,
            DroneState.DISARMED,
            DroneState.IDLE,
        ]
        
        for event, expected_state in zip(events, expected_states):
            assert fsm.process_event(event)
            assert fsm.get_state() == expected_state


if __name__ == '__main__':
    pytest.main([__file__, '-v'])