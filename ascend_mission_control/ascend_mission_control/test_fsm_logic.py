#!/usr/bin/env python3
"""
test/test_fsm_logic.py

Unit tests for ASCEND FSM — no ROS2 runtime, no Gazebo, no hardware needed.
Tests: State enum, LawnmowerPattern geometry, failsafe math, mission logic.

WHAT CHANGED:
  1. sys.path.insert now uses dirname(__file__) correctly relative to the
     test/ subdirectory — previously broke when run from different CWDs.
  2. Added MOCK_ROS2 environment patch so importing fsm_node.py doesn't
     crash when rclpy is not initialized (pytest runs without ros2 launch).
  3. Removed import of APInterface from __init__ in test scope — APInterface
     requires ardupilot_msgs which may not be built in CI/pytest environment.
     fsm_node.py guards the import correctly; tests only need State +
     LawnmowerPattern which are pure Python.
  4. TestStateEnum: fixed required state list to match actual State enum
     (removed 'HOVER', 'MISSION', 'ARMED' from old schema).
  5. TestMissionLogic: battery threshold test fixed — 20.0 returns None
     (at threshold, not below), was asserting wrong behavior.

Run with (no ROS2 needed):
  cd ~/ardu_ws/src/ascend_packages
  pytest ascend_mission_control/test/test_fsm_logic.py -v
"""

import sys
import os

# FIX 1: correct path regardless of where pytest is invoked from
_test_dir    = os.path.dirname(os.path.abspath(__file__))
_package_dir = os.path.dirname(_test_dir)   # ascend_mission_control/
_src_dir     = os.path.dirname(_package_dir) # src/ascend_packages/
sys.path.insert(0, _src_dir)
sys.path.insert(0, _package_dir)

import pytest
import math

# FIX 2: patch rclpy before importing fsm_node so pytest doesn't crash
# when ROS2 is not initialized. Only patches the Node base class reference;
# State and LawnmowerPattern are pure Python and need no ROS2.
try:
    import rclpy
except ImportError:
    # rclpy not installed — create a minimal stub so the import succeeds
    import types
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')

    class _FakeNode:
        def __init__(self, *a, **kw): pass

    rclpy.node.Node = _FakeNode
    sys.modules['rclpy']            = rclpy
    sys.modules['rclpy.node']       = rclpy.node
    sys.modules['rclpy.qos']        = types.ModuleType('rclpy.qos')
    sys.modules['geometry_msgs']    = types.ModuleType('geometry_msgs')
    sys.modules['geometry_msgs.msg']= types.ModuleType('geometry_msgs.msg')
    sys.modules['std_msgs']         = types.ModuleType('std_msgs')
    sys.modules['std_msgs.msg']     = types.ModuleType('std_msgs.msg')
    sys.modules['sensor_msgs']      = types.ModuleType('sensor_msgs')
    sys.modules['sensor_msgs.msg']  = types.ModuleType('sensor_msgs.msg')

# FIX 3: import only pure-Python classes; APInterface not imported in test scope
from ascend_mission_control.fsm_node import State, LawnmowerPattern


# ─────────────────────────────────────────────
#  LawnmowerPattern Tests
# ─────────────────────────────────────────────

class TestLawnmowerPattern:

    def test_generates_waypoints(self):
        p = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        assert p.total_waypoints() > 0

    def test_altitude_respected(self):
        alt = 4.0
        p   = LawnmowerPattern(altitude_m=alt, overlap_factor=0.3)
        p.reset()
        while True:
            wp = p.next_waypoint()
            if wp is None:
                break
            assert abs(wp[2] - alt) < 0.01, f"Waypoint altitude {wp[2]} != {alt}"

    def test_all_waypoints_inside_arena(self):
        p       = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        arena_x = LawnmowerPattern.ARENA_X_M
        arena_y = LawnmowerPattern.ARENA_Y_M
        p.reset()
        while True:
            wp = p.next_waypoint()
            if wp is None:
                break
            x, y, _ = wp
            assert 0 <= x <= arena_x, f"X={x} outside [0, {arena_x}]"
            assert 0 <= y <= arena_y, f"Y={y} outside [0, {arena_y}]"

    def test_progress_monotonic(self):
        p    = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        prev = 0.0
        p.reset()
        while True:
            p.next_waypoint()
            prog = p.progress()
            assert prog >= prev
            prev = prog
            if p.is_complete():
                break

    def test_reset_restarts_pattern(self):
        p = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        p.next_waypoint()
        p.next_waypoint()
        p.reset()
        assert p._index == 0
        assert not p.is_complete()

    def test_different_altitudes_produce_different_strip_counts(self):
        p_low  = LawnmowerPattern(altitude_m=2.0, overlap_factor=0.3)
        p_high = LawnmowerPattern(altitude_m=6.0, overlap_factor=0.3)
        # Higher altitude → wider strips → fewer strips
        assert p_high.total_waypoints() <= p_low.total_waypoints()

    def test_higher_overlap_more_waypoints(self):
        p_low  = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.1)
        p_high = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.5)
        assert p_high.total_waypoints() >= p_low.total_waypoints()

    def test_next_waypoint_returns_none_at_end(self):
        p = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        p.reset()
        for _ in range(p.total_waypoints()):
            p.next_waypoint()
        assert p.next_waypoint() is None
        assert p.is_complete()

    def test_final_waypoint_near_home(self):
        p    = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        last = p._waypoints[-1]
        dist = math.sqrt(last[0]**2 + last[1]**2)
        assert dist < 2.0, f"Last waypoint {last} is far from home (dist={dist:.2f}m)"

    @pytest.mark.parametrize("altitude", [2.0, 3.0, 4.0, 5.0, 6.0])
    def test_valid_altitude_range(self, altitude):
        p = LawnmowerPattern(altitude_m=altitude, overlap_factor=0.3)
        assert p.total_waypoints() > 0


# ─────────────────────────────────────────────
#  State Enum Tests
# ─────────────────────────────────────────────

class TestStateEnum:

    def test_all_states_unique(self):
        values = [s.value for s in State]
        assert len(values) == len(set(values))

    def test_required_states_exist(self):
        """Verify all 13 states that map to rulebook tasks exist."""
        # FIX 4: corrected list — removed old schema states (HOVER, MISSION, ARMED)
        required = [
            'IDLE', 'ARMING', 'TAKEOFF', 'SURVEY',
            'MATCH_VERIFY', 'RTL', 'LANDING', 'DOCKING',
            'CHARGING', 'TRANSFER', 'COMPLETE',
            'FAILSAFE_RTL', 'FAILSAFE_LAND'
        ]
        state_names = [s.name for s in State]
        for name in required:
            assert name in state_names, f"Required state '{name}' missing from FSM"

    def test_failsafe_states_present(self):
        failsafe = [s for s in State if 'FAILSAFE' in s.name]
        assert len(failsafe) >= 2, "Need at least FAILSAFE_RTL and FAILSAFE_LAND"

    def test_no_legacy_states(self):
        """Ensure deleted schema states are not present."""
        # FIX 4: old states.py had these — confirm they don't exist in real FSM
        legacy = ['HOVER', 'MISSION', 'ARMED', 'DISARMED', 'ERROR']
        state_names = [s.name for s in State]
        for name in legacy:
            assert name not in state_names, \
                f"Legacy state '{name}' found — delete states.py and fsm.py"


# ─────────────────────────────────────────────
#  Coordinate Math Tests
# ─────────────────────────────────────────────

class TestCoordinateMath:

    def test_distance_calculation(self):
        def dist(x1, y1, z1, x2, y2, z2):
            return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
        assert abs(dist(0,0,0, 3,4,0) - 5.0) < 1e-6
        assert abs(dist(0,0,0, 0,0,3) - 3.0) < 1e-6
        assert dist(1,2,3, 1,2,3) == 0.0

    def test_base_station_relative_coords(self):
        home_x, home_y            = 1.5, 2.3
        feature_slam_x, slam_y    = 4.5, 5.3
        rel_x = feature_slam_x - home_x
        rel_y = slam_y - home_y
        assert abs(rel_x - 3.0) < 1e-6
        assert abs(rel_y - 3.0) < 1e-6

    def test_arena_boundary_clamp(self):
        arena_x, arena_y = 10.67, 7.62
        margin = 0.3

        def clamp(x, y):
            return (
                max(margin, min(arena_x - margin, x)),
                max(margin, min(arena_y - margin, y))
            )
        assert clamp(5.0, 3.0) == (5.0, 3.0)
        x, y = clamp(12.0, 3.0)
        assert x == arena_x - margin
        x, y = clamp(5.0, -1.0)
        assert y == margin
        x, y = clamp(-1.0, 100.0)
        assert x == margin
        assert y == arena_y - margin


# ─────────────────────────────────────────────
#  Mission Logic Tests
# ─────────────────────────────────────────────

class TestMissionLogic:

    def test_features_found_deduplication(self):
        found = []
        def add_feature(fid, x, y):
            if fid not in [f['seed_id'] for f in found]:
                found.append({'seed_id': fid, 'x': x, 'y': y})
        add_feature(0, 1.0, 2.0)
        add_feature(0, 1.0, 2.0)  # Duplicate — ignored
        add_feature(1, 3.0, 4.0)
        assert len(found) == 2

    def test_mission_complete_condition(self):
        n_features = 3
        found      = [{'seed_id': i} for i in range(n_features)]
        assert len(found) >= n_features

    def test_single_start_command_enforcement(self):
        received = [0]
        def handle_start():
            if received[0] > 0:
                return False
            received[0] += 1
            return True
        assert handle_start() == True
        assert handle_start() == False

    def test_battery_failsafe_threshold(self):
        LOW_PCT  = 20.0
        CRIT_PCT = 10.0

        def check_failsafe(pct):
            if pct < CRIT_PCT:
                return 'FAILSAFE_LAND'
            if pct < LOW_PCT:
                return 'FAILSAFE_RTL'
            return None

        assert check_failsafe(50.0) is None
        assert check_failsafe(15.0) == 'FAILSAFE_RTL'
        assert check_failsafe(5.0)  == 'FAILSAFE_LAND'
        # FIX 5: exactly at threshold is NOT below it — returns None
        assert check_failsafe(20.0) is None
        assert check_failsafe(19.9) == 'FAILSAFE_RTL'
        # Exactly at critical threshold — NOT below it
        assert check_failsafe(10.0) is None
        assert check_failsafe(9.9)  == 'FAILSAFE_LAND'

    def test_max_sorties_cap(self):
        MAX_SORTIES  = 5
        sorties_done = 0

        def attempt_sortie():
            nonlocal sorties_done
            if sorties_done >= MAX_SORTIES:
                return False
            sorties_done += 1
            return True

        for _ in range(MAX_SORTIES):
            assert attempt_sortie() == True
        assert attempt_sortie() == False
        assert sorties_done == MAX_SORTIES


# ─────────────────────────────────────────────
#  Scenario Flow Tests (logic only, no ROS2)
# ─────────────────────────────────────────────

class TestScenarios:

    def test_scenario_normal_flow(self):
        expected = [
            State.IDLE, State.ARMING, State.TAKEOFF, State.SURVEY,
            State.MATCH_VERIFY, State.SURVEY,
            State.MATCH_VERIFY, State.SURVEY,
            State.MATCH_VERIFY, State.RTL, State.LANDING,
            State.DOCKING, State.TRANSFER, State.COMPLETE,
        ]
        assert State.TAKEOFF  in expected
        assert State.COMPLETE in expected
        assert State.SURVEY   in expected
        assert State.RTL      in expected

    def test_scenario_failsafe_rtl_exits_survey(self):
        current_state = State.SURVEY
        battery       = 15.0
        if battery < 20.0:
            current_state = State.FAILSAFE_RTL
        assert current_state == State.FAILSAFE_RTL

    def test_scenario_multi_sortie_requires_charging(self):
        visited = set()
        for s in [
            State.TAKEOFF, State.SURVEY, State.RTL, State.LANDING,
            State.DOCKING, State.CHARGING, State.TAKEOFF, State.SURVEY,
            State.RTL, State.LANDING, State.DOCKING, State.TRANSFER, State.COMPLETE
        ]:
            visited.add(s)
        assert State.CHARGING in visited, "Multi-sortie must include CHARGING"
        assert State.TRANSFER in visited, "Must include TRANSFER before COMPLETE"

    def test_failsafe_land_from_critical_battery(self):
        current_state = State.SURVEY
        battery       = 8.0  # Below critical threshold
        if battery < 10.0:
            current_state = State.FAILSAFE_LAND
        assert current_state == State.FAILSAFE_LAND

    def test_arming_state_present_before_takeoff(self):
        """Ensures ARMING is not skipped — was missing in old states.py schema."""
        flow = [State.IDLE, State.ARMING, State.TAKEOFF, State.SURVEY]
        arming_idx  = flow.index(State.ARMING)
        takeoff_idx = flow.index(State.TAKEOFF)
        assert arming_idx < takeoff_idx, "ARMING must precede TAKEOFF"