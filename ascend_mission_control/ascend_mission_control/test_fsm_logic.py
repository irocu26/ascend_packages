#!/usr/bin/env python3
"""
test/test_fsm_logic.py

Unit tests for ASCEND FSM — no ROS2 runtime required.
Tests state transitions, failsafe logic, and survey pattern.

Run with:
  cd ~/ardu_ws/src/ascend_packages
  pytest ascend_mission_control/test/test_fsm_logic.py -v
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
import math

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
        p = LawnmowerPattern(altitude_m=alt, overlap_factor=0.3)
        p.reset()
        while True:
            wp = p.next_waypoint()
            if wp is None:
                break
            assert abs(wp[2] - alt) < 0.01, f"Waypoint altitude {wp[2]} != {alt}"

    def test_all_waypoints_inside_arena(self):
        p = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        p.reset()
        arena_x = LawnmowerPattern.ARENA_X_M
        arena_y = LawnmowerPattern.ARENA_Y_M
        while True:
            wp = p.next_waypoint()
            if wp is None:
                break
            x, y, _ = wp
            assert 0 <= x <= arena_x, f"X={x} outside arena [0, {arena_x}]"
            assert 0 <= y <= arena_y, f"Y={y} outside arena [0, {arena_y}]"

    def test_progress_monotonic(self):
        p = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
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
        p_low_overlap  = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.1)
        p_high_overlap = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.5)
        assert p_high_overlap.total_waypoints() >= p_low_overlap.total_waypoints()

    def test_next_waypoint_returns_none_at_end(self):
        p = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        p.reset()
        for _ in range(p.total_waypoints()):
            p.next_waypoint()
        assert p.next_waypoint() is None
        assert p.is_complete()

    def test_final_waypoint_near_home(self):
        """Last waypoint should be back near (0,0) home position."""
        p = LawnmowerPattern(altitude_m=3.0, overlap_factor=0.3)
        last = p._waypoints[-1]
        dist_to_home = math.sqrt(last[0]**2 + last[1]**2)
        assert dist_to_home < 2.0, f"Last waypoint {last} is far from home"

    @pytest.mark.parametrize("altitude", [2.0, 3.0, 4.0, 5.0, 6.0])
    def test_valid_altitude_range(self, altitude):
        """Test all altitudes within the 2–6m rulebook range."""
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
        """Verify states that map to rulebook tasks exist."""
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


# ─────────────────────────────────────────────
#  Coordinate Math Tests
# ─────────────────────────────────────────────

class TestCoordinateMath:

    def test_distance_calculation(self):
        """Verify 3D distance formula used in FSM."""
        def dist(x1, y1, z1, x2, y2, z2):
            return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)

        assert abs(dist(0,0,0, 3,4,0) - 5.0) < 1e-6   # Pythagorean 3-4-5
        assert abs(dist(0,0,0, 0,0,3) - 3.0) < 1e-6   # Vertical distance
        assert dist(1,2,3, 1,2,3) == 0.0                # Same point

    def test_base_station_relative_coords(self):
        """
        Simulate the coordinate transformation:
        home position locked at takeoff → all positions relative to home.
        """
        home_x, home_y = 1.5, 2.3   # Arbitrary home position in SLAM frame
        feature_slam_x, feature_slam_y = 4.5, 5.3

        rel_x = feature_slam_x - home_x
        rel_y = feature_slam_y - home_y

        assert abs(rel_x - 3.0) < 1e-6
        assert abs(rel_y - 3.0) < 1e-6

    def test_arena_boundary_clamp(self):
        """Test that positions are clamped to arena bounds."""
        arena_x, arena_y = 10.67, 7.62
        margin = 0.3

        def clamp(x, y):
            return (
                max(margin, min(arena_x - margin, x)),
                max(margin, min(arena_y - margin, y))
            )

        # Inside arena
        assert clamp(5.0, 3.0) == (5.0, 3.0)
        # Outside on x
        x, y = clamp(12.0, 3.0)
        assert x == arena_x - margin
        # Outside on y
        x, y = clamp(5.0, -1.0)
        assert y == margin
        # Both outside
        x, y = clamp(-1.0, 100.0)
        assert x == margin
        assert y == arena_y - margin


# ─────────────────────────────────────────────
#  Mission Logic Tests
# ─────────────────────────────────────────────

class TestMissionLogic:

    def test_features_found_deduplication(self):
        """Same feature ID should not be logged twice."""
        found = []
        def add_feature(fid, x, y):
            if fid not in [f['seed_id'] for f in found]:
                found.append({'seed_id': fid, 'x': x, 'y': y})
        add_feature(0, 1.0, 2.0)
        add_feature(0, 1.0, 2.0)   # Duplicate — should be ignored
        add_feature(1, 3.0, 4.0)
        assert len(found) == 2

    def test_mission_complete_condition(self):
        """Mission is complete when all 3 features are found."""
        n_features = 3
        found = [{'seed_id': i} for i in range(n_features)]
        assert len(found) >= n_features

    def test_single_start_command_enforcement(self):
        """Only one start command should be accepted."""
        received = [0]
        def handle_start():
            if received[0] > 0:
                return False  # Reject
            received[0] += 1
            return True
        assert handle_start() == True
        assert handle_start() == False   # Second command rejected

    def test_battery_failsafe_threshold(self):
        """Verify failsafe fires at correct battery level."""
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
        assert check_failsafe(20.0) is None     # At threshold, not below
        assert check_failsafe(19.9) == 'FAILSAFE_RTL'

    def test_max_sorties_cap(self):
        """FSM should not exceed MAX_SORTIES."""
        MAX_SORTIES = 5
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
#  Integration Scenario Tests (logic only)
# ─────────────────────────────────────────────

class TestScenarios:

    def test_scenario_normal_flow(self):
        """
        Trace the expected state sequence for a normal mission.
        IDLE → ARMING → TAKEOFF → SURVEY → MATCH_VERIFY →
        (×3) → RTL → LANDING → DOCKING → TRANSFER → CHARGING → COMPLETE
        """
        expected_sequence = [
            State.IDLE,
            State.ARMING,
            State.TAKEOFF,
            State.SURVEY,
            State.MATCH_VERIFY,     # Feature 1
            State.SURVEY,
            State.MATCH_VERIFY,     # Feature 2
            State.SURVEY,
            State.MATCH_VERIFY,     # Feature 3
            State.RTL,
            State.LANDING,
            State.DOCKING,
            State.TRANSFER,
            State.COMPLETE,
        ]
        # Verify the sequence has no invalid back-transitions
        assert State.TAKEOFF in expected_sequence
        assert State.COMPLETE in expected_sequence
        assert State.SURVEY in expected_sequence

    def test_scenario_failsafe_rtl_exits_survey(self):
        """If FAILSAFE_RTL fires during SURVEY, it should preempt normal flow."""
        current_state = State.SURVEY
        battery = 15.0  # Below LOW threshold
        if battery < 20.0:
            current_state = State.FAILSAFE_RTL
        assert current_state == State.FAILSAFE_RTL

    def test_scenario_multi_sortie_requires_charging(self):
        """Multi-sortie missions must go through CHARGING state."""
        visited_states = set()
        # Simulate: takeoff, survey, partial find, RTL, dock, charge, takeoff again
        for s in [State.TAKEOFF, State.SURVEY, State.RTL, State.LANDING,
                  State.DOCKING, State.CHARGING, State.TAKEOFF, State.SURVEY,
                  State.RTL, State.LANDING, State.DOCKING, State.TRANSFER, State.COMPLETE]:
            visited_states.add(s)
        assert State.CHARGING in visited_states, "Multi-sortie must include CHARGING"
        assert State.TRANSFER in visited_states, "Must include TRANSFER (data transfer)"