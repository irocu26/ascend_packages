#!/usr/bin/env python3
"""
test_fsm_logic.py

Pure-Python unit tests for the ASCEND FSM — no ROS2 runtime, no Gazebo, no
hardware. Covers the bits that are testable without instantiating the node:
the State enum and the LawnmowerPattern survey geometry, plus a few mission
maths invariants. Node/state-handler behaviour is exercised in test/test_fsm.py
(which does spin up a real AscendFSMNode).

Run (no ROS2 launch needed):
  cd ~/ardu_ws/src/ascend_packages
  pytest ascend_mission_control/ascend_mission_control/test_fsm_logic.py -v

NOTE on the LawnmowerPattern API: it takes `strip_spacing_m` (metres between
adjacent rows) and `step_along_row_m` — there is no `overlap_factor` (that was an
older API). Coverage density is set by strip spacing, independent of altitude.
"""

import sys
import os

# Correct path regardless of where pytest is invoked from.
_test_dir = os.path.dirname(os.path.abspath(__file__))
_package_dir = os.path.dirname(_test_dir)    # ascend_mission_control/
_src_dir = os.path.dirname(_package_dir)     # src/ascend_packages/
sys.path.insert(0, _src_dir)
sys.path.insert(0, _package_dir)

import pytest
import math

# fsm_node guards its ardupilot_msgs import, so State + LawnmowerPattern import
# even when ardupilot_msgs/rclpy aren't present (e.g. a bare CI runner).
from ascend_mission_control.fsm_node import State, LawnmowerPattern


# ─────────────────────────────────────────────
#  LawnmowerPattern Tests
# ─────────────────────────────────────────────

class TestLawnmowerPattern:

    def test_generates_waypoints(self):
        p = LawnmowerPattern(altitude_m=3.0)
        assert p.total_waypoints() > 0

    def test_altitude_respected(self):
        alt = 4.0
        p = LawnmowerPattern(altitude_m=alt)
        p.reset()
        while True:
            wp = p.next_waypoint()
            if wp is None:
                break
            assert abs(wp[2] - alt) < 0.01, f"Waypoint altitude {wp[2]} != {alt}"

    def test_all_waypoints_inside_arena(self):
        p = LawnmowerPattern(altitude_m=3.0, arena_x_m=10.67, arena_y_m=7.62)
        p.reset()
        while True:
            wp = p.next_waypoint()
            if wp is None:
                break
            x, y, _ = wp
            assert 0 <= x <= 10.67, f"X={x} outside [0, 10.67]"
            assert 0 <= y <= 7.62, f"Y={y} outside [0, 7.62]"

    def test_waypoints_respect_wall_margin(self):
        margin = 0.5
        p = LawnmowerPattern(altitude_m=3.0, margin_m=margin,
                             arena_x_m=10.67, arena_y_m=7.62)
        p.reset()
        while True:
            wp = p.next_waypoint()
            if wp is None:
                break
            x, y, _ = wp
            # Allow a tiny epsilon for float arithmetic.
            assert margin - 1e-6 <= x <= 10.67 - margin + 1e-6
            assert margin - 1e-6 <= y <= 7.62 - margin + 1e-6

    def test_progress_monotonic(self):
        p = LawnmowerPattern(altitude_m=3.0)
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
        p = LawnmowerPattern(altitude_m=3.0)
        p.next_waypoint()
        p.next_waypoint()
        p.reset()
        assert p._index == 0
        assert not p.is_complete()

    def test_smaller_strip_spacing_more_waypoints(self):
        # Denser rows -> more coverage passes -> at least as many waypoints.
        p_coarse = LawnmowerPattern(altitude_m=3.0, strip_spacing_m=2.0)
        p_dense = LawnmowerPattern(altitude_m=3.0, strip_spacing_m=0.5)
        assert p_dense.total_waypoints() >= p_coarse.total_waypoints()

    def test_step_along_row_adds_intermediate_points(self):
        # Intermediate in-row points should only increase the waypoint count.
        p_plain = LawnmowerPattern(altitude_m=3.0, step_along_row_m=0.0)
        p_stepped = LawnmowerPattern(altitude_m=3.0, step_along_row_m=1.0)
        assert p_stepped.total_waypoints() >= p_plain.total_waypoints()

    def test_next_waypoint_returns_none_at_end(self):
        p = LawnmowerPattern(altitude_m=3.0)
        p.reset()
        for _ in range(p.total_waypoints()):
            p.next_waypoint()
        assert p.next_waypoint() is None
        assert p.is_complete()

    def test_final_waypoint_near_home(self):
        p = LawnmowerPattern(altitude_m=3.0)
        last = p._waypoints[-1]
        dist = math.sqrt(last[0] ** 2 + last[1] ** 2)
        assert dist < 2.0, f"Last waypoint {last} far from home (dist={dist:.2f}m)"

    @pytest.mark.parametrize("altitude", [2.0, 3.0, 4.0, 5.0, 6.0])
    def test_valid_altitude_range(self, altitude):
        p = LawnmowerPattern(altitude_m=altitude)
        assert p.total_waypoints() > 0


# ─────────────────────────────────────────────
#  State Enum Tests
# ─────────────────────────────────────────────

class TestStateEnum:

    def test_all_states_unique(self):
        values = [s.value for s in State]
        assert len(values) == len(set(values))

    def test_required_states_exist(self):
        """All states that map to mission phases must exist."""
        required = [
            'IDLE', 'ARMING', 'TAKEOFF', 'SURVEY', 'MATCH_VERIFY',
            'NAV_DEGRADED', 'RTL', 'LANDING', 'DOCKING', 'CHARGING',
            'TRANSFER', 'COMPLETE', 'FAILSAFE_RTL', 'FAILSAFE_LAND',
        ]
        state_names = [s.name for s in State]
        for name in required:
            assert name in state_names, f"Required state '{name}' missing"

    def test_failsafe_states_present(self):
        failsafe = [s for s in State if 'FAILSAFE' in s.name]
        assert len(failsafe) >= 2, "Need FAILSAFE_RTL and FAILSAFE_LAND"

    def test_no_legacy_states(self):
        """The old event-driven schema (states.py/fsm.py) must be gone."""
        legacy = ['HOVER', 'MISSION', 'ARMED', 'DISARMED', 'ERROR']
        state_names = [s.name for s in State]
        for name in legacy:
            assert name not in state_names, f"Legacy state '{name}' resurfaced"


# ─────────────────────────────────────────────
#  Coordinate Math Tests
# ─────────────────────────────────────────────

class TestCoordinateMath:

    def test_distance_calculation(self):
        def dist(x1, y1, z1, x2, y2, z2):
            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        assert abs(dist(0, 0, 0, 3, 4, 0) - 5.0) < 1e-6
        assert abs(dist(0, 0, 0, 0, 0, 3) - 3.0) < 1e-6
        assert dist(1, 2, 3, 1, 2, 3) == 0.0

    def test_base_station_relative_coords(self):
        home_x, home_y = 1.5, 2.3
        feat_x, feat_y = 4.5, 5.3
        assert abs((feat_x - home_x) - 3.0) < 1e-6
        assert abs((feat_y - home_y) - 3.0) < 1e-6


# ─────────────────────────────────────────────
#  Mission Logic Tests (threshold / maths invariants)
# ─────────────────────────────────────────────

class TestMissionLogic:

    def test_features_found_deduplication(self):
        found = []

        def add_feature(fid, x, y):
            if fid not in [f['seed_id'] for f in found]:
                found.append({'seed_id': fid, 'x': x, 'y': y})
        add_feature(0, 1.0, 2.0)
        add_feature(0, 1.0, 2.0)   # duplicate — ignored
        add_feature(1, 3.0, 4.0)
        assert len(found) == 2

    def test_battery_failsafe_threshold(self):
        """Mirrors _check_failsafe_conditions: strictly-below thresholds."""
        LOW_PCT, CRIT_PCT = 20.0, 10.0

        def check(pct):
            if pct < CRIT_PCT:
                return 'FAILSAFE_LAND'
            if pct < LOW_PCT:
                return 'FAILSAFE_RTL'
            return None

        assert check(50.0) is None
        assert check(20.0) is None              # at low threshold is NOT below it
        assert check(19.9) == 'FAILSAFE_RTL'
        assert check(10.0) == 'FAILSAFE_RTL'    # not below critical, but below low
        assert check(9.9) == 'FAILSAFE_LAND'

    def test_max_sorties_cap(self):
        MAX_SORTIES = 5
        sorties_done = 0

        def attempt():
            nonlocal sorties_done
            if sorties_done >= MAX_SORTIES:
                return False
            sorties_done += 1
            return True

        for _ in range(MAX_SORTIES):
            assert attempt() is True
        assert attempt() is False
        assert sorties_done == MAX_SORTIES


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
