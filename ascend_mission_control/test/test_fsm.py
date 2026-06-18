#!/usr/bin/env python3
"""
test/test_fsm.py

Node-level regression tests for the ASCEND FSM. These instantiate the REAL
AscendFSMNode (rclpy + ardupilot_msgs required) with a stubbed APInterface, then
drive the state handlers directly and assert the transitions/flags. Each test is
tied to a specific fix from the FSM audit (see ascend_mission_control/README.md
changelog) so regressions surface immediately.

Plus the geo_utils maths tests (that module is still exported from __init__).

Run:
  cd ~/ardu_ws/src/ascend_packages
  pytest ascend_mission_control/test/test_fsm.py -v
"""

import os
import sys
import math
from types import SimpleNamespace

import pytest

# Make the inner ascend_mission_control package importable regardless of CWD.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ascend_mission_control.geo_utils import (         # noqa: E402
    haversine_distance, calculate_bearing, is_within_radius, lat_lon_offset,
)


# ─────────────────────────────────────────────────────────────────────────────
#  Test doubles for APInterface
# ─────────────────────────────────────────────────────────────────────────────

class _Resp:
    """Stand-in for a service response (carries .status and/or .result)."""
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Future:
    """Stand-in for an rclpy future — already 'done' with a fixed result."""
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result


class FakeAP:
    """Records commands and returns configurable futures, so the handlers can be
    exercised without a live ArduPilot DDS link."""
    def __init__(self):
        self.ready = True
        self.mode_resp = _Resp(status=True, curr_mode=0)
        self.arm_resp = _Resp(result=True)
        self.takeoff_resp = _Resp(status=True)
        self.modes = []          # mode numbers commanded
        self.velocities = []     # (vx, vy, vz, frame) published
        self.disarms = 0
        self.arms = 0
        self.takeoffs = 0

    def is_service_ready(self):
        return self.ready

    def set_mode_async(self, mode):
        self.modes.append(mode)
        return _Future(self.mode_resp)

    def arm_async(self):
        self.arms += 1
        return _Future(self.arm_resp)

    def disarm_async(self):
        self.disarms += 1
        return _Future(_Resp(result=True))

    def takeoff_async(self, alt):
        self.takeoffs += 1
        return _Future(self.takeoff_resp)

    def publish_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, frame_id='map'):
        self.velocities.append((vx, vy, vz, frame_id))

    def hover(self):
        self.velocities.append((0.0, 0.0, 0.0, 'hover'))


def _pose(x, y, z):
    """Minimal PoseStamped stand-in (only .pose.position.{x,y,z} is read)."""
    return SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z))
    )


def _pose_yaw(x, y, z, yaw):
    """PoseStamped stand-in with a yaw quaternion (about +z)."""
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
    ))


# ─────────────────────────────────────────────────────────────────────────────
#  Fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope='module')
def ros():
    import rclpy
    started = False
    if not rclpy.ok():
        rclpy.init()
        started = True
    yield
    if started:
        rclpy.shutdown()


@pytest.fixture
def node(ros):
    from ascend_mission_control.fsm_node import AscendFSMNode
    n = AscendFSMNode()
    n._ap = FakeAP()            # swap the real DDS interface for the stub
    yield n
    n.destroy_node()


def _stale(node, seconds):
    """A Time `seconds` in the past, for forcing elapsed-in-state timeouts."""
    from rclpy.duration import Duration
    return node.get_clock().now() - Duration(seconds=seconds)


def _State(node):
    from ascend_mission_control.fsm_node import State
    return State


# ─────────────────────────────────────────────────────────────────────────────
#  Sanity
# ─────────────────────────────────────────────────────────────────────────────

def test_initial_state_is_idle(node):
    State = _State(node)
    assert node._state == State.IDLE


# ─────────────────────────────────────────────────────────────────────────────
#  #1 — arming checks the service RESPONSE status, not just completion
# ─────────────────────────────────────────────────────────────────────────────

def _enter_arming(node):
    State = _State(node)
    node._state = State.ARMING
    node._ap_services_ready = True
    node._arming_step = 0
    node._mode_future = None
    node._arm_future = None
    node._arm_confirmed = False
    node._state_entry_time = node.get_clock().now()


def test_arming_accepted_advances_to_takeoff(node):
    State = _State(node)
    _enter_arming(node)
    node._state_arming()   # send GUIDED
    node._state_arming()   # GUIDED ok -> send arm
    node._state_arming()   # arm ok -> TAKEOFF
    assert node._arm_confirmed is True
    assert node._state == State.TAKEOFF


def test_arming_rejected_mode_does_not_advance(node):
    State = _State(node)
    _enter_arming(node)
    node._ap.mode_resp = _Resp(status=False)   # AP refuses GUIDED
    node._state_arming()   # send GUIDED
    node._state_arming()   # GUIDED rejected -> retry, stay step 0
    assert node._arming_step == 0
    assert node._arm_confirmed is False
    assert node._state == State.ARMING


def test_arming_rejected_arm_does_not_confirm(node):
    State = _State(node)
    _enter_arming(node)
    node._ap.arm_resp = _Resp(result=False)    # AP refuses to arm
    node._state_arming()   # GUIDED
    node._state_arming()   # GUIDED ok -> send arm
    node._state_arming()   # arm rejected -> retry, not confirmed
    assert node._arm_confirmed is False
    assert node._state == State.ARMING


# ─────────────────────────────────────────────────────────────────────────────
#  #2 — arming timeout is reachable and aborts to IDLE
# ─────────────────────────────────────────────────────────────────────────────

def test_arming_timeout_aborts_to_idle(node):
    State = _State(node)
    _enter_arming(node)
    node._arming_step = 1
    node._arm_future = _Future(_Resp(result=False))   # perpetual rejection
    node._start_cmd_received = True
    node._state_entry_time = _stale(node, node.arming_timeout_s + 1.0)
    node._state_arming()
    assert node._state == State.IDLE
    assert node._start_cmd_received is False           # re-armed for retry


# ─────────────────────────────────────────────────────────────────────────────
#  #3 — position comes ONLY from /ap/pose/filtered (no raw SLAM pose)
# ─────────────────────────────────────────────────────────────────────────────

def test_no_raw_slam_pose_attribute(node):
    assert not hasattr(node, '_current_pose')


def test_position_reads_from_ap_pose(node):
    node._ap_pose = _pose(1.0, 2.0, 3.0)
    assert node._get_current_x() == 1.0
    assert node._get_current_y() == 2.0
    assert node._get_current_z() == 3.0


# ─────────────────────────────────────────────────────────────────────────────
#  #4 — docking always transfers; charging re-commands the charger
# ─────────────────────────────────────────────────────────────────────────────

def test_docking_always_goes_to_transfer(node):
    State = _State(node)
    node._state = State.DOCKING
    # The old sticky flags that used to skip TRANSFER on sortie 2+:
    node._charge_triggered = True
    node._transfer_done = True
    node._state_entry_time = _stale(node, 3.0)
    node._state_docking()
    assert node._state == State.TRANSFER


def test_charging_rearms_charge_flag_and_reenters_arming(node):
    State = _State(node)
    node._state = State.CHARGING
    node._features_found = []          # still need features
    node._sorties_done = 1
    node._charge_triggered = True
    node._charging_done = True
    node._state_entry_time = node.get_clock().now()
    node._state_charging()
    assert node._charge_triggered is False     # re-armed so charger fires again
    assert node._state == State.ARMING         # re-arm path, not TAKEOFF


# ─────────────────────────────────────────────────────────────────────────────
#  #5 — per-waypoint hover dwell (timed from arrival, not time-in-state)
# ─────────────────────────────────────────────────────────────────────────────

def test_survey_dwells_before_advancing(node):
    State = _State(node)
    node._state = State.SURVEY
    node._pattern.reset()
    node._current_wp = node._pattern.next_waypoint()
    wp0 = node._current_wp
    node._ap_pose = _pose(*wp0)                 # sitting on the waypoint
    node._wp_arrival_time = None
    node._state_entry_time = node.get_clock().now()

    node._state_survey()                        # arrival tick: start dwell only
    assert node._wp_arrival_time is not None
    assert node._current_wp == wp0              # has NOT advanced yet

    node._wp_arrival_time = _stale(node, node.hover_dwell_s + 0.5)
    node._state_survey()                        # dwell satisfied -> advance
    assert node._current_wp != wp0


# ─────────────────────────────────────────────────────────────────────────────
#  #6 — MATCH_VERIFY holds over the feature and logs the held position
# ─────────────────────────────────────────────────────────────────────────────

def test_match_verify_holds_position_not_next_waypoint(node):
    State = _State(node)
    node._state = State.MATCH_VERIFY
    node._pending_match = {'seed_id': 1, 'confidence': 0.9, 'hd_path': ''}
    node._current_wp = (10.0, 7.0, 3.0)         # far-away survey waypoint
    node._ap_pose = _pose(3.0, 2.0, 3.0)        # where the feature was seen
    node._match_hold = None
    node._state_entry_time = node.get_clock().now()

    node._state_match_verify()
    assert node._match_hold == (3.0, 2.0, 3.0)  # captured current pos, not wp
    vx, vy, vz, _ = node._ap.velocities[-1]
    assert abs(vx) < 1e-6 and abs(vy) < 1e-6    # holding, not flying to (10,7)


def test_match_confirm_logs_held_position(node):
    State = _State(node)
    node._state = State.MATCH_VERIFY
    node._pending_match = {'seed_id': 2, 'confidence': 0.8, 'hd_path': 'x.png'}
    node._match_hold = (4.2, 1.1, 3.0)
    node._ap_pose = _pose(9.9, 9.9, 3.0)        # drifted away since detection
    node._features_found = []
    node._state_entry_time = node.get_clock().now()

    node._confirm_match()
    assert len(node._features_found) == 1
    feat = node._features_found[0]
    assert feat['x'] == 4.2 and feat['y'] == 1.1   # logged the HOLD, not drift
    assert node._match_hold is None


def test_match_confirm_backprojects_bearing_to_arena_coord(node):
    """#5: with a feature bearing + altitude + yaw, the logged coordinate is the
    back-projected feature position, not just the drone position."""
    State = _State(node)
    node._state = State.MATCH_VERIFY
    node._features_found = []
    # Drone holding at (5,5), 4 m up, facing +x (yaw 0).
    node._match_hold = (5.0, 5.0, 4.0)
    node._ap_pose = _pose_yaw(5.0, 5.0, 4.0, 0.0)
    # Default camera_to_body_rotation = identity (RDF camera == FRD body):
    #   fwd = alt*tan(angle_x), right = alt*tan(angle_y).
    ax = math.atan(0.5)     # fwd   = 4 * 0.5  = 2.0 m
    ay = math.atan(0.25)    # right = 4 * 0.25 = 1.0 m
    node._pending_match = {'seed_id': 7, 'confidence': 0.9, 'hd_path': '',
                           'angle_x': ax, 'angle_y': ay}
    node._state_entry_time = node.get_clock().now()

    node._confirm_match()
    feat = node._features_found[0]
    # yaw 0: x = base_x + fwd, y = base_y - right
    assert abs(feat['x'] - 7.0) < 0.05   # 5 + 2
    assert abs(feat['y'] - 4.0) < 0.05   # 5 - 1


# ─────────────────────────────────────────────────────────────────────────────
#  #7 — TAKEOFF / LANDING watchdogs
# ─────────────────────────────────────────────────────────────────────────────

def test_takeoff_timeout_triggers_failsafe_land(node):
    State = _State(node)
    node._state = State.TAKEOFF
    node._takeoff_sent = True
    node._takeoff_future = None
    node._ap_pose = _pose(0.0, 0.0, 0.2)        # never climbed
    node._state_entry_time = _stale(node, node.takeoff_timeout_s + 1.0)
    node._state_takeoff()
    assert node._state == State.FAILSAFE_LAND


def test_takeoff_rejected_resends_and_does_not_climb_check(node):
    node._state = _State(node).TAKEOFF
    node._takeoff_sent = True
    node._ap.takeoff_resp = _Resp(status=False)        # AP refuses takeoff
    node._takeoff_future = node._ap.takeoff_async(3.0)
    node._ap_pose = _pose(0.0, 0.0, 0.0)
    node._state_entry_time = node.get_clock().now()
    before = node._ap.takeoffs
    node._state_takeoff()
    assert node._ap.takeoffs == before + 1             # re-sent on rejection


def test_landing_timeout_triggers_failsafe_land(node):
    State = _State(node)
    node._state = State.LANDING
    node._plnd_started = True
    node._plnd_complete = False
    node._state_entry_time = _stale(node, node.landing_timeout_s + 1.0)
    node._state_landing()
    assert node._state == State.FAILSAFE_LAND


def test_landing_complete_disarms_and_docks(node):
    State = _State(node)
    node._state = State.LANDING
    node._plnd_started = True
    node._plnd_complete = True
    node._state_entry_time = node.get_clock().now()
    node._state_landing()
    assert node._ap.disarms == 1
    assert node._state == State.DOCKING


# ─────────────────────────────────────────────────────────────────────────────
#  re-arm gap — repeat sortie goes back through ARMING, never TAKEOFF disarmed
# ─────────────────────────────────────────────────────────────────────────────

def test_takeoff_only_reached_from_confirmed_arm(node):
    """ARMING is the only entry to TAKEOFF (so the vehicle is always armed)."""
    State = _State(node)
    _enter_arming(node)
    node._state_arming()
    node._state_arming()
    node._state_arming()
    assert node._state == State.TAKEOFF and node._arm_confirmed


# ─────────────────────────────────────────────────────────────────────────────
#  #L — FAILSAFE_LAND latches terminal (disarms once, then quiescent)
# ─────────────────────────────────────────────────────────────────────────────

def test_failsafe_land_latches_and_is_idempotent(node):
    node._state = _State(node).FAILSAFE_LAND
    node._failsafe_landed = False
    node._failsafe_land_confirmed = True        # pretend LAND mode confirmed
    node._ap_pose = _pose(0.0, 0.0, 0.05)       # on the ground
    node._state_entry_time = node.get_clock().now()

    node._state_failsafe_land()
    assert node._failsafe_landed is True
    assert node._ap.disarms == 1

    vel_before = len(node._ap.velocities)
    node._state_failsafe_land()                 # quiescent now
    assert node._ap.disarms == 1                # not re-disarmed
    assert len(node._ap.velocities) == vel_before   # no new commands


def test_failsafe_land_commands_land_mode(node):
    node._state = _State(node).FAILSAFE_LAND
    node._failsafe_landed = False
    node._failsafe_land_confirmed = False
    node._failsafe_land_future = None
    node._state_entry_time = node.get_clock().now()
    node._state_failsafe_land()
    assert node.LAND_MODE in node._ap.modes


# ─────────────────────────────────────────────────────────────────────────────
#  #K — battery telemetry handling
# ─────────────────────────────────────────────────────────────────────────────

def test_no_battery_telemetry_does_not_failsafe(node):
    State = _State(node)
    node._state = State.SURVEY
    node._battery_valid = False                 # never received a reading
    node._battery_pct = 100.0
    node._last_pose_time = None                 # keep lost-link guard inert
    node._check_failsafe_conditions()
    assert node._state == State.SURVEY          # warns, does not failsafe


def test_low_battery_triggers_rtl(node):
    State = _State(node)
    node._state = State.SURVEY
    node._battery_valid = True
    node._last_battery_time = node.get_clock().now()
    node._battery_pct = 15.0                     # below low (20)
    node._check_failsafe_conditions()
    assert node._state == State.FAILSAFE_RTL


def test_critical_battery_triggers_land(node):
    State = _State(node)
    node._state = State.SURVEY
    node._battery_valid = True
    node._last_battery_time = node.get_clock().now()
    node._battery_pct = 5.0                       # below critical (10)
    node._check_failsafe_conditions()
    assert node._state == State.FAILSAFE_LAND


def test_stale_battery_telemetry_triggers_failsafe(node):
    State = _State(node)
    node._state = State.SURVEY
    node._battery_valid = True
    node._battery_pct = 80.0                      # healthy level...
    node._last_battery_time = _stale(node, node.battery_timeout_s + 1.0)
    node._check_failsafe_conditions()
    assert node._state == State.FAILSAFE_RTL      # ...but telemetry went stale


def test_battery_callback_rejects_nan(node):
    node._battery_valid = False
    node._cb_battery(SimpleNamespace(percentage=float('nan')))
    assert node._battery_valid is False           # NaN ignored
    node._cb_battery(SimpleNamespace(percentage=0.5))
    assert node._battery_valid is True
    assert abs(node._battery_pct - 50.0) < 1e-6   # 0..1 fraction -> percent


# ─────────────────────────────────────────────────────────────────────────────
#  geo_utils (still exported from the package __init__)
# ─────────────────────────────────────────────────────────────────────────────

class TestGeoUtils:

    def test_haversine_distance_same_point(self):
        assert abs(haversine_distance(40.0, -74.0, 40.0, -74.0)) < 0.1

    def test_haversine_distance_known(self):
        # NYC -> Boston is ~300 km.
        dist = haversine_distance(40.7128, -74.0060, 42.3601, -71.0589)
        assert 250000 < dist < 350000

    def test_calculate_bearing_north(self):
        bearing = calculate_bearing(40.0, -74.0, 41.0, -74.0)
        assert bearing > 350 or bearing < 10

    def test_calculate_bearing_east(self):
        bearing = calculate_bearing(40.0, -74.0, 40.0, -73.0)
        assert 80 < bearing < 100

    def test_is_within_radius_true(self):
        assert is_within_radius(40.0, -74.0, 40.0001, -74.0001, 100) is True

    def test_is_within_radius_false(self):
        assert is_within_radius(40.0, -74.0, 40.1, -74.0, 100) is False

    def test_lat_lon_offset(self):
        lat, lon = lat_lon_offset(40.0, -74.0, 1000, 1000)
        assert abs(lat - 40.009) < 0.001
        # Longitude offset is cos(lat)-corrected: 1000 m east at 40degN.
        assert abs(lon - (-73.988)) < 0.001


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
