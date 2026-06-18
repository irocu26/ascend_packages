#!/usr/bin/env python3
"""
ascend_mission_control/fsm_node.py

Finite State Machine node for ASCEND autonomous drone.
Controls full mission: takeoff → survey → match → RTL → dock → charge → repeat.

CHANGES FROM ORIGINAL:
  1. Added APInterface import — arm/disarm/takeoff now call real ArduPilot
     DDS services instead of sim stubs.
  2. _send_arm_command() now calls self._ap.guided_and_arm() in both sim
     and hardware modes (AP handles sim correctly too).
  3. _send_disarm_command() calls self._ap.disarm().
  4. Added max_sorties and critical_battery_pct as declared parameters
     (they were class constants but not ROS parameters — couldn't be set
     from mission_params.yaml at runtime).
  5. _check_failsafe_conditions(): pose-timeout failsafe now only fires
     AFTER the mission has started (state != IDLE and != ARMING), so the
     drone doesn't immediately failsafe before SLAM/AP pose arrives.
  6. _state_arming(): now waits for APInterface services to be ready
     before sending arm command. Fails gracefully if services unavailable.
  7. Added 'max_sorties' and 'critical_battery_pct' declare_parameter calls.

Topics consumed:
  /ascend/vision/match_result        (ascend_msgs/MatchResult)    — from matcher_node
  /ap/battery_status                 (sensor_msgs/BatteryState)   — from ArduPilot uXRCE-DDS
  /ap/pose/filtered                  (geometry_msgs/PoseStamped)  — fused EKF pose from
                                       ArduPilot uXRCE-DDS; SOLE position source for the FSM
                                       (already reflects the active EKF source set, so the raw
                                       SLAM pose is NOT consumed here — it feeds the EKF as
                                       EK3_SRC1 external nav)
  /ascend/mission_control/start_cmd  (std_msgs/Empty)             — single start trigger
  /ascend/ground_station/charge_done (std_msgs/Bool)              — charging complete signal
  /ascend/ground_station/transfer_done (std_msgs/Bool)            — data transfer complete

Topics published:
  /ascend/mission_control/state      (std_msgs/String)
  /ascend/mission_control/target_pose (geometry_msgs/PoseStamped)
  /ascend/mission_control/coord_log_str (std_msgs/String JSON)    — feature coords
  /ascend/mission_control/start_transfer (std_msgs/Bool)
  /ascend/mission_control/start_charge   (std_msgs/Bool)
  /ascend/mission_control/failsafe_triggered (std_msgs/String)
  /ap/cmd_vel        (geometry_msgs/TwistStamped)                 — via APInterface
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
from enum import Enum, auto

from std_msgs.msg import String, Empty, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState

# CHANGE 1: Import APInterface (replaces sim stubs in _send_arm_command).
# Guarded so the pure-logic classes (State, LawnmowerPattern) stay importable
# for unit tests even when ardupilot_msgs isn't built; the node needs it at run.
try:
    from .ap_interface import APInterface
except ImportError:
    APInterface = None

# Custom messages — defined in ascend_msgs package
# Falls back to String JSON if ascend_msgs not yet built
try:
    from ascend_msgs.msg import MatchResult, CoordLog
    CUSTOM_MSGS = True
except ImportError:
    CUSTOM_MSGS = False


# ─────────────────────────────────────────────
#  FSM State Enum
# ─────────────────────────────────────────────
class State(Enum):
    IDLE            = auto()
    ARMING          = auto()
    TAKEOFF         = auto()
    SURVEY          = auto()
    MATCH_VERIFY    = auto()
    NAV_DEGRADED    = auto()
    RTL             = auto()
    LANDING         = auto()
    DOCKING         = auto()
    CHARGING        = auto()
    TRANSFER        = auto()
    COMPLETE        = auto()
    FAILSAFE_RTL    = auto()
    FAILSAFE_LAND   = auto()


# ─────────────────────────────────────────────
#  Survey Pattern Generator
# ─────────────────────────────────────────────
class LawnmowerPattern:
    """
    Boustrophedon sweep over the IRoC-U arena.
    Arena: 35ft × 25ft = 10.67m × 7.62m (rulebook Section 10.2).
    Home/base station at corner (0, 0).
    """

    ARENA_X_M = 10.67
    ARENA_Y_M = 7.62

    def __init__(self, altitude_m: float = 3.0, strip_spacing_m: float = 1.0,
                 arena_x_m: float = 10.67, arena_y_m: float = 7.62,
                 margin_m: float = 0.5, step_along_row_m: float = 0.0):
        """
        altitude_m       : survey altitude.
        strip_spacing_m  : distance between adjacent lawnmower rows (smaller =
                           denser coverage of the whole arena). This is the key
                           knob — at 3 m altitude a ~1.0 m spacing gives full
                           overlapping coverage instead of just 2 edge passes.
        arena_x_m/y_m    : arena dimensions.
        margin_m         : keep-out margin from the arena walls.
        step_along_row_m : if > 0, insert intermediate waypoints along each row
                           every this many metres. Gives the SIFT matcher more
                           steady frames per row instead of one long dash.
        """
        self.altitude    = altitude_m
        self.strip_step  = max(0.3, strip_spacing_m)
        self.ARENA_X_M   = arena_x_m
        self.ARENA_Y_M   = arena_y_m
        self.margin      = margin_m
        self.row_step    = step_along_row_m
        self._waypoints  = []
        self._index = 0
        self._generate()

    def _row_points(self, x_start, x_end, y):
        """Return waypoints sweeping from x_start to x_end, optionally with
        intermediate points every row_step metres."""
        if self.row_step <= 0.0:
            return [(x_start, y, self.altitude), (x_end, y, self.altitude)]
        pts = []
        n = int(abs(x_end - x_start) / self.row_step)
        for i in range(n + 1):
            frac = i / max(1, n)
            x = x_start + (x_end - x_start) * frac
            pts.append((x, y, self.altitude))
        pts.append((x_end, y, self.altitude))
        return pts

    def _generate(self):
        self._waypoints.clear()
        x_start = self.margin
        x_end   = self.ARENA_X_M - self.margin
        y_start = self.margin
        y_end   = self.ARENA_Y_M - self.margin

        y   = y_start
        row = 0
        last_y = y
        while y <= y_end + 1e-6:
            if row % 2 == 0:
                self._waypoints.extend(self._row_points(x_start, x_end, y))
            else:
                self._waypoints.extend(self._row_points(x_end, x_start, y))
            last_y = y
            y += self.strip_step
            row += 1

        # If the fixed spacing didn't land on the far edge, add a final row so
        # the top of the arena is still covered (no missed strip).
        if last_y < y_end - 0.1:
            if row % 2 == 0:
                self._waypoints.extend(self._row_points(x_start, x_end, y_end))
            else:
                self._waypoints.extend(self._row_points(x_end, x_start, y_end))

        # Return to home corner at survey altitude
        self._waypoints.append((self.margin, self.margin, self.altitude))

    def reset(self):
        self._index = 0

    def next_waypoint(self):
        if self._index >= len(self._waypoints):
            return None
        wp = self._waypoints[self._index]
        self._index += 1
        return wp

    def current_waypoint(self):
        idx = max(0, self._index - 1)
        return self._waypoints[idx] if self._waypoints else None

    def is_complete(self):
        return self._index >= len(self._waypoints)

    def total_waypoints(self):
        return len(self._waypoints)

    def progress(self):
        return self._index / max(1, len(self._waypoints))


# ─────────────────────────────────────────────
#  Main FSM Node
# ─────────────────────────────────────────────
class AscendFSMNode(Node):

    SURVEY_ALTITUDE_M       = 3.0
    TAKEOFF_ALTITUDE_M      = 3.0
    WP_ACCEPTANCE_RADIUS_M  = 0.4
    HOVER_DWELL_S           = 1.5
    MATCH_HOVER_S           = 2.0
    LOW_BATTERY_PCT         = 20.0
    CRITICAL_BATTERY_PCT    = 10.0
    LINK_TIMEOUT_S          = 3.0
    BATTERY_TIMEOUT_S       = 5.0     # max gap in battery telemetry before failsafe
    DOCK_TARGET_X           = 0.0
    DOCK_TARGET_Y           = 0.0
    DOCK_TARGET_Z           = 0.0
    TOTAL_FEATURES          = 3
    MAX_SORTIES             = 5
    CHARGE_TIMEOUT_S        = 300.0
    TRANSFER_TIMEOUT_S      = 60.0
    ARMING_TIMEOUT_S        = 10.0
    TAKEOFF_TIMEOUT_S       = 20.0    # max climb-to-altitude time before abort
    LANDING_TIMEOUT_S       = 60.0    # max precision-landing time before blind-land fallback
    # ArduCopter flight-mode numbers
    GUIDED_MODE             = 4
    LOITER_MODE             = 5
    LAND_MODE               = 9

    def __init__(self):
        super().__init__('ascend_fsm_node')

        # ── Parameters ───────────────────────
        self.declare_parameter('survey_altitude',       self.SURVEY_ALTITUDE_M)
        self.declare_parameter('wp_accept_radius',      self.WP_ACCEPTANCE_RADIUS_M)
        self.declare_parameter('low_battery_pct',       self.LOW_BATTERY_PCT)
        self.declare_parameter('total_features',        self.TOTAL_FEATURES)
        self.declare_parameter('arena_x_m',             10.67)
        self.declare_parameter('arena_y_m',             7.62)
        self.declare_parameter('sim_mode',              True)
        # CHANGE 4: these were class constants but not ROS params — now tunable from yaml
        self.declare_parameter('max_sorties',           self.MAX_SORTIES)
        self.declare_parameter('critical_battery_pct',  self.CRITICAL_BATTERY_PCT)
        # Coverage + speed tuning for full-area SIFT survey
        self.declare_parameter('survey_strip_spacing',  1.0)   # m between rows
        self.declare_parameter('survey_speed',          0.4)   # m/s horizontal
        self.declare_parameter('row_step',              1.0)   # m between in-row points
        # Timing / timeout knobs — previously class constants only, so the
        # matching mission_params.yaml entries were silently ignored at runtime.
        self.declare_parameter('hover_dwell_s',      self.HOVER_DWELL_S)
        self.declare_parameter('match_hover_s',      self.MATCH_HOVER_S)
        self.declare_parameter('arming_timeout_s',   self.ARMING_TIMEOUT_S)
        self.declare_parameter('takeoff_timeout_s',  self.TAKEOFF_TIMEOUT_S)
        self.declare_parameter('landing_timeout_s',  self.LANDING_TIMEOUT_S)
        self.declare_parameter('charge_timeout_s',   self.CHARGE_TIMEOUT_S)
        self.declare_parameter('transfer_timeout_s', self.TRANSFER_TIMEOUT_S)
        self.declare_parameter('link_timeout_s',     self.LINK_TIMEOUT_S)
        self.declare_parameter('battery_timeout_s',  self.BATTERY_TIMEOUT_S)
        # Camera->body rotation for feature back-projection (#5), as a flat
        # row-major 3x3. Maps the camera optical ray (RDF: x-right, y-down,
        # z-forward) into the Pixhawk body frame (FRD: x-forward, y-right,
        # z-down). Default = identity: camera +x/+y/+z aligned with body
        # +x/+y/+z (down-facing mount). Edit this in mission_params.yaml to
        # re-orient the camera WITHOUT touching code.
        self.declare_parameter(
            'camera_to_body_rotation',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )

        self.survey_alt      = self.get_parameter('survey_altitude').value
        self.wp_radius       = self.get_parameter('wp_accept_radius').value
        self.low_bat_pct     = self.get_parameter('low_battery_pct').value
        self.n_features      = self.get_parameter('total_features').value
        self.sim_mode        = self.get_parameter('sim_mode').value
        self.max_sorties     = self.get_parameter('max_sorties').value
        self.crit_bat_pct    = self.get_parameter('critical_battery_pct').value
        self.arena_x         = self.get_parameter('arena_x_m').value
        self.arena_y         = self.get_parameter('arena_y_m').value
        self.strip_spacing   = self.get_parameter('survey_strip_spacing').value
        self.survey_speed    = self.get_parameter('survey_speed').value
        self.row_step        = self.get_parameter('row_step').value
        self.hover_dwell_s      = self.get_parameter('hover_dwell_s').value
        self.match_hover_s      = self.get_parameter('match_hover_s').value
        self.arming_timeout_s   = self.get_parameter('arming_timeout_s').value
        self.takeoff_timeout_s  = self.get_parameter('takeoff_timeout_s').value
        self.landing_timeout_s  = self.get_parameter('landing_timeout_s').value
        self.charge_timeout_s   = self.get_parameter('charge_timeout_s').value
        self.transfer_timeout_s = self.get_parameter('transfer_timeout_s').value
        self.link_timeout_s     = self.get_parameter('link_timeout_s').value
        self.battery_timeout_s  = self.get_parameter('battery_timeout_s').value
        _R = list(self.get_parameter('camera_to_body_rotation').value or [])
        if len(_R) != 9:
            self.get_logger().warn(
                f'camera_to_body_rotation needs 9 values, got {len(_R)} — '
                'falling back to identity.'
            )
            _R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._cam_to_body_R = [list(_R[0:3]), list(_R[3:6]), list(_R[6:9])]

        # ── Internal state ───────────────────
        self._state              = State.IDLE
        self._prev_state         = None
        # Single source of truth for vehicle position: ArduPilot's fused EKF
        # output (/ap/pose/filtered). It already reflects whichever EKF source
        # set the ekf_source_manager has selected — SLAM external nav (EK3_SRC1)
        # when tracking is healthy, optical flow + rangefinder (EK3_SRC2) when
        # SLAM is lost. The FSM must NOT also consume the raw SLAM pose: that is
        # a redundant EKF input which goes stale exactly when AP has switched
        # away from it, which would drive the velocity controller off a frozen
        # position (flyaway).
        self._ap_pose            = None
        self._battery_pct        = 100.0
        self._battery_valid      = False  # True once a real (non-NaN) reading arrives
        self._last_battery_time  = None   # freshness of battery telemetry
        self._last_pose_time     = None
        self._state_entry_time   = None
        self._start_cmd_received = False
        self._arm_confirmed      = False
        self._sorties_done       = 0
        self._features_found     = []
        self._pending_match      = None
        self._match_hold         = None   # (x,y,z) held over the feature in MATCH_VERIFY
        self._charging_done      = False
        self._transfer_done      = False
        self._charge_triggered   = False
        self._transfer_triggered = False
        self._failsafe_reason    = ''
        self._failsafe_landed         = False  # FAILSAFE_LAND terminal latch
        self._failsafe_land_confirmed = False  # AP LAND mode switch confirmed
        self._failsafe_land_future    = None   # pending LAND mode-switch future
        # CHANGE 6: track whether AP services are ready
        self._ap_services_ready  = False
        self._mode_future        = None   # pending async mode-switch future
        self._arm_future         = None   # pending async arm future
        self._arming_step        = 0      # 0=mode, 1=arm
        self._takeoff_sent       = False  # takeoff service called this sortie
        self._takeoff_future     = None   # pending async takeoff future
        self._plnd_started       = False
        self._plnd_complete      = False
        # SLAM-degraded hold (NAV_DEGRADED) — driven by ekf_source_manager's slam_ok.
        self._slam_ok            = True   # assume OK until told otherwise
        self._resume_state       = None   # state to return to after recovery
        self._loiter_sent        = False  # LOITER commanded for this degrade

        # ── Survey pattern ───────────────────
        self._pattern    = LawnmowerPattern(
            altitude_m=self.survey_alt,
            strip_spacing_m=self.strip_spacing,
            arena_x_m=self.arena_x,
            arena_y_m=self.arena_y,
            step_along_row_m=self.row_step,
        )
        self._current_wp = None
        # Wall-clock arrival time at the current survey waypoint, for the
        # per-waypoint hover dwell. None = not yet settled within wp_radius.
        # (Deliberately NOT _elapsed_in_state — see _state_survey.)
        self._wp_arrival_time = None
        self.get_logger().info(
            f'Survey pattern: {self._pattern.total_waypoints()} waypoints, '
            f'strip spacing {self.strip_spacing}m, speed {self.survey_speed}m/s'
        )

        # CHANGE 1: APInterface attaches to this node so futures are processed
        self._ap = APInterface(self)

        # ── QoS ─────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscribers ──────────────────────
        # Position comes solely from AP's fused EKF estimate. The raw SLAM pose
        # (/ascend/localization/pose) is an INPUT to that EKF (EK3_SRC1 external
        # nav), not a separate nav source for the FSM — see _ap_pose above.
        self.sub_ap_pose = self.create_subscription(
            PoseStamped, '/ap/pose/filtered', self._cb_ap_pose, sensor_qos
        )
        self.sub_battery = self.create_subscription(
            BatteryState, '/ap/battery_status', self._cb_battery, sensor_qos
        )
        self.sub_start = self.create_subscription(
            Empty, '/ascend/mission_control/start_cmd', self._cb_start, reliable_qos
        )
        self.sub_charge_done = self.create_subscription(
            Bool, '/ascend/ground_station/charge_done', self._cb_charge_done, reliable_qos
        )
        self.sub_transfer_done = self.create_subscription(
            Bool, '/ascend/ground_station/transfer_done', self._cb_transfer_done, reliable_qos
        )

        self.sub_plnd_complete = self.create_subscription(
            Bool,
            '/ascend/precision_landing/complete',
            self._cb_plnd_complete,
            reliable_qos
        )

        self.sub_slam_ok = self.create_subscription(
            Bool, '/ascend/localization/slam_ok', self._cb_slam_ok, reliable_qos
        )

        if CUSTOM_MSGS:
            self.sub_match = self.create_subscription(
                MatchResult, '/ascend/vision/match_result', self._cb_match, reliable_qos
            )
        else:
            self.sub_match = self.create_subscription(
                String, '/ascend/vision/match_result_str', self._cb_match_str, reliable_qos
            )

        # ── Publishers ───────────────────────
        self.pub_state          = self.create_publisher(String,      '/ascend/mission_control/state',            reliable_qos)
        self.pub_target_pose    = self.create_publisher(PoseStamped, '/ascend/mission_control/target_pose',      reliable_qos)
        self.pub_cmd_vel        = self.create_publisher(TwistStamped,'/ap/cmd_vel',                              reliable_qos)
        self.pub_start_transfer = self.create_publisher(Bool,        '/ascend/mission_control/start_transfer',   reliable_qos)
        self.pub_start_charge   = self.create_publisher(Bool,        '/ascend/mission_control/start_charge',     reliable_qos)
        self.pub_failsafe       = self.create_publisher(String,      '/ascend/mission_control/failsafe_triggered', reliable_qos)
        self.pub_plnd_start = self.create_publisher(
            Bool,
            '/ascend/precision_landing/start',
            reliable_qos
        )



        if CUSTOM_MSGS:
            self.pub_coord_log = self.create_publisher(CoordLog, '/ascend/mission_control/coord_log', reliable_qos)
        else:
            self.pub_coord_log = self.create_publisher(String, '/ascend/mission_control/coord_log_str', reliable_qos)

        # ── 10 Hz main loop ──────────────────
        self.timer = self.create_timer(0.1, self._fsm_tick)

        self.get_logger().info('ASCEND FSM node initialized — waiting for start command.')
        self._publish_state()

    # ─────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────

    def _cb_ap_pose(self, msg: PoseStamped):
        # The fused EKF pose is the FSM's only position source AND its
        # link-liveness signal. Update the timestamp on EVERY message so the
        # lost-link failsafe tracks the topic we actually navigate on (it keeps
        # flowing even when SLAM drops and AP falls back to optical flow).
        self._ap_pose = msg
        self._last_pose_time = self.get_clock().now()

    def _cb_battery(self, msg: BatteryState):
        # AP_DDS publishes percentage as a 0..1 fraction, or NaN when no battery
        # monitor is configured (e.g. bare SITL). The >= 0.0 test rejects both
        # NaN (NaN >= 0 is False) and negatives, so only a real reading updates
        # the level — and the freshness stamp that gates the battery failsafe.
        if msg.percentage >= 0.0:
            self._battery_pct       = msg.percentage * 100.0
            self._battery_valid     = True
            self._last_battery_time = self.get_clock().now()

    def _cb_start(self, _msg: Empty):
        if self._state != State.IDLE:
            self.get_logger().warn('Start command received but FSM not in IDLE — ignoring.')
            return
        if self._start_cmd_received:
            self.get_logger().warn('Start command already received — one allowed per rulebook.')
            return
        self._start_cmd_received = True
        self.get_logger().info('START COMMAND received — beginning autonomous mission.')
        self._transition(State.ARMING)

    def _cb_match(self, msg):
        if self._state not in (State.SURVEY, State.MATCH_VERIFY):
            return
        found_ids = [f['seed_id'] for f in self._features_found]
        if msg.seed_id in found_ids:
            return
        self._pending_match = {
            'seed_id':    msg.seed_id,
            'confidence': msg.confidence,
            'hd_path':    msg.hd_image_path
        }
        self.get_logger().info(f'Match candidate: seed_id={msg.seed_id} confidence={msg.confidence:.2f}')
        self._transition(State.MATCH_VERIFY)

    def _cb_match_str(self, msg: String):
        import json
        try:
            data = json.loads(msg.data)
            if self._state not in (State.SURVEY, State.MATCH_VERIFY):
                return
            found_ids = [f['seed_id'] for f in self._features_found]
            if data.get('seed_id') in found_ids:
                return
            self._pending_match = data
            self.get_logger().info(f'Match candidate (str): {data}')
            self._transition(State.MATCH_VERIFY)
        except Exception as e:
            self.get_logger().error(f'Failed to parse match string: {e}')

    def _cb_charge_done(self, msg: Bool):
        if msg.data:
            self._charging_done = True
            self.get_logger().info('Charging complete signal received.')

    def _cb_transfer_done(self, msg: Bool):
        if msg.data:
            self._transfer_done = True
            self.get_logger().info('Data transfer complete signal received.')

    def _cb_plnd_complete(self, msg: Bool):

        if msg.data:
            self._plnd_complete = True

            self.get_logger().info(
                'Precision landing completed.'
            )

    def _cb_slam_ok(self, msg: Bool):
        self._slam_ok = bool(msg.data)


    # ─────────────────────────────────────────
    #  FSM Tick (10 Hz)
    # ─────────────────────────────────────────

    def _fsm_tick(self):
        self._check_failsafe_conditions()
        self._check_nav_degraded()

        if   self._state == State.IDLE:          self._state_idle()
        elif self._state == State.ARMING:         self._state_arming()
        elif self._state == State.TAKEOFF:        self._state_takeoff()
        elif self._state == State.SURVEY:         self._state_survey()
        elif self._state == State.MATCH_VERIFY:   self._state_match_verify()
        elif self._state == State.NAV_DEGRADED:   self._state_nav_degraded()
        elif self._state == State.RTL:            self._state_rtl()
        elif self._state == State.LANDING:        self._state_landing()
        elif self._state == State.DOCKING:        self._state_docking()
        elif self._state == State.CHARGING:       self._state_charging()
        elif self._state == State.TRANSFER:       self._state_transfer()
        elif self._state == State.COMPLETE:       self._state_complete()
        elif self._state == State.FAILSAFE_RTL:   self._state_failsafe_rtl()
        elif self._state == State.FAILSAFE_LAND:  self._state_failsafe_land()

    # ─────────────────────────────────────────
    #  Failsafe Guard
    # ─────────────────────────────────────────

    def _check_failsafe_conditions(self):
        safe_states = {
            State.IDLE, State.ARMING,           # CHANGE 5: ARMING added — no failsafe during startup
            State.FAILSAFE_RTL, State.FAILSAFE_LAND,
            State.DOCKING, State.CHARGING, State.COMPLETE
        }
        if self._state in safe_states:
            return

        # Battery — only meaningful once AP has sent a real reading. AP_DDS sends
        # NaN when no monitor is configured, so a phantom default 100% must NOT
        # be used to clear the failsafe (the old default hid a dead/absent pack).
        if self._battery_valid:
            # Telemetry went stale mid-flight → lost a safety-critical sensor.
            dt_batt = (self.get_clock().now() - self._last_battery_time).nanoseconds / 1e9
            if dt_batt > self.battery_timeout_s:
                self._trigger_failsafe(
                    f'BATTERY_TELEMETRY_LOST ({dt_batt:.1f}s)', State.FAILSAFE_RTL
                )
                return
            # 1. Critical battery → land immediately
            if self._battery_pct < self.crit_bat_pct:
                self._trigger_failsafe('CRITICAL_BATTERY', State.FAILSAFE_LAND)
                return
            # 2. Low battery → RTL
            if self._battery_pct < self.low_bat_pct:
                self._trigger_failsafe('LOW_BATTERY', State.FAILSAFE_RTL)
                return
        else:
            # Never received a valid reading — the battery failsafe is disabled.
            # Surface it loudly rather than let it be a silent safety gap.
            self.get_logger().warn(
                'No valid battery telemetry — battery failsafe disabled.',
                throttle_duration_sec=10.0
            )

        # 3. Lost link — CHANGE 5: only after mission has started and pose was
        #    received at least once (prevents failsafe before SLAM connects)
        if self._last_pose_time is not None:
            dt = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
            if dt > self.link_timeout_s:
                self._trigger_failsafe(f'LOST_LINK ({dt:.1f}s no pose)', State.FAILSAFE_RTL)
                return

    def _trigger_failsafe(self, reason: str, target_state: State):
        if self._state in (State.FAILSAFE_RTL, State.FAILSAFE_LAND):
            return
        self._failsafe_reason = reason
        self.get_logger().error(f'FAILSAFE TRIGGERED: {reason} → {target_state.name}')
        msg = String()
        msg.data = f'{reason}|{target_state.name}'
        self.pub_failsafe.publish(msg)
        self._transition(target_state)

    def _check_nav_degraded(self):
        """SLAM-lost hold. If SLAM nav drops out during an autonomous nav state,
        hold position in LOITER until it recovers, then resume. The
        ekf_source_manager handles the EKF source switch (SLAM<->optical flow);
        here we only manage flight behaviour. Battery/link failsafes are checked
        first and take priority over this."""
        nav_states = {State.TAKEOFF, State.SURVEY, State.MATCH_VERIFY, State.RTL}
        if self._slam_ok:
            return
        if self._state in nav_states:
            self._resume_state = self._state
            self._loiter_sent  = False
            self.get_logger().warn(
                f'SLAM nav lost in {self._state.name} — holding (LOITER) until recovery.'
            )
            self._transition(State.NAV_DEGRADED)

    # ─────────────────────────────────────────
    #  State Handlers
    # ─────────────────────────────────────────

    def _state_idle(self):
        pass  # Waiting for _cb_start

    def _state_arming(self):
        elapsed = self._elapsed_in_state()

        # Abort guard — checked on EVERY tick, BEFORE the phase branches below.
        # Each of those branches returns early, so a timeout placed after them is
        # unreachable. This bounds the whole ARMING state: services that never
        # appear, a GUIDED switch AP keeps refusing, or an arm it keeps rejecting
        # (failed pre-arm checks) would otherwise loop forever — and ARMING is in
        # safe_states, so no failsafe would rescue it. On timeout we fall back to
        # IDLE and re-arm the start trigger so the operator can just start again.
        if elapsed > self.arming_timeout_s:
            self.get_logger().error(
                f'Arming did not complete within {self.arming_timeout_s:.0f}s '
                f'(stuck at step {self._arming_step}) — aborting to IDLE.'
            )
            self._mode_future        = None
            self._arm_future         = None
            self._arming_step        = 0
            self._start_cmd_received = False  # allow a fresh start command
            self._transition(State.IDLE)
            return

        # Wait for AP services to be ready
        if not self._ap_services_ready:
            if self._ap.is_service_ready():
                self._ap_services_ready = True
                self.get_logger().info('AP services ready — switching to GUIDED mode.')
                self._mode_future = self._ap.set_mode_async(4)  # GUIDED
            else:
                self.get_logger().info('Waiting for AP DDS services...')
            return

        # Step 0: wait for GUIDED mode confirmation
        if self._arming_step == 0:
            if self._mode_future is None:
                self._mode_future = self._ap.set_mode_async(4)
                return
            if self._mode_future.done():
                resp = self._mode_future.result()
                # ModeSwitch.Response carries `bool status`. A completed call only
                # means AP answered — `status` is what says the mode actually
                # changed. AP rejects GUIDED when the EKF/prearm isn't ready, so
                # treating any non-None result as success would arm in the wrong
                # mode.
                if resp is not None and resp.status:
                    self.get_logger().info('GUIDED mode confirmed — sending arm.')
                    self._arming_step = 1
                    self._arm_future = self._ap.arm_async()
                else:
                    reason = 'rejected by AP' if resp is not None else 'no response'
                    self.get_logger().error(
                        f'GUIDED switch {reason} — retrying.',
                        throttle_duration_sec=1.0
                    )
                    self._mode_future = self._ap.set_mode_async(4)
            return

        # Step 1: wait for arm confirmation
        if self._arming_step == 1:
            if self._arm_future is not None and self._arm_future.done():
                resp = self._arm_future.result()
                # ArmMotors.Response carries `bool result`. A completed call only
                # means AP answered — `result` is what says the motors actually
                # armed. Proceeding to TAKEOFF on a rejected arm is unsafe.
                if resp is not None and resp.result:
                    self._arm_confirmed = True
                    self.get_logger().info('Armed successfully — initiating takeoff.')
                    self._transition(State.TAKEOFF)
                    return
                else:
                    reason = 'rejected by AP' if resp is not None else 'no response'
                    self.get_logger().warn(
                        f'Arm {reason} — retrying.',
                        throttle_duration_sec=1.0
                    )
                    self._arm_future = self._ap.arm_async()
            return

    def _state_takeoff(self):
        # Climb watchdog — checked every tick, BEFORE the send/confirm branches
        # below (which return early, so a check after them would be unreachable,
        # cf. the arming timeout). If we haven't reached altitude in time the
        # takeoff isn't happening (silent reject, no climb, motor fault, or a
        # disarmed vehicle on a repeat sortie). TAKEOFF is past ARMING, so the
        # drone is armed and may be partly airborne — bring it down via
        # failsafe-land rather than hang here or drop to IDLE.
        if self._elapsed_in_state() > self.takeoff_timeout_s:
            cz = self._get_current_z()
            z_str = f'{cz:.1f}m' if cz is not None else 'no pose'
            self._takeoff_sent   = False
            self._takeoff_future = None
            self._trigger_failsafe(
                f'TAKEOFF_TIMEOUT ({self.takeoff_timeout_s:.0f}s, z={z_str})',
                State.FAILSAFE_LAND
            )
            return

        # Send the ArduPilot takeoff service ONCE. In GUIDED mode the drone
        # will not leave the ground from velocity setpoints alone — it needs
        # an explicit takeoff command. We must NOT spam cmd_vel during the
        # climb or it overrides the takeoff controller and the drone stays put.
        if not self._takeoff_sent:
            self.get_logger().info(f'Sending takeoff to {self.TAKEOFF_ALTITUDE_M}m...')
            self._takeoff_future = self._ap.takeoff_async(self.TAKEOFF_ALTITUDE_M)
            self._takeoff_sent = True
            return

        # Confirm AP actually ACCEPTED the takeoff (Takeoff.Response.status). A
        # completed service call alone doesn't mean the vehicle is climbing — AP
        # rejects takeoff if it isn't armed or isn't in GUIDED. Re-send on
        # rejection instead of silently waiting out the climb watchdog.
        if self._takeoff_future is not None:
            if not self._takeoff_future.done():
                return
            resp = self._takeoff_future.result()
            if resp is None or not resp.status:
                reason = 'rejected by AP' if resp is not None else 'no response'
                self.get_logger().warn(
                    f'Takeoff {reason} — re-sending.', throttle_duration_sec=1.0
                )
                self._takeoff_future = self._ap.takeoff_async(self.TAKEOFF_ALTITUDE_M)
                return
            self._takeoff_future = None  # accepted — now wait for the climb
            self.get_logger().info('Takeoff accepted — climbing.')

        # Just publish the target pose for visualization while climbing.
        self.pub_target_pose.publish(
            self._make_pose(0.5, 0.5, self.TAKEOFF_ALTITUDE_M)
        )

        current_z = self._get_current_z()
        if current_z is not None and current_z >= (self.TAKEOFF_ALTITUDE_M - 0.3):
            self.get_logger().info(
                f'Reached takeoff altitude {current_z:.2f}m — starting survey. '
                f'Sortie #{self._sorties_done + 1}'
            )
            self._takeoff_sent = False  # reset for next sortie
            self._pattern.reset()
            self._current_wp = self._pattern.next_waypoint()
            self._wp_arrival_time = None  # fresh dwell timer for the new sweep
            self._transition(State.SURVEY)

    def _state_survey(self):
        if self._current_wp is None:
            self.get_logger().info(
                f'Survey complete. Features: {len(self._features_found)}/{self.n_features}. RTL.'
            )
            self._transition(State.RTL)
            return

        x, y, z = self._current_wp
        self._send_position_cmd(x, y, z)

        # Continuously report drone position + survey progress during the sweep.
        cx, cy, cz = self._get_current_x(), self._get_current_y(), self._get_current_z()
        if cx is not None:
            self.get_logger().info(
                f'SURVEY pos=({cx:+.2f}, {cy:+.2f}, {cz:.2f})m  '
                f'→ wp=({x:.1f}, {y:.1f})  '
                f'progress={self._pattern.progress()*100:.0f}%  '
                f'features={len(self._features_found)}/{self.n_features}',
                throttle_duration_sec=1.0
            )

        if self._distance_to(x, y, z) < self.wp_radius:
            # Hover-dwell at THIS waypoint before advancing. _elapsed_in_state()
            # is the wrong clock: the whole sweep runs inside one SURVEY state,
            # so it measures time-in-survey (always > dwell after the first
            # 1.5 s) and every waypoint past the first would advance the instant
            # it's reached — no pause, no steady frames for SIFT. Time from
            # arrival at the current waypoint instead.
            if self._wp_arrival_time is None:
                self._wp_arrival_time = self.get_clock().now()
            else:
                dwell = (self.get_clock().now() - self._wp_arrival_time).nanoseconds / 1e9
                if dwell >= self.hover_dwell_s:
                    self._current_wp = self._pattern.next_waypoint()
                    self._wp_arrival_time = None  # re-arm for the next waypoint

    def _state_match_verify(self):
        if self._pending_match is None:
            self._match_hold = None
            self._transition(State.SURVEY)
            return

        # HOLD over the feature for the whole verification — do NOT keep flying
        # toward the next survey waypoint. Flying on drifts the logged coordinate
        # ~MATCH_HOVER_S of travel away from where the feature was actually seen,
        # and denies the matcher the steady frames it needs to confirm. Capture
        # the position once (first tick over the feature) and command back to it
        # each tick so the P-controller resists drift.
        if self._match_hold is None:
            cx, cy, cz = self._get_current_x(), self._get_current_y(), self._get_current_z()
            if cx is not None:
                self._match_hold = (cx, cy, cz)
        if self._match_hold is not None:
            self._send_position_cmd(*self._match_hold)

        if self._elapsed_in_state() >= self.match_hover_s:
            self._confirm_match()

    def _yaw_from_pose(self, pose):
        """Yaw in radians (about +z) from a PoseStamped quaternion; 0 if None."""
        if pose is None:
            return 0.0
        q = pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _confirm_match(self):
        if self._pending_match is None:
            return
        # Base = where the drone was when the feature was seen (held position),
        # not a fresh read taken MATCH_HOVER_S later.
        if self._match_hold is not None:
            base_x, base_y, alt = self._match_hold
        else:
            pose = self._ap_pose
            base_x = pose.pose.position.x if pose else 0.0
            base_y = pose.pose.position.y if pose else 0.0
            alt = pose.pose.position.z if pose else 0.0
        x, y = base_x, base_y

        # Back-project the feature's bearing to an arena coordinate (#5).
        # The vision node reports angle_x (+right) / angle_y (+down), so the
        # camera optical ray (RDF: x-right, y-down, z-forward) is
        # (tan ax, tan ay, 1). Rotate it into the FRD body frame with the
        # configurable camera_to_body_rotation, intersect the ground plane
        # (body +z = down, depth = alt), then rotate the body (forward, right)
        # offset into the world (map ENU) frame by the drone yaw and add it to
        # the drone position — so we log WHERE the feature is, not just where
        # the drone was (which can be off by up to half the camera footprint).
        # ASSUMES roughly level flight and /ap/pose/filtered in map ENU, yaw CCW
        # from +x. VERIFY in sim over a known feature; if the coordinate is
        # mirrored/rotated, fix the camera_to_body_rotation in the yaml.
        ax = self._pending_match.get('angle_x')
        ay = self._pending_match.get('angle_y')
        if ax is not None and ay is not None and alt and alt > 0.1:
            ray = (math.tan(float(ax)), math.tan(float(ay)), 1.0)  # camera RDF
            R = self._cam_to_body_R
            bx = R[0][0] * ray[0] + R[0][1] * ray[1] + R[0][2] * ray[2]  # forward
            by = R[1][0] * ray[0] + R[1][1] * ray[1] + R[1][2] * ray[2]  # right
            bz = R[2][0] * ray[0] + R[2][1] * ray[1] + R[2][2] * ray[2]  # down
            if bz > 1e-6:                       # ray must point at the ground
                s = alt / bz                    # scale to the ground plane
                fwd, right = bx * s, by * s
                yaw = self._yaw_from_pose(self._ap_pose)
                x = base_x + fwd * math.cos(yaw) + right * math.sin(yaw)
                y = base_y + fwd * math.sin(yaw) - right * math.cos(yaw)
                self.get_logger().info(
                    f'Feature back-projected: drone=({base_x:.2f},{base_y:.2f}) '
                    f'fwd={fwd:+.2f} right={right:+.2f} '
                    f'yaw={math.degrees(yaw):.0f}deg -> feature=({x:.2f},{y:.2f})m'
                )

        feature = {
            'seed_id':    self._pending_match['seed_id'],
            'x':          round(x, 3),
            'y':          round(y, 3),
            'hd_path':    self._pending_match.get('hd_path', ''),
            'confidence': self._pending_match.get('confidence', 0.0)
        }
        self._features_found.append(feature)
        self._pending_match = None
        self._match_hold    = None

        self.get_logger().info(
            f'Feature confirmed: id={feature["seed_id"]} '
            f'at ({feature["x"]:.3f}, {feature["y"]:.3f})m '
            f'confidence={feature["confidence"]:.2f}'
        )
        self._publish_coord_log(feature)

        if len(self._features_found) >= self.n_features:
            self.get_logger().info(f'All {self.n_features} features found! RTL.')
            self._transition(State.RTL)
        else:
            self.get_logger().info(
                f'{len(self._features_found)}/{self.n_features} found — resuming survey.'
            )
            self._transition(State.SURVEY)

    def _state_rtl(self):
        self._send_position_cmd(
            self.DOCK_TARGET_X + 0.5,
            self.DOCK_TARGET_Y + 0.5,
            self.survey_alt
        )
        dist = self._distance_to(
            self.DOCK_TARGET_X + 0.5,
            self.DOCK_TARGET_Y + 0.5,
            self.survey_alt
        )
        if dist < self.wp_radius:
            self.get_logger().info('Above dock — beginning landing sequence.')
            self._transition(State.LANDING)

    def _state_landing(self):

        # Trigger precision landing node ONCE
        if not self._plnd_started:

            msg = Bool()
            msg.data = True

            self.pub_plnd_start.publish(msg)

            self._plnd_started = True
            self._plnd_complete = False

            self.get_logger().info(
                'Precision landing initiated.'
            )

            return

        # Wait for precision landing node to finish
        if self._plnd_complete:

            self.get_logger().info(
                'Precision landing successful — entering docking.'
            )

            self._send_disarm_command()

            self._plnd_started = False
            self._plnd_complete = False

            self._transition(State.DOCKING)
            return

        # Watchdog: the precision lander reports `complete` ONLY on success. If
        # it aborts (marker lost past its search timeout) it just hovers and
        # never reports — and it won't restart from a re-published `start` — so
        # without this the FSM would hang in LANDING forever. On an aborted
        # landing the lander is silent and the vehicle is still in GUIDED above
        # the dock, so fall back to the FSM's own descent-and-disarm.
        if self._elapsed_in_state() > self.landing_timeout_s:
            self._plnd_started  = False
            self._plnd_complete = False
            self._trigger_failsafe(
                f'PRECISION_LANDING_TIMEOUT ({self.landing_timeout_s:.0f}s)',
                State.FAILSAFE_LAND
            )

    def _state_docking(self):
        if self._elapsed_in_state() > 2.0:
            self.get_logger().info('Docking complete.')
            self._sorties_done += 1
            # Upload this sortie's data after every landing, then let TRANSFER
            # decide whether to charge for another sortie or finish. The old
            # branch keyed off _charge_triggered/_transfer_done, which are sticky
            # across sorties — so after sortie 1 it skipped TRANSFER entirely and
            # the features found on later sorties were never uploaded.
            self._transition(State.TRANSFER)

    def _state_charging(self):
        if not self._charge_triggered:
            self._charge_triggered = True
            self._charging_done    = False
            msg = Bool()
            msg.data = True
            self.pub_start_charge.publish(msg)
            self.get_logger().info('Charging sequence initiated.')

        if self._charging_done:
            self.get_logger().info('Battery charged — ready for next sortie.')
            # Re-arm BOTH flags so the next sortie actually re-commands the
            # charger. Leaving _charge_triggered set means sortie 2+ skips the
            # start_charge publish and just waits out the timeout — the battery
            # never really recharges, so repeated sorties aren't possible.
            self._charge_triggered = False
            self._charging_done    = False
            if len(self._features_found) < self.n_features and self._sorties_done < self.max_sorties:
                self.get_logger().info(
                    f'Starting sortie #{self._sorties_done + 1}. '
                    f'Still need {self.n_features - len(self._features_found)} feature(s).'
                )
                # Re-ARM before the next sortie — do NOT jump straight to TAKEOFF.
                # Every landing disarms the vehicle (_send_disarm_command clears
                # _arm_confirmed and resets the arming sub-state), so a repeat
                # sortie must re-enter GUIDED + arm first. ARMING already does
                # exactly that, with the same result-checks and timeout abort;
                # going to TAKEOFF would command takeoff on a disarmed vehicle.
                self._transition(State.ARMING)
            else:
                self._transition(State.COMPLETE)
            return

        if self._elapsed_in_state() > self.charge_timeout_s:
            self.get_logger().warn('Charging timed out — proceeding.')
            self._charging_done = True

    def _state_transfer(self):
        if not self._transfer_triggered:
            self._transfer_triggered = True
            self._transfer_done      = False
            msg = Bool()
            msg.data = True
            self.pub_start_transfer.publish(msg)
            self.get_logger().info(
                f'Data transfer initiated: {len(self._features_found)} features.'
            )

        if self._transfer_done:
            self.get_logger().info('Data transfer verified.')
            self._transfer_triggered = False
            if len(self._features_found) < self.n_features and self._sorties_done < self.max_sorties:
                self._transition(State.CHARGING)
            else:
                self._transition(State.COMPLETE)
            return

        if self._elapsed_in_state() > self.transfer_timeout_s:
            self.get_logger().warn('Data transfer timed out — marking attempted.')
            self._transfer_done = True

    def _state_complete(self):
        if self._state != self._prev_state:
            self.get_logger().info(
                f'=== MISSION COMPLETE ===\n'
                f'Sorties: {self._sorties_done}\n'
                f'Features: {len(self._features_found)}/{self.n_features}'
            )
            for f in self._features_found:
                self.get_logger().info(
                    f'  Feature {f["seed_id"]}: ({f["x"]:.3f}, {f["y"]:.3f})m '
                    f'confidence={f["confidence"]:.2f}'
                )

    def _state_failsafe_rtl(self):
        self._send_position_cmd(self.DOCK_TARGET_X, self.DOCK_TARGET_Y, self.survey_alt)
        dist_xy = math.sqrt(
            (self._get_current_x() - self.DOCK_TARGET_X)**2 +
            (self._get_current_y() - self.DOCK_TARGET_Y)**2
        ) if self._ap_pose else 99.0

        if dist_xy < self.wp_radius:
            self.get_logger().info('Failsafe RTL: above home — landing.')
            self._transition(State.FAILSAFE_LAND)

    def _state_failsafe_land(self):
        # Terminal once we're down — go fully quiescent. The old handler kept
        # re-publishing a descent setpoint AND re-disarming AND re-logging every
        # tick (10 Hz) after touchdown, forever; it never latched.
        if self._failsafe_landed:
            return

        # Hand the descent to ArduPilot LAND mode: it has ground detection and
        # auto-disarm and does not depend on the FSM pose estimate, so it is far
        # more robust than stepping a velocity setpoint down — and unlike GUIDED
        # it actually disarms on the ground. Confirm the switch took before
        # trusting it; a silently-rejected switch would leave us hovering.
        if not self._failsafe_land_confirmed:
            if self._failsafe_land_future is None:
                self.get_logger().error(
                    f'FAILSAFE LAND ({self._failsafe_reason}) — commanding AP LAND mode.'
                )
                self._failsafe_land_future = self._ap.set_mode_async(self.LAND_MODE)
                return
            if self._failsafe_land_future.done():
                resp = self._failsafe_land_future.result()
                if resp is not None and resp.status:
                    self._failsafe_land_confirmed = True
                    self.get_logger().info('AP LAND mode confirmed — descending.')
                else:
                    self.get_logger().warn(
                        'LAND switch rejected — retrying.', throttle_duration_sec=1.0
                    )
                    self._failsafe_land_future = self._ap.set_mode_async(self.LAND_MODE)
            return

        # In LAND mode AP brings it down and disarms itself. Latch terminal once
        # we read near-ground altitude: disarm once (belt-and-suspenders) and
        # stop. If pose is unavailable we never read touchdown, but AP LAND still
        # lands and disarms on its own — the FSM just idles quietly here.
        current_z = self._get_current_z()
        if current_z is not None and current_z < 0.15:
            self._send_disarm_command()
            self._failsafe_landed = True
            self.get_logger().error(
                f'FAILSAFE LANDING COMPLETE (z={current_z:.2f}m). '
                f'Reason: {self._failsafe_reason}'
            )

    def _state_nav_degraded(self):
        # SLAM lost: hold position in LOITER (EKF is now on optical flow via the
        # ekf_source_manager) and wait for SLAM to recover. We deliberately send
        # NO cmd_vel here so AP holds. On recovery, return to GUIDED and resume
        # the interrupted state (survey progress / waypoint index is preserved).
        if not self._loiter_sent:
            self._ap.set_mode_async(self.LOITER_MODE)
            self._loiter_sent = True
            self.get_logger().warn('NAV_DEGRADED: holding in LOITER, waiting for SLAM.')
            return

        if self._slam_ok:
            self._ap.set_mode_async(self.GUIDED_MODE)
            self._loiter_sent = False
            resume = self._resume_state or State.SURVEY
            self.get_logger().info(f'SLAM recovered — GUIDED, resuming {resume.name}.')
            self._transition(resume)

    # ─────────────────────────────────────────
    #  Transition Helper
    # ─────────────────────────────────────────

    def _transition(self, new_state: State):
        if new_state == self._state:
            return
        self.get_logger().info(f'FSM: {self._state.name} → {new_state.name}')
        self._prev_state       = self._state
        self._state            = new_state
        self._state_entry_time = self.get_clock().now()
        self._publish_state()

    def _elapsed_in_state(self) -> float:
        if self._state_entry_time is None:
            return 0.0
        return (self.get_clock().now() - self._state_entry_time).nanoseconds / 1e9

    # ─────────────────────────────────────────
    #  ArduPilot Command Helpers
    # ─────────────────────────────────────────

    def _send_arm_command(self):
        pass  # arming is now handled step-by-step in _state_arming()

    def _is_armed(self) -> bool:
        return self._arm_confirmed

    def _send_disarm_command(self):
        self._ap.disarm_async()
        self._arm_confirmed = False
        self._arming_step   = 0
        self._mode_future   = None
        self._arm_future    = None

    def _send_position_cmd(self, x: float, y: float, z: float):
        """P-controller velocity setpoint → /ap/cmd_vel via APInterface."""
        current_x = self._get_current_x() or 0.0
        current_y = self._get_current_y() or 0.0
        current_z = self._get_current_z() or 0.0

        kp_xy, kp_z    = 0.5, 0.8
        max_v_xy       = self.survey_speed   # slow enough for SIFT to detect
        max_v_z        = 0.8

        vx = max(-max_v_xy, min(max_v_xy, kp_xy * (x - current_x)))
        vy = max(-max_v_xy, min(max_v_xy, kp_xy * (y - current_y)))
        vz = max(-max_v_z,  min(max_v_z,  kp_z  * (z - current_z)))

        # Publish in 'map' (world ENU) frame — NOT base_link. These velocities
        # are computed from world-frame position errors, so they must be sent
        # in the world frame. Using base_link makes ArduPilot rotate them by
        # the drone's yaw, which creates positive feedback and a flyaway.
        self._ap.publish_velocity(vx, vy, vz, frame_id='map')

        # Also publish target pose for visualization
        self.pub_target_pose.publish(self._make_pose(x, y, z))

    def _make_pose(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    # ─────────────────────────────────────────
    #  Pose Accessors
    # ─────────────────────────────────────────

    def _get_current_x(self):
        if self._ap_pose:
            return self._ap_pose.pose.position.x
        return None

    def _get_current_y(self):
        if self._ap_pose:
            return self._ap_pose.pose.position.y
        return None

    def _get_current_z(self):
        if self._ap_pose:
            return self._ap_pose.pose.position.z
        return None

    def _distance_to(self, x: float, y: float, z: float) -> float:
        cx, cy, cz = self._get_current_x(), self._get_current_y(), self._get_current_z()
        if cx is None or cy is None or cz is None:
            return 99.0
        return math.sqrt((x-cx)**2 + (y-cy)**2 + (z-cz)**2)

    # ─────────────────────────────────────────
    #  Publishers
    # ─────────────────────────────────────────

    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self.pub_state.publish(msg)

    def _publish_coord_log(self, feature: dict):
        if CUSTOM_MSGS:
            msg = CoordLog()
            msg.feature_id    = int(feature['seed_id'])
            msg.x             = float(feature['x'])
            msg.y             = float(feature['y'])
            msg.confidence    = float(feature['confidence'])
            msg.hd_image_path = str(feature['hd_path'])
            self.pub_coord_log.publish(msg)
        else:
            import json
            msg = String()
            msg.data = json.dumps(feature)
            self.pub_coord_log.publish(msg)


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AscendFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('FSM node shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()