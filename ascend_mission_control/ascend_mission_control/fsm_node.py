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
  /ascend/localization/pose          (geometry_msgs/PoseStamped)  — from slam_bridge_node
  /ascend/vision/match_result        (ascend_msgs/MatchResult)    — from matcher_node
  /ap/battery_status                 (sensor_msgs/BatteryState)   — from ArduPilot uXRCE-DDS
  /ap/pose/filtered                  (geometry_msgs/PoseStamped)  — from ArduPilot uXRCE-DDS
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

# CHANGE 1: Import APInterface (replaces sim stubs in _send_arm_command)
from .ap_interface import APInterface

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

    def __init__(self, altitude_m: float = 3.0, overlap_factor: float = 0.3):
        self.altitude = altitude_m
        self.overlap  = overlap_factor
        strip_width_m = 2.0 * altitude_m * math.tan(math.radians(45))
        self.strip_step = strip_width_m * (1.0 - overlap_factor)
        self._waypoints = []
        self._index = 0
        self._generate()

    def _generate(self):
        self._waypoints.clear()
        y = 0.0
        row = 0
        x_start = 0.5
        x_end   = self.ARENA_X_M - 0.5
        y_end   = self.ARENA_Y_M - 0.5

        while y <= y_end:
            if row % 2 == 0:
                self._waypoints.append((x_start, y, self.altitude))
                self._waypoints.append((x_end,   y, self.altitude))
            else:
                self._waypoints.append((x_end,   y, self.altitude))
                self._waypoints.append((x_start, y, self.altitude))
            y += self.strip_step
            row += 1

        # Return to home corner at survey altitude
        self._waypoints.append((0.5, 0.5, self.altitude))

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
    DOCK_TARGET_X           = 0.0
    DOCK_TARGET_Y           = 0.0
    DOCK_TARGET_Z           = 0.0
    TOTAL_FEATURES          = 3
    MAX_SORTIES             = 5
    CHARGE_TIMEOUT_S        = 300.0
    TRANSFER_TIMEOUT_S      = 60.0
    ARMING_TIMEOUT_S        = 10.0

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

        self.survey_alt      = self.get_parameter('survey_altitude').value
        self.wp_radius       = self.get_parameter('wp_accept_radius').value
        self.low_bat_pct     = self.get_parameter('low_battery_pct').value
        self.n_features      = self.get_parameter('total_features').value
        self.sim_mode        = self.get_parameter('sim_mode').value
        self.max_sorties     = self.get_parameter('max_sorties').value
        self.crit_bat_pct    = self.get_parameter('critical_battery_pct').value

        # ── Internal state ───────────────────
        self._state              = State.IDLE
        self._prev_state         = None
        self._current_pose       = None
        self._ap_pose            = None
        self._battery_pct        = 100.0
        self._last_pose_time     = None
        self._state_entry_time   = None
        self._start_cmd_received = False
        self._arm_confirmed      = False
        self._sorties_done       = 0
        self._features_found     = []
        self._pending_match      = None
        self._charging_done      = False
        self._transfer_done      = False
        self._charge_triggered   = False
        self._transfer_triggered = False
        self._failsafe_reason    = ''
        # CHANGE 6: track whether AP services are ready
        self._ap_services_ready  = False

        # ── Survey pattern ───────────────────
        self._pattern    = LawnmowerPattern(altitude_m=self.survey_alt, overlap_factor=0.3)
        self._current_wp = None

        # CHANGE 1: create APInterface node — handles all ArduPilot DDS comms
        self._ap = APInterface()

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
        self.sub_pose = self.create_subscription(
            PoseStamped, '/ascend/localization/pose', self._cb_pose, sensor_qos
        )
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

    def _cb_pose(self, msg: PoseStamped):
        self._current_pose  = msg
        self._last_pose_time = self.get_clock().now()

    def _cb_ap_pose(self, msg: PoseStamped):
        self._ap_pose = msg
        # Use AP pose as fallback for link-timeout tracking when SLAM not running
        if self._last_pose_time is None:
            self._last_pose_time = self.get_clock().now()

    def _cb_battery(self, msg: BatteryState):
        if msg.percentage >= 0.0:
            self._battery_pct = msg.percentage * 100.0

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

    # ─────────────────────────────────────────
    #  FSM Tick (10 Hz)
    # ─────────────────────────────────────────

    def _fsm_tick(self):
        self._check_failsafe_conditions()

        if   self._state == State.IDLE:          self._state_idle()
        elif self._state == State.ARMING:         self._state_arming()
        elif self._state == State.TAKEOFF:        self._state_takeoff()
        elif self._state == State.SURVEY:         self._state_survey()
        elif self._state == State.MATCH_VERIFY:   self._state_match_verify()
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

        # 1. Critical battery → land immediately
        if self._battery_pct < self.crit_bat_pct:
            self._trigger_failsafe('CRITICAL_BATTERY', State.FAILSAFE_LAND)
            return

        # 2. Low battery → RTL
        if self._battery_pct < self.low_bat_pct:
            self._trigger_failsafe('LOW_BATTERY', State.FAILSAFE_RTL)
            return

        # 3. Lost link — CHANGE 5: only after mission has started and pose was
        #    received at least once (prevents failsafe before SLAM connects)
        if self._last_pose_time is not None:
            dt = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
            if dt > self.LINK_TIMEOUT_S:
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

    # ─────────────────────────────────────────
    #  State Handlers
    # ─────────────────────────────────────────

    def _state_idle(self):
        pass  # Waiting for _cb_start

    def _state_arming(self):
        elapsed = self._elapsed_in_state()

        # CHANGE 6: on first tick, check services and send arm via APInterface
        if elapsed < 1.0:
            if not self._ap_services_ready:
                if self._ap.is_service_ready():
                    self._ap_services_ready = True
                    self.get_logger().info('AP services ready — sending arm command.')
                    self._send_arm_command()
                else:
                    self.get_logger().info('Waiting for AP DDS services...')
            return

        if self._is_armed():
            self.get_logger().info('Armed successfully — initiating takeoff.')
            self._transition(State.TAKEOFF)
            return

        if elapsed > self.ARMING_TIMEOUT_S:
            self.get_logger().error('Arming timed out — aborting mission.')
            self._transition(State.IDLE)
            self._start_cmd_received = False  # Allow retry

    def _state_takeoff(self):
        target = self._make_pose(0.5, 0.5, self.TAKEOFF_ALTITUDE_M)
        self.pub_target_pose.publish(target)
        self._send_position_cmd(0.5, 0.5, self.TAKEOFF_ALTITUDE_M)

        current_z = self._get_current_z()
        if current_z is not None and current_z >= (self.TAKEOFF_ALTITUDE_M - 0.3):
            self.get_logger().info(
                f'Reached takeoff altitude {current_z:.2f}m — starting survey. '
                f'Sortie #{self._sorties_done + 1}'
            )
            self._pattern.reset()
            self._current_wp = self._pattern.next_waypoint()
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

        if self._distance_to(x, y, z) < self.wp_radius:
            if self._elapsed_in_state() > self.HOVER_DWELL_S or self._state_entry_time is None:
                self._current_wp = self._pattern.next_waypoint()

    def _state_match_verify(self):
        if self._pending_match is None:
            self._transition(State.SURVEY)
            return
        if self._current_wp:
            x, y, z = self._current_wp
            self._send_position_cmd(x, y, z)
        if self._elapsed_in_state() >= self.MATCH_HOVER_S:
            self._confirm_match()

    def _confirm_match(self):
        if self._pending_match is None:
            return
        pose = self._current_pose
        x = pose.pose.position.x if pose else 0.0
        y = pose.pose.position.y if pose else 0.0

        feature = {
            'seed_id':    self._pending_match['seed_id'],
            'x':          round(x, 3),
            'y':          round(y, 3),
            'hd_path':    self._pending_match.get('hd_path', ''),
            'confidence': self._pending_match.get('confidence', 0.0)
        }
        self._features_found.append(feature)
        self._pending_match = None

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
        current_z = self._get_current_z() or self.survey_alt
        target_z  = max(0.0, current_z - 0.05)
        self._send_position_cmd(self.DOCK_TARGET_X, self.DOCK_TARGET_Y, target_z)

        if current_z < 0.15:
            self.get_logger().info('Landed — entering docking state.')
            self._send_disarm_command()
            self._transition(State.DOCKING)

    def _state_docking(self):
        if self._elapsed_in_state() > 2.0:
            self.get_logger().info('Docking complete.')
            self._sorties_done += 1

            if not self._charge_triggered:
                self._transition(State.TRANSFER)
            elif not self._transfer_done:
                self._transition(State.TRANSFER)
            elif len(self._features_found) < self.n_features and self._sorties_done < self.max_sorties:
                self._transition(State.CHARGING)
            else:
                self._transition(State.COMPLETE)

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
            self._charging_done = False
            if len(self._features_found) < self.n_features and self._sorties_done < self.max_sorties:
                self.get_logger().info(
                    f'Starting sortie #{self._sorties_done + 1}. '
                    f'Still need {self.n_features - len(self._features_found)} feature(s).'
                )
                self._transition(State.TAKEOFF)
            else:
                self._transition(State.COMPLETE)
            return

        if self._elapsed_in_state() > self.CHARGE_TIMEOUT_S:
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

        if self._elapsed_in_state() > self.TRANSFER_TIMEOUT_S:
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
        ) if self._current_pose else 99.0

        if dist_xy < self.wp_radius:
            self.get_logger().info('Failsafe RTL: above home — landing.')
            self._transition(State.FAILSAFE_LAND)

    def _state_failsafe_land(self):
        current_z = self._get_current_z() or 0.5
        target_z  = max(0.0, current_z - 0.1)
        self._send_position_cmd(
            self._get_current_x() or 0.0,
            self._get_current_y() or 0.0,
            target_z
        )
        if current_z < 0.1:
            self._send_disarm_command()
            self.get_logger().error(f'FAILSAFE LANDING COMPLETE. Reason: {self._failsafe_reason}')

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
        """
        CHANGE 2: now calls APInterface.guided_and_arm() for both sim and hardware.
        ArduPilot handles sim correctly — no need for separate sim stub.
        """
        self.get_logger().info('Sending GUIDED + ARM via APInterface...')
        success = self._ap.guided_and_arm()
        if success:
            self._arm_confirmed = True
            self.get_logger().info('Armed successfully via APInterface.')
        else:
            self.get_logger().error('guided_and_arm() failed — will retry via arming timeout.')

    def _is_armed(self) -> bool:
        return self._arm_confirmed

    def _send_disarm_command(self):
        """CHANGE 3: calls APInterface.disarm() — works on hardware and sim."""
        self._ap.disarm()
        self._arm_confirmed = False

    def _send_position_cmd(self, x: float, y: float, z: float):
        """P-controller velocity setpoint → /ap/cmd_vel via APInterface."""
        current_x = self._get_current_x() or 0.0
        current_y = self._get_current_y() or 0.0
        current_z = self._get_current_z() or 0.0

        kp_xy, kp_z    = 0.5, 0.8
        max_v_xy       = 1.5
        max_v_z        = 0.8

        vx = max(-max_v_xy, min(max_v_xy, kp_xy * (x - current_x)))
        vy = max(-max_v_xy, min(max_v_xy, kp_xy * (y - current_y)))
        vz = max(-max_v_z,  min(max_v_z,  kp_z  * (z - current_z)))

        # Publish via APInterface (which owns the /ap/cmd_vel publisher)
        self._ap.publish_velocity(vx, vy, vz)

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
        if self._current_pose:
            return self._current_pose.pose.position.x
        if self._ap_pose:
            return self._ap_pose.pose.position.x
        return None

    def _get_current_y(self):
        if self._current_pose:
            return self._current_pose.pose.position.y
        if self._ap_pose:
            return self._ap_pose.pose.position.y
        return None

    def _get_current_z(self):
        if self._current_pose:
            return self._current_pose.pose.position.z
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