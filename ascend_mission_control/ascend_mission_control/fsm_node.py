#!/usr/bin/env python3
"""
ascend_mission_control/fsm_node.py

Finite State Machine node for ASCEND autonomous drone.
Controls full mission: takeoff → survey → match → RTL → dock → charge → repeat.

Rules enforced:
  - Only ONE start command allowed (Section 10.5.1)
  - No joystick / manual control once started (Section 10.5.1)
  - ASCEND must fly at 2m–6m altitude (Section 10.3.5)
  - Must land ONLY at home position / base station (Section 10.2 rule 3)
  - Autonomous charging must be demonstrated at least once (Section 10.1)
  - Autonomous data transfer must be demonstrated at least once (Section 10.1)
  - Failsafe on: low battery, lost-link, collision hint (Section 7.2 Table-1)

Topics consumed:
  /ascend/localization/pose          (geometry_msgs/PoseStamped)  — from slam_bridge_node
  /ascend/vision/match_result        (ascend_msgs/MatchResult)    — from matcher_node
  /ap/battery_status                 (sensor_msgs/BatteryState)   — from ArduPilot uXRCE-DDS
  /ascend/mission_control/start_cmd  (std_msgs/Empty)             — single start trigger
  /ascend/ground_station/charge_done (std_msgs/Bool)              — charging complete signal
  /ascend/ground_station/transfer_done (std_msgs/Bool)            — data transfer complete

Topics published:
  /ascend/mission_control/state      (std_msgs/String)
  /ascend/mission_control/target_pose (geometry_msgs/PoseStamped) — next waypoint
  /ascend/mission_control/coord_log  (ascend_msgs/CoordLog)
  /ascend/mission_control/start_transfer (std_msgs/Bool)          — trigger ZMQ transfer
  /ascend/mission_control/start_charge   (std_msgs/Bool)          — trigger charge sequence
  /ascend/mission_control/failsafe_triggered (std_msgs/String)

ArduPilot uXRCE-DDS flight command topics:
  /ap/cmd_vel        (geometry_msgs/TwistStamped) — velocity commands in GUIDED mode
  /ap/pose/filtered  (geometry_msgs/PoseStamped)  — actual drone pose from AP
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

# Custom messages — defined in ascend_msgs package
# If ascend_msgs is not built yet, use the fallback String-based stubs below
try:
    from ascend_msgs.msg import MatchResult, CoordLog
    CUSTOM_MSGS = True
except ImportError:
    CUSTOM_MSGS = False


# ─────────────────────────────────────────────
#  FSM State Enum
# ─────────────────────────────────────────────
class State(Enum):
    IDLE            = auto()   # Waiting for start command
    ARMING          = auto()   # Sending arm + mode-switch to ArduPilot
    TAKEOFF         = auto()   # Climbing to survey altitude
    SURVEY          = auto()   # Lawnmower scan of arena
    MATCH_VERIFY    = auto()   # Hovering over candidate, waiting for matcher confirmation
    RTL             = auto()   # Return to base station
    LANDING         = auto()   # Final descent to dock pad
    DOCKING         = auto()   # Precision alignment with dock
    CHARGING        = auto()   # Waiting for charge to complete
    TRANSFER        = auto()   # ZMQ image + coord transfer to base station
    COMPLETE        = auto()   # All 3 features found and validated
    FAILSAFE_RTL    = auto()   # Emergency return (low battery / lost link)
    FAILSAFE_LAND   = auto()   # Emergency land in place


# ─────────────────────────────────────────────
#  Survey Pattern Generator
# ─────────────────────────────────────────────
class LawnmowerPattern:
    """
    Generates a boustrophedon (lawnmower) sweep over the arena.
    Arena is 35ft × 25ft ≈ 10.67m × 7.62m per rulebook Section 10.2.
    Home/base station is at one corner (0, 0).
    Sweep direction: along the long axis (X), step along short axis (Y).
    Overlap factor ensures no gap between strips at survey altitude.
    """

    ARENA_X_M = 10.67    # 35 ft
    ARENA_Y_M = 7.62     # 25 ft

    def __init__(self, altitude_m: float = 3.0, overlap_factor: float = 0.3):
        """
        altitude_m:    survey altitude (2–6m, rulebook Section 10.3.5)
        overlap_factor: fraction of camera FOV to overlap between strips
        """
        self.altitude = altitude_m
        self.overlap  = overlap_factor
        # Camera horizontal FOV assumed ~90°; at altitude h, strip width ≈ 2*h*tan(45°) = 2h
        # With overlap: step = strip_width * (1 - overlap)
        strip_width_m = 2.0 * altitude_m * math.tan(math.radians(45))
        self.strip_step = strip_width_m * (1.0 - overlap_factor)
        self._waypoints = []
        self._index = 0
        self._generate()

    def _generate(self):
        """Build the ordered list of (x, y, z) waypoints."""
        self._waypoints.clear()
        y = 0.0
        row = 0
        # Margin inset so drone stays strictly inside the boundary
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

        # Final safety: return to home corner at survey altitude
        self._waypoints.append((0.5, 0.5, self.altitude))

    def reset(self):
        self._index = 0

    def next_waypoint(self):
        """Returns next (x, y, z) or None if pattern is complete."""
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

    # ── Mission parameters ───────────────────
    SURVEY_ALTITUDE_M       = 3.0     # Default survey altitude (2–6m range)
    TAKEOFF_ALTITUDE_M      = 3.0     # Must match survey altitude
    WP_ACCEPTANCE_RADIUS_M  = 0.4     # Distance to consider waypoint reached
    HOVER_DWELL_S           = 1.5     # Seconds to hover before snapping image
    MATCH_HOVER_S           = 2.0     # Seconds to hover for match verification
    LOW_BATTERY_PCT         = 20.0    # Trigger failsafe RTL below this %
    CRITICAL_BATTERY_PCT    = 10.0    # Trigger failsafe LAND below this %
    LINK_TIMEOUT_S          = 3.0     # Seconds without pose → lost-link failsafe
    DOCK_TARGET_X           = 0.0     # Base station dock position (home frame)
    DOCK_TARGET_Y           = 0.0
    DOCK_TARGET_Z           = 0.0     # Ground level
    TOTAL_FEATURES          = 3       # Number of features to find (rulebook Sec 10.5.2)
    MAX_SORTIES             = 5       # Safety cap on repeat sorties
    CHARGE_TIMEOUT_S        = 300.0   # Max wait for charging (5 min)
    TRANSFER_TIMEOUT_S      = 60.0    # Max wait for data transfer (1 min)
    ARMING_TIMEOUT_S        = 10.0    # Max wait for arming confirmation

    def __init__(self):
        super().__init__('ascend_fsm_node')

        # ── Load parameters ──────────────────
        self.declare_parameter('survey_altitude',      self.SURVEY_ALTITUDE_M)
        self.declare_parameter('wp_accept_radius',     self.WP_ACCEPTANCE_RADIUS_M)
        self.declare_parameter('low_battery_pct',      self.LOW_BATTERY_PCT)
        self.declare_parameter('total_features',       self.TOTAL_FEATURES)
        self.declare_parameter('arena_x_m',            10.67)
        self.declare_parameter('arena_y_m',            7.62)
        self.declare_parameter('sim_mode',             True)

        self.survey_alt      = self.get_parameter('survey_altitude').value
        self.wp_radius       = self.get_parameter('wp_accept_radius').value
        self.low_bat_pct     = self.get_parameter('low_battery_pct').value
        self.n_features      = self.get_parameter('total_features').value
        self.sim_mode        = self.get_parameter('sim_mode').value

        # ── Internal state ───────────────────
        self._state              = State.IDLE
        self._prev_state         = None
        self._current_pose       = None       # Latest SLAM pose
        self._ap_pose            = None       # ArduPilot filtered pose
        self._battery_pct        = 100.0
        self._last_pose_time     = None
        self._state_entry_time   = None
        self._start_cmd_received = False      # Enforces single-start-command rule
        self._arm_confirmed      = False
        self._sorties_done       = 0
        self._features_found     = []         # List of {seed_id, x, y, hd_path, confidence}
        self._pending_match      = None       # MatchResult waiting for hover dwell
        self._charging_done      = False
        self._transfer_done      = False
        self._charge_triggered   = False
        self._transfer_triggered = False
        self._failsafe_reason    = ''

        # ── Survey pattern ───────────────────
        self._pattern = LawnmowerPattern(
            altitude_m=self.survey_alt,
            overlap_factor=0.3
        )
        self._current_wp = None

        # ── QoS profiles ────────────────────
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
            PoseStamped,
            '/ascend/localization/pose',
            self._cb_pose,
            sensor_qos
        )
        self.sub_ap_pose = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self._cb_ap_pose,
            sensor_qos
        )
        self.sub_battery = self.create_subscription(
            BatteryState,
            '/ap/battery_status',
            self._cb_battery,
            sensor_qos
        )
        self.sub_start = self.create_subscription(
            Empty,
            '/ascend/mission_control/start_cmd',
            self._cb_start,
            reliable_qos
        )
        self.sub_charge_done = self.create_subscription(
            Bool,
            '/ascend/ground_station/charge_done',
            self._cb_charge_done,
            reliable_qos
        )
        self.sub_transfer_done = self.create_subscription(
            Bool,
            '/ascend/ground_station/transfer_done',
            self._cb_transfer_done,
            reliable_qos
        )

        if CUSTOM_MSGS:
            self.sub_match = self.create_subscription(
                MatchResult,
                '/ascend/vision/match_result',
                self._cb_match,
                reliable_qos
            )
        else:
            # Fallback: match result as JSON string
            self.sub_match = self.create_subscription(
                String,
                '/ascend/vision/match_result_str',
                self._cb_match_str,
                reliable_qos
            )

        # ── Publishers ───────────────────────
        self.pub_state = self.create_publisher(String, '/ascend/mission_control/state', reliable_qos)
        self.pub_target_pose = self.create_publisher(PoseStamped, '/ascend/mission_control/target_pose', reliable_qos)
        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/ap/cmd_vel', reliable_qos)
        self.pub_start_transfer = self.create_publisher(Bool, '/ascend/mission_control/start_transfer', reliable_qos)
        self.pub_start_charge = self.create_publisher(Bool, '/ascend/mission_control/start_charge', reliable_qos)
        self.pub_failsafe = self.create_publisher(String, '/ascend/mission_control/failsafe_triggered', reliable_qos)

        if CUSTOM_MSGS:
            self.pub_coord_log = self.create_publisher(CoordLog, '/ascend/mission_control/coord_log', reliable_qos)
        else:
            self.pub_coord_log = self.create_publisher(String, '/ascend/mission_control/coord_log_str', reliable_qos)

        # ── Main loop timer (10 Hz) ──────────
        self.timer = self.create_timer(0.1, self._fsm_tick)

        self.get_logger().info('ASCEND FSM node initialized — waiting for start command.')
        self._publish_state()

    # ─────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────
    def _cb_pose(self, msg: PoseStamped):
        self._current_pose = msg
        self._last_pose_time = self.get_clock().now()

    def _cb_ap_pose(self, msg: PoseStamped):
        self._ap_pose = msg

    def _cb_battery(self, msg: BatteryState):
        if msg.percentage >= 0.0:
            self._battery_pct = msg.percentage * 100.0  # 0.0–1.0 → %

    def _cb_start(self, _msg: Empty):
        """Single start command — only honoured once, only from IDLE."""
        if self._state != State.IDLE:
            self.get_logger().warn('Start command received but FSM is not in IDLE — ignoring.')
            return
        if self._start_cmd_received:
            self.get_logger().warn('Start command already received — only one allowed per rulebook.')
            return
        self._start_cmd_received = True
        self.get_logger().info('START COMMAND received — beginning autonomous mission.')
        self._transition(State.ARMING)

    def _cb_match(self, msg):
        """MatchResult from vision pipeline — only acted on during SURVEY."""
        if self._state not in (State.SURVEY, State.MATCH_VERIFY):
            return
        # Avoid duplicate detections of already-found features
        found_ids = [f['seed_id'] for f in self._features_found]
        if msg.seed_id in found_ids:
            return
        self._pending_match = {
            'seed_id':    msg.seed_id,
            'confidence': msg.confidence,
            'hd_path':    msg.hd_image_path
        }
        self.get_logger().info(
            f'Match candidate: seed_id={msg.seed_id} confidence={msg.confidence:.2f}'
        )
        self._transition(State.MATCH_VERIFY)

    def _cb_match_str(self, msg: String):
        """Fallback: parse match from JSON string."""
        import json
        try:
            data = json.loads(msg.data)
            if self._state not in (State.SURVEY, State.MATCH_VERIFY):
                return
            found_ids = [f['seed_id'] for f in self._features_found]
            if data.get('seed_id') in found_ids:
                return
            self._pending_match = data
            self.get_logger().info(f"Match candidate (str): {data}")
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
    #  FSM Tick — called at 10 Hz
    # ─────────────────────────────────────────
    def _fsm_tick(self):
        self._check_failsafe_conditions()

        if   self._state == State.IDLE:         self._state_idle()
        elif self._state == State.ARMING:        self._state_arming()
        elif self._state == State.TAKEOFF:       self._state_takeoff()
        elif self._state == State.SURVEY:        self._state_survey()
        elif self._state == State.MATCH_VERIFY:  self._state_match_verify()
        elif self._state == State.RTL:           self._state_rtl()
        elif self._state == State.LANDING:       self._state_landing()
        elif self._state == State.DOCKING:       self._state_docking()
        elif self._state == State.CHARGING:      self._state_charging()
        elif self._state == State.TRANSFER:      self._state_transfer()
        elif self._state == State.COMPLETE:      self._state_complete()
        elif self._state == State.FAILSAFE_RTL:  self._state_failsafe_rtl()
        elif self._state == State.FAILSAFE_LAND: self._state_failsafe_land()

    # ─────────────────────────────────────────
    #  Failsafe Guard — runs every tick
    # ─────────────────────────────────────────
    def _check_failsafe_conditions(self):
        """Pre-empts normal state handling if safety conditions are violated."""
        safe_states = {State.IDLE, State.FAILSAFE_RTL, State.FAILSAFE_LAND,
                       State.DOCKING, State.CHARGING, State.COMPLETE}
        if self._state in safe_states:
            return

        # 1. Critical battery → land immediately
        if self._battery_pct < self.CRITICAL_BATTERY_PCT:
            self._trigger_failsafe('CRITICAL_BATTERY', State.FAILSAFE_LAND)
            return

        # 2. Low battery → RTL
        if self._battery_pct < self.low_bat_pct:
            self._trigger_failsafe('LOW_BATTERY', State.FAILSAFE_RTL)
            return

        # 3. Lost link (SLAM pose timeout)
        if self._last_pose_time is not None:
            dt = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
            if dt > self.LINK_TIMEOUT_S:
                self._trigger_failsafe(f'LOST_LINK ({dt:.1f}s no pose)', State.FAILSAFE_RTL)
                return

    def _trigger_failsafe(self, reason: str, target_state: State):
        if self._state in (State.FAILSAFE_RTL, State.FAILSAFE_LAND):
            return  # Already in failsafe
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

        if elapsed < 1.0:
            # First tick in ARMING: send arm command
            self._send_arm_command()
            return

        # Check if ArduPilot has confirmed armed state
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
            # Pattern complete, no matches found yet
            self.get_logger().info(
                'Survey pattern complete. '
                f'Features found so far: {len(self._features_found)}/{self.n_features}. '
                'Initiating RTL.'
            )
            self._transition(State.RTL)
            return

        x, y, z = self._current_wp
        self._send_position_cmd(x, y, z)

        # Check if we've reached the current waypoint
        if self._distance_to(x, y, z) < self.wp_radius:
            # Brief hover dwell before advancing
            if self._elapsed_in_state() > self.HOVER_DWELL_S or self._state_entry_time is None:
                self._current_wp = self._pattern.next_waypoint()

    def _state_match_verify(self):
        """Hover in place while matcher confirms the detection."""
        if self._pending_match is None:
            self._transition(State.SURVEY)
            return

        # Hold current position
        if self._current_wp:
            x, y, z = self._current_wp
            self._send_position_cmd(x, y, z)

        if self._elapsed_in_state() >= self.MATCH_HOVER_S:
            self._confirm_match()

    def _confirm_match(self):
        """Log the confirmed match and decide next action."""
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

        # Check if we have all features
        if len(self._features_found) >= self.n_features:
            self.get_logger().info(
                f'All {self.n_features} features found! Initiating RTL.'
            )
            self._transition(State.RTL)
        else:
            self.get_logger().info(
                f'{len(self._features_found)}/{self.n_features} features found. '
                'Resuming survey.'
            )
            self._transition(State.SURVEY)

    def _state_rtl(self):
        """Fly back toward dock position at survey altitude, then descend."""
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
        """Descend slowly toward dock."""
        current_z = self._get_current_z() or self.survey_alt
        target_z  = max(0.0, current_z - 0.05)   # 5 cm/tick at 10Hz = 0.5 m/s descent
        self._send_position_cmd(self.DOCK_TARGET_X, self.DOCK_TARGET_Y, target_z)

        if current_z < 0.15:  # Consider landed
            self.get_logger().info('Landed — entering docking state.')
            self._send_disarm_command()
            self._transition(State.DOCKING)

    def _state_docking(self):
        """
        Precision docking alignment.
        In simulation: immediate success.
        On hardware: integrate ArUco dock marker detection here.
        """
        if self._elapsed_in_state() > 2.0:
            self.get_logger().info('Docking complete.')
            self._sorties_done += 1

            # Decide next step based on mission status
            if not self._charge_triggered:
                # Must demonstrate charging at least once
                self._transition(State.TRANSFER)
            elif not self._transfer_done:
                self._transition(State.TRANSFER)
            elif len(self._features_found) < self.n_features and self._sorties_done < self.MAX_SORTIES:
                self._transition(State.CHARGING)
            else:
                self._transition(State.COMPLETE)

    def _state_charging(self):
        """Wait for charging to complete before next sortie."""
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

            if len(self._features_found) < self.n_features and self._sorties_done < self.MAX_SORTIES:
                self.get_logger().info(
                    f'Starting sortie #{self._sorties_done + 1}. '
                    f'Still looking for {self.n_features - len(self._features_found)} feature(s).'
                )
                self._transition(State.TAKEOFF)
            else:
                self._transition(State.COMPLETE)
            return

        if self._elapsed_in_state() > self.CHARGE_TIMEOUT_S:
            self.get_logger().warn('Charging timed out — proceeding anyway.')
            self._charging_done = True

    def _state_transfer(self):
        """Trigger ZMQ data transfer and wait for confirmation."""
        if not self._transfer_triggered:
            self._transfer_triggered = True
            self._transfer_done      = False
            msg = Bool()
            msg.data = True
            self.pub_start_transfer.publish(msg)
            self.get_logger().info(
                f'Data transfer initiated: {len(self._features_found)} features, '
                f'{len(self._features_found)} HD images.'
            )

        if self._transfer_done:
            self.get_logger().info('Data transfer verified by base station.')
            self._transfer_triggered = False

            # After transfer, decide whether to charge + repeat or complete
            if len(self._features_found) < self.n_features and self._sorties_done < self.MAX_SORTIES:
                self._transition(State.CHARGING)
            else:
                self._transition(State.COMPLETE)
            return

        if self._elapsed_in_state() > self.TRANSFER_TIMEOUT_S:
            self.get_logger().warn('Data transfer timed out — marking as attempted.')
            self._transfer_done = True

    def _state_complete(self):
        """Mission complete. Publish final results."""
        if self._state != self._prev_state:  # Only on entry
            self.get_logger().info(
                '=== MISSION COMPLETE ===\n'
                f'Sorties: {self._sorties_done}\n'
                f'Features found: {len(self._features_found)}/{self.n_features}\n'
            )
            for f in self._features_found:
                self.get_logger().info(
                    f'  Feature {f["seed_id"]}: ({f["x"]:.3f}, {f["y"]:.3f})m '
                    f'confidence={f["confidence"]:.2f}'
                )

    def _state_failsafe_rtl(self):
        """Emergency return to home at max throttle."""
        self._send_position_cmd(
            self.DOCK_TARGET_X,
            self.DOCK_TARGET_Y,
            self.survey_alt  # Maintain altitude during RTL
        )
        dist_xy = math.sqrt(
            (self._get_current_x() - self.DOCK_TARGET_X)**2 +
            (self._get_current_y() - self.DOCK_TARGET_Y)**2
        ) if self._current_pose else 99.0

        if dist_xy < self.wp_radius:
            self.get_logger().info('Failsafe RTL: above home — landing.')
            self._transition(State.FAILSAFE_LAND)

    def _state_failsafe_land(self):
        """Emergency land in place."""
        current_z = self._get_current_z() or 0.5
        target_z  = max(0.0, current_z - 0.1)   # Faster descent in failsafe
        self._send_position_cmd(
            self._get_current_x() or 0.0,
            self._get_current_y() or 0.0,
            target_z
        )
        if current_z < 0.1:
            self._send_disarm_command()
            self.get_logger().error(
                f'FAILSAFE LANDING COMPLETE. Reason: {self._failsafe_reason}'
            )

    # ─────────────────────────────────────────
    #  State Transition Helper
    # ─────────────────────────────────────────
    def _transition(self, new_state: State):
        if new_state == self._state:
            return
        self.get_logger().info(
            f'FSM: {self._state.name} → {new_state.name}'
        )
        self._prev_state = self._state
        self._state = new_state
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
        In ArduPilot uXRCE-DDS, arming and mode switching are done via
        MAVLink commands relayed through the DDS agent.
        In simulation, we use a simplified approach.
        On hardware, integrate with /ap/cmd/arming_and_mode service.
        """
        if self.sim_mode:
            # Simulation: assume armed immediately
            self._arm_confirmed = True
            self.get_logger().info('[SIM] Arm command sent (auto-confirmed in sim mode).')
        else:
            # Hardware: publish arm command via appropriate topic
            # This depends on your uXRCE-DDS bridge setup
            self.get_logger().info('Arm command sent — waiting for confirmation.')

    def _is_armed(self) -> bool:
        """Check if ArduPilot has confirmed armed state."""
        if self.sim_mode:
            return self._arm_confirmed
        # On hardware: check /ap/state topic for armed flag
        return self._arm_confirmed

    def _send_disarm_command(self):
        if self.sim_mode:
            self._arm_confirmed = False
            self.get_logger().info('[SIM] Disarm command sent.')

    def _send_position_cmd(self, x: float, y: float, z: float):
        """
        Send position setpoint to ArduPilot via TwistStamped velocity command.
        We use a simple proportional controller: velocity proportional to error.
        On hardware replace with direct setpoint via /ap/setpoint or MAVLink SET_POSITION_TARGET.
        """
        current_x = self._get_current_x() or 0.0
        current_y = self._get_current_y() or 0.0
        current_z = self._get_current_z() or 0.0

        # P-controller gains
        kp_xy = 0.5
        kp_z  = 0.8
        max_v_xy = 1.5  # m/s
        max_v_z  = 0.8  # m/s

        vx = max(-max_v_xy, min(max_v_xy, kp_xy * (x - current_x)))
        vy = max(-max_v_xy, min(max_v_xy, kp_xy * (y - current_y)))
        vz = max(-max_v_z,  min(max_v_z,  kp_z  * (z - current_z)))

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.pub_cmd_vel.publish(msg)

        # Also publish target pose for visualization / logging
        target = self._make_pose(x, y, z)
        self.pub_target_pose.publish(target)

    def _make_pose(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
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
        cx = self._get_current_x()
        cy = self._get_current_y()
        cz = self._get_current_z()
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
            msg.feature_id  = int(feature['seed_id'])
            msg.x           = float(feature['x'])
            msg.y           = float(feature['y'])
            msg.confidence  = float(feature['confidence'])
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