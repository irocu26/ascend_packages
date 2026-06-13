#!/usr/bin/env python3
"""
ascend_mission_control/precision_landing_node.py

Precision landing on ArUco marker — pure ROS2 velocity control via APInterface.

This is the ROS2-native replacement for the old ManualXYAutoLand + plnd.py classes
that used mavlink/mavproxy. No LANDING_TARGET MAVLink message is used — we control
the drone entirely through /ap/cmd_vel velocity setpoints, exactly as fsm_node does.

HOW IT WORKS:
  1. Subscribes to /ascend/vision/aruco_detection (from aruco_detector_node)
  2. Reads /ap/pose/filtered for current altitude
  3. Runs a P-controller: velocity = error * Kp
  4. Publishes velocities via APInterface.publish_velocity()
  5. When aligned (error < threshold) for ALIGN_HOLD_TIME_S consecutive seconds,
     switches to LAND mode via APInterface.set_mode(LAND_MODE=9)
  6. Publishes landing status on /ascend/precision_landing/status (String)
  7. Publishes landing_complete on /ascend/precision_landing/complete (Bool)
     so the FSM can transition from LANDING → DOCKING

STATE MACHINE (internal, separate from main FSM):
  IDLE        → waiting for /ascend/precision_landing/start (Bool True)
  DESCENDING  → reducing altitude toward DESCENT_TARGET_M using velocity,
                simultaneously centering over marker
  ALIGNING    → at or below DESCENT_TARGET_M, fine alignment only
  ALIGNED     → holding stable, counting ALIGN_HOLD_TIME_S
  LANDING     → LAND mode commanded, sending ALIGNED status, waiting for disarm
  ABORTED     → marker lost for too long during descent

Integration with FSM:
  - FSM publishes /ascend/precision_landing/start (Bool True) when entering LANDING state
  - This node publishes /ascend/precision_landing/complete (Bool True) when done
  - FSM subscribes to complete and transitions to DOCKING

Topics subscribed:
  /ascend/vision/aruco_detection     (std_msgs/String JSON)   — from aruco_detector_node
  /ap/pose/filtered                  (geometry_msgs/PoseStamped) — altitude
  /ascend/precision_landing/start    (std_msgs/Bool)           — trigger from FSM

Topics published:
  /ascend/precision_landing/status   (std_msgs/String)        — human-readable state
  /ascend/precision_landing/complete (std_msgs/Bool)          — True when landed
  /ap/cmd_vel                        (geometry_msgs/TwistStamped) — via APInterface

Parameters (all in mission_params.yaml under precision_landing_node):
  kp_horizontal     : 0.4    P-gain for XY centering
  kp_descent        : 0.3    P-gain for Z descent
  max_vel_xy_m_s    : 0.5    Max horizontal velocity during landing
  max_vel_z_m_s     : 0.4    Max descent velocity
  error_margin_m    : 0.12   Alignment threshold in metres
  align_hold_time_s : 2.0    Seconds to hold alignment before commanding LAND
  descent_target_m  : 1.2    Descend to this altitude (AGL) before fine-align
  search_timeout_s  : 8.0    Seconds without marker → abort
  brake_on_lost     : true   Apply brake when marker is lost
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import json
import time
from enum import Enum, auto

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from .ap_interface import APInterface, LAND_MODE


class PLNDState(Enum):
    IDLE       = auto()
    DESCENDING = auto()
    ALIGNING   = auto()
    ALIGNED    = auto()
    LANDING    = auto()
    COMPLETE   = auto()
    ABORTED    = auto()


class PrecisionLandingNode(Node):

    def __init__(self):
        super().__init__('precision_landing_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('kp_horizontal',    0.4)
        self.declare_parameter('kp_descent',       0.3)
        self.declare_parameter('max_vel_xy_m_s',   0.5)
        self.declare_parameter('max_vel_z_m_s',    0.4)
        self.declare_parameter('error_margin_m',   0.12)
        self.declare_parameter('align_hold_time_s',2.0)
        self.declare_parameter('descent_target_m', 1.2)
        self.declare_parameter('search_timeout_s', 8.0)
        self.declare_parameter('brake_on_lost',    True)

        self._kp_xy        = self.get_parameter('kp_horizontal').value
        self._kp_z         = self.get_parameter('kp_descent').value
        self._max_vxy      = self.get_parameter('max_vel_xy_m_s').value
        self._max_vz       = self.get_parameter('max_vel_z_m_s').value
        self._margin       = self.get_parameter('error_margin_m').value
        self._hold_time    = self.get_parameter('align_hold_time_s').value
        self._descent_tgt  = self.get_parameter('descent_target_m').value
        self._search_timeout_s = self.get_parameter('search_timeout_s').value
        self._brake_lost   = self.get_parameter('brake_on_lost').value

        # ── Internal state ────────────────────────────────────────────────
        self._state            = PLNDState.IDLE
        self._is_found         = False
        self._angle_x          = 0.0
        self._angle_y          = 0.0
        self._distance_m       = 0.0
        self._current_alt      = 3.0      # fallback altitude in metres
        self._aligned_since    = None     # monotonic time when alignment started
        self._last_seen_time   = None     # monotonic time of last marker detection
        self._brake_applied    = False
        self._land_cmd_sent    = False

        # ── AP interface ─────────────────────────────────────────────────
        self._ap = APInterface(self)

        # ── QoS ──────────────────────────────────────────────────────────
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

        # ── Subscribers ──────────────────────────────────────────────────
        self.sub_aruco = self.create_subscription(
            String, '/ascend/vision/aruco_detection',
            self._cb_aruco, sensor_qos
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, '/ap/pose/filtered',
            self._cb_pose, sensor_qos
        )
        self.sub_start = self.create_subscription(
            Bool, '/ascend/precision_landing/start',
            self._cb_start, reliable_qos
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_status   = self.create_publisher(String, '/ascend/precision_landing/status',   reliable_qos)
        self.pub_complete = self.create_publisher(Bool,   '/ascend/precision_landing/complete', reliable_qos)

        # ── Main control loop at 20 Hz ────────────────────────────────────
        self.create_timer(0.05, self._control_tick)

        self.get_logger().info(
            f'Precision landing node ready. '
            f'margin={self._margin}m hold={self._hold_time}s '
            f'descent_target={self._descent_tgt}m max_vxy={self._max_vxy}m/s'
        )

    # ─────────────────────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────────────────────

    def _cb_aruco(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._is_found   = bool(data.get('is_found', False))
            self._angle_x    = float(data.get('angle_x', 0.0))
            self._angle_y    = float(data.get('angle_y', 0.0))
            self._distance_m = float(data.get('distance_m', 0.0))
            if self._is_found:
                self._last_seen_time = time.monotonic()
                self._brake_applied  = False
        except Exception as e:
            self.get_logger().error(f'ArUco parse error: {e}')

    def _cb_pose(self, msg: PoseStamped):
        z = msg.pose.position.z
        if z > 0.05:   # ignore ground-clamp noise
            self._current_alt = z

    def _cb_start(self, msg: Bool):
        if msg.data and self._state == PLNDState.IDLE:
            self.get_logger().info('Precision landing START received.')
            self._state          = PLNDState.DESCENDING
            self._aligned_since  = None
            self._last_seen_time = None
            self._land_cmd_sent  = False
            self._brake_applied  = False

    # ─────────────────────────────────────────────────────────────────────
    #  Control tick — 20 Hz
    # ─────────────────────────────────────────────────────────────────────

    def _control_tick(self):
        if self._state == PLNDState.IDLE:
            return
        if self._state in (PLNDState.COMPLETE, PLNDState.ABORTED):
            return

        status = self._run_state_machine()
        self._publish_status(status)

    def _run_state_machine(self) -> str:

        # ── Already in LAND mode — just monitor ───────────────────────────
        if self._state == PLNDState.LANDING:
            if not self._land_cmd_sent:
                result = self._ap.set_mode(LAND_MODE)
                if result:
                    self._land_cmd_sent = True
                    self.get_logger().info('LAND mode commanded.')
                else:
                    self.get_logger().warn('LAND mode command failed — retrying.')
                    return 'PLND | LAND CMD RETRY'

            # Once we're very close to the ground, call it done
            if self._current_alt < 0.15:
                self._state = PLNDState.COMPLETE
                msg = Bool()
                msg.data = True
                self.pub_complete.publish(msg)
                self.get_logger().info('Precision landing COMPLETE.')
                return 'PLND | COMPLETE'

            return f'PLND | LANDING | alt={self._current_alt:.2f}m'

        # ── Marker lost check ─────────────────────────────────────────────
        if not self._is_found:
            if self._last_seen_time is not None:
                lost_for = time.monotonic() - self._last_seen_time
                if lost_for > self._search_timeout_s:
                    self._state = PLNDState.ABORTED
                    self._ap.hover()
                    self.get_logger().error(
                        f'PRECISION LANDING ABORTED — marker lost for {lost_for:.1f}s'
                    )
                    return 'PLND | ABORTED — MARKER LOST'

            if self._brake_lost and not self._brake_applied:
                self._ap.hover()
                self._brake_applied = True

            if self._last_seen_time is None:
                return 'PLND | SEARCHING — NO MARKER'
            return f'PLND | SEARCHING | lost {time.monotonic()-self._last_seen_time:.1f}s'

        # ── Marker found — calculate metric error ─────────────────────────
        # camera frame: image +X=right, image +Y=down
        # body frame:   +X=forward, +Y=right
        # angle_y in image = forward/backward error
        # angle_x in image = right/left error
        # negate angle_y because image-down maps to drone-forward
        alt = max(0.3, self._current_alt)
        dist_forward = alt * math.tan(-self._angle_y)   # m, positive = target is fwd
        dist_right   = alt * math.tan( self._angle_x)   # m, positive = target is right

        err_x = abs(dist_forward)
        err_y = abs(dist_right)
        aligned = (err_x < self._margin) and (err_y < self._margin)

        # ── DESCENDING state ──────────────────────────────────────────────
        if self._state == PLNDState.DESCENDING:
            # Center AND descend simultaneously
            vx = self._clamp(self._kp_xy * dist_forward, self._max_vxy)
            vy = self._clamp(self._kp_xy * dist_right,   self._max_vxy)

            # Descend toward descent_target_m
            if self._current_alt > self._descent_tgt:
                vz = -self._clamp(
                    self._kp_z * (self._current_alt - self._descent_tgt),
                    self._max_vz
                )
            else:
                vz = 0.0
                self._state = PLNDState.ALIGNING
                self.get_logger().info(
                    f'Reached descent target {self._descent_tgt}m — switching to fine align.'
                )

            self._ap.publish_velocity(vx, vy, vz, frame_id='base_link')
            return (
                f'PLND | DESCENDING | '
                f'alt={alt:.2f}m fwd={dist_forward:+.2f}m rgt={dist_right:+.2f}m'
            )

        # ── ALIGNING state ────────────────────────────────────────────────
        if self._state == PLNDState.ALIGNING:
            if not aligned:
                self._aligned_since = None

                # Deadband: only move on axes that are out of tolerance
                vx = self._clamp(self._kp_xy * dist_forward, self._max_vxy) \
                     if err_x > self._margin else 0.0
                vy = self._clamp(self._kp_xy * dist_right,   self._max_vxy) \
                     if err_y > self._margin else 0.0
                self._ap.publish_velocity(vx, vy, 0.0, frame_id='base_link')

                return (
                    f'PLND | ALIGNING | '
                    f'fwd={dist_forward:+.3f}m ({err_x:.3f}>{self._margin}) '
                    f'rgt={dist_right:+.3f}m ({err_y:.3f}>{self._margin})'
                )
            else:
                # Transition into ALIGNED
                self._ap.hover()
                if self._aligned_since is None:
                    self._aligned_since = time.monotonic()
                    self._state = PLNDState.ALIGNED
                    return 'PLND | ALIGNED | STABILIZING...'

        # ── ALIGNED state ─────────────────────────────────────────────────
        if self._state == PLNDState.ALIGNED:
            # Re-check — if marker drifted out during hold, go back to ALIGNING
            if not aligned:
                self._state = PLNDState.ALIGNING
                self._aligned_since = None
                return 'PLND | DRIFT DETECTED — re-aligning'

            self._ap.hover()   # Hold position

            held_for  = time.monotonic() - self._aligned_since
            remaining = max(0.0, self._hold_time - held_for)

            if held_for >= self._hold_time:
                self._state = PLNDState.LANDING
                self.get_logger().info(
                    f'Alignment held for {self._hold_time}s — commanding LAND mode.'
                )
                return 'PLND | ALIGNMENT CONFIRMED — COMMANDING LAND'

            return f'PLND | ALIGNED | landing in {remaining:.1f}s'

        return f'PLND | UNKNOWN STATE: {self._state}'

    # ─────────────────────────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────────────────────────

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(text, throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()