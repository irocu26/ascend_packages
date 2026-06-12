#!/usr/bin/env python3
"""
ascend_mission_control/ap_interface.py

ArduPilot uXRCE-DDS flight interface.

WHAT CHANGED FROM dds_flight_manager.py:
  - Renamed DDSFlightManager → APInterface
  - Removed the hardcoded flight sequence from main() — that was a demo script,
    not suitable for integration. main() now just connects and waits, so you can
    test this node alone with SITL running.
  - Added wait_for_services() helper so fsm_node can call it on startup.
  - Added guided_and_arm() convenience method combining mode switch + arm,
    which is what fsm_node needs in ARMING state.
  - All rclpy.spin_until_future_complete() calls now pass timeout_sec=5.0
    and check result — previously they blocked forever on failure.
  - publish_velocity() now accepts a frame_id parameter (default 'base_link').
  - Added is_service_ready() so fsm_node can check before calling.
  - move_for_duration() removed — fsm_node drives its own 10 Hz loop,
    duration-based blocking loops inside a node are wrong.

TOPICS / SERVICES (all correct ArduPilot uXRCE-DDS names):
  Publishers:
    /ap/cmd_vel          (geometry_msgs/TwistStamped)
  Service clients:
    /ap/arm_motors       (ardupilot_msgs/ArmMotors)
    /ap/mode_switch      (ardupilot_msgs/ModeSwitch)
    /ap/experimental/takeoff (ardupilot_msgs/Takeoff)

HOW TO TEST STANDALONE (no FSM, no SLAM — just SITL):
  Terminal A:  ros2 launch ardupilot_sitl sitl.launch.py
  Terminal B:  MicroXRCEAgent udp4 -p 2019
  Terminal C:  gz sim -r ~/ardu_ws/src/ardupilot_gazebo/worlds/iris_arena_ascend.sdf
  Terminal D:  ros2 run ascend_mission_control ap_interface
               (arms, switches to GUIDED, takes off to 3m, hovers, lands)
"""

import time
import rclpy
from rclpy.node import Node  # used only by standalone main()
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

# ArduPilot mode numbers (from ardupilot_msgs/ModeSwitch)
GUIDED_MODE = 4
LAND_MODE   = 9
LOITER_MODE = 5


class APInterface:
    """
    Thin wrapper around ArduPilot uXRCE-DDS services and topics.
    NOT a ROS2 node — attaches clients/publishers to the parent node so
    its futures are processed by the parent's executor.

    Example (inside AscendFSMNode.__init__):
        self._ap = APInterface(self)

    Example (inside _state_arming):
        if self._ap.is_service_ready():
            self._mode_future = self._ap.set_mode_async(4)
    """

    def __init__(self, node):
        self._node = node

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publisher ────────────────────────────────────────────────────
        self.cmd_vel_pub = node.create_publisher(
            TwistStamped, '/ap/cmd_vel', qos
        )

        # ── Service clients ──────────────────────────────────────────────
        self.arm_client = node.create_client(
            ArmMotors, '/ap/arm_motors'
        )
        self.mode_client = node.create_client(
            ModeSwitch, '/ap/mode_switch'
        )
        self.takeoff_client = node.create_client(
            Takeoff, '/ap/experimental/takeoff'
        )

        node.get_logger().info('APInterface ready — waiting for ArduPilot DDS services.')

    # ── Service availability ─────────────────────────────────────────────

    def is_service_ready(self) -> bool:
        """Returns True only when all three AP services are available."""
        return (
            self.arm_client.service_is_ready() and
            self.mode_client.service_is_ready() and
            self.takeoff_client.service_is_ready()
        )

    def wait_for_services(self, timeout_sec: float = 30.0) -> bool:
        """Block until all AP services are available or timeout."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.is_service_ready():
                self._node.get_logger().info('All ArduPilot DDS services available.')
                return True
            rclpy.spin_once(self._node, timeout_sec=0.5)
        self._node.get_logger().error('Timed out waiting for ArduPilot DDS services.')
        return False

    # ── Mode switch ──────────────────────────────────────────────────────

    def set_mode_async(self, mode: int):
        """Fire-and-forget mode switch. Returns a Future — caller polls it."""
        req = ModeSwitch.Request()
        req.mode = mode
        return self.mode_client.call_async(req)

    def set_mode(self, mode: int) -> bool:
<<<<<<< HEAD
        """Blocking mode switch — only use outside a timer callback."""
        req = ModeSwitch.Request()
        req.mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            self._node.get_logger().info(f'Mode switched to {mode}.')
            return True
        self._node.get_logger().error(f'Mode switch to {mode} failed or timed out.')
        return False
=======
        """
        Send mode switch request asynchronously.
        Non-blocking version safe for FSM callbacks.
        """

        req = ModeSwitch.Request()
        req.mode = mode

        self.mode_client.call_async(req)

        self.get_logger().info(
            f'Mode switch request sent: {mode}'
        )

        return True


>>>>>>> 616afd7 (Local mission control updates)

    def guided_mode(self) -> bool:
        return self.set_mode(GUIDED_MODE)

    def land_mode(self) -> bool:
        return self.set_mode(LAND_MODE)

    def loiter_mode(self) -> bool:
        return self.set_mode(LOITER_MODE)

    # ── Arm / disarm ─────────────────────────────────────────────────────

<<<<<<< HEAD
    def arm_async(self):
        """Fire-and-forget arm. Returns a Future — caller polls it."""
        req = ArmMotors.Request()
        req.arm = True
        return self.arm_client.call_async(req)

    def disarm_async(self):
        """Fire-and-forget disarm. Returns a Future — caller polls it."""
        req = ArmMotors.Request()
        req.arm = False
        return self.arm_client.call_async(req)

    def arm(self) -> bool:
        """Blocking arm — only use outside a timer callback."""
        req = ArmMotors.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            self._node.get_logger().info('Motors armed.')
            return True
        self._node.get_logger().error('Arm command failed or timed out.')
        return False

    def disarm(self) -> bool:
        """Blocking disarm — only use outside a timer callback."""
        req = ArmMotors.Request()
        req.arm = False
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            self._node.get_logger().info('Motors disarmed.')
            return True
        self._node.get_logger().error('Disarm command failed or timed out.')
        return False
=======
    
    def arm(self) -> bool:
        """
        Send arm command asynchronously.
        """

        req = ArmMotors.Request()
        req.arm = True

        self.arm_client.call_async(req)

        self.get_logger().info(
            'Arm command sent.'
        )

        return True


    def disarm(self) -> bool:
        """
        Send disarm command asynchronously.
        """

        req = ArmMotors.Request()
        req.arm = False

        self.arm_client.call_async(req)

        self.get_logger().info(
            'Disarm command sent.'
        )

        return True


>>>>>>> 616afd7 (Local mission control updates)

    def guided_and_arm(self) -> bool:
        """Blocking guided+arm — only use outside a timer callback."""
        if not self.guided_mode():
            return False
        time.sleep(0.5)
        return self.arm()

    # ── Takeoff ──────────────────────────────────────────────────────────

    def takeoff_async(self, altitude: float = 3.0):
        """Fire-and-forget takeoff. Returns a Future — caller polls it."""
        req = Takeoff.Request()
        req.alt = float(altitude)
        return self.takeoff_client.call_async(req)

    def takeoff(self, altitude: float = 3.0) -> bool:
        """
        Send takeoff request asynchronously.
        """

        req = Takeoff.Request()
        req.alt = float(altitude)
<<<<<<< HEAD
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            self._node.get_logger().info(f'Takeoff command sent — target {altitude:.1f}m AGL.')
            return True
        self._node.get_logger().error('Takeoff command failed or timed out.')
        return False
=======

        self.takeoff_client.call_async(req)

        self.get_logger().info(
            f'Takeoff request sent — target {altitude:.1f}m AGL.'
        )

        return True
>>>>>>> 616afd7 (Local mission control updates)

    # ── Velocity commands ────────────────────────────────────────────────

    def publish_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw_rate: float = 0.0,
        frame_id: str = 'base_link'
    ):
        """
        Publish a velocity setpoint to ArduPilot via /ap/cmd_vel.
        Called from fsm_node._send_position_cmd() at 10 Hz.

        vx, vy: horizontal velocity in m/s (body frame)
        vz:     vertical velocity in m/s (positive = up)
        yaw_rate: rotation rate in rad/s
        frame_id: coordinate frame ('base_link' for body, 'map' for world)
        """
        msg = TwistStamped()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.z = yaw_rate
        self.cmd_vel_pub.publish(msg)

    def hover(self):
        """Send zero velocity — hold position."""
        self.publish_velocity(0.0, 0.0, 0.0, 0.0)


# ── Standalone test entry point ──────────────────────────────────────────────
def main(args=None):
    """
    Standalone test: arm → GUIDED → takeoff 3m → hover 5s → land.
    Run with SITL + DDS agent + Gazebo already running.
    This tests the full AP command chain WITHOUT fsm_node or SLAM.

    ros2 run ascend_mission_control ap_interface
    """
    rclpy.init(args=args)
    ros_node = Node('ap_interface_test')
    ap = APInterface(ros_node)

    ros_node.get_logger().info('Waiting for ArduPilot DDS services...')
    if not ap.wait_for_services(timeout_sec=30.0):
        ros_node.get_logger().error('Services not available. Is SITL + DDS agent running?')
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    ros_node.get_logger().info('=== AP Interface standalone test ===')

    # Step 1: GUIDED mode
    ros_node.get_logger().info('Step 1: Switching to GUIDED mode...')
    ap.guided_mode()
    time.sleep(1.0)

    # Step 2: Arm
    ros_node.get_logger().info('Step 2: Arming motors...')
    ap.arm()
    time.sleep(1.0)

    # Step 3: Takeoff to 3m
    ros_node.get_logger().info('Step 3: Takeoff to 3.0m...')
    ap.takeoff(3.0)
    time.sleep(8.0)

    # Step 4: Hover for 5s
    ros_node.get_logger().info('Step 4: Hovering for 5s...')
    start = time.time()
    while time.time() - start < 5.0:
        ap.hover()
        rclpy.spin_once(ros_node, timeout_sec=0.1)
        time.sleep(0.1)

    # Step 5: Move forward at 0.5 m/s for 3s
    ros_node.get_logger().info('Step 5: Moving forward at 0.5 m/s for 3s...')
    start = time.time()
    while time.time() - start < 3.0:
        ap.publish_velocity(vx=0.5)
        rclpy.spin_once(ros_node, timeout_sec=0.1)
        time.sleep(0.1)

    # Step 6: Hover again
    ros_node.get_logger().info('Step 6: Hovering 3s...')
    start = time.time()
    while time.time() - start < 3.0:
        ap.hover()
        rclpy.spin_once(ros_node, timeout_sec=0.1)
        time.sleep(0.1)

    # Step 7: Land
    ros_node.get_logger().info('Step 7: Switching to LAND mode...')
    ap.land_mode()
    ros_node.get_logger().info('=== AP Interface test complete ===')

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()