
"""
FCU Bridge - ROS2 telemetry bridge for Gazebo + ArduPilot setup.
Handles odometry, IMU, GPS telemetry and simple motion commands.
"""

import logging
from typing import Dict

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist

from .states import GPSPoint

logger = logging.getLogger(__name__)


class FCUBridge(Node):

    def __init__(self):
        super().__init__('fcu_bridge')

        # ======================== Subscribers ========================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry',
            self._on_odometry,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self._on_imu,
            10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/navsat',
            self._on_gps,
            10
        )

        # ======================== Publishers ========================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ======================== State ========================

        self.gps_fix = False

        self.gps_position = GPSPoint(
            latitude=0.0,
            longitude=0.0,
            altitude=0.0
        )

        self.local_position = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }

        self.velocity = {
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0
        }

        self.attitude = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0
        }

        logger.info("FCU Bridge initialized")

        
        # ======================== Debug Timer ========================

        self.debug_timer = self.create_timer(
            2.0,
            self._debug_telemetry
        )

        # ======================== Motion Test Timer ========================

        self.test_timer = self.create_timer(
            1.0,
            self._test_motion
        )





    # ======================== Callbacks ========================

    def _on_odometry(self, msg: Odometry):

        self.local_position = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z
        }

        self.velocity = {
            "vx": msg.twist.twist.linear.x,
            "vy": msg.twist.twist.linear.y,
            "vz": msg.twist.twist.linear.z
        }

    def _on_imu(self, msg: Imu):

        self.attitude = {
            "roll": msg.orientation.x,
            "pitch": msg.orientation.y,
            "yaw": msg.orientation.z
        }

    def _on_gps(self, msg: NavSatFix):

        self.gps_fix = True

        self.gps_position = GPSPoint(
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude
        )

    # ======================== Debug ========================

    def _debug_telemetry(self):

        self.get_logger().info(
            f"POS: "
            f"x={self.local_position['x']:.2f}, "
            f"y={self.local_position['y']:.2f}, "
            f"z={self.local_position['z']:.2f} | "
            f"GPS FIX: {self.gps_fix}"
        )
    
    def _test_motion(self):

        self.move_forward(0.5)

        self.get_logger().info(
            "Publishing forward velocity command"
        )


    # ======================== Motion Commands ========================

    def hover(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def move_forward(self, speed: float = 1.0):

        cmd = Twist()

        cmd.linear.x = speed

        self.cmd_vel_pub.publish(cmd)

    def move_backward(self, speed: float = 1.0):

        cmd = Twist()

        cmd.linear.x = -speed

        self.cmd_vel_pub.publish(cmd)

    def move_up(self, speed: float = 0.5):

        cmd = Twist()

        cmd.linear.z = speed

        self.cmd_vel_pub.publish(cmd)

    def move_down(self, speed: float = 0.5):

        cmd = Twist()

        cmd.linear.z = -speed

        self.cmd_vel_pub.publish(cmd)

    # ======================== State Queries ========================

    def has_gps_fix(self) -> bool:
        return self.gps_fix

    def get_gps_position(self) -> GPSPoint:
        return self.gps_position

    def get_velocity(self) -> Dict[str, float]:
        return self.velocity.copy()

    def get_attitude(self) -> Dict[str, float]:
        return self.attitude.copy()

    def get_altitude(self) -> float:
        return abs(self.local_position["z"])

    def get_telemetry(self) -> Dict:

        return {
            "gps_fix": self.gps_fix,
            "position": self.gps_position,
            "local_position": self.local_position.copy(),
            "velocity": self.velocity.copy(),
            "attitude": self.attitude.copy(),
        }


def main(args=None):

    rclpy.init(args=args)

    node = FCUBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

