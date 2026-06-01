
import time

import rclpy

from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy

from geometry_msgs.msg import TwistStamped

from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff


GUIDED_MODE = 4
LAND_MODE = 9


class DDSFlightManager(Node):

    def __init__(self):

        super().__init__("dds_flight_manager")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==================================================
        # Publisher
        # ==================================================

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            "/ap/cmd_vel",
            qos
        )

        # ==================================================
        # Service Clients
        # ==================================================

        self.arm_client = self.create_client(
            ArmMotors,
            "/ap/arm_motors"
        )

        self.mode_client = self.create_client(
            ModeSwitch,
            "/ap/mode_switch"
        )

        self.takeoff_client = self.create_client(
            Takeoff,
            "/ap/experimental/takeoff"
        )

        self.get_logger().info(
            "DDS Flight Manager Started"
        )

    # ======================================================
    # Publish velocity command
    # ======================================================

    def publish_velocity(
        self,
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw_rate=0.0
    ):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = "base_link"

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz

        msg.twist.angular.z = yaw_rate

        self.cmd_vel_pub.publish(msg)

    # ======================================================
    # Mode switch
    # ======================================================

    def guided_mode(self):

        req = ModeSwitch.Request()

        req.mode = GUIDED_MODE

        future = self.mode_client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            "GUIDED mode enabled"
        )

    def land(self):

        req = ModeSwitch.Request()

        req.mode = LAND_MODE

        future = self.mode_client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            "LAND mode enabled"
        )

    # ======================================================
    # Arm
    # ======================================================

    def arm(self):

        req = ArmMotors.Request()

        req.arm = True

        future = self.arm_client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            "Drone armed"
        )

    # ======================================================
    # Takeoff
    # ======================================================

    def takeoff(self, altitude=3.0):

        req = Takeoff.Request()

        req.alt = altitude

        future = self.takeoff_client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            "Takeoff command sent"
        )

    # ======================================================
    # Continuous movement helper
    # ======================================================

    def move_for_duration(
        self,
        duration,
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw_rate=0.0
    ):

        start = time.time()

        while time.time() - start < duration:

            self.publish_velocity(
                vx,
                vy,
                vz,
                yaw_rate
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            time.sleep(0.1)

    # ======================================================
    # Hover
    # ======================================================

    def hover(self, duration=3.0):

        self.get_logger().info(
            "Hovering..."
        )

        self.move_for_duration(
            duration,
            0.0,
            0.0,
            0.0,
            0.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = DDSFlightManager()

    node.get_logger().info(
        "Waiting for DDS services..."
    )

    node.arm_client.wait_for_service()

    node.mode_client.wait_for_service()

    node.takeoff_client.wait_for_service()

    node.get_logger().info(
        "All DDS services available"
    )

    try:

        # ==============================================
        # GUIDED
        # ==============================================

        node.guided_mode()

        time.sleep(1)

        # ==============================================
        # ARM
        # ==============================================

        node.arm()

        time.sleep(1)

        # ==============================================
        # TAKEOFF
        # ==============================================

        node.takeoff(3.0)

        time.sleep(8)

        # ==============================================
        # MOVE FORWARD
        # ==============================================

        node.get_logger().info(
            "Moving forward..."
        )

        node.move_for_duration(
            duration=5,
            vx=1.0
        )

        # ==============================================
        # HOVER
        # ==============================================

        node.hover(3)

        # ==============================================
        # MOVE BACKWARD
        # ==============================================

        node.get_logger().info(
            "Moving backward..."
        )

        node.move_for_duration(
            duration=5,
            vx=-1.0
        )

        # ==============================================
        # HOVER
        # ==============================================

        node.hover(3)

        # ==============================================
        # LAND
        # ==============================================

        node.land()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()

