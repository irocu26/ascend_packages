#!/usr/bin/env python3
"""
ascend_mission_control/survey_planner_node.py

Standalone node that:
  1. Publishes the full lawnmower waypoint list as a JSON array (for visualization)
  2. Watches /ascend/mission_control/state and publishes next waypoint
     as the FSM progresses through SURVEY state
  3. Enforces arena boundary — rejects any waypoint outside 35ft × 25ft

This is separate from fsm_node so the survey pattern can be
reconfigured / visualized without touching the FSM.

Topics published:
  /ascend/survey/waypoint_list   (std_msgs/String — JSON array of {x,y,z})
  /ascend/survey/current_wp      (geometry_msgs/PoseStamped)
  /ascend/survey/progress        (std_msgs/String — "N/M")

Topics subscribed:
  /ascend/mission_control/state  (std_msgs/String)
  /ascend/localization/pose      (geometry_msgs/PoseStamped)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import math

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class SurveyPlannerNode(Node):

    def __init__(self):
        super().__init__('survey_planner_node')

        self.declare_parameter('survey_altitude', 3.0)
        self.declare_parameter('strip_overlap',   0.3)
        self.declare_parameter('arena_x_m',       10.67)
        self.declare_parameter('arena_y_m',       7.62)
        self.declare_parameter('wp_radius',       0.4)
        self.declare_parameter('camera_hfov_deg', 90.0)

        alt          = self.get_parameter('survey_altitude').value
        overlap      = self.get_parameter('strip_overlap').value
        self.arena_x = self.get_parameter('arena_x_m').value
        self.arena_y = self.get_parameter('arena_y_m').value
        self.wp_r    = self.get_parameter('wp_radius').value
        hfov         = self.get_parameter('camera_hfov_deg').value

        # Build waypoints
        strip_w = 2.0 * alt * math.tan(math.radians(hfov / 2.0))
        step    = strip_w * (1.0 - overlap)
        self._waypoints = self._build_lawnmower(alt, step)
        self._wp_idx    = 0
        self._active    = False
        self._pose      = None

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

        self.sub_state = self.create_subscription(
            String, '/ascend/mission_control/state', self._cb_state, reliable_qos
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, '/ascend/localization/pose', self._cb_pose, sensor_qos
        )

        self.pub_list    = self.create_publisher(String,       '/ascend/survey/waypoint_list', reliable_qos)
        self.pub_wp      = self.create_publisher(PoseStamped,  '/ascend/survey/current_wp',    reliable_qos)
        self.pub_progress = self.create_publisher(String,      '/ascend/survey/progress',      reliable_qos)

        # Publish waypoint list once on startup
        self.create_timer(2.0, self._publish_waypoint_list_once)
        # Update timer
        self.create_timer(0.5, self._tick)

        total = len(self._waypoints)
        self.get_logger().info(
            f'Survey planner ready: {total} waypoints, '
            f'altitude={alt}m, strip_step={step:.2f}m'
        )

    def _build_lawnmower(self, altitude: float, step: float):
        """Build boustrophedon sweep. Returns list of (x, y, z)."""
        wps = []
        margin = 0.5
        y = margin
        row = 0
        while y <= self.arena_y - margin:
            if row % 2 == 0:
                wps.append((margin,              y, altitude))
                wps.append((self.arena_x - margin, y, altitude))
            else:
                wps.append((self.arena_x - margin, y, altitude))
                wps.append((margin,              y, altitude))
            y += step
            row += 1
        # Final: return to home corner
        wps.append((margin, margin, altitude))
        return wps

    def _cb_state(self, msg: String):
        state = msg.data
        if state == 'SURVEY' and not self._active:
            self._active = True
            self._wp_idx = 0
            self.get_logger().info('Survey started — publishing waypoints.')
        elif state not in ('SURVEY', 'MATCH_VERIFY'):
            self._active = False

    def _cb_pose(self, msg: PoseStamped):
        self._pose = msg

    def _tick(self):
        if not self._active or not self._waypoints:
            return
        if self._wp_idx >= len(self._waypoints):
            return

        x, y, z = self._waypoints[self._wp_idx]

        # Check if close enough to advance
        if self._pose:
            cx = self._pose.pose.position.x
            cy = self._pose.pose.position.y
            cz = self._pose.pose.position.z
            dist = math.sqrt((x-cx)**2 + (y-cy)**2 + (z-cz)**2)
            if dist < self.wp_r and self._wp_idx < len(self._waypoints) - 1:
                self._wp_idx += 1
                x, y, z = self._waypoints[self._wp_idx]

        # Enforce arena boundary
        x = max(0.3, min(self.arena_x - 0.3, x))
        y = max(0.3, min(self.arena_y - 0.3, y))

        # Publish current waypoint
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        self.pub_wp.publish(ps)

        # Publish progress
        prog = String()
        prog.data = f'{self._wp_idx + 1}/{len(self._waypoints)}'
        self.pub_progress.publish(prog)

    def _publish_waypoint_list_once(self):
        """Publish full waypoint list as JSON (for RViz / debug)."""
        data = [{'x': x, 'y': y, 'z': z} for x, y, z in self._waypoints]
        msg = String()
        msg.data = json.dumps(data)
        self.pub_list.publish(msg)
        # Only do this once
        self.destroy_timer(self.get_timers()[-1])


def main(args=None):
    rclpy.init(args=args)
    node = SurveyPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()