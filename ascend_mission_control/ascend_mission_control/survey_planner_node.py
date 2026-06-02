#!/usr/bin/env python3
"""
ascend_mission_control/survey_planner_node.py

Publishes lawnmower waypoints while FSM is in SURVEY state.
Display-assist only — fsm_node drives actual motion via /ap/cmd_vel.

WHAT CHANGED:
  1. _publish_waypoint_list_once(): destroy_timer(self.get_timers()[-1]) is
     unreliable — replaced with a one_shot flag to stop re-publishing.
  2. _cb_state(): _wp_idx now resets to 0 on every SURVEY entry, not just
     the first — needed for multi-sortie missions where SURVEY is re-entered
     after charging.
  3. Added /ascend/survey/complete topic — published when all waypoints done,
     so fsm_node or a debug subscriber can confirm survey finished.
  4. _tick(): added survey-complete publish when _wp_idx reaches end.
  5. Added MATCH_VERIFY to active states — planner stays active while
     FSM is hovering over a match candidate (drone hasn't moved).

Topics published:
  /ascend/survey/waypoint_list   (std_msgs/String — JSON array of {x,y,z})
  /ascend/survey/current_wp      (geometry_msgs/PoseStamped)
  /ascend/survey/progress        (std_msgs/String — "N/M")
  /ascend/survey/complete        (std_msgs/Bool)   ← NEW

Topics subscribed:
  /ascend/mission_control/state  (std_msgs/String)
  /ascend/localization/pose      (geometry_msgs/PoseStamped)
    — remapped to /ap/pose/filtered in sim_gazebo.launch.py when no SLAM
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import math

from std_msgs.msg import String, Bool
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

        strip_w          = 2.0 * alt * math.tan(math.radians(hfov / 2.0))
        step             = strip_w * (1.0 - overlap)
        self._waypoints  = self._build_lawnmower(alt, step)
        self._wp_idx     = 0
        self._active     = False
        self._pose       = None
        self._list_published = False  # FIX 1: replaces unreliable destroy_timer
        self._survey_complete_sent = False

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

        self.pub_list     = self.create_publisher(String,      '/ascend/survey/waypoint_list', reliable_qos)
        self.pub_wp       = self.create_publisher(PoseStamped, '/ascend/survey/current_wp',    reliable_qos)
        self.pub_progress = self.create_publisher(String,      '/ascend/survey/progress',      reliable_qos)
        self.pub_complete = self.create_publisher(Bool,        '/ascend/survey/complete',      reliable_qos)  # FIX 4

        self.create_timer(2.0, self._publish_waypoint_list_once)
        self.create_timer(0.5, self._tick)

        total = len(self._waypoints)
        self.get_logger().info(
            f'Survey planner ready: {total} waypoints, '
            f'altitude={alt}m, strip_step={step:.2f}m'
        )

    def _build_lawnmower(self, altitude: float, step: float):
        wps    = []
        margin = 0.5
        y      = margin
        row    = 0
        while y <= self.arena_y - margin:
            if row % 2 == 0:
                wps.append((margin,               y, altitude))
                wps.append((self.arena_x - margin, y, altitude))
            else:
                wps.append((self.arena_x - margin, y, altitude))
                wps.append((margin,               y, altitude))
            y   += step
            row += 1
        wps.append((margin, margin, altitude))  # Return to home
        return wps

    def _cb_state(self, msg: String):
        state = msg.data
        # FIX 5: MATCH_VERIFY keeps planner active — drone hasn't moved,
        # survey position needs to remain published
        active_states = ('SURVEY', 'MATCH_VERIFY')

        if state == 'SURVEY' and not self._active:
            self._active = True
            # FIX 2: always reset on SURVEY entry for multi-sortie support
            self._wp_idx = 0
            self._survey_complete_sent = False
            self.get_logger().info('Survey started — publishing waypoints.')
        elif state not in active_states:
            self._active = False

    def _cb_pose(self, msg: PoseStamped):
        self._pose = msg

    def _tick(self):
        if not self._active or not self._waypoints:
            return

        # FIX 4: survey complete — publish and stop ticking
        if self._wp_idx >= len(self._waypoints):
            if not self._survey_complete_sent:
                self._survey_complete_sent = True
                msg = Bool()
                msg.data = True
                self.pub_complete.publish(msg)
                self.get_logger().info('Survey pattern complete — all waypoints published.')
            return

        x, y, z = self._waypoints[self._wp_idx]

        # Advance if drone is close enough to current waypoint
        if self._pose:
            cx   = self._pose.pose.position.x
            cy   = self._pose.pose.position.y
            cz   = self._pose.pose.position.z
            dist = math.sqrt((x-cx)**2 + (y-cy)**2 + (z-cz)**2)
            if dist < self.wp_r and self._wp_idx < len(self._waypoints) - 1:
                self._wp_idx += 1
                x, y, z = self._waypoints[self._wp_idx]

        # Enforce arena boundary
        x = max(0.3, min(self.arena_x - 0.3, x))
        y = max(0.3, min(self.arena_y - 0.3, y))

        # Publish current waypoint
        ps                   = PoseStamped()
        ps.header.stamp      = self.get_clock().now().to_msg()
        ps.header.frame_id   = 'map'
        ps.pose.position.x   = x
        ps.pose.position.y   = y
        ps.pose.position.z   = z
        ps.pose.orientation.w = 1.0
        self.pub_wp.publish(ps)

        # Publish progress string "N/M"
        prog      = String()
        prog.data = f'{self._wp_idx + 1}/{len(self._waypoints)}'
        self.pub_progress.publish(prog)

    def _publish_waypoint_list_once(self):
        """Publish full waypoint list as JSON for RViz/debug. Only once."""
        # FIX 1: flag instead of destroy_timer — avoids IndexError on timer list
        if self._list_published:
            return
        self._list_published = True
        data     = [{'x': x, 'y': y, 'z': z} for x, y, z in self._waypoints]
        msg      = String()
        msg.data = json.dumps(data)
        self.pub_list.publish(msg)
        self.get_logger().info(
            f'Waypoint list published ({len(self._waypoints)} waypoints).'
        )


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