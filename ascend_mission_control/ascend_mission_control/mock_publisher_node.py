#!/usr/bin/env python3
"""
ascend_mission_control/mock_publisher_node.py

Test harness that simulates all external inputs to the FSM
so you can test the full state machine WITHOUT Gazebo, SLAM, or real hardware.

What it fakes:
  - SLAM pose (smooth linear motion following the target waypoint)
  - ArduPilot pose (same as SLAM)
  - Battery state (slow drain + recharge when charging signal active)
  - Vision match results (fires after drone lingers near "feature" coords)
  - Charge complete signal
  - Data transfer complete signal

Usage:
  Terminal 1: ros2 run ascend_mission_control fsm_node --ros-args -p sim_mode:=true
  Terminal 2: ros2 run ascend_mission_control mock_publisher_node
  Terminal 3: ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once
  Terminal 4: ros2 run ascend_mission_control mission_monitor_node

Configurable test scenarios via ROS2 parameters:
  scenario:
    'normal'           — full successful mission
    'low_battery_rtl'  — battery drops to 20% mid-survey → failsafe RTL
    'multi_sortie'     — only 1 feature found per sortie, needs 3 sorties
    'no_match'         — no matches found, full survey then RTL
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import json
import time
import random

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState


# ── Fake feature locations in the arena (meters from home) ─────────────────
FAKE_FEATURES = [
    {'seed_id': 0, 'x': 3.0, 'y': 2.0, 'confidence': 0.87},
    {'seed_id': 1, 'x': 7.5, 'y': 5.5, 'confidence': 0.92},
    {'seed_id': 2, 'x': 9.0, 'y': 1.5, 'confidence': 0.78},
]
MATCH_TRIGGER_RADIUS_M = 0.6   # Publish match when drone is within this radius


class MockPublisherNode(Node):

    def __init__(self):
        super().__init__('mock_publisher_node')

        self.declare_parameter('scenario',          'normal')
        self.declare_parameter('drone_speed_m_s',   1.0)
        self.declare_parameter('publish_rate_hz',   20.0)

        self.scenario    = self.get_parameter('scenario').value
        self.speed       = self.get_parameter('drone_speed_m_s').value
        rate             = self.get_parameter('publish_rate_hz').value

        # Simulated drone state
        self._x          = 0.5
        self._y          = 0.5
        self._z          = 0.0
        self._target_x   = 0.5
        self._target_y   = 0.5
        self._target_z   = 0.0
        self._battery    = 1.0      # 0.0 – 1.0
        self._charging   = False
        self._fsm_state  = 'IDLE'
        self._found_ids  = set()
        self._mission_started = False
        self._sortie_features_published = 0

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.create_subscription(PoseStamped, '/ascend/mission_control/target_pose',
                                 self._cb_target_pose, reliable_qos)
        self.create_subscription(String, '/ascend/mission_control/state',
                                 self._cb_fsm_state, reliable_qos)
        self.create_subscription(Bool, '/ascend/mission_control/start_charge',
                                 self._cb_start_charge, reliable_qos)
        self.create_subscription(Bool, '/ascend/mission_control/start_transfer',
                                 self._cb_start_transfer, reliable_qos)

        # Publishers
        self.pub_pose    = self.create_publisher(PoseStamped, '/ascend/localization/pose',    reliable_qos)
        self.pub_ap_pose = self.create_publisher(PoseStamped, '/ap/pose/filtered',             reliable_qos)
        self.pub_battery = self.create_publisher(BatteryState, '/ap/battery_status',           reliable_qos)
        self.pub_match   = self.create_publisher(String,       '/ascend/vision/match_result_str', reliable_qos)
        self.pub_charge_done   = self.create_publisher(Bool, '/ascend/ground_station/charge_done',   reliable_qos)
        self.pub_transfer_done = self.create_publisher(Bool, '/ascend/ground_station/transfer_done', reliable_qos)

        dt = 1.0 / rate
        self.create_timer(dt, self._tick)

        self.get_logger().info(
            f'Mock publisher started — scenario: {self.scenario}  speed: {self.speed} m/s'
        )
        self.get_logger().info(
            'Fake feature locations:'
        )
        for f in FAKE_FEATURES:
            self.get_logger().info(f'  seed_id={f["seed_id"]} at ({f["x"]},{f["y"]})m')

    def _cb_target_pose(self, msg: PoseStamped):
        self._target_x = msg.pose.position.x
        self._target_y = msg.pose.position.y
        self._target_z = msg.pose.position.z

    def _cb_fsm_state(self, msg: String):
        old = self._fsm_state
        self._fsm_state = msg.data
        if old != msg.data:
            self.get_logger().info(f'[MOCK] FSM → {msg.data}')
        if msg.data == 'TAKEOFF':
            self._mission_started = True
        if msg.data in ('IDLE', 'COMPLETE', 'FAILSAFE_LAND'):
            self._charging = False

    def _cb_start_charge(self, msg: Bool):
        if msg.data:
            self._charging = True
            self.get_logger().info('[MOCK] Charging started — will complete in 5s')
            self.create_timer(5.0, self._complete_charge_once)

    def _cb_start_transfer(self, msg: Bool):
        if msg.data:
            self.get_logger().info('[MOCK] Data transfer started — will complete in 2s')
            self.create_timer(2.0, self._complete_transfer_once)

    def _complete_charge_once(self):
        self._charging = False
        self._battery  = min(1.0, self._battery + 0.7)  # Recharge to ~70%+
        msg = Bool()
        msg.data = True
        self.pub_charge_done.publish(msg)
        self.get_logger().info(f'[MOCK] Charging complete — battery at {self._battery*100:.0f}%')

    def _complete_transfer_once(self):
        msg = Bool()
        msg.data = True
        self.pub_transfer_done.publish(msg)
        self.get_logger().info('[MOCK] Data transfer complete.')

    def _tick(self):
        dt = 1.0 / 20.0  # matches publish_rate_hz default

        # ── Move toward target ───────────────
        dx = self._target_x - self._x
        dy = self._target_y - self._y
        dz = self._target_z - self._z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist > 0.05:
            move = min(self.speed * dt, dist)
            self._x += (dx / dist) * move
            self._y += (dy / dist) * move
            self._z += (dz / dist) * move

        # ── Battery drain ────────────────────
        if self._fsm_state in ('SURVEY', 'TAKEOFF', 'RTL', 'MATCH_VERIFY'):
            if self.scenario == 'low_battery_rtl' and self._mission_started:
                self._battery -= 0.0015 * dt * 20   # Fast drain
            else:
                self._battery -= 0.0001 * dt * 20   # Normal drain
            self._battery = max(0.0, self._battery)

        # ── Publish pose ─────────────────────
        now = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.stamp    = now
        ps.header.frame_id = 'map'
        ps.pose.position.x = self._x
        ps.pose.position.y = self._y
        ps.pose.position.z = self._z
        ps.pose.orientation.w = 1.0
        self.pub_pose.publish(ps)
        self.pub_ap_pose.publish(ps)

        # ── Publish battery ──────────────────
        bat = BatteryState()
        bat.header.stamp = now
        bat.percentage   = float(self._battery)
        bat.voltage      = 11.1 * self._battery + 10.0  # Fake voltage
        self.pub_battery.publish(bat)

        # ── Check for feature matches ────────
        if self._fsm_state == 'SURVEY' and self.scenario != 'no_match':
            for feat in FAKE_FEATURES:
                if feat['seed_id'] in self._found_ids:
                    continue
                dist_to = math.sqrt(
                    (self._x - feat['x'])**2 +
                    (self._y - feat['y'])**2
                )
                if dist_to < MATCH_TRIGGER_RADIUS_M:
                    # Only fire one match per visit
                    if self.scenario == 'multi_sortie':
                        # One feature per sortie
                        if self._sortie_features_published >= 1:
                            continue
                    self._found_ids.add(feat['seed_id'])
                    self._sortie_features_published += 1
                    payload = {
                        'seed_id':      feat['seed_id'],
                        'confidence':   feat['confidence'] + random.uniform(-0.05, 0.05),
                        'hd_image_path': f'/tmp/ascend_hd/feature_{feat["seed_id"]}.jpg',
                    }
                    msg = String()
                    msg.data = json.dumps(payload)
                    self.pub_match.publish(msg)
                    self.get_logger().info(
                        f'[MOCK] Match published: seed_id={feat["seed_id"]} '
                        f'at ({self._x:.2f},{self._y:.2f})'
                    )

        # Reset per-sortie counter when a new sortie starts
        if self._fsm_state == 'TAKEOFF':
            self._sortie_features_published = 0


def main(args=None):
    rclpy.init(args=args)
    node = MockPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()