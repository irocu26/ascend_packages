#!/usr/bin/env python3
"""
ascend_mission_control/mission_monitor_node.py

Display-only terminal dashboard + JSON mission logger.
Satisfies IRoC-U rulebook §10.6.2 — base station screen display.
This node never sends commands.

WHAT CHANGED:
  1. Hardcoded '3' in 'Features: X/3' replaced with total_features parameter
     read from mission_params.yaml — consistent with fsm_node.
  2. Added subscription to /ascend/survey/progress — shows survey waypoint
     count (e.g. "12/20") in dashboard.
  3. Added subscription to /ap/pose/filtered as fallback pose when
     /ascend/localization/pose (SLAM) is not running. Uses whichever
     arrives — SLAM takes priority if both are live.
  4. _cb_battery: added guard for msg.percentage == -1.0 (ArduPilot sends
     -1 when battery percentage is unknown — was causing 101600% display).
  5. _display_tick: added survey progress line to dashboard output.

Topics subscribed:
  /ascend/mission_control/state           (std_msgs/String)
  /ascend/localization/pose               (geometry_msgs/PoseStamped) — SLAM
  /ap/pose/filtered                       (geometry_msgs/PoseStamped) — AP fallback
  /ap/battery_status                      (sensor_msgs/BatteryState)
  /ascend/mission_control/coord_log_str   (std_msgs/String JSON)
  /ascend/mission_control/failsafe_triggered (std_msgs/String)
  /ascend/survey/progress                 (std_msgs/String "N/M") ← NEW
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import os
import time
from datetime import datetime

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState


STATE_DISPLAY = {
    'IDLE':          '⏸  IDLE          — waiting for start command',
    'ARMING':        '🔧  ARMING        — sending arm command',
    'TAKEOFF':       '🚀  TAKEOFF       — climbing to survey altitude',
    'SURVEY':        '🔍  SURVEY        — scanning arena',
    'MATCH_VERIFY':  '🎯  MATCH_VERIFY  — hovering over candidate feature',
    'RTL':           '🏠  RTL           — returning to base station',
    'LANDING':       '🛬  LANDING       — descending to dock',
    'DOCKING':       '🔌  DOCKING       — aligning with dock pad',
    'CHARGING':      '⚡  CHARGING      — recharging battery',
    'TRANSFER':      '📡  TRANSFER      — sending data to base station',
    'COMPLETE':      '✅  COMPLETE      — mission finished',
    'FAILSAFE_RTL':  '🚨  FAILSAFE RTL  — emergency return',
    'FAILSAFE_LAND': '🚨  FAILSAFE LAND — emergency landing',
}


class MissionMonitorNode(Node):

    def __init__(self):
        super().__init__('mission_monitor_node')

        self.declare_parameter('log_dir',        '/tmp/ascend_logs')
        # FIX 1: read total_features from params instead of hardcoding 3
        self.declare_parameter('total_features', 3)

        log_dir         = self.get_parameter('log_dir').value
        self._n_features = self.get_parameter('total_features').value

        os.makedirs(log_dir, exist_ok=True)
        ts                = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path    = os.path.join(log_dir, f'mission_{ts}.jsonl')
        self._result_path = os.path.join(log_dir, f'results_{ts}.json')

        # Internal state
        self._state         = 'IDLE'
        self._pose          = (0.0, 0.0, 0.0)
        self._pose_source   = 'none'       # FIX 3: track which source
        self._battery       = 100.0
        self._features      = []
        self._failsafe_msg  = ''
        self._survey_progress = '—'        # FIX 2: survey waypoint counter
        self._mission_start = None

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

        # Core subscriptions
        self.create_subscription(
            String,       '/ascend/mission_control/state',
            self._cb_state,    reliable_qos
        )
        self.create_subscription(
            BatteryState, '/ap/battery_status',
            self._cb_battery,  sensor_qos
        )
        self.create_subscription(
            String,       '/ascend/mission_control/coord_log_str',
            self._cb_coord,    reliable_qos
        )
        self.create_subscription(
            String,       '/ascend/mission_control/failsafe_triggered',
            self._cb_failsafe, reliable_qos
        )

        # FIX 2: survey progress
        self.create_subscription(
            String,       '/ascend/survey/progress',
            self._cb_survey_progress, reliable_qos
        )

        # FIX 3: SLAM pose (primary) and AP pose (fallback)
        self.create_subscription(
            PoseStamped,  '/ascend/localization/pose',
            self._cb_slam_pose, sensor_qos
        )
        self.create_subscription(
            PoseStamped,  '/ap/pose/filtered',
            self._cb_ap_pose,   sensor_qos
        )

        # Display at 1 Hz
        self.create_timer(1.0, self._display_tick)

        self.get_logger().info(f'Mission monitor started. Log: {self._log_path}')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cb_state(self, msg: String):
        if msg.data != self._state:
            self._state = msg.data
            if msg.data == 'TAKEOFF' and self._mission_start is None:
                self._mission_start = time.time()
            self._write_log({
                'event':   'state_change',
                'state':   self._state,
                'battery': self._battery,
                'pose':    list(self._pose)
            })

    def _cb_slam_pose(self, msg: PoseStamped):
        """SLAM pose — highest priority position source."""
        p               = msg.pose.position
        self._pose      = (round(p.x, 3), round(p.y, 3), round(p.z, 3))
        self._pose_source = 'SLAM'

    def _cb_ap_pose(self, msg: PoseStamped):
        """AP EKF pose — fallback when SLAM is not running."""
        # FIX 3: only use AP pose if SLAM hasn't provided one recently
        if self._pose_source != 'SLAM':
            p               = msg.pose.position
            self._pose      = (round(p.x, 3), round(p.y, 3), round(p.z, 3))
            self._pose_source = 'AP'

    def _cb_battery(self, msg: BatteryState):
        # FIX 4: ArduPilot sends -1.0 when percentage is unknown
        if msg.percentage >= 0.0:
            self._battery = round(msg.percentage * 100.0, 1)

    def _cb_coord(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._features.append(data)
            self._write_log({'event': 'feature_found', 'feature': data})
            self._write_results()
        except Exception as e:
            self.get_logger().warn(f'coord_log parse error: {e}')

    def _cb_failsafe(self, msg: String):
        self._failsafe_msg = msg.data
        self._write_log({'event': 'failsafe', 'reason': msg.data})

    def _cb_survey_progress(self, msg: String):
        """FIX 2: update survey waypoint counter string e.g. '12/20'."""
        self._survey_progress = msg.data

    # ── Display ────────────────────────────────────────────────────────────

    def _display_tick(self):
        elapsed = ''
        if self._mission_start:
            s       = int(time.time() - self._mission_start)
            elapsed = f'{s//60:02d}:{s%60:02d}'

        print('\033[2J\033[H', end='')
        print('=' * 62)
        print('  ASCEND MISSION CONTROL — IRoC-U 2026')
        print('=' * 62)
        print(f'  State    : {STATE_DISPLAY.get(self._state, self._state)}')
        print(f'  Position : X={self._pose[0]:+.3f}m  Y={self._pose[1]:+.3f}m'
              f'  Z={self._pose[2]:.3f}m  [{self._pose_source}]')  # FIX 3: show source
        print(f'  Battery  : {self._battery:.1f}%'
              + ('  ⚠ LOW' if self._battery < 25 else ''))
        # FIX 1: use self._n_features instead of hardcoded 3
        print(f'  Features : {len(self._features)}/{self._n_features} found')
        # FIX 2: survey waypoint progress
        if self._state in ('SURVEY', 'MATCH_VERIFY'):
            print(f'  Survey WP: {self._survey_progress}')
        if elapsed:
            print(f'  Elapsed  : {elapsed}')
        if self._features:
            print()
            print('  Found features:')
            for f in self._features:
                print(f'    [{f.get("seed_id","?")}] '
                      f'X={f.get("x", 0):.3f}m Y={f.get("y", 0):.3f}m '
                      f'conf={f.get("confidence", 0):.2f}')
        if self._failsafe_msg:
            print()
            print(f'  ⚠ FAILSAFE: {self._failsafe_msg}')
        print('=' * 62)
        print(f'  Log: {self._log_path}')

    # ── Logging ────────────────────────────────────────────────────────────

    def _write_log(self, entry: dict):
        entry['timestamp'] = datetime.now().isoformat()
        with open(self._log_path, 'a') as f:
            f.write(json.dumps(entry) + '\n')

    def _write_results(self):
        results = {
            'timestamp':   datetime.now().isoformat(),
            'features':    self._features,
            'total_found': len(self._features),
        }
        with open(self._result_path, 'w') as f:
            json.dump(results, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()