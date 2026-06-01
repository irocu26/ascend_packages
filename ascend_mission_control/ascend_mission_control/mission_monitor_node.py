#!/usr/bin/env python3
"""
ascend_mission_control/mission_monitor_node.py

Subscribes to all ASCEND topics and provides:
  1. Real-time terminal dashboard (printed at 1 Hz)
  2. Mission log file (JSON lines, one entry per state change)
  3. Feature result summary on mission complete

This node is display-only — it never sends commands.
It satisfies the rulebook requirement (Section 10.6.2) to show
ASCEND state on the base station screen during recording.

Run alongside fsm_node. Does not depend on custom msgs.
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

        self.declare_parameter('log_dir', '/tmp/ascend_logs')
        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = os.path.join(log_dir, f'mission_{ts}.jsonl')
        self._result_path = os.path.join(log_dir, f'results_{ts}.json')

        # Internal snapshot
        self._state          = 'IDLE'
        self._pose           = (0.0, 0.0, 0.0)
        self._battery        = 100.0
        self._features       = []
        self._failsafe_msg   = ''
        self._survey_wp      = 0
        self._state_changed  = False
        self._last_state     = ''
        self._mission_start  = None

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

        self.create_subscription(String,       '/ascend/mission_control/state',     self._cb_state,    reliable_qos)
        self.create_subscription(PoseStamped,  '/ascend/localization/pose',          self._cb_pose,     sensor_qos)
        self.create_subscription(BatteryState, '/ap/battery_status',                 self._cb_battery,  sensor_qos)
        self.create_subscription(String,       '/ascend/mission_control/coord_log_str', self._cb_coord, reliable_qos)
        self.create_subscription(String,       '/ascend/mission_control/failsafe_triggered', self._cb_failsafe, reliable_qos)

        # Display timer: 1 Hz
        self.create_timer(1.0, self._display_tick)
        # Log timer: on state change (handled in _cb_state)

        self.get_logger().info(f'Mission monitor started. Log: {self._log_path}')

    def _cb_state(self, msg: String):
        if msg.data != self._state:
            self._state = msg.data
            self._state_changed = True
            if msg.data == 'TAKEOFF' and self._mission_start is None:
                self._mission_start = time.time()
            self._write_log({'event': 'state_change', 'state': self._state,
                             'battery': self._battery, 'pose': list(self._pose)})

    def _cb_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self._pose = (round(p.x, 3), round(p.y, 3), round(p.z, 3))

    def _cb_battery(self, msg: BatteryState):
        if msg.percentage >= 0:
            self._battery = round(msg.percentage * 100.0, 1)

    def _cb_coord(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._features.append(data)
            self._write_log({'event': 'feature_found', 'feature': data})
            self._write_results()
        except Exception:
            pass

    def _cb_failsafe(self, msg: String):
        self._failsafe_msg = msg.data
        self._write_log({'event': 'failsafe', 'reason': msg.data})

    def _display_tick(self):
        elapsed = ''
        if self._mission_start:
            s = int(time.time() - self._mission_start)
            elapsed = f'{s//60:02d}:{s%60:02d}'

        # Clear terminal and print dashboard
        print('\033[2J\033[H', end='')   # Clear screen
        print('=' * 60)
        print('  ASCEND MISSION CONTROL — IRoC-U 2026')
        print('=' * 60)
        print(f'  State    : {STATE_DISPLAY.get(self._state, self._state)}')
        print(f'  Position : X={self._pose[0]:+.3f}m  Y={self._pose[1]:+.3f}m  Z={self._pose[2]:.3f}m')
        print(f'  Battery  : {self._battery:.1f}%'
              + ('  ⚠ LOW' if self._battery < 25 else ''))
        print(f'  Features : {len(self._features)}/3 found')
        if elapsed:
            print(f'  Elapsed  : {elapsed}')
        if self._features:
            print()
            print('  Found features:')
            for f in self._features:
                print(f'    [{f.get("seed_id","?")}] '
                      f'X={f.get("x",0):.3f}m Y={f.get("y",0):.3f}m '
                      f'conf={f.get("confidence",0):.2f}')
        if self._failsafe_msg:
            print()
            print(f'  ⚠ FAILSAFE: {self._failsafe_msg}')
        print('=' * 60)
        print(f'  Log: {self._log_path}')

    def _write_log(self, entry: dict):
        entry['timestamp'] = datetime.now().isoformat()
        with open(self._log_path, 'a') as f:
            f.write(json.dumps(entry) + '\n')

    def _write_results(self):
        results = {
            'timestamp':  datetime.now().isoformat(),
            'features':   self._features,
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