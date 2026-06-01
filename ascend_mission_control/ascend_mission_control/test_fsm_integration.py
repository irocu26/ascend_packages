#!/usr/bin/env python3
"""
test/test_fsm_integration.py

ROS2 integration test — requires a live ROS2 environment (no hardware/sim needed).
Spins up the FSM node and mock publisher together, injects a start command,
and verifies the state machine progresses through expected states.

Run with:
  cd ~/ardu_ws
  source install/setup.bash
  python3 src/ascend_packages/ascend_mission_control/test/test_fsm_integration.py

Or with pytest (with ros2 test runner):
  colcon test --packages-select ascend_mission_control
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import threading
import sys

from std_msgs.msg import String, Empty, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState

# Import FSM and mock nodes
sys.path.insert(0, '..')
from ascend_mission_control.fsm_node import AscendFSMNode, State
from ascend_mission_control.mock_publisher_node import MockPublisherNode


TIMEOUT_S = 60.0   # Max test duration


class TestObserverNode(Node):
    """Observes FSM state transitions during integration test."""

    def __init__(self):
        super().__init__('test_observer')
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.visited_states = []
        self.current_state  = 'IDLE'
        self.coord_logs     = []
        self.failsafes      = []

        self.sub_state = self.create_subscription(
            String, '/ascend/mission_control/state', self._cb_state, reliable_qos
        )
        self.sub_coords = self.create_subscription(
            String, '/ascend/mission_control/coord_log_str', self._cb_coords, reliable_qos
        )
        self.sub_failsafe = self.create_subscription(
            String, '/ascend/mission_control/failsafe_triggered', self._cb_failsafe, reliable_qos
        )
        self.pub_start = self.create_publisher(Empty, '/ascend/mission_control/start_cmd', reliable_qos)

    def _cb_state(self, msg):
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.visited_states.append(msg.data)
            print(f'[TEST] State → {msg.data}')

    def _cb_coords(self, msg):
        import json
        try:
            self.coord_logs.append(json.loads(msg.data))
        except Exception:
            pass

    def _cb_failsafe(self, msg):
        self.failsafes.append(msg.data)
        print(f'[TEST] FAILSAFE: {msg.data}')

    def send_start(self):
        self.pub_start.publish(Empty())
        print('[TEST] Start command sent.')


def run_integration_test(scenario='normal'):
    rclpy.init()

    fsm_node   = AscendFSMNode()
    mock_node  = MockPublisherNode()
    observer   = TestObserverNode()

    # Override mock scenario
    mock_node._scenario = scenario

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fsm_node)
    executor.add_node(mock_node)
    executor.add_node(observer)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait for nodes to initialize
    time.sleep(2.0)

    # Send start command
    observer.send_start()

    # Wait for mission to complete or timeout
    start = time.time()
    while time.time() - start < TIMEOUT_S:
        if observer.current_state in ('COMPLETE', 'FAILSAFE_LAND', 'FAILSAFE_RTL'):
            break
        time.sleep(0.5)

    # ── Assertions ────────────────────────────────────────────────────────
    print(f'\n[TEST] Visited states: {observer.visited_states}')
    print(f'[TEST] Final state: {observer.current_state}')
    print(f'[TEST] Coord logs: {observer.coord_logs}')
    print(f'[TEST] Failsafes: {observer.failsafes}')

    if scenario == 'normal':
        assert 'TAKEOFF' in observer.visited_states,  "Never took off"
        assert 'SURVEY'  in observer.visited_states,  "Never surveyed"
        assert 'COMPLETE' == observer.current_state,  "Did not reach COMPLETE"
        assert len(observer.coord_logs) == 3,          "Did not find all 3 features"
        assert len(observer.failsafes) == 0,           "Unexpected failsafe in normal scenario"
        print('[TEST] ✅ Normal scenario PASSED')

    elif scenario == 'low_battery_rtl':
        assert 'FAILSAFE_RTL' in observer.visited_states, "Failsafe RTL not triggered"
        assert any('LOW_BATTERY' in f or 'BATTERY' in f for f in observer.failsafes), \
            "Failsafe should be battery-related"
        print('[TEST] ✅ Low battery failsafe scenario PASSED')

    elif scenario == 'no_match':
        assert 'SURVEY' in observer.visited_states,  "Never surveyed"
        assert 'RTL'    in observer.visited_states,  "Should have RTL'd after survey"
        assert len(observer.coord_logs) == 0,         "Should have 0 features in no_match"
        print('[TEST] ✅ No match scenario PASSED')

    # Cleanup
    executor.shutdown()
    fsm_node.destroy_node()
    mock_node.destroy_node()
    observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    scenario = sys.argv[1] if len(sys.argv) > 1 else 'normal'
    print(f'Running integration test — scenario: {scenario}')
    run_integration_test(scenario)