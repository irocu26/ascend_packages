#!/usr/bin/env python3
"""
dashboard_node.py — ASCEND IRoC-U 2026 live result dashboard + report server.

A single process that runs BOTH:
  • an rclpy node subscribing to every mission-relevant ASCEND topic, and
  • a Flask web server that streams the live mission state to a browser
    dashboard (Server-Sent Events) and generates the final result report
    (JSON + HTML + PDF) on demand and automatically at mission COMPLETE.

This is the base-station console captured in the elimination-round video:
it shows the 7 rulebook tasks, live telemetry, an arena map with the drone and
the determined feature coordinates, and a one-click "Download Report" button.

────────────────────────────────────────────────────────────────────────────
RUN (after sourcing your ROS 2 workspace):

    source ~/ardu_ws/install/setup.bash
    python3 dashboard_node.py

Then open  http://<base-station-ip>:8080  in a browser.

Optional ROS parameters (override with --ros-args -p name:=value):
    http_host        (default 0.0.0.0)
    http_port        (default 8080)
    team_name        (default ASCEND)
    report_dir       (default ~/ascend_results)
    arena_x_m        (default 10.67)
    arena_y_m        (default 7.62)
    total_features   (default 3)
    dock_x / dock_y  (default 0.0 / 0.0)
    auto_report      (default True — write report files when COMPLETE reached)
    pose_source_label (default "Optical Flow" — set to "Manual" if flying manual)
    direct_vision_match (default False — feed SIFT matches straight into the
                         report, bypassing the FSM; for manual testing only)

Position source: the displayed pose comes from the ArduPilot EKF
(/ap/pose/filtered), which fuses the optical-flow sensor; /ascend/localization/pose
is used only as a fallback. ORB-SLAM3 is NOT relied upon.

This node is DISPLAY-ONLY: it never publishes a command. Safe to start/stop at
any time without affecting the flight.
────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time

# Make sibling modules importable regardless of launch cwd.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState

from flask import Flask, Response, jsonify, request, send_from_directory

from mission_tracker import MissionTracker
import report_generator

PUBLIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'public')


# ════════════════════════════════════════════════════════════════════════════
#  ROS node
# ════════════════════════════════════════════════════════════════════════════
class ResultDashboardNode(Node):

    def __init__(self, tracker: MissionTracker, *, direct_vision_match: bool = False):
        super().__init__('result_dashboard_node')
        self.tracker = tracker
        self._last_state = None

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE, depth=10)
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE, depth=5)

        # FSM / mission-control topics
        self.create_subscription(String, '/ascend/mission_control/state',
                                 self._cb_state, reliable)
        self.create_subscription(String, '/ascend/mission_control/coord_log_str',
                                 self._cb_coord, reliable)
        self.create_subscription(String, '/ascend/mission_control/failsafe_triggered',
                                 self._cb_failsafe, reliable)

        # Survey planner
        self.create_subscription(String, '/ascend/survey/progress',
                                 self._cb_survey_progress, reliable)
        self.create_subscription(Bool, '/ascend/survey/complete',
                                 self._cb_survey_complete, reliable)

        # Ground-station / landing acks
        self.create_subscription(Bool, '/ascend/ground_station/charge_done',
                                 self._cb_charge_done, reliable)
        self.create_subscription(Bool, '/ascend/ground_station/transfer_done',
                                 self._cb_transfer_done, reliable)
        self.create_subscription(Bool, '/ascend/precision_landing/complete',
                                 self._cb_landing_complete, reliable)

        # Telemetry — optical-flow / manual localization + battery.
        # Primary position = ArduPilot EKF pose (fuses the optical-flow sensor).
        # /ascend/localization/pose is only a fallback. We do NOT rely on
        # ORB-SLAM3 for the displayed position.
        self.create_subscription(PoseStamped, '/ap/pose/filtered',
                                 self._cb_ap_pose, sensor)
        self.create_subscription(PoseStamped, '/ascend/localization/pose',
                                 self._cb_localization_pose, sensor)
        self.create_subscription(BatteryState, '/ap/battery_status',
                                 self._cb_battery, sensor)

        # Optional MANUAL-test path: ingest SIFT matches straight from the vision
        # node, bypassing the FSM. In a real mission the FSM republishes verified
        # feature coords on coord_log_str (subscribed above) and is the single
        # source of truth; but in manual joystick testing the FSM isn't surveying
        # and drops every match, so the report would stay empty. Off by default.
        if direct_vision_match:
            self.create_subscription(String, '/ascend/vision/match_result_str',
                                     self._cb_vision_match, reliable)
            self.get_logger().warn(
                'direct_vision_match ON — feeding /ascend/vision/match_result_str '
                'into the report directly (FSM bypassed). Manual testing only.')

        self.get_logger().info('Result dashboard node subscribed to mission topics.')

    # ── callbacks ────────────────────────────────────────────────────────────
    def _cb_state(self, msg: String):
        self.tracker.on_state(msg.data)
        # Trigger the auto-report exactly once when COMPLETE first arrives.
        if msg.data == 'COMPLETE' and self._last_state != 'COMPLETE':
            self._maybe_auto_report()
        self._last_state = msg.data

    def _cb_coord(self, msg: String):
        try:
            self.tracker.on_feature(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warn(f'coord_log parse error: {e}')

    def _cb_vision_match(self, msg: String):
        """Manual-test path: a raw SIFT match becomes a report feature directly.

        Payload mirrors the FSM's coord_log JSON (seed_id, confidence, hd_path,
        plus x/y when the vision node has a pose-derived coordinate). on_feature
        de-dups by seed_id, so this never double-counts a feature the FSM also
        reports.
        """
        try:
            self.tracker.on_feature(json.loads(msg.data))
        except Exception as e:
            self.get_logger().warn(f'vision match parse error: {e}')

    def _cb_failsafe(self, msg: String):
        self.tracker.on_failsafe(msg.data)

    def _cb_survey_progress(self, msg: String):
        try:
            done, total = msg.data.split('/')
            self.tracker.on_survey_progress(int(done), int(total))
        except Exception:
            pass

    def _cb_survey_complete(self, msg: Bool):
        if msg.data:
            self.tracker.on_survey_complete()

    def _cb_charge_done(self, msg: Bool):
        if msg.data:
            self.tracker.on_charge_done()

    def _cb_transfer_done(self, msg: Bool):
        if msg.data:
            self.tracker.on_transfer_done()

    def _cb_landing_complete(self, msg: Bool):
        if msg.data:
            self.tracker.on_landing_complete()

    def _cb_ap_pose(self, msg: PoseStamped):
        # Primary position: ArduPilot EKF pose (fuses the optical-flow sensor).
        p = msg.pose.position
        self.tracker.on_pose(p.x, p.y, p.z, self._pose_label, primary=True)

    def _cb_localization_pose(self, msg: PoseStamped):
        # Optional external localization feed — used only as a fallback.
        p = msg.pose.position
        self.tracker.on_pose(p.x, p.y, p.z, self._pose_label, primary=False)

    def _cb_battery(self, msg: BatteryState):
        # ArduPilot sends -1.0 when percentage is unknown (mission_monitor FIX 4).
        if msg.percentage >= 0.0:
            self.tracker.on_battery(msg.percentage * 100.0)

    # ── auto report ───────────────────────────────────────────────────────────
    def _maybe_auto_report(self):
        if not self._auto_report:
            return
        try:
            paths = report_generator.generate(self.tracker.snapshot(), self._report_dir)
            self.tracker.mark_report_generated(paths)
            self.get_logger().info(f'Auto-generated mission report: {paths}')
        except Exception as e:
            self.get_logger().error(f'Auto-report failed: {e}')

    # set by main() after param read
    _auto_report = True
    _report_dir = os.path.expanduser('~/ascend_results')
    _pose_label = 'Optical Flow'   # how the position source is shown on screen


# ════════════════════════════════════════════════════════════════════════════
#  Flask app
# ════════════════════════════════════════════════════════════════════════════
def build_flask(tracker: MissionTracker, node: ResultDashboardNode,
                report_dir: str) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.route('/')
    def index():
        return send_from_directory(PUBLIC_DIR, 'index.html')

    @app.route('/static/<path:fname>')
    def static_files(fname):
        return send_from_directory(PUBLIC_DIR, fname)

    @app.route('/api/state')
    def api_state():
        return jsonify(tracker.snapshot())

    @app.route('/api/stream')
    def api_stream():
        def gen():
            while True:
                payload = json.dumps(tracker.snapshot())
                yield f'data: {payload}\n\n'
                time.sleep(0.5)
        return Response(gen(), mimetype='text/event-stream',
                        headers={'Cache-Control': 'no-cache',
                                 'X-Accel-Buffering': 'no'})

    @app.route('/api/report', methods=['POST', 'GET'])
    def api_report():
        """Generate (or regenerate) the report files, return their paths."""
        paths = report_generator.generate(tracker.snapshot(), report_dir)
        tracker.mark_report_generated(paths)
        return jsonify({'ok': True, 'paths': paths,
                        'download': {fmt: f'/api/report/file/{fmt}'
                                     for fmt in paths}})

    @app.route('/api/report/file/<fmt>')
    def api_report_file(fmt):
        paths = tracker.snapshot().get('report_paths', {})
        if fmt not in paths or not paths[fmt] or not os.path.exists(paths[fmt]):
            # Generate on the fly if not present yet.
            paths = report_generator.generate(tracker.snapshot(), report_dir)
            tracker.mark_report_generated(paths)
        path = paths.get(fmt)
        if not path or not os.path.exists(path):
            return jsonify({'ok': False, 'error': f'{fmt} not available'}), 404
        directory, fname = os.path.split(path)
        as_attach = fmt in ('json', 'pdf')
        return send_from_directory(directory, fname, as_attachment=as_attach)

    return app


# ════════════════════════════════════════════════════════════════════════════
#  main
# ════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    # Read parameters via a throwaway node so we can size the tracker first.
    pre = Node('result_dashboard_params')
    p = lambda name, default: pre.declare_parameter(name, default).value
    http_host = p('http_host', '0.0.0.0')
    http_port = int(p('http_port', 8080))
    team_name = p('team_name', 'ASCEND')
    report_dir = os.path.expanduser(p('report_dir', '~/ascend_results'))
    arena_x = float(p('arena_x_m', 10.67))
    arena_y = float(p('arena_y_m', 7.62))
    total_features = int(p('total_features', 3))
    dock_x = float(p('dock_x', 0.0))
    dock_y = float(p('dock_y', 0.0))
    auto_report = bool(p('auto_report', True))
    pose_source_label = p('pose_source_label', 'Optical Flow')
    direct_vision_match = bool(p('direct_vision_match', False))
    pre.destroy_node()

    os.makedirs(report_dir, exist_ok=True)

    tracker = MissionTracker(team_name=team_name, arena_x=arena_x, arena_y=arena_y,
                             dock_x=dock_x, dock_y=dock_y, total_features=total_features)

    node = ResultDashboardNode(tracker, direct_vision_match=direct_vision_match)
    node._auto_report = auto_report
    node._report_dir = report_dir
    node._pose_label = pose_source_label

    app = build_flask(tracker, node, report_dir)

    # Flask in a daemon thread; rclpy.spin owns the main thread.
    def serve():
        app.run(host=http_host, port=http_port, threaded=True,
                debug=False, use_reloader=False)

    flask_thread = threading.Thread(target=serve, daemon=True)
    flask_thread.start()

    node.get_logger().info('━' * 56)
    node.get_logger().info('ASCEND RESULT DASHBOARD — IRoC-U 2026')
    node.get_logger().info(f'  Open:        http://{_lan_ip()}:{http_port}')
    node.get_logger().info(f'  Report dir:  {report_dir}')
    node.get_logger().info(f'  Auto-report: {auto_report}')
    node.get_logger().info('━' * 56)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Dashboard shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _lan_ip() -> str:
    """Best-effort LAN IP for the log banner (no external connection made)."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip


if __name__ == '__main__':
    main()
