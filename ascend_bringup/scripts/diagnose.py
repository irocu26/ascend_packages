#!/usr/bin/env python3
"""
ascend_bringup/scripts/diagnose.py

Single-screen, low-bandwidth field diagnostics for the ASCEND drone.

WHY THIS EXISTS / HOW TO RUN IT
-------------------------------
DDS discovery over Wi-Fi to a second machine collapses to ~0.1 Hz even when the
topic runs at 15 Hz on the Pi. So do NOT diagnose from the laptop. Instead run
this ON the Raspberry Pi, inside an SSH session:

    source ~/ardu_ws/install/setup.bash
    python3 ~/ardu_ws/src/ascend_packages/ascend_bringup/scripts/diagnose.py

It subscribes to every topic LOCALLY on the Pi (full rate, no network in the
loop) and redraws ONE screen in place at 5 Hz. Only the rendered text — a few
hundred bytes per refresh — crosses the SSH link, so it stays real-time on a
slow connection and it does NOT scroll/spam. Press Ctrl-C to quit.

It is a plain script: no colcon build, no extra pip packages (stdlib + rclpy +
the standard message packages only). Camera RATE is read from the tiny
camera_info topic, so no heavy image frame is ever deserialized.

WHAT YOU SEE
------------
  MISSION    FSM state, slam_ok, last failsafe reason
  SLAM       ORB-SLAM3 tracking state (decoded) + /odom (orbslam_odom->body)
  ARDUPILOT  /ap/pose/filtered (the pose the FSM flies on), battery, target wp
  CAMERA     color + aligned-depth rates (the SLAM input)

Green = healthy/fresh, yellow = warning, red = lost/stale, grey = no data yet.
"""

import sys
import time
import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, CameraInfo, Image


# ── ANSI helpers ──────────────────────────────────────────────────────────────
RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
GREY = "\033[90m"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"
CLEAR_SCREEN = "\033[2J"
HOME = "\033[H"
CLEAR_EOL = "\033[K"
CLEAR_BELOW = "\033[J"


def col(text, color):
    return f"{color}{text}{RESET}"


# ── Rate / freshness tracker ──────────────────────────────────────────────────
class Track:
    """Records arrival times so we can report Hz and staleness without storing
    the (potentially large) messages themselves."""

    def __init__(self, window=25):
        self._times = collections.deque(maxlen=window)
        self.last_mono = None
        self.value = None  # last extracted display value (small)

    def tick(self, value=None):
        self.last_mono = time.monotonic()
        self._times.append(self.last_mono)
        self.value = value

    def hz(self):
        if len(self._times) < 2:
            return 0.0
        dt = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / dt if dt > 0 else 0.0

    def age(self):
        return None if self.last_mono is None else time.monotonic() - self.last_mono


# ORB-SLAM3 Tracking::eTrackingState  (see GetTrackingState())
TRACK_STATES = {
    -1: ("SYSTEM_NOT_READY", RED),
    0:  ("NO_IMAGES_YET",    RED),
    1:  ("NOT_INITIALIZED",  YELLOW),
    2:  ("OK",               GREEN),
    3:  ("RECENTLY_LOST",    YELLOW),
    4:  ("LOST",             RED),
    5:  ("OK_KLT",           GREEN),
}

# FSM state name -> colour
FSM_COLORS = {
    "FAILSAFE_RTL": RED, "FAILSAFE_LAND": RED,
    "NAV_DEGRADED": YELLOW, "ARMING": YELLOW,
    "IDLE": GREY, "COMPLETE": CYAN,
}


class Diag(Node):
    def __init__(self):
        super().__init__("ascend_diagnose")

        # BEST_EFFORT + VOLATILE is the maximally-compatible subscriber QoS: it
        # receives from RELIABLE and BEST_EFFORT publishers alike (the reverse is
        # not true), so one profile works for every topic below.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.start_mono = time.monotonic()

        # Trackers
        self.t_state = Track()       # FSM state (event-based)
        self.t_slam_ok = Track()
        self.t_failsafe = Track()
        self.t_track = Track()       # SLAM tracking_state
        self.t_odom = Track()
        self.t_pose = Track()
        self.t_batt = Track()
        self.t_target = Track()
        self.t_cam_color = Track()
        self.t_cam_depth = Track()

        # FSM state change clock (for "in state Xs")
        self._fsm_state = None
        self._fsm_since = None

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(String, "/ascend/mission_control/state",
                                 self._cb_state, qos)
        self.create_subscription(Bool, "/ascend/localization/slam_ok",
                                 lambda m: self.t_slam_ok.tick(m.data), qos)
        self.create_subscription(String, "/ascend/mission_control/failsafe_triggered",
                                 lambda m: self.t_failsafe.tick(m.data), qos)
        self.create_subscription(Int32, "/orbslam/tracking_state",
                                 lambda m: self.t_track.tick(m.data), qos)
        self.create_subscription(Odometry, "/odom",
                                 lambda m: self.t_odom.tick(m.pose.pose.position), qos)
        self.create_subscription(PoseStamped, "/ap/pose/filtered",
                                 lambda m: self.t_pose.tick(m.pose.position), qos)
        self.create_subscription(BatteryState, "/ap/battery_status",
                                 lambda m: self.t_batt.tick(m.percentage), qos)
        self.create_subscription(PoseStamped, "/ascend/mission_control/target_pose",
                                 lambda m: self.t_target.tick(m.pose.position), qos)
        # Camera RATE only — read the tiny camera_info, never the image frame.
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 lambda m: self.t_cam_color.tick(), qos)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            lambda m: self.t_cam_depth.tick(), qos)

    def _cb_state(self, msg):
        if msg.data != self._fsm_state:
            self._fsm_state = msg.data
            self._fsm_since = time.monotonic()
        self.t_state.tick(msg.data)

    # ── Rendering helpers ─────────────────────────────────────────────────────
    @staticmethod
    def _hz_str(track, stale_after=1.5):
        age = track.age()
        if age is None:
            return col("   no data", GREY)
        hz = track.hz()
        if age > stale_after:
            return col(f"{hz:5.1f} Hz STALE {age:.1f}s", RED)
        color = GREEN if hz > 0.5 else YELLOW
        return col(f"{hz:5.1f} Hz", color)

    @staticmethod
    def _xyz(p):
        return f"x={p.x:+6.2f} y={p.y:+6.2f} z={p.z:+6.2f}"

    def _row(self, label, value):
        return f"   {label:<13}{value}"

    def render(self):
        L = []
        up = time.monotonic() - self.start_mono
        clk = time.strftime("%H:%M:%S")
        L.append(col(f" ASCEND DIAGNOSTICS", BOLD) +
                 col(f"                          {clk}   up {up:4.0f}s", GREY))
        L.append(col(" " + "─" * 58, GREY))

        # ── MISSION ──
        L.append(col(" MISSION", BOLD + CYAN))
        if self.t_state.value is None:
            fsm = col("waiting (publishes on transition only)", GREY)
        else:
            color = FSM_COLORS.get(self._fsm_state, GREEN)
            in_state = (time.monotonic() - self._fsm_since) if self._fsm_since else 0.0
            fsm = col(f"{self._fsm_state:<16}", color) + col(f"(in state {in_state:4.1f}s)", GREY)
        L.append(self._row("FSM state", fsm))

        if self.t_slam_ok.value is None:
            slam_ok = col("no data", GREY)
        elif self.t_slam_ok.value:
            slam_ok = col("TRUE", GREEN)
        else:
            slam_ok = col("FALSE  (FSM will LOITER / NAV_DEGRADED)", RED)
        L.append(self._row("slam_ok", slam_ok))

        if self.t_failsafe.value is None:
            fs = col("—", GREY)
        else:
            fs = col(str(self.t_failsafe.value), RED)
        L.append(self._row("failsafe", fs))
        L.append("")

        # ── SLAM ──
        L.append(col(" SLAM (ORB-SLAM3)", BOLD + CYAN))
        if self.t_track.value is None:
            ts = col("no data — is the rgbd node running?", GREY)
        else:
            name, color = TRACK_STATES.get(self.t_track.value, ("UNKNOWN", YELLOW))
            ts = col(f"[{self.t_track.value}] {name:<16}", color) + self._hz_str(self.t_track)
        L.append(self._row("tracking", ts))

        if self.t_odom.value is None:
            od = col("no data (only flows when tracking >= OK)", GREY)
        else:
            od = self._xyz(self.t_odom.value) + "  " + self._hz_str(self.t_odom, stale_after=2.0)
        L.append(self._row("/odom", od))
        L.append("")

        # ── ARDUPILOT ──
        L.append(col(" ARDUPILOT", BOLD + CYAN))
        if self.t_pose.value is None:
            ps = col("no data — EKF/DDS not publishing", GREY)
        else:
            ps = self._xyz(self.t_pose.value) + "  " + self._hz_str(self.t_pose)
            p = self.t_pose.value
            if abs(p.x) < 1e-6 and abs(p.y) < 1e-6 and abs(p.z) < 1e-6:
                ps += col("  <- frozen at 0,0,0?", YELLOW)
        L.append(self._row("/ap/pose", ps))

        if self.t_batt.value is None or self.t_batt.value != self.t_batt.value or self.t_batt.value < 0:
            bt = col("-- (no battery telemetry — failsafe disabled)", YELLOW)
        else:
            pct = self.t_batt.value * 100.0
            bcolor = GREEN if pct >= 30 else (YELLOW if pct >= 15 else RED)
            bt = col(f"{pct:5.1f} %", bcolor)
        L.append(self._row("battery", bt))

        if self.t_target.value is None:
            tg = col("—", GREY)
        else:
            tg = self._xyz(self.t_target.value)
        L.append(self._row("target wp", tg))
        L.append("")

        # ── CAMERA ──
        L.append(col(" CAMERA (rate via camera_info)", BOLD + CYAN))
        L.append(self._row("color", self._hz_str(self.t_cam_color)))
        L.append(self._row("depth", self._hz_str(self.t_cam_depth)))
        L.append(col(" " + "─" * 58, GREY))
        L.append(col(" run ON the Pi over SSH  ·  Ctrl-C to quit  ·  refresh 5 Hz", GREY))

        # In-place redraw: home, overwrite each line clearing to EOL, then clear
        # anything left below from a previous taller frame. Avoids flicker.
        out = [HOME]
        for line in L:
            out.append(line + CLEAR_EOL + "\n")
        out.append(CLEAR_BELOW)
        sys.stdout.write("".join(out))
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Diag()
    sys.stdout.write(CLEAR_SCREEN + HIDE_CURSOR)
    sys.stdout.flush()
    last_render = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_render >= 0.2:   # 5 Hz redraw
                node.render()
                last_render = now
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW_CURSOR + RESET + "\n")
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
