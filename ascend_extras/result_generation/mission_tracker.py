#!/usr/bin/env python3
"""
mission_tracker.py — ASCEND IRoC-U 2026 elimination-round result tracker.

Pure, thread-safe state object. The dashboard node feeds it ROS events
(state changes, telemetry, features, ground-station acks); it maps those onto
the SEVEN elimination-round tasks defined by the rulebook / video instructions:

    1. Autonomous Take-Off
    2. Autonomous Survey
    3. Coordinate Determination
    4. Autonomous Landing
    5. Autonomous Charging
    6. Autonomous Data Validation
    7. Reporting of Result/Tasks

It also keeps an event timeline, the live telemetry snapshot, the drone trail,
and the determined feature coordinates. `snapshot()` returns a single
JSON-serialisable dict consumed by both the live web UI and the report
generator — there is no ROS or Flask dependency in this file, so it is trivially
unit-testable.
"""

from __future__ import annotations

import threading
import time
from datetime import datetime, timezone


# ── FSM state → human label (mirrors mission_monitor_node STATE_DISPLAY) ──────
STATE_LABEL = {
    'IDLE':          'Waiting for start command',
    'ARMING':        'Arming motors',
    'TAKEOFF':       'Climbing to survey altitude',
    'SURVEY':        'Scanning the arena',
    'MATCH_VERIFY':  'Verifying candidate feature',
    'RTL':           'Returning to base station',
    'LANDING':       'Descending to dock',
    'DOCKING':       'Aligning with dock pad',
    'CHARGING':      'Recharging battery',
    'TRANSFER':      'Transferring / validating data',
    'COMPLETE':      'Mission finished',
    'FAILSAFE_RTL':  'FAILSAFE — emergency return',
    'FAILSAFE_LAND': 'FAILSAFE — emergency landing',
}

# Terminal / abnormal states.
FAILSAFE_STATES = ('FAILSAFE_RTL', 'FAILSAFE_LAND')

# Task keys in display order, with their rulebook labels.
TASK_ORDER = [
    ('takeoff',    'Autonomous Take-Off'),
    ('survey',     'Autonomous Survey'),
    ('coordinate', 'Coordinate Determination'),
    ('landing',    'Autonomous Landing'),
    ('charging',   'Autonomous Charging'),
    ('validation', 'Autonomous Data Validation'),
    ('reporting',  'Reporting of Result/Tasks'),
]

# Status values: pending → active → done | failed
PENDING, ACTIVE, DONE, FAILED = 'pending', 'active', 'done', 'failed'

# Cap stored trail / event-log lengths so memory stays bounded on the Pi.
MAX_TRAIL = 4000
MAX_EVENTS = 300


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')


class _Task:
    __slots__ = ('key', 'label', 'status', 'started_at', 'completed_at', 'detail')

    def __init__(self, key: str, label: str):
        self.key = key
        self.label = label
        self.status = PENDING
        self.started_at = None      # epoch seconds
        self.completed_at = None    # epoch seconds
        self.detail = ''

    def to_dict(self) -> dict:
        return {
            'key': self.key,
            'label': self.label,
            'status': self.status,
            'started_at': self.started_at,
            'completed_at': self.completed_at,
            'started_iso': _iso_or_none(self.started_at),
            'completed_iso': _iso_or_none(self.completed_at),
            'duration_s': (
                round(self.completed_at - self.started_at, 1)
                if self.started_at and self.completed_at else None
            ),
            'detail': self.detail,
        }


def _iso_or_none(epoch):
    if epoch is None:
        return None
    return datetime.fromtimestamp(epoch, timezone.utc).astimezone().isoformat(timespec='seconds')


class MissionTracker:
    """Thread-safe mission/result state. All public methods take the lock."""

    def __init__(self, *, team_name='ASCEND', arena_x=10.67, arena_y=7.62,
                 dock_x=0.0, dock_y=0.0, total_features=3):
        self._lock = threading.RLock()

        self.team_name = team_name
        self.arena_x = float(arena_x)
        self.arena_y = float(arena_y)
        self.dock_x = float(dock_x)
        self.dock_y = float(dock_y)
        self.total_features = int(total_features)

        # Tasks
        self._tasks = {key: _Task(key, label) for key, label in TASK_ORDER}

        # Mission status
        self.state = 'IDLE'
        self.failsafe = ''
        self.mission_start = None      # epoch — set at first TAKEOFF-class state
        self.mission_end = None        # epoch — set at COMPLETE or terminal failsafe

        # Telemetry
        self.pose = (0.0, 0.0, 0.0)
        self.pose_source = 'none'
        self._primary_pose_seen = False   # a trusted (optical-flow/manual) feed is live
        self.battery = 100.0
        self.battery_seen = False

        # Survey
        self.survey_done_wp = 0
        self.survey_total_wp = 0

        # Features (determined coordinates)
        self.features = []             # list of dicts {seed_id,x,y,confidence,hd_path}

        # History
        self.trail = []                # [(x, y), ...]
        self.events = []               # [{t, t_iso, kind, text}, ...]
        self.report_paths = {}         # {'json':..,'html':..,'pdf':..}

        self._add_event('system', 'Result dashboard online — waiting for mission.')

    # ── Event log helper ────────────────────────────────────────────────────
    def _add_event(self, kind: str, text: str):
        t = time.time()
        self.events.append({'t': t, 't_iso': _iso_or_none(t), 'kind': kind, 'text': text})
        if len(self.events) > MAX_EVENTS:
            del self.events[:-MAX_EVENTS]

    # ── Task helpers (assume lock held) ─────────────────────────────────────
    def _start(self, key: str, detail: str = ''):
        t = self._tasks[key]
        if t.status in (PENDING, FAILED):
            t.status = ACTIVE
            t.started_at = t.started_at or time.time()
            if detail:
                t.detail = detail
            self._add_event('task', f'▶ {t.label} started')

    def _finish(self, key: str, detail: str = '', failed: bool = False):
        t = self._tasks[key]
        if t.status == DONE:
            return
        if t.started_at is None:
            t.started_at = time.time()
        t.status = FAILED if failed else DONE
        t.completed_at = time.time()
        if detail:
            t.detail = detail
        verb = '✖ failed' if failed else '✔ done'
        self._add_event('task', f'{verb}: {t.label}')

    # ════════════════════════════════════════════════════════════════════════
    #  ROS-fed updates
    # ════════════════════════════════════════════════════════════════════════
    def on_state(self, new_state: str):
        with self._lock:
            if new_state == self.state:
                return
            prev = self.state
            self.state = new_state
            self._add_event('state', f'State: {prev} → {new_state}')

            # First climb marks the mission clock start.
            if new_state in ('ARMING', 'TAKEOFF') and self.mission_start is None:
                self.mission_start = time.time()

            if new_state in ('ARMING', 'TAKEOFF'):
                self._start('takeoff')

            elif new_state in ('SURVEY', 'MATCH_VERIFY'):
                self._finish('takeoff', 'Reached survey altitude')
                self._start('survey')
                self._start('coordinate')

            elif new_state == 'RTL':
                self._finish('takeoff', 'Reached survey altitude')
                self._finish('survey', self._survey_detail())
                self._finish_coordinate()
                self._start('landing')

            elif new_state in ('LANDING', 'DOCKING'):
                self._start('landing')

            elif new_state == 'CHARGING':
                self._finish('landing', 'Touchdown on dock')
                self._start('charging')

            elif new_state == 'TRANSFER':
                self._finish('charging', 'Battery recharged')
                self._start('validation')

            elif new_state == 'COMPLETE':
                # Anything not yet finished is implicitly complete by now.
                for key in ('takeoff', 'survey', 'landing', 'charging', 'validation'):
                    self._finish(key)
                self._finish_coordinate()
                self._start('reporting')
                self.mission_end = self.mission_end or time.time()
                self._add_event('system', 'Mission COMPLETE.')

            elif new_state in FAILSAFE_STATES:
                self.failsafe = STATE_LABEL.get(new_state, new_state)
                self.mission_end = self.mission_end or time.time()
                # Mark whatever is mid-flight as failed; leave finished tasks.
                for t in self._tasks.values():
                    if t.status == ACTIVE:
                        self._finish(t.key, f'Aborted by {new_state}', failed=True)
                self._add_event('failsafe', f'FAILSAFE entered: {new_state}')

    def on_pose(self, x: float, y: float, z: float, source: str, primary: bool = True):
        with self._lock:
            # Position comes from optical-flow (or manual) sources — we do NOT
            # depend on ORB-SLAM3. `primary` marks the trusted feed (the
            # optical-flow EKF pose); a non-primary feed is only used as a
            # fallback while the primary one is silent.
            if primary:
                self._primary_pose_seen = True
            elif self._primary_pose_seen:
                return
            self.pose = (round(x, 3), round(y, 3), round(z, 3))
            self.pose_source = source
            # Only record trail while airborne-ish to avoid a blob at the dock.
            if z > 0.3:
                if not self.trail or _dist2(self.trail[-1], (x, y)) > 0.04:
                    self.trail.append((round(x, 3), round(y, 3)))
                    if len(self.trail) > MAX_TRAIL:
                        del self.trail[:-MAX_TRAIL]

    def on_battery(self, pct: float):
        with self._lock:
            self.battery = round(pct, 1)
            self.battery_seen = True

    def on_survey_progress(self, done_wp: int, total_wp: int):
        with self._lock:
            self.survey_done_wp = int(done_wp)
            self.survey_total_wp = int(total_wp)

    def on_survey_complete(self):
        with self._lock:
            self._finish('survey', self._survey_detail())

    def on_feature(self, feature: dict):
        """feature: {seed_id, x, y, confidence, hd_path}."""
        with self._lock:
            sid = feature.get('seed_id')
            if any(f.get('seed_id') == sid for f in self.features):
                return  # de-dup — FSM already enforces this, belt & braces
            f = {
                'seed_id': sid,
                'x': round(float(feature.get('x', 0.0)), 3),
                'y': round(float(feature.get('y', 0.0)), 3),
                'confidence': round(float(feature.get('confidence', 0.0)), 3),
                'hd_path': feature.get('hd_path', ''),
                'found_at': time.time(),
                'found_iso': _now_iso(),
            }
            self.features.append(f)
            self._add_event(
                'feature',
                f'Feature {sid} located at ({f["x"]:+.2f}, {f["y"]:+.2f}) m '
                f'conf={f["confidence"]:.2f}')
            self._start('coordinate')
            if len(self.features) >= self.total_features:
                self._finish('coordinate', self._coord_detail())

    def on_charge_done(self):
        with self._lock:
            self._finish('charging', 'Charge-complete ack received')

    def on_transfer_done(self):
        with self._lock:
            self._finish('validation', 'Data-validation ack received')

    def on_landing_complete(self):
        with self._lock:
            self._finish('landing', 'Precision landing complete')

    def on_failsafe(self, reason: str):
        with self._lock:
            self.failsafe = reason or self.failsafe or 'Unspecified failsafe'
            self._add_event('failsafe', f'Failsafe trigger: {reason}')

    def mark_report_generated(self, paths: dict):
        with self._lock:
            self.report_paths = dict(paths)
            self._finish('reporting', 'Report files generated')
            files = ', '.join(p for p in paths.values() if p)
            self._add_event('system', f'Report generated: {files}')

    # ── Derived detail strings (assume lock held) ────────────────────────────
    def _survey_detail(self) -> str:
        if self.survey_total_wp:
            return f'{self.survey_done_wp}/{self.survey_total_wp} waypoints covered'
        return 'Survey sweep complete'

    def _coord_detail(self) -> str:
        return f'{len(self.features)}/{self.total_features} feature coordinates determined'

    def _finish_coordinate(self):
        n = len(self.features)
        if n >= self.total_features:
            self._finish('coordinate', self._coord_detail())
        elif n > 0:
            self._finish('coordinate', self._coord_detail())
        else:
            self._finish('coordinate', 'No features located', failed=True)

    # ════════════════════════════════════════════════════════════════════════
    #  Snapshot (single source of truth for UI + reports)
    # ════════════════════════════════════════════════════════════════════════
    def elapsed_s(self) -> int:
        if self.mission_start is None:
            return 0
        end = self.mission_end or time.time()
        return int(end - self.mission_start)

    def snapshot(self, *, trail_limit=1500, events_limit=60) -> dict:
        with self._lock:
            done = sum(1 for t in self._tasks.values() if t.status == DONE)
            failed = any(t.status == FAILED for t in self._tasks.values())
            survey_pct = (
                round(100.0 * self.survey_done_wp / self.survey_total_wp, 1)
                if self.survey_total_wp else 0.0
            )
            # Downsample the trail for the wire.
            trail = self.trail
            if len(trail) > trail_limit:
                step = len(trail) // trail_limit + 1
                trail = trail[::step]

            return {
                'team': self.team_name,
                'generated': _now_iso(),
                'arena': {
                    'x': self.arena_x, 'y': self.arena_y,
                    'dock_x': self.dock_x, 'dock_y': self.dock_y,
                },
                'mission': {
                    'state': self.state,
                    'state_label': STATE_LABEL.get(self.state, self.state),
                    'elapsed_s': self.elapsed_s(),
                    'started': self.mission_start is not None,
                    'complete': self.state == 'COMPLETE',
                    'failsafe': self.failsafe,
                    'in_failsafe': self.state in FAILSAFE_STATES,
                    'tasks_done': done,
                    'tasks_total': len(self._tasks),
                    'any_failed': failed,
                },
                'telemetry': {
                    'x': self.pose[0], 'y': self.pose[1], 'z': self.pose[2],
                    'pose_source': self.pose_source,
                    'battery': self.battery,
                    'battery_seen': self.battery_seen,
                    'battery_low': self.battery_seen and self.battery < 25.0,
                },
                'survey': {
                    'done_wp': self.survey_done_wp,
                    'total_wp': self.survey_total_wp,
                    'progress': (f'{self.survey_done_wp}/{self.survey_total_wp}'
                                 if self.survey_total_wp else '—'),
                    'pct': survey_pct,
                },
                'features': {
                    'found': len(self.features),
                    'total': self.total_features,
                    'list': [dict(f) for f in self.features],
                },
                'tasks': [self._tasks[k].to_dict() for k, _ in TASK_ORDER],
                'trail': [list(p) for p in trail],
                'events': list(self.events[-events_limit:][::-1]),  # newest first
                'report_paths': dict(self.report_paths),
            }


def _dist2(a, b) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
