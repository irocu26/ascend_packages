## build the workspace properly 

## Run the normal gz ardupilot simulation
``` bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

## check the topic list
``` bash
ros2 topic list #should contain /ap topics
```
## Check apinterface node
The iris should takeoff and hover for 5 seconds and move with 0.5m/s velocity and then land
``` bash
ros2 run ascend_mission_control ap_interface
```

## For running full mission control, Run in different terminals 

``` bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

``` bash
ros2 launch ascend_bringup ascend.launch.py
```
``` bash
ros2 topic pub --once /ascend/mission_control/start_cmd std_msgs/msg/Empty "{}"
```

---

# Changelog — FSM Audit & Hardening (2026-06-16)

A logic audit of `fsm_node.py` surfaced a set of correctness/safety bugs. All
were fixed in order; each entry below is **Problem → Fix** so it can be verified
against the code. Tags (`#N`) refer to the audit findings.

### 1. Arm/mode service responses now actually checked (#1)
- **Problem:** `_state_arming` judged success by `future.result() is not None`,
  which is true for *any* completed service call. A **rejected** GUIDED switch or
  arm was treated as success → the FSM advanced to TAKEOFF on an unarmed / wrong-
  mode vehicle.
- **Fix:** check the real response fields — `ModeSwitch.status` and
  `ArmMotors.result` — and re-send (throttled) on rejection.

### 2. Arming timeout made reachable (#2)
- **Problem:** the `arming_timeout_s` abort sat *after* three early-returning
  branches, so it was unreachable; with ARMING also failsafe-exempt, a hung
  arm / missing DDS services hung the FSM forever.
- **Fix:** moved the timeout check to the top of `_state_arming`; on timeout it
  resets the arming sub-state, aborts to `IDLE`, and re-arms the start trigger.

### 3. Single pose source of truth — no redundant SLAM (#3)
- **Problem:** the FSM preferred the raw SLAM pose (`/ascend/localization/pose`)
  and only fell back to `/ap/pose/filtered`. `_current_pose` was never cleared,
  so once SLAM arrived the fused-pose fallback was dead — and a stale SLAM pose
  kept driving the velocity controller after SLAM dropped (flyaway risk).
- **Fix:** removed the raw SLAM subscription and `_current_pose` entirely. All
  position, feature logging, and link-liveness now use `/ap/pose/filtered` — the
  EKF-fused output that already reflects whichever EK3 source set the
  `ekf_source_manager` has selected (SLAM ext-nav ↔ optical-flow). `slam_ok` is
  still consumed, only as the NAV_DEGRADED event.

### 4. Multi-sortie transfer + charge (#4)
- **Problem:** `_charge_triggered` was never reset and doubled as a
  "have we ever charged?" flag, so sortie 2+ skipped TRANSFER (later sorties'
  data never uploaded) and CHARGING never re-commanded the charger.
- **Fix:** DOCKING now always → TRANSFER (upload every sortie); CHARGING resets
  `_charge_triggered` so the charger is re-commanded each sortie.

### 5. Per-waypoint hover dwell (#5)
- **Problem:** dwell used `_elapsed_in_state()` (time-in-SURVEY), so after the
  first 1.5 s every waypoint advanced instantly — no pause, no steady SIFT frames.
- **Fix:** added `_wp_arrival_time`; the dwell is now timed from arrival at each
  waypoint.

### 6. MATCH_VERIFY holds over the feature (#6)
- **Problem:** during verification it kept flying toward the next survey
  waypoint, then logged the position ~`match_hover_s` of travel downrange.
- **Fix:** captures the position at detection, holds it (P-controller resists
  drift), and logs that held position.

### 7. TAKEOFF / LANDING watchdogs (#7)
- **Problem:** TAKEOFF ignored the takeoff-service result and had no climb
  timeout; LANDING hung forever if precision landing aborted (it only publishes
  `complete` on success).
- **Fix:** added `takeoff_timeout_s` + a takeoff result-check/re-send, and
  `landing_timeout_s`. Both fall back to `FAILSAFE_LAND`.

### 8. Multi-sortie re-arm gap
- **Problem:** every landing disarms the vehicle, but repeat sorties went
  `CHARGING → TAKEOFF`, which never re-arms → takeoff commanded on a disarmed
  vehicle. Second sorties could not launch.
- **Fix:** repeat sorties now go `CHARGING → ARMING → TAKEOFF`, re-entering
  GUIDED + arm (with the #1/#2 checks) before each takeoff.

### 9. YAML parameters wired up (#8)
- **Problem:** `hover_dwell_s`, `match_hover_s`, `charge_timeout_s`,
  `transfer_timeout_s`, `arming_timeout_s`, `link_timeout_s` were present in
  `mission_params.yaml` but never declared/read — silently ignored at runtime.
- **Fix:** declared/read all of them plus the new `takeoff_timeout_s`,
  `landing_timeout_s`, `battery_timeout_s`. Every timing knob is now tunable from
  the yaml (21/21 declared ↔ yaml parity).

### 10. FAILSAFE_LAND terminates cleanly (#L)
- **Problem:** after touchdown it re-published a descent setpoint, re-disarmed,
  and re-logged **every tick forever**, never latching — and could command
  *upward* when pose was missing (GUIDED never auto-disarms on the ground).
- **Fix:** commands ArduPilot **LAND mode** (ground detection + auto-disarm,
  pose-independent) with a confirmed mode switch, then latches a terminal state —
  disarm once, then fully quiescent.

### 11. Battery telemetry handling (#K)
- **Problem:** `_battery_pct` defaulted to 100 %, so an absent / `NaN` / stale
  battery never triggered a failsafe.
- **Fix:** only real (non-NaN) readings update the level; if no valid telemetry
  has arrived it warns loudly (failsafe disabled, not silently "full"); mid-flight
  telemetry loss (> `battery_timeout_s`) triggers RTL. (AP_DDS sends `percentage`
  as a 0–1 fraction, so the `×100` conversion is correct.)

### 12. Test suite repaired
- **Problem:** `test/test_fsm.py` imported deleted `states`/`fsm` modules (the old
  event-driven FSM) and `test_fsm_logic.py` used the removed `overlap_factor`
  arg — both failed at collection, so there was no real coverage.
- **Fix:** `test_fsm_logic.py` now matches the real `LawnmowerPattern`/`State`
  API (pure-Python, no ROS2 needed). `test/test_fsm.py` is rewritten as
  **node-level regression tests** that spin up the real `AscendFSMNode` (with a
  stubbed APInterface) and assert each fix above, plus the preserved `geo_utils`
  maths tests.

**Verify:**
``` bash
cd ~/ardu_ws/src/ascend_packages
pytest ascend_mission_control/ascend_mission_control/test_fsm_logic.py \
       ascend_mission_control/test/test_fsm.py -v
# 55 passed
```