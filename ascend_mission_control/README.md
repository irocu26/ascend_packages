# ascend_mission_control

Autonomous mission control package for the ASCEND drone system — IRoC-U 2026.

This package implements the complete **Finite State Machine (FSM)** that governs all autonomous behaviour: takeoff, lawnmower survey, feature detection, RTL, precision landing, charging, data transfer, and failsafe handling.

All behaviour strictly follows the **IRoC-U 2026 Elimination Round Rulebook v3.0**.

---

## Package Contents

```
ascend_mission_control/
├── ascend_mission_control/
│   ├── __init__.py
│   ├── fsm_node.py              ← Core FSM — the autonomous brain
│   ├── survey_planner_node.py   ← Lawnmower waypoint generator
│   ├── mission_monitor_node.py  ← Terminal dashboard + mission logger
│   └── mock_publisher_node.py   ← Test harness (simulates all sensors)
├── launch/
│   ├── sim_mock.launch.py       ← Test WITHOUT Gazebo/hardware
│   └── sim_gazebo.launch.py     ← Full Gazebo + SITL integration
├── config/
│   └── mission_params.yaml      ← All tunable parameters
├── test/
│   ├── test_fsm_logic.py        ← Pure Python unit tests (no ROS2 needed)
│   └── test_fsm_integration.py  ← ROS2 live integration tests
├── resource/
│   └── ascend_mission_control
├── package.xml
└── setup.py
```

---

## FSM States

The FSM enforces the full mission flow as required by the rulebook.

```
IDLE ──start_cmd──► ARMING ──armed──► TAKEOFF ──at_alt──► SURVEY
                                                               │
                    ◄────────── RTL ◄── all_found ────────────┤
                    │                                          │
                    │           ┌──────────── MATCH_VERIFY ◄──┘
                    │           │  (hover + confirm)
                    │           │
                    ▼           ▼ confirmed
                 LANDING ◄──── RTL
                    │
                    ▼
                 DOCKING
                    │
              ┌─────┴─────┐
              ▼           ▼
          TRANSFER     CHARGING
              │           │
              └─────┬─────┘
                    ▼
             (more sorties?) ──yes──► TAKEOFF
                    │
                    ▼ no
               COMPLETE

Any state ──battery<20%──► FAILSAFE_RTL ──above_home──► FAILSAFE_LAND
Any state ──battery<10%──► FAILSAFE_LAND
Any state ──pose_timeout──► FAILSAFE_RTL
```

### State descriptions

| State | Rulebook ref | Description |
|---|---|---|
| `IDLE` | §10.5.1 | Waiting for single start command |
| `ARMING` | §10.5.1 | Arming ArduPilot, switching to GUIDED mode |
| `TAKEOFF` | §10.5.1 | Climbing to survey altitude (2–6m) |
| `SURVEY` | §10.5.2 | Lawnmower scan, captures HD frames, triggers vision matcher |
| `MATCH_VERIFY` | §10.5.3 | Hovers over candidate, waits for matcher confirmation, logs coords |
| `RTL` | §10.5.4 | Returns to base station at survey altitude |
| `LANDING` | §10.5.4 | Descends to dock pad |
| `DOCKING` | §10.5.5 | Precision alignment with charging dock |
| `CHARGING` | §10.5.5 | Waits for charge complete signal |
| `TRANSFER` | §10.5.6 | Triggers ZMQ image + coordinate transfer |
| `COMPLETE` | §10.5.7 | Mission done, publishes final results |
| `FAILSAFE_RTL` | §7.2 Table-1 | Emergency return — low battery or lost link |
| `FAILSAFE_LAND` | §7.2 Table-1 | Emergency land in place — critical battery |

---

## Topic Interface

### Subscribed topics

| Topic | Type | Source | Purpose |
|---|---|---|---|
| `/ascend/mission_control/start_cmd` | `std_msgs/Empty` | Base station | **Single** mission start trigger |
| `/ascend/localization/pose` | `geometry_msgs/PoseStamped` | `ascend_localization` | SLAM position in base-station frame |
| `/ap/pose/filtered` | `geometry_msgs/PoseStamped` | ArduPilot uXRCE-DDS | Fallback position from SITL |
| `/ap/battery_status` | `sensor_msgs/BatteryState` | ArduPilot uXRCE-DDS | Battery percentage for failsafe |
| `/ascend/vision/match_result_str` | `std_msgs/String` (JSON) | `ascend_vision` | Feature match detection |
| `/ascend/ground_station/charge_done` | `std_msgs/Bool` | `ascend_ground_station` | Charging complete |
| `/ascend/ground_station/transfer_done` | `std_msgs/Bool` | `ascend_ground_station` | Data transfer complete |

### Published topics

| Topic | Type | Consumer | Purpose |
|---|---|---|---|
| `/ascend/mission_control/state` | `std_msgs/String` | All nodes / monitor | Current FSM state |
| `/ascend/mission_control/target_pose` | `geometry_msgs/PoseStamped` | Vizualization / SLAM | Next waypoint |
| `/ap/cmd_vel` | `geometry_msgs/TwistStamped` | ArduPilot uXRCE-DDS | Velocity commands to drone |
| `/ascend/mission_control/coord_log_str` | `std_msgs/String` (JSON) | Ground station | Feature ID + coords + HD path |
| `/ascend/mission_control/start_transfer` | `std_msgs/Bool` | `image_sharing` | Trigger ZMQ transfer |
| `/ascend/mission_control/start_charge` | `std_msgs/Bool` | `ascend_ground_station` | Trigger charging |
| `/ascend/mission_control/failsafe_triggered` | `std_msgs/String` | Monitor / logs | Failsafe reason |

---

## Building

```bash
# From your workspace root
cd ~/ardu_ws

# Build only this package
colcon build --packages-select ascend_mission_control

# Source
source install/setup.bash
```

---

## Testing — 4 levels

### Level 1: Pure Python unit tests (fastest, no ROS2 needed)

Tests FSM logic, lawnmower geometry, coordinate math, and failsafe thresholds
with plain `pytest`. No ROS2 runtime, no nodes, no simulator.

```bash
cd ~/ardu_ws/src/ascend_packages
pytest ascend_mission_control/test/test_fsm_logic.py -v
```

Expected output:
```
test_fsm_logic.py::TestLawnmowerPattern::test_generates_waypoints            PASSED
test_fsm_logic.py::TestLawnmowerPattern::test_altitude_respected             PASSED
test_fsm_logic.py::TestLawnmowerPattern::test_all_waypoints_inside_arena     PASSED
...
test_fsm_logic.py::TestScenarios::test_scenario_failsafe_rtl_exits_survey    PASSED
27 passed in 0.12s
```

---

### Level 2: Mock simulation (no Gazebo, no hardware)

Runs the full ROS2 node stack with `mock_publisher_node` faking all sensors.
The simulated drone moves toward target waypoints and triggers fake matches
when near the pre-set feature coordinates.

**Terminal 1 — Launch the stack:**
```bash
source ~/ardu_ws/install/setup.bash
ros2 launch ascend_mission_control sim_mock.launch.py
```

**Terminal 2 — Send start command (after nodes are ready):**
```bash
source ~/ardu_ws/install/setup.bash
ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once
```

**Terminal 3 — Watch state topic:**
```bash
ros2 topic echo /ascend/mission_control/state
```

**Terminal 4 — Watch coordinate log:**
```bash
ros2 topic echo /ascend/mission_control/coord_log_str
```

#### Test scenarios

Change the scenario with a launch argument:

```bash
# Normal full mission (default)
ros2 launch ascend_mission_control sim_mock.launch.py scenario:=normal

# Battery drops mid-survey → failsafe RTL
ros2 launch ascend_mission_control sim_mock.launch.py scenario:=low_battery_rtl

# Only 1 feature found per sortie → 3 sorties needed
ros2 launch ascend_mission_control sim_mock.launch.py scenario:=multi_sortie

# No features match → survey completes, RTL, transfer empty results
ros2 launch ascend_mission_control sim_mock.launch.py scenario:=no_match
```

---

### Level 3: ROS2 integration test (automated)

Programmatic test that starts nodes, injects a start command, and asserts
expected state sequences without human interaction.

```bash
source ~/ardu_ws/install/setup.bash

# Normal scenario
python3 src/ascend_packages/ascend_mission_control/test/test_fsm_integration.py normal

# Battery failsafe scenario
python3 src/ascend_packages/ascend_mission_control/test/test_fsm_integration.py low_battery_rtl

# No match scenario
python3 src/ascend_packages/ascend_mission_control/test/test_fsm_integration.py no_match
```

---

### Level 4: Full Gazebo + SITL simulation

Full end-to-end test with real SLAM, real flight controller, real arena.

**Prerequisites — run each in a separate terminal:**

```bash
# Terminal A — ArduPilot SITL
cd ~/ardu_ws
ros2 launch ardupilot_sitl sitl.launch.py

# Terminal B — Micro-XRCE-DDS Agent
MicroXRCEAgent udp4 -p 2019

# Terminal C — Gazebo with arena world
gz sim -r ~/ardu_ws/src/ardupilot_gazebo/worlds/iris_arena_ascend.sdf

# Terminal D — ORB-SLAM3 (monocular or RGB-D)
cd $ORB_SLAM3_ROOT_PATH/ORB-SLAM3
./Examples/Monocular/mono_webcam Vocabulary/ORBvoc.txt Examples/Monocular/Webcam.yaml

# Terminal E — SLAM bridge + ASCEND stack
source ~/ardu_ws/install/setup.bash
ros2 launch ascend_mission_control sim_gazebo.launch.py

# Terminal F — Send start command when position is stable
ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once
```

---

## Useful debug commands

```bash
# Watch all ASCEND topics at once
ros2 topic list | grep ascend

# Check FSM state
ros2 topic echo /ascend/mission_control/state --once

# Check drone position
ros2 topic echo /ascend/localization/pose --once

# Check battery
ros2 topic echo /ap/battery_status --once

# Check survey waypoints
ros2 topic echo /ascend/survey/progress

# View coordinate log (JSON)
ros2 topic echo /ascend/mission_control/coord_log_str

# Override survey altitude without recompiling
ros2 param set /ascend_fsm_node survey_altitude 4.0

# Check node graph
ros2 node list
ros2 node info /ascend_fsm_node

# Record a full mission for post-analysis
ros2 bag record -a -o /tmp/mission_bag

# Replay and inspect
ros2 bag play /tmp/mission_bag
```

---

## Parameters reference

All parameters are in `config/mission_params.yaml`. Key values:

| Parameter | Default | Notes |
|---|---|---|
| `survey_altitude` | `3.0` | meters; rulebook range: 2–6m |
| `wp_accept_radius` | `0.4` | meters; decrease for more precise survey |
| `low_battery_pct` | `20.0` | % below which failsafe RTL fires |
| `total_features` | `3` | per rulebook §10.5.2 |
| `max_sorties` | `5` | safety cap on repeat flights |
| `arena_x_m` | `10.67` | 35 ft — per rulebook §10.2 |
| `arena_y_m` | `7.62` | 25 ft — per rulebook §10.2 |
| `sim_mode` | `true` | set `false` for real hardware |

---

## Rulebook compliance checklist

| Requirement | Rule | Status |
|---|---|---|
| Single start command only | §10.5.1 | ✅ Enforced in `_cb_start` |
| No manual intervention after start | §10.5.1 | ✅ FSM locks out further input |
| Flight at 2–6m altitude | §10.3.5 | ✅ Enforced in `TAKEOFF` and survey planner |
| Land only at home/base station | §10.2 rule 3 | ✅ `LANDING` targets `DOCK_TARGET_X/Y` |
| Autonomous charging (at least once) | §10.1 | ✅ `CHARGING` state mandatory before second sortie |
| Autonomous data transfer (at least once) | §10.1 | ✅ `TRANSFER` state before `COMPLETE` |
| Failsafe: low battery | §7.2 Table-1 | ✅ `FAILSAFE_RTL` at <20%, `FAILSAFE_LAND` at <10% |
| Failsafe: lost link | §7.2 Table-1 | ✅ Pose timeout → `FAILSAFE_RTL` |
| Coordinate logging (base-station-relative) | §10.5.3 | ✅ From SLAM frame, home locked at takeoff |
| Coord + HD image reported | §10.5.7 | ✅ Published in `coord_log` |
| Stay within arena boundary | §10.2 | ✅ Planner insets 0.5m from all walls |
| Repeat sorties if needed | §10.1 | ✅ Up to `max_sorties` after recharge |

---

## Integration with other packages

```
ascend_localization  →  /ascend/localization/pose  →  fsm_node
ascend_vision        →  /ascend/vision/match_result_str  →  fsm_node
fsm_node             →  /ascend/mission_control/start_transfer  →  image_sharing
fsm_node             →  /ascend/mission_control/start_charge    →  ascend_ground_station
image_sharing        →  /ascend/ground_station/transfer_done    →  fsm_node
ascend_ground_station → /ascend/ground_station/charge_done      →  fsm_node
```

---

## Known limitations / TODO

- `_send_arm_command()` and `_send_disarm_command()` are stubbed in `sim_mode=True`.
  On hardware, replace with the actual ArduPilot arming service call via uXRCE-DDS.
- `_send_position_cmd()` uses a simple P-controller for simulation.
  On hardware, replace with direct `TrajectorySetpoint` or `SET_POSITION_TARGET_LOCAL_NED`
  via the ArduPilot MAVLink interface.
- `DOCKING` state assumes immediate dock in simulation.
  On hardware, integrate ArUco marker detection from `ascend_vision` for precision alignment.
- `ascend_msgs` custom message package is referenced but not yet created.
  The FSM falls back to JSON strings via `std_msgs/String` until `ascend_msgs` is built.