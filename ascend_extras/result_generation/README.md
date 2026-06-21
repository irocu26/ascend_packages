# 🛰️ ASCEND Result Dashboard — IRoC-U 2026

A **live base-station console + result-report generator** for the ASCEND
autonomous drone. It subscribes to the mission's ROS 2 topics and shows, in a
browser, exactly what the elimination-round judges expect to see on the base
station screen:

- the **7 elimination tasks** with live pass/in-progress/fail status & timestamps,
- live **telemetry** (state, battery, position, survey coverage),
- a top-down **arena map** with the drone, its trail, the base zone and the
  **determined feature coordinates**,
- a **mission timeline**, and
- a **one-click result report** (JSON + HTML + PDF) for the
  *"Reporting of Result/Tasks"* deliverable — generated automatically when the
  mission reaches `COMPLETE`.

It is **display-only** — it never publishes a command, so you can start or stop
it at any time without affecting the flight.

> Maps the 7 tasks from *Elimination-Round-Instructions V2*: Take-Off, Survey,
> Coordinate Determination, Landing, Charging, Data Validation, Reporting.

---

## 1 — Install dependencies

`rclpy` and the message packages come from your ROS 2 install. Only Flask and
(optionally) ReportLab are extra:

```bash
pip install -r requirements.txt          # Flask + reportlab
# PDF is optional — without reportlab you still get JSON + HTML reports.
```

## 2 — Run

In a terminal **with the workspace sourced** (so the ROS topics are visible):

```bash
source ~/ardu_ws/install/setup.bash
cd ~/ardu_ws/src/ascend_packages/ascend_extras/result_generation
python3 dashboard_node.py
```

The startup banner prints the URL. Open it on the base-station laptop (or any
device on the same network):

```
http://<base-station-ip>:8080
```

> 💡 For the elimination video, put this browser window on the base-station
> screen so it is captured alongside the drone, as the instructions require.

### Or via `ros2 launch`

```bash
ros2 launch result_dashboard.launch.py            # defaults
ros2 launch result_dashboard.launch.py http_port:=9000 team_name:="Team ASCEND"
```

### Typical full run order (sim)

```
Terminal A:  ros2 launch ardupilot_sitl sitl.launch.py
Terminal B:  MicroXRCEAgent udp4 -p 2019
Terminal C:  gz sim -r .../iris_arena_ascend.sdf
Terminal D:  ros2 launch ascend_mission_control sim_gazebo.launch.py
Terminal E:  python3 dashboard_node.py            # ← this dashboard
Terminal F:  ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once
```

---

## 3 — The result report

- Auto-generated into `report_dir` (default `~/ascend_results/`) when the FSM
  reaches `COMPLETE`.
- Or click **“Generate & Download Report”** any time — useful for a partial run.

Files produced (timestamped):

| File | Contents |
|------|----------|
| `results_<ts>.json` | Full machine-readable snapshot: tasks, timing, feature coordinates, timeline |
| `report_<ts>.html`  | Self-contained printable report (arena map + task table + validation notes) |
| `report_<ts>.pdf`   | Same, rendered with ReportLab *(only if reportlab is installed)* |

The **Data Validation** section documents that each feature was confirmed by
matching the on-board HD capture against the pre-uploaded **128×128 LR** seed
images (SIFT + RANSAC), with the confidence = normalised inlier ratio — i.e. the
methodology the rulebook asks you to corroborate in Video-2 / the design report.

---

## 4 — Topics consumed

| Topic | Type | Used for |
|-------|------|----------|
| `/ascend/mission_control/state` | `std_msgs/String` | FSM state → task status |
| `/ascend/mission_control/coord_log_str` | `std_msgs/String` (JSON) | Determined feature coordinates |
| `/ascend/mission_control/failsafe_triggered` | `std_msgs/String` | Failsafe banner |
| `/ascend/survey/progress` | `std_msgs/String` (`N/M`) | Survey coverage |
| `/ascend/survey/complete` | `std_msgs/Bool` | Survey task done |
| `/ascend/ground_station/charge_done` | `std_msgs/Bool` | Charging task done |
| `/ascend/ground_station/transfer_done` | `std_msgs/Bool` | Data-validation task done |
| `/ascend/precision_landing/complete` | `std_msgs/Bool` | Landing task done |
| `/ap/pose/filtered` (primary) / `/ascend/localization/pose` (fallback) | `geometry_msgs/PoseStamped` | Drone position — optical-flow EKF pose; **does not rely on ORB-SLAM3** |
| `/ap/battery_status` | `sensor_msgs/BatteryState` | Battery gauge |

## 5 — Parameters

Override with `--ros-args -p name:=value`:

| Param | Default | Meaning |
|-------|---------|---------|
| `http_host` | `0.0.0.0` | Bind address |
| `http_port` | `8080` | Web port |
| `team_name` | `ASCEND` | Shown in header & report |
| `report_dir` | `~/ascend_results` | Where reports are written |
| `arena_x_m` / `arena_y_m` | `10.67` / `7.62` | Arena size (matches `mission_params.yaml`) |
| `total_features` | `3` | Features to find |
| `dock_x` / `dock_y` | `0.0` / `0.0` | Base-station / dock origin |
| `auto_report` | `true` | Write report files at `COMPLETE` |
| `pose_source_label` | `Optical Flow` | Label shown for the position source — set to `Manual` if flying manually |

> Position is taken from the ArduPilot EKF pose (`/ap/pose/filtered`), which
> fuses the **optical-flow** sensor; `/ascend/localization/pose` is only a
> fallback. **ORB-SLAM3 is not relied upon.**

---

## 6 — Files

```
result_generation/
├── dashboard_node.py          # rclpy node + Flask SSE server (entry point)
├── mission_tracker.py         # thread-safe 7-task tracker (pure logic)
├── report_generator.py        # JSON + HTML + PDF report builder
├── result_dashboard.launch.py # ros2 launch wrapper
├── requirements.txt
├── README.md
└── public/                    # browser dashboard
    ├── index.html
    ├── style.css
    └── app.js
```

`mission_tracker.py` and `report_generator.py` have **no ROS/Flask imports**, so
they can be unit-tested directly (`python3 report_generator.py /tmp/demo` writes
a sample report you can open to preview the layout).
