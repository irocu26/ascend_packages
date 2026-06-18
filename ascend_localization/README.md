# ascend_localization

ORB-SLAM3 RGBD visual odometry bring-up for the Ascend drone, with selectable
camera targets and motion-blur (exposure) controls for bench-vs-motors testing.

---

## Quick test matrix — copy / paste

Each command is fully self-contained with **nominal values** spelled out, so you
can paste it as-is during testing. Tune `exposure` / `gain` to chase motion blur
when motors are running.

> Defaults if you omit an arg: `auto_exposure:=true exposure:=8500 gain:=16
> fps:=30 width:=640 height:=480 emitter_enabled:=0`

### 1) D435i — COLOR RGBD (rolling shutter)
```bash
ros2 launch ascend_localization localization.launch.py \
  target:=d435i type:=n \
  auto_exposure:=true exposure:=8500 gain:=16 \
  fps:=30 width:=640 height:=480
```

### 2) D435i — INFRA RGBD (global-shutter left IR + raw depth)
```bash
ros2 launch ascend_localization localization.launch.py \
  target:=d435i type:=i \
  auto_exposure:=true exposure:=8500 gain:=16 \
  fps:=30 width:=640 height:=480 \
  emitter_enabled:=0
```

### 3) D455 — COLOR RGBD (global-shutter RGB)
```bash
ros2 launch ascend_localization localization.launch.py \
  target:=d455 type:=n \
  auto_exposure:=true exposure:=8500 gain:=16 \
  fps:=30 width:=640 height:=480
```

### Motion-blur kill (motors-on test — short exposure, lift gain)
Disable auto-exposure and force a short integration window; raise gain (and add
light) to keep the image bright. Swap `target`/`type` for whichever unit:
```bash
ros2 launch ascend_localization localization.launch.py \
  target:=d435i type:=n \
  auto_exposure:=false exposure:=3000 gain:=64 \
  fps:=30 width:=640 height:=480
```

### Legacy / sim (unchanged)
```bash
ros2 launch ascend_localization localization.launch.py target:=sim   # Gazebo SITL
ros2 launch ascend_localization localization.launch.py target:=hw    # D435i color (alias)
```

### From the master launch (whole stack)
`ascend_bringup` forwards the SLAM camera + blur knobs. `slam_target` defaults to
`target`, so the rest of the stack (mission-control, vision) stays on `hw` while
you pick the SLAM camera:
```bash
# D435i color
ros2 launch ascend_bringup ascend.launch.py target:=hw slam_target:=d435i type:=n

# D435i infra (global-shutter IR)
ros2 launch ascend_bringup ascend.launch.py target:=hw slam_target:=d435i type:=i emitter_enabled:=0

# D455 color (global-shutter RGB)
ros2 launch ascend_bringup ascend.launch.py target:=hw slam_target:=d455 type:=n

# motion-blur kill, whole stack
ros2 launch ascend_bringup ascend.launch.py target:=hw slam_target:=d435i type:=n \
  auto_exposure:=false exposure:=3000 gain:=64
```

---

## Launch arguments

| Arg | Default | Meaning |
|---|---|---|
| `target` | `sim` | Camera: `sim`, `hw`, `d435i`, `d455` |
| `type` | `n` | Stream: `n` color/normal, `i` infra/IR (`d435i` only) |
| `auto_exposure` | `true` | Manual `exposure` applies only when `false` |
| `exposure` | `8500` | Manual exposure, microseconds (shorter → less blur) |
| `gain` | `16` | Sensor gain (raise to offset short exposure) |
| `fps` | `30` | Stream frame rate |
| `width` | `640` | Stream width (must match calib YAML intrinsics) |
| `height` | `480` | Stream height (must match calib YAML intrinsics) |
| `emitter_enabled` | `0` | IR projector: `0` off, `1` on, `2` auto (infra only) |

Config files live in [`config/rgbd/`](config/rgbd/): `RealSense_D435i.yaml`
(color), `RealSense_D435i_infra.yaml` (IR — **placeholder intrinsics**),
`RealSense_D455.yaml` (color — **placeholder intrinsics**), `gazebo_rgbd.yaml`.

> ⚠️ The two placeholder configs use nominal datasheet intrinsics. Calibrate each
> unit before relying on the pose. Also confirm the `realsense2_camera` parameter
> names against your installed version: `ros2 param list /camera/camera`.

---

## Changelog

### 2026-06-19
- Added multi-target camera selection to `localization.launch.py`:
  `target:=d435i type:={n|i}` and `target:=d455 type:=n`; `sim`/`hw` preserved.
- Added motion-blur / exposure launch args: `auto_exposure`, `exposure`, `gain`,
  `fps`, `width`, `height`, `emitter_enabled`.
- Infra path (`type:=i`) uses raw depth in the left-IR frame (no align-to-color),
  enables `infra1`, and exposes the IR projector via `emitter_enabled`.
- New configs: `RealSense_D435i_infra.yaml`, `RealSense_D455.yaml` (placeholder
  intrinsics, pending calibration).
- `ascend_bringup/ascend.launch.py` now forwards the SLAM camera + blur knobs:
  new `slam_target` (defaults to `target`) plus `type` and all exposure args.
  Existing `target:=sim` / `target:=hw` runs are unchanged.
