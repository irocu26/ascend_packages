# ASCEND Mission Control - Autonomous Flight System

Production-ready autonomous mission control for ASCEND drone using ROS 2 and Micro-XRCE-DDS communication with ArduPilot/PX4.

## Overview

**Architecture**: Modular, testable FSM-based system
- **fsm.py** - Pure Python state machine (testable without ROS 2)
- **fcu_bridge.py** - All FCU I/O via uXRCE-DDS (10 Hz offboard heartbeat)
- **mission_executor.py** - 10 Hz control loop evaluating sensor thresholds and dispatching FSM events
- **states.py** - Data structures & transition table
- **geo_utils.py** - Haversine distance, bearing, radius checks

## Flight Capabilities

✅ **Mission States**
- IDLE → ARMED → TAKEOFF → HOVER → MISSION → LAND → DISARMED → IDLE

✅ **Flight Operations**
- ARM/DISARM
- TAKEOFF to target AGL altitude
- GPS-guided waypoint navigation
- HOVER with configurable duration at each waypoint
- LAND with automatic disarm
- EMERGENCY_LAND from any state

✅ **Sensor Integration**
- GPS position (Haversine distance check)
- Altitude (AGL from barometer)
- Attitude (roll/pitch/yaw from IMU)
- Velocity (NED frame)

✅ **Offboard Safety**
- Mandatory 10 Hz control heartbeat to FCU
- Automatic failsafe if heartbeat lost

## Installation

### Prerequisites
```bash
# ROS 2 Humble (Ubuntu 22.04)
sudo apt-get install ros-humble-desktop python3-colcon-common-extensions


```

### Build
```bash
cd ~/ardu_ws
colcon build --packages-select ascend_mission_control
source install/setup.bash
```

### Run Tests (No ROS 2 Needed)
```bash
cd src/ascend_packages/ascend_mission_control
pytest test/test_fsm.py -v  # 26 unit tests
```

## Quick Start

### 1. Configure Mission (Before Flight)

Edit `config/mission_params.yaml`:

```yaml
mission:
  ros__parameters:
    name: "GPS Waypoint Mission"
    home:
      latitude: 40.7128      # YOUR HOME LAT
      longitude: -74.0060    # YOUR HOME LON
      altitude: 0.0
    takeoff_altitude: 10.0   # AGL metres
    mission_speed: 5.0       # m/s
    waypoint_reach_radius: 2.0
    
    waypoints:
      - name: "Home Hover"
        position:
          latitude: 40.7128
          longitude: -74.0060
          altitude: 10.0
        hover_duration: 3.0
        speed: 0.0
      
      - name: "Waypoint 1 (50m North)"
        position:
          latitude: 40.7133   # ~55m north
          longitude: -74.0060
          altitude: 10.0
        hover_duration: 2.0
        speed: 5.0
      
      - name: "Return Home"
        position:
          latitude: 40.7128
          longitude: -74.0060
          altitude: 10.0
        hover_duration: 2.0
        speed: 5.0
```

### 2. Launch System

```bash
# Terminal 1 - Launch nodes
ros2 launch ascend_mission_control mission.launch.py

# Terminal 2 - ARM drone
ros2 topic pub --once /ascend/mission/cmd std_msgs/String "data: 'ARM'"

# Terminal 3 - START mission
ros2 topic pub --once /ascend/mission/cmd std_msgs/String "data: 'START_MISSION'"

# Monitor mission status
ros2 topic echo /ascend/mission/status
ros2 topic echo /ascend/mission/position
```

### 3. Mission Commands

| Command | Effect |
|---------|--------|
| `ARM` | Arm motors, set home position |
| `DISARM` | Disarm motors (must be landed) |
| `TAKEOFF` | Takeoff to configured altitude |
| `LAND` | Land and auto-disarm |
| `EMERGENCY_LAND` | Immediate land from any state |
| `START_MISSION` | Execute waypoint mission |

**Example**:
```bash
# Emergency landing
ros2 topic pub --once /ascend/mission/cmd std_msgs/String "data: 'EMERGENCY_LAND'"
```

## Topic Reference

### Published Topics
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/ascend/mission/status` | `std_msgs/String` | 10 Hz | `state=...,waypoint=.../.,alt=...m,gps_fix=...` |
| `/ascend/mission/position` | `geometry_msgs/PoseStamped` | 10 Hz | Current GPS position + altitude |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | 10 Hz | Offboard heartbeat |
| `/fmu/in/vehicle_command` | `VehicleCommand` | On-demand | ARM/DISARM/TAKEOFF/LAND |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | On-demand | Position/velocity setpoints |

### Subscribed Topics
| Topic | Type | From | Description |
|-------|------|------|-------------|
| `/ascend/mission/cmd` | `std_msgs/String` | User | Mission commands (ARM, LAND, etc.) |
| `/fmu/out/vehicle_status` | `VehicleStatus` | FCU | Arm state, in-air flag |
| `/fmu/out/sensor_gps` | `SensorGps` | FCU | GPS position, fix quality |
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | FCU | NED position, velocity |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | FCU | Attitude quaternion |

## Architecture Deep Dive

### FSM State Machine (Pure Python)

**states.py** defines:
- `DroneState` enum (IDLE, ARMED, TAKEOFF, HOVER, MISSION, LAND, DISARMED, ERROR)
- `MissionEvent` enum (EV_ARM, EV_ALTITUDE_REACHED, EV_WAYPOINT_REACHED, etc.)
- `TRANSITIONS` table: all valid (state, event) → new_state mappings

**fsm.py** implements:
- Event-driven state transitions
- Callbacks for state entry/exit and transitions
- `process_event()` method returns True/False if transition valid
- No ROS 2 dependencies → testable with pytest

```python
# Example usage
from ascend_mission_control.fsm import FlightFSM
from ascend_mission_control.states import DroneState, MissionEvent

fsm = FlightFSM()

# Register callbacks
fsm.register_enter_callback(DroneState.TAKEOFF, lambda ctx: print("Taking off!"))

# Process event
if fsm.process_event(MissionEvent.EV_ARM):
    print(f"Now in state: {fsm.get_state().value}")
```

### FCU Bridge (ROS 2 Node)

**fcu_bridge.py** owns all Flight Control Unit I/O:

1. **Publishers**
   - `/fmu/in/vehicle_command` - Send ARM/DISARM/TAKEOFF/LAND commands
   - `/fmu/in/offboard_control_mode` - 10 Hz heartbeat (mandatory)
   - `/fmu/in/trajectory_setpoint` - Position/velocity setpoints

2. **Subscribers**
   - `/fmu/out/vehicle_status` - Parse armed state, in-air flag
   - `/fmu/out/sensor_gps` - Parse GPS (lat/lon/alt, fix quality)
   - `/fmu/out/vehicle_local_position` - Parse NED position, velocity
   - `/fmu/out/vehicle_attitude` - Parse quaternion → Euler angles

3. **API Methods**
   - `arm()` / `disarm()` - Send control commands
   - `set_home()` - Set home at current GPS location
   - `takeoff(altitude)` - Command takeoff to AGL altitude
   - `land()` - Command landing
   - `move_to_local_offset(x, y, z, speed)` - Move in NED frame
   - `hover()` - Zero velocity (hold position)
   - `get_telemetry()` - Return dict with all sensor readings

**Key Design**: Nothing else touches the FCU directly. All I/O goes through fcu_bridge.

### Mission Executor (ROS 2 Node)

**mission_executor.py** orchestrates missions at 10 Hz:

1. **Subscriptions**
   - `/ascend/mission/cmd` - Receive STRING commands (ARM, LAND, START_MISSION, etc.)
   - Calls `fsm.process_event()` to transition states

2. **Control Loop** (10 Hz)
   - Polls sensor telemetry from fcu_bridge
   - **TAKEOFF state**: Check if `altitude_agl >= takeoff_altitude - threshold`
     → Trigger `EV_ALTITUDE_REACHED` → Enter HOVER
   - **HOVER state**: Check hover elapsed time
     → Trigger `EV_HOVER_COMPLETE` → Resume MISSION
   - **MISSION state**: Check if reached waypoint (Haversine distance ≤ threshold)
     → Trigger `EV_WAYPOINT_REACHED` → Enter HOVER
     → Move to next waypoint via `fcu_bridge.move_to_local_offset()`
   - **LAND state**: Check if `altitude_agl ≤ 0.5m`
     → Trigger `EV_LANDED` → Auto-disarm

3. **Publications**
   - `/ascend/mission/status` - Mission state, waypoint count, altitude
   - `/ascend/mission/position` - Current GPS pose

**Flow**:
```
User sends ARM → mission_executor → fsm.process_event(EV_ARM) → fcu_bridge.arm()
              ↓
        fcu_bridge publishes /fmu/in/vehicle_command
              ↓
        ArduPilot receives & arms
```

### Geographic Utilities

**geo_utils.py** (pure Python, no ROS 2):

- `haversine_distance(lat1, lon1, lat2, lon2)` → distance in metres
- `calculate_bearing(lat1, lon1, lat2, lon2)` → bearing 0-360°
- `is_within_radius(lat1, lon1, lat2, lon2, radius)` → bool
- `lat_lon_offset(lat, lon, north, east)` → (new_lat, new_lon)

Used by mission_executor to:
1. Check if drone reached waypoint: `haversine_distance(current, waypoint) ≤ 2m`
2. Generate movement commands to next waypoint

## Testing

### Unit Tests (No ROS 2 Required)

26 comprehensive tests in `test/test_fsm.py`:

```bash
pytest test/test_fsm.py -v

# Covers:
# - State transitions (valid & invalid)
# - Event dispatch
# - Callbacks
# - Emergency landing
# - Full mission sequence
# - Haversine distance & bearing
# - Radius checks
# - Mission plan creation
```

### Example Test

```python
def test_fsm_full_mission_sequence(self):
    """Verify complete ARM → TAKEOFF → MISSION → LAND → DISARM flow."""
    fsm = FlightFSM()
    
    events = [
        MissionEvent.EV_ARM,
        MissionEvent.EV_TAKEOFF_START,
        MissionEvent.EV_ALTITUDE_REACHED,
        MissionEvent.EV_MISSION_START,
        MissionEvent.EV_WAYPOINT_REACHED,
        MissionEvent.EV_MISSION_COMPLETE,
        MissionEvent.EV_LANDED,
        MissionEvent.EV_DISARM,
    ]
    
    for event in events:
        assert fsm.process_event(event)
    
    assert fsm.get_state() == DroneState.IDLE
```

## Troubleshooting

### "No GPS fix"
- Ensure FCU has GPS lock (green LED)
- Wait 30-60s after power-up for GPS acquisition
- Set realistic home coordinates in mission_params.yaml

### "Cannot transition from IDLE"
- Run `ros2 topic pub --once /ascend/mission/cmd std_msgs/String "data: 'ARM'"` first

### Offboard heartbeat lost
- Check `/fmu/in/offboard_control_mode` publishing at 10 Hz
- If stopped, FCU will failsafe (RTH or Land)

### Waypoints not reached
- Increase `waypoint_reach_radius` in config
- Check GPS accuracy is < `gps_accuracy_threshold`
- Verify drone hasn't hit GPS_DRIFT_MAX (PX4 param)

## Performance

- **FSM transition latency**: < 1 ms (pure Python)
- **Control loop rate**: 10 Hz (100 ms cycle)
- **Offboard heartbeat**: 10 Hz (mandatory, PX4/ArduPilot requirement)
- **GPS position update**: 1-5 Hz (depending on receiver)

## Production Checklist

Before first flight:

- [ ] **config/mission_params.yaml** - Replace 0.0 placeholders with real GPS coordinates
- [ ] `takeoff_altitude` - Set to your desired AGL altitude
- [ ] `waypoint_reach_radius` - Verify acceptable acceptance radius
- [ ] **FCU Configuration**
  - [ ] ArduPilot: Set `SERIAL*_PROTOCOL = 39` (uXRCE-DDS)
  - [ ] PX4: Set `UXRCE_DDS_CFG = 0` (enabled)
- [ ] **GPS Lock** - Confirm green LED before arming
- [ ] **Test in SITL** - Run in Gazebo simulator first
- [ ] **Unit Tests** - `pytest test/test_fsm.py -v` passes
- [ ] **ROS 2 Network** - Check topics flowing: `ros2 topic list`

## Future Enhancements

- [ ] Geofence validation
- [ ] Wind compensation
- [ ] Vision-based landing
- [ ] RTH (Return to Home) failsafe
- [ ] Mission logging/telemetry recording
- [ ] Web dashboard for mission planning
- [ ] Multi-vehicle swarm control

## License

BSD-3-Clause

## Support

For issues, feature requests, or questions:
- Open an issue on GitHub
- Contact: team@ascend.com