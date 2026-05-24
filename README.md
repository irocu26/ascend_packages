# Before cloning this repo make sure your Ardupilot is of 4.7 , if you dont know it is at which version you can simply copy paste this command 

```
cd ~/ardu_ws/src/ardupilot
git fetch --all --tags
git checkout ArduPilot-4.7
git submodule update --init --recursive
```
> Next step (May take while)
```
cd ~/ardu_ws/src/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y

```
> Final Prerequisite Step
```
cd ~/ardu_ws/
colcon build  --packages-select ardupilot_sitl
```

# To Clone this repo : 
``` 
mkdir -p ~/ardu_ws/src && cd ~/ardu_ws/src && git clone "https://github.com/irocu26/ascend_packages.git"
```


# ASCEND Robotics Workspace

This repository contains the modular software architecture for the **ASCEND Autonomous Drone System**.  
The architecture is designed around **Hardware Abstraction**, allowing the same software stack to run seamlessly in:

- Gazebo SITL Simulation
- Raspberry Pi 5 onboard hardware

The goal is to maintain identical high-level logic across simulation and real-world deployment while isolating hardware-specific implementations.

---

# Workspace Structure

```text
~/ardu_ws/src/ascend_packages/
├── ascend_bringup/          # Hardware interface, launch files, and system configuration
├── ascend_localization/     # SLAM / VIO configuration and localization pipeline
├── ascend_vision/           # OpenCV-based perception and vision processing
├── ascend_mission_control/  # Autonomous flight logic and state machine
└── ascend_ground_station/   # Validation, telemetry analysis, and reporting tools
```




---
## Note : The topics are not yet decided kindly push your code with care, the naming shoudld generally be  `ascend/{node_sub_name}/{topic_name}`


---

# Package Responsibilities

| Package | Role | Key Components |
|---|---|---|
| `ascend_bringup` | Hardware Interface Layer | ROS2 launch files, sensor parameters, camera calibration YAML files, hardware remappings, device initialization |
| `ascend_localization` | Position Estimation Engine | OpenVINS / RTAB-Map configuration, TF tree management, visual odometry|
| `ascend_vision` | Perception Pipeline | Image downsampling, cropping, OpenCV preprocessing, ArUco detection, 2D-to-3D ray projection |
| `ascend_mission_control` | Autonomous Decision Layer | Finite State Machine (FSM), Micro-XRCE-DDS communication, autonomous navigation and flight command logic |
| `ascend_ground_station` | Data Validation and Analytics | Coordinate formatting, mission validation scripts, telemetry parsing, post-flight reporting |

---

