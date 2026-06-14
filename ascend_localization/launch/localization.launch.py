"""
ascend_localization/launch/localization.launch.py

Single-parameter SLAM bring-up. The SAME high-level pipeline runs in Gazebo
simulation and on the Raspberry Pi 5 + RealSense hardware; only the camera
source, calibration file, image topics and clock differ. Those differences are
captured entirely in the `target` argument and the per-target settings YAML
(the rgbd node reads its mount rotation from `Mount.R` in that YAML).

This is a LOCAL launch file: it brings up only this package's nodes. The master
launch (ascend_bringup/ascend.launch.py) includes it and forwards `target`.

Usage (standalone, for testing this package alone):
    ros2 launch ascend_localization localization.launch.py target:=sim   # Gazebo (default)
    ros2 launch ascend_localization localization.launch.py target:=hw    # RealSense D435i

The ORB vocabulary is large and ships with ORB-SLAM3, so it is read from the
ORB-SLAM3 install rather than the package share:
    export ORB_SLAM3_ROOT=/path/to/ORB_SLAM3   (defaults to ~/ardu_ws/ORB_SLAM3)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Per-target profile: everything that differs between sim and hardware.
PROFILES = {
    'sim': {
        'calib': 'gazebo_rgbd.yaml',
        'rgb_topic': '/rgbd_camera/image',
        'depth_topic': '/rgbd_camera/depth_image',
        'use_sim_time': True,
        'start_realsense': False,
        'camera_frame': None,          # Gazebo publishes the camera TF itself
    },
    'hw': {
        'calib': 'RealSense_D435i.yaml',
        'rgb_topic': '/camera/camera/color/image_raw',
        'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
        'use_sim_time': False,
        'start_realsense': True,
        'camera_frame': 'camera_link',  # no robot_state_publisher on hardware
    },
}


def launch_setup(context, *args, **kwargs):
    target = LaunchConfiguration('target').perform(context)
    if target not in PROFILES:
        raise RuntimeError(
            f"target must be one of {list(PROFILES)}, got '{target}'")
    p = PROFILES[target]

    pkg_share = get_package_share_directory('ascend_localization')
    calib = os.path.join(pkg_share, 'config', 'rgbd', p['calib'])

    # Vocabulary lives with the ORB-SLAM3 install (portable via env var,
    # mirrors the ORB_SLAM3_ROOT logic in CMakeLists.txt).
    orb_root = os.environ.get(
        'ORB_SLAM3_ROOT', os.path.expanduser('~/ardu_ws/ORB_SLAM3'))
    vocab = os.path.join(orb_root, 'Vocabulary', 'ORBvoc.txt')

    use_sim_time = {'use_sim_time': p['use_sim_time']}
    nodes = []

    # ── Hardware camera driver (hw only; Gazebo provides images in sim) ──────
    if p['start_realsense']:
        nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'align_depth.enable': True,   # depth registered to color
                'enable_sync': True,          # time-sync color + depth
                # intrinsics in RealSense_D435i.yaml are for 640x480:
                # 'rgb_camera.color_profile': '640x480x15',
                # 'depth_module.depth_profile': '640x480x15',
            }],
        ))

    # ── ORB-SLAM3 RGBD visual odometry ───────────────────────────────────────
    #    argv: <vocabulary> <settings.yaml> <doRectify> <CLAHE>
    nodes.append(Node(
        package='ascend_localization',
        executable='rgbd',
        name='rgbd',
        output='screen',
        arguments=[vocab, calib, 'false', 'true'],
        remappings=[
            ('/camera/rgb', p['rgb_topic']),
            ('/camera/depth', p['depth_topic']),
        ],
        parameters=[use_sim_time],
    ))

    # ── SLAM -> ArduPilot relay ──────────────────────────────────────────────
    #    Subscribes /orbslam/tf_to_ap (published by rgbd above) and /ap/time,
    #    republishes the pose on /ap/tf for the ArduPilot DDS intake.
    nodes.append(Node(
        package='ascend_localization',
        executable='slam_to_ap',
        name='slam_to_ap',
        output='screen',
        parameters=[use_sim_time],
    ))

    # ── EKF source manager ───────────────────────────────────────────────────
    #    Watches /orbslam/tracking_state, publishes /ascend/localization/slam_ok
    #    for the FSM, and switches the ArduPilot EKF source set (SLAM <-> optical
    #    flow) via /ap/joy on the RCx_OPTION=90 aux channel. Target-agnostic.
    nodes.append(Node(
        package='ascend_localization',
        executable='ekf_source_manager',
        name='ekf_source_manager',
        output='screen',
        parameters=[use_sim_time],
    ))

    # ── Static TF base_link -> camera (hw only) ──────────────────────────────
    if p['camera_frame'] is not None:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', p['camera_frame']],
            parameters=[use_sim_time],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target',
            default_value='sim',
            description="Run target: 'sim' (Gazebo SITL) or 'hw' (RealSense D435i).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
