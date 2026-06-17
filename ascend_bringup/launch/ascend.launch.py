"""
ascend_bringup/launch/ascend.launch.py

MASTER launch — the single entry point for the whole ASCEND stack. It declares
ONE argument, `target` (sim | hw), and fans it out to each package's local
launch file. The same command runs the system in simulation or on hardware:

    ros2 launch ascend_bringup ascend.launch.py target:=sim   # Gazebo (default)
    ros2 launch ascend_bringup ascend.launch.py target:=hw    # Raspberry Pi 5 + RealSense

This master does NOT start nodes directly; it includes the per-package launch
files (localization, mission-control, vision) and forwards `target` so each
package configures its own sim/hardware specifics.

Run the simulation/hardware base layer separately FIRST (Gazebo + ArduPilot
SITL + micro-ROS DDS agent), e.g.:

    ros2 launch ardupilot_gz_bringup iris_runway.launch.py

Vision (sift_node) subscribes to the camera and publishes match results on
/ascend/vision/match_result_str, which the FSM consumes to find features.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target = LaunchConfiguration('target')
    image_topic = LaunchConfiguration('image_topic')
    seed_images_dir = LaunchConfiguration('seed_images_dir')
    # SIFT matcher tuning — forwarded to vision.launch.py (see its --show-args).
    process_max_dim = LaunchConfiguration('process_max_dim')
    min_inliers = LaunchConfiguration('min_inliers')
    min_match_count = LaunchConfiguration('min_match_count')
    square_resize = LaunchConfiguration('square_resize')

    declare_target = DeclareLaunchArgument(
        'target',
        default_value='sim',
        description="Run target for the whole stack: 'sim' (Gazebo SITL) or "
                    "'hw' (Raspberry Pi 5 + RealSense D435i).",
    )
    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='',
        description='Override camera image topic for the SIFT node. '
                    'Empty = pick automatically from target.',
    )
    declare_seed_dir = DeclareLaunchArgument(
        'seed_images_dir',
        default_value=os.path.expanduser('~/ardu_ws/seed_images'),
        description='Directory of seed images to match against.',
    )
    declare_process_max_dim = DeclareLaunchArgument(
        'process_max_dim',
        default_value='0',
        description='SIFT scene long-side cap in px. 0 = full native res (#3).',
    )
    declare_min_inliers = DeclareLaunchArgument(
        'min_inliers',
        default_value='10',
        description='RANSAC inliers required to accept a SIFT match (#1).',
    )
    declare_min_match_count = DeclareLaunchArgument(
        'min_match_count',
        default_value='10',
        description='Ratio-test matches required before RANSAC (#2).',
    )
    declare_square_resize = DeclareLaunchArgument(
        'square_resize',
        default_value='false',
        description='Anamorphic squash to square for SIFT (#4). false = '
                    'aspect-preserving (recommended).',
    )

    # ── Localization: ORB-SLAM3 RGBD odometry + ArduPilot relay ─────────────
    #    Forwards `target` so it picks the sim/hw camera, calib and topics.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ascend_localization'),
                'launch',
                'localization.launch.py',
            ])
        ),
        launch_arguments={'target': target}.items(),
    )

    # ── Mission-control stack (FSM + survey planner + monitor) ──────────────
    #    Forwards `target` so the FSM runs in sim_mode / use_sim_time to match.
    mission_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ascend_mission_control'),
                'launch',
                'sim_gazebo.launch.py',
            ])
        ),
        launch_arguments={'target': target}.items(),
    )

    # ── Vision: SIFT matcher feeding the FSM ────────────────────────────────
    #    Forwards `target` so it picks the sim/hw color topic automatically.
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ascend_vision'),
                'launch',
                'vision.launch.py',
            ])
        ),
        launch_arguments={
            'target': target,
            'image_topic': image_topic,
            'seed_images_dir': seed_images_dir,
            'process_max_dim': process_max_dim,
            'min_inliers': min_inliers,
            'min_match_count': min_match_count,
            'square_resize': square_resize,
        }.items(),
    )

    return LaunchDescription([
        declare_target,
        declare_image_topic,
        declare_seed_dir,
        declare_process_max_dim,
        declare_min_inliers,
        declare_min_match_count,
        declare_square_resize,
        localization,
        mission_control,
        vision,
    ])
