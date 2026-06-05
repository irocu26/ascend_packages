"""
ascend_bringup/launch/ascend.launch.py

Top-level ASCEND application launch — brings up the mission-control stack
(FSM + survey planner + monitor) together with the SIFT vision node.

This is the APPLICATION layer. Run the simulation/hardware layer separately
FIRST (Gazebo + ArduPilot SITL + micro-ROS DDS agent), e.g.:

    ros2 launch ardupilot_gz_bringup iris_runway.launch.py

Then launch the application:

    ros2 launch ascend_bringup ascend.launch.py

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
    image_topic = LaunchConfiguration('image_topic')
    seed_images_dir = LaunchConfiguration('seed_images_dir')

    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/rgbd_camera/image',
        description='Camera image topic the SIFT node subscribes to.',
    )
    declare_seed_dir = DeclareLaunchArgument(
        'seed_images_dir',
        default_value=os.path.expanduser('~/ardu_ws/seed_images'),
        description='Directory of seed images to match against.',
    )

    # ── Mission-control stack (FSM + survey planner + monitor) ──────────────
    mission_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ascend_mission_control'),
                'launch',
                'sim_gazebo.launch.py',
            ])
        )
    )

    # ── Vision: SIFT matcher feeding the FSM ────────────────────────────────
    sift_node = Node(
        package='ascend_vision',
        executable='sift_node',
        name='sift_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'seed_images_dir': seed_images_dir,
        }],
    )

    return LaunchDescription([
        declare_image_topic,
        declare_seed_dir,
        mission_control,
        sift_node,
    ])
