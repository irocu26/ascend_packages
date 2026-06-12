from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('ascend_localization')
    rviz_config = os.path.join(pkg_dir, 'config', 'slam_comparison.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Gazebo'
        ),

        # ── RGBD SLAM node ────────────────────────────────────────────────
        Node(
            package='ascend_localization',
            executable='rgbd_slam_node',
            name='rgbd_slam_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # ── RViz ─────────────────────────────────────────────────────────
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config],
    output='screen'
),
    ])