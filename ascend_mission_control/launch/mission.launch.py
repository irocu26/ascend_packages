
"""
Launch file for ASCEND Mission Control.
Starts:
- Gazebo ROS bridges
- FCU Bridge
- Mission Executor
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory(
        'ascend_mission_control'
    )

    config_dir = os.path.join(
        pkg_dir,
        'config'
    )

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=os.path.join(
            config_dir,
            'mission_params.yaml'
        ),
        description='Path to mission YAML file'
    )

    # ==========================================================
    # ODOMETRY BRIDGE
    # ==========================================================

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        arguments=[
            '/model/iris/odometry'
            '@nav_msgs/msg/Odometry'
            '@gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/iris/odometry', '/odometry')
        ]
    )

    # ==========================================================
    # IMU BRIDGE
    # ==========================================================

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=[
            '/world/map/model/iris/link/imu_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu'
            '@gz.msgs.IMU'
        ],
        remappings=[
            (
                '/world/map/model/iris/link/imu_link/sensor/imu_sensor/imu',
                '/imu'
            )
        ]
    )

    # ==========================================================
    # GPS BRIDGE
    # ==========================================================

    gps_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gps_bridge',
        output='screen',
        arguments=[
            '/world/map/model/iris/link/base_link/sensor/navsat_sensor/navsat'
            '@sensor_msgs/msg/NavSatFix'
            '@gz.msgs.NavSat'
        ],
        remappings=[
            (
                '/world/map/model/iris/link/base_link/sensor/navsat_sensor/navsat',
                '/navsat'
            )
        ]
    )

    # ==========================================================
    # FCU BRIDGE NODE
    # ==========================================================

    fcu_bridge_node = Node(
        package='ascend_mission_control',
        executable='fcu_bridge',
        name='fcu_bridge',
        output='screen',
        parameters=[
            os.path.join(
                config_dir,
                'mission_params.yaml'
            )
        ]
    )

    # ==========================================================
    # MISSION EXECUTOR NODE
    # ==========================================================

    mission_executor_node = Node(
        package='ascend_mission_control',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        parameters=[
            os.path.join(
                config_dir,
                'mission_params.yaml'
            ),
            {
                'mission_file':
                LaunchConfiguration('mission_file')
            }
        ]
    )

    return LaunchDescription([

        mission_file_arg,

        odom_bridge,
        imu_bridge,
        gps_bridge,

        fcu_bridge_node,
        mission_executor_node,
    ])
