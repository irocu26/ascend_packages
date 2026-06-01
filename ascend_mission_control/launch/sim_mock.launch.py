"""
launch/sim_mock.launch.py

Launch the full ASCEND mission stack with mock sensors.
Use this to test the FSM WITHOUT Gazebo or SLAM.

Usage:
  ros2 launch ascend_mission_control sim_mock.launch.py
  # Then in another terminal:
  ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once

Optional overrides:
  ros2 launch ascend_mission_control sim_mock.launch.py scenario:=low_battery_rtl
  ros2 launch ascend_mission_control sim_mock.launch.py survey_altitude:=4.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('ascend_mission_control')
    params_file = os.path.join(pkg, 'config', 'mission_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='normal',
        description='Test scenario: normal | low_battery_rtl | multi_sortie | no_match'
    )
    altitude_arg = DeclareLaunchArgument(
        'survey_altitude',
        default_value='3.0',
        description='Survey altitude in meters (2–6m per rulebook)'
    )

    scenario       = LaunchConfiguration('scenario')
    survey_altitude = LaunchConfiguration('survey_altitude')

    # ── Nodes ─────────────────────────────────────────────────────────────
    fsm_node = Node(
        package='ascend_mission_control',
        executable='fsm_node',
        name='ascend_fsm_node',
        output='screen',
        parameters=[
            params_file,
            {
                'sim_mode':       True,
                'survey_altitude': survey_altitude,
            }
        ],
        remappings=[]
    )

    survey_planner_node = Node(
        package='ascend_mission_control',
        executable='survey_planner_node',
        name='survey_planner_node',
        output='screen',
        parameters=[
            params_file,
            {'survey_altitude': survey_altitude}
        ]
    )

    monitor_node = Node(
        package='ascend_mission_control',
        executable='mission_monitor_node',
        name='mission_monitor_node',
        output='screen',
        parameters=[params_file]
    )

    mock_node = Node(
        package='ascend_mission_control',
        executable='mock_publisher_node',
        name='mock_publisher_node',
        output='screen',
        parameters=[
            params_file,
            {'scenario': scenario}
        ]
    )

    return LaunchDescription([
        scenario_arg,
        altitude_arg,
        LogInfo(msg='Starting ASCEND mock simulation stack...'),
        fsm_node,
        survey_planner_node,
        monitor_node,
        mock_node,
        LogInfo(msg='All nodes launched. Send start command:'),
        LogInfo(msg="  ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once"),
    ])