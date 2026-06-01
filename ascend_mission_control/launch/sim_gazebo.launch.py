"""
launch/sim_gazebo.launch.py

Full simulation launch: Gazebo + ArduPilot SITL + uXRCE-DDS + ORB-SLAM3 + ASCEND stack.

Prerequisites (run before this launch):
  Terminal A — ArduPilot SITL:
    cd ~/ardu_ws && ros2 launch ardupilot_sitl sitl.launch.py

  Terminal B — Micro-XRCE-DDS Agent:
    MicroXRCEAgent udp4 -p 2019

  Terminal C — Gazebo:
    gz sim -r ~/ardu_ws/src/ardupilot_gazebo/worlds/iris_arena_ascend.sdf

  Terminal D — This launch:
    ros2 launch ascend_mission_control sim_gazebo.launch.py

  Terminal E — Start command (when all nodes are ready):
    ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('ascend_mission_control')
    params_file = os.path.join(pkg, 'config', 'mission_params.yaml')

    altitude_arg = DeclareLaunchArgument(
        'survey_altitude',
        default_value='3.0',
        description='Survey altitude in meters (2–6m)'
    )
    survey_altitude = LaunchConfiguration('survey_altitude')

    # ── ASCEND FSM ────────────────────────────────────────────────────────
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
        ]
    )

    # ── Survey Planner ───────────────────────────────────────────────────
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

    # ── Mission Monitor ──────────────────────────────────────────────────
    monitor_node = Node(
        package='ascend_mission_control',
        executable='mission_monitor_node',
        name='mission_monitor_node',
        output='screen',
        parameters=[params_file]
    )

    # ── SLAM Bridge (from ascend_localization package) ───────────────────
    slam_bridge_node = Node(
        package='ascend_localization',
        executable='slam_bridge_node',
        name='slam_bridge_node',
        output='screen',
        parameters=[
            {'home_frame_topic': '/ap/pose/filtered'}
        ]
    )

    return LaunchDescription([
        altitude_arg,
        LogInfo(msg='Starting ASCEND Gazebo simulation stack...'),
        LogInfo(msg='Ensure SITL + DDS Agent + Gazebo are already running.'),
        fsm_node,
        survey_planner_node,
        monitor_node,
        # Delay slam bridge slightly to let SITL connect
        TimerAction(
            period=3.0,
            actions=[slam_bridge_node]
        ),
        LogInfo(msg='Stack ready. Send start command when position is stable.'),
    ])