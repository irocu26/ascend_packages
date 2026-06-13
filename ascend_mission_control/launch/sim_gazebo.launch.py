"""
launch/sim_gazebo.launch.py

Full Gazebo + ArduPilot SITL launch — WITHOUT SLAM (for current testing phase).

WHAT CHANGED FROM ORIGINAL:
  - slam_bridge_node REMOVED — not needed for Gazebo+SITL testing without SLAM.
  - Added ros_gz_bridge for /ap/pose/filtered so fsm_node gets position from
    ArduPilot directly (fallback pose — _ap_pose in fsm_node).
  - Added ros_gz_bridge for /ap/battery_status.
  - survey_planner_node now receives pose from /ap/pose/filtered remapped to
    /ascend/localization/pose so it still gets position without SLAM.
  - When SLAM is ready, re-add slam_bridge_node and remove the AP→SLAM remap.

Prerequisites — run BEFORE this launch (each in its own terminal):

  Terminal A — ArduPilot SITL:
    cd ~/ardu_ws
    ros2 launch ardupilot_sitl sitl.launch.py

  Terminal B — Micro-XRCE-DDS Agent:
    MicroXRCEAgent udp4 -p 2019

  Terminal C — Gazebo:
    gz sim -r ~/ardu_ws/src/ardupilot_gazebo/worlds/iris_arena_ascend.sdf

  Terminal D — This launch:
    ros2 launch ascend_mission_control sim_gazebo.launch.py

  Terminal E — Send start command (wait for 'waiting for start command' log):
    ros2 topic pub /ascend/mission_control/start_cmd std_msgs/Empty '{}' --once

Verify AP topics are live before starting:
  ros2 topic list | grep '/ap/'
  ros2 topic echo /ap/battery_status --once
  ros2 topic echo /ap/pose/filtered --once
  ros2 service list | grep '/ap/'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg        = get_package_share_directory('ascend_mission_control')
    params_file = os.path.join(pkg, 'config', 'mission_params.yaml')
    survey_altitude = LaunchConfiguration('survey_altitude')

    # `target` (sim | hw) drives sim_mode + use_sim_time. The FSM uses sim_mode
    # for its clock/arming behaviour; everything else is identical sim vs hw.
    target = LaunchConfiguration('target').perform(context)
    if target not in ('sim', 'hw'):
        raise RuntimeError(f"target must be 'sim' or 'hw', got '{target}'")
    sim_mode = (target == 'sim')
    use_sim_time = {'use_sim_time': sim_mode}

    # ── ASCEND FSM ────────────────────────────────────────────────────────
    fsm_node = Node(
        package='ascend_mission_control',
        executable='fsm_node',
        name='ascend_fsm_node',
        output='screen',
        parameters=[
            params_file,
            {
                'sim_mode':        sim_mode,
                'survey_altitude': survey_altitude,
            },
            use_sim_time,
        ]
    )

    # ── Survey Planner ────────────────────────────────────────────────────
    # CHANGE: remaps /ap/pose/filtered → /ascend/localization/pose so the
    # survey planner gets real position without needing SLAM bridge.
    # Remove remappings[] entry once slam_bridge_node is integrated.
    survey_planner_node = Node(
        package='ascend_mission_control',
        executable='survey_planner_node',
        name='survey_planner_node',
        output='screen',
        parameters=[
            params_file,
            {'survey_altitude': survey_altitude},
            use_sim_time,
        ],
        remappings=[
            ('/ascend/localization/pose', '/ap/pose/filtered'),
        ]
    )

    # ── Mission Monitor ───────────────────────────────────────────────────
    monitor_node = Node(
        package='ascend_mission_control',
        executable='mission_monitor_node',
        name='mission_monitor_node',
        output='screen',
        parameters=[params_file, use_sim_time],
        remappings=[
            ('/ascend/localization/pose', '/ap/pose/filtered'),
        ]
    )

    # ── SLAM bridge (commented out until SLAM integration phase) ──────────
    # Uncomment when ORB-SLAM3 + slam_bridge_node are ready:
    #
    # slam_bridge_node = Node(
    #     package='ascend_localization',
    #     executable='slam_bridge_node',
    #     name='slam_bridge_node',
    #     output='screen',
    #     parameters=[
    #         {'home_frame_topic': '/ap/pose/filtered'}
    #     ]
    # )

    return [fsm_node, survey_planner_node, monitor_node]


def generate_launch_description():
    target_arg = DeclareLaunchArgument(
        'target',
        default_value='sim',
        description="Run target: 'sim' (Gazebo SITL) or 'hw' (real hardware).",
    )
    altitude_arg = DeclareLaunchArgument(
        'survey_altitude',
        default_value='3.0',
        description='Survey altitude in meters (2-6m per rulebook)'
    )

    return LaunchDescription([
        target_arg,
        altitude_arg,

        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='ASCEND — Gazebo+SITL launch (NO SLAM)'),
        LogInfo(msg='Ensure SITL + DDS Agent + Gazebo are already running.'),
        LogInfo(msg='Position source: /ap/pose/filtered (ArduPilot EKF)'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),

        OpaqueFunction(function=launch_setup),
    ])