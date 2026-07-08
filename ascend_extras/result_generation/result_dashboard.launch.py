"""
result_dashboard.launch.py — start the ASCEND result dashboard alongside the
mission.

The dashboard is a standalone Python process (it is NOT a colcon entry point),
so this launch file runs it with ExecuteProcess. It is display-only and never
commands the vehicle, so it is safe to add to your main bringup or to launch
separately.

    ros2 launch <this file> result_dashboard.launch.py
    # or, with overrides:
    ros2 launch <this file> result_dashboard.launch.py http_port:=9000 team_name:=ASCEND

Then open  http://<base-station-ip>:8080  in a browser.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(here, 'dashboard_node.py')

    http_port = LaunchConfiguration('http_port')
    team_name = LaunchConfiguration('team_name')
    report_dir = LaunchConfiguration('report_dir')
    direct_vision_match = LaunchConfiguration('direct_vision_match')

    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('team_name', default_value='ASCEND'),
        DeclareLaunchArgument('report_dir',
                              default_value=os.path.expanduser('~/ascend_results')),
        DeclareLaunchArgument(
            'direct_vision_match', default_value='false',
            description='Feed SIFT matches straight into the report, bypassing '
                        'the FSM. Enable for manual joystick SIFT testing.'),

        ExecuteProcess(
            cmd=[
                'python3', script,
                '--ros-args',
                '-p', ['http_port:=', http_port],
                '-p', ['team_name:=', team_name],
                '-p', ['report_dir:=', report_dir],
                '-p', ['direct_vision_match:=', direct_vision_match],
            ],
            output='screen',
        ),
    ])
