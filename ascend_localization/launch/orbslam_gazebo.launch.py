from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ascend_localization',
            executable='orbslam_node',
            name='orbslam_node',
            output='screen',
            remappings=[
                # Change right side to match your actual Gazebo camera topic
                ('/camera/image', '/camera/image_raw'),
            ],
            parameters=[{
                'use_sim_time': True,   # important for simulation timestamps
            }]
        )
    ])