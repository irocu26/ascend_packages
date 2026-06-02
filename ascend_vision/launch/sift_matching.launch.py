from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/rgbd_camera/image',
            description='Topic to subscribe for the video stream'
        ),
        DeclareLaunchArgument(
            'seed_images_dir',
            default_value=os.path.expanduser('~/ardu_ws/seed_images'),
            description='Path to seed images directory'
        ),
        Node(
            package='ascend_vision',
            executable='sift_node',
            name='sift_node',
            parameters=[
                {'seed_images_dir': LaunchConfiguration('seed_images_dir')},
                {'image_topic': LaunchConfiguration('image_topic')},
            ],
            output='screen'
        )
    ])
