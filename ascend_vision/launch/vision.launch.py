"""
ascend_vision/launch/vision.launch.py

LOCAL launch for the perception pipeline (SIFT matcher). Like the other ASCEND
packages it takes a single `target` (sim | hw) and picks the camera color topic
to subscribe to. The master launch (ascend_bringup) includes this and forwards
`target`.

    ros2 launch ascend_vision vision.launch.py target:=sim   # /rgbd_camera/image
    ros2 launch ascend_vision vision.launch.py target:=hw    # /camera/camera/color/image_raw

`image_topic` may be set explicitly to override the per-target default (leave it
empty to let `target` choose).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Color-image topic per target — matches the RGB topic localization consumes.
IMAGE_TOPIC = {
    'sim': '/rgbd_camera/image',
    'hw': '/camera/camera/color/image_raw',
}


def launch_setup(context, *args, **kwargs):
    target = LaunchConfiguration('target').perform(context)
    if target not in IMAGE_TOPIC:
        raise RuntimeError(
            f"target must be one of {list(IMAGE_TOPIC)}, got '{target}'")

    # Explicit override wins; empty string means "derive from target".
    image_topic = LaunchConfiguration('image_topic').perform(context)
    if not image_topic:
        image_topic = IMAGE_TOPIC[target]

    return [Node(
        package='ascend_vision',
        executable='sift_node',
        name='sift_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'seed_images_dir': LaunchConfiguration('seed_images_dir'),
            'use_sim_time': (target == 'sim'),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target',
            default_value='sim',
            description="Run target: 'sim' (Gazebo SITL) or 'hw' (RealSense D435i).",
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='',
            description='Override color image topic. Empty = pick from target.',
        ),
        DeclareLaunchArgument(
            'seed_images_dir',
            default_value=os.path.expanduser('~/ardu_ws/seed_images'),
            description='Directory of seed images to match against.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
