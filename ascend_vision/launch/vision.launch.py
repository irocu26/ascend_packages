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

# Matching CameraInfo per target, for the feature back-projection (#5).
CAMERA_INFO_TOPIC = {
    'sim': '/rgbd_camera/camera_info',
    'hw': '/camera/camera/color/camera_info',
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
    camera_info_topic = CAMERA_INFO_TOPIC[target]

    # SIFT tuning knobs. Launch args arrive as strings; cast to int so the node
    # receives the integer types it declares (a string param would break the
    # numeric comparisons inside the node).
    process_max_dim = int(LaunchConfiguration('process_max_dim').perform(context))
    min_inliers = int(LaunchConfiguration('min_inliers').perform(context))
    min_match_count = int(LaunchConfiguration('min_match_count').perform(context))
    square_resize = LaunchConfiguration('square_resize').perform(context).lower() \
        in ('true', '1', 'yes')

    return [Node(
        package='ascend_vision',
        executable='sift_node',
        name='sift_node',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'seed_images_dir': LaunchConfiguration('seed_images_dir'),
            'process_max_dim': process_max_dim,
            'min_inliers': min_inliers,
            'min_match_count': min_match_count,
            'square_resize': square_resize,
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
        DeclareLaunchArgument(
            'process_max_dim',
            default_value='0',
            description="Cap scene long-side (px) before SIFT. 0 = full native "
                        "res (default). Only raise above 0 once you confirm the "
                        "feature still spans >~40px after the cap (#3).",
        ),
        DeclareLaunchArgument(
            'min_inliers',
            default_value='10',
            description="RANSAC inliers required to ACCEPT a match (#1 geometric "
                        "verification gate). Lower if real matches fall short at "
                        "low res; keep high enough to reject junk.",
        ),
        DeclareLaunchArgument(
            'min_match_count',
            default_value='10',
            description="Ratio-test matches required before attempting RANSAC "
                        "(#2). Keep >= min_inliers.",
        ),
        DeclareLaunchArgument(
            'square_resize',
            default_value='false',
            description="Anamorphic squash to a square for SIFT (#4). false "
                        "(default) = aspect-preserving, no distortion. true only "
                        "if your references were squashed from 1280x720 AND the "
                        "feature fills the frame.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
