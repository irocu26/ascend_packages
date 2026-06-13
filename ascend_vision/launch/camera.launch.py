# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():

#     return LaunchDescription([
#         Node(
#             package='usb_cam',
#             executable='usb_cam_node_exe',
#             name='usb_cam',
#             parameters=[{
#                 'video_device': '/dev/video0',
#                 'image_width': 640,
#                 'image_height': 480,
#                 'framerate': 30.0,
#                 'pixel_format': 'yuyv'
#             }]
#         )
#     ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Camera device'
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': video_device,
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv'
            }]
        )
    ])