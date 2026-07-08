"""
ascend_extras/manual_sift_test.launch.py

ONE-SHOT bring-up for MANUAL SIFT testing + live result report — no autonomy,
no FSM. Use it to fly the drone by hand (joystick via ArduPilot/QGC) over the
seed images and confirm both SIFT matching and report generation work IRL.

It starts three things:
  1. RealSense D455 colour stream  — realsense2_camera_node (colour only; SIFT
     does not need depth).
  2. ascend_vision SIFT matcher    — vision.launch.py target:=hw, subscribing to
     /camera/camera/color/image_raw + .../camera_info.
  3. Result dashboard + report     — result_dashboard.launch.py with
     direct_vision_match:=true, so a raw SIFT match becomes a report feature
     WITHOUT the FSM in the loop (the FSM only accepts matches while surveying).

Camera intrinsics: the RealSense driver already publishes the D455 factory
intrinsics on .../color/camera_info (sift_node uses those for the feature
bearing). The one value the world-coordinate maths needs separately is the
horizontal FOV, which we derive from
ascend_localization/config/rgbd/RealSense_D455.yaml (Camera.fx / Camera.width)
and forward to the SIFT node — so the report's coordinates use the real D455
geometry, not the D435i default of 69.4 deg.

    ros2 launch ~/ardu_ws/src/ascend_packages/ascend_extras/manual_sift_test.launch.py

Then:
  • put reference images in ~/ardu_ws/seed_images/
  • open  http://localhost:8080  and click "Download Report"
  • (optional)  ros2 run rqt_image_view rqt_image_view /sift_matches

Notes
  • Real arena X/Y in the report needs /ap/pose/filtered (the ArduPilot DDS/MAVROS
    bridge running separately); without it matching still works and coordinates
    read 0,0.
  • The joystick itself is handled by ArduPilot/QGC — nothing to launch here.
"""
import math
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _d455_hfov_deg(default=79.28):
    """Horizontal FOV (deg) from the D455 ORB-SLAM yaml: 2*atan(width / 2*fx).

    That file is OpenCV %YAML:1.0 with custom !!opencv-matrix tags, so we scrape
    fx/width with a regex rather than a YAML parser. Falls back to the nominal
    D455 colour HFOV if the file or fields are missing.
    """
    try:
        share = get_package_share_directory('ascend_localization')
        path = os.path.join(share, 'config', 'rgbd', 'RealSense_D455.yaml')
        with open(path) as f:
            text = f.read()
        fx = float(re.search(r'Camera\.fx:\s*([0-9.]+)', text).group(1))
        width = float(re.search(r'Camera\.width:\s*([0-9.]+)', text).group(1))
        return round(math.degrees(2 * math.atan(width / (2 * fx))), 2)
    except Exception:
        return default


def launch_setup(context, *args, **kwargs):
    width = LaunchConfiguration('width').perform(context)
    height = LaunchConfiguration('height').perform(context)
    fps = LaunchConfiguration('fps').perform(context)
    seed_dir = LaunchConfiguration('seed_images_dir').perform(context)
    http_port = LaunchConfiguration('http_port').perform(context)
    profile = f'{width}x{height}x{fps}'
    fov_h = _d455_hfov_deg()

    here = os.path.dirname(os.path.abspath(__file__))
    dashboard_launch = os.path.join(
        here, 'result_generation', 'result_dashboard.launch.py')
    vision_launch = os.path.join(
        get_package_share_directory('ascend_vision'), 'launch', 'vision.launch.py')

    # 1. RealSense D455 — colour only (SIFT needs no depth). Publishes
    #    /camera/camera/color/image_raw + .../camera_info, which is exactly what
    #    vision.launch.py target:=hw subscribes to.
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_sync': True,
            'enable_color': True,
            'rgb_camera.color_profile': profile,
            'enable_depth': False,
            'align_depth.enable': False,
        }],
    )

    # 2. SIFT matcher (existing local launch), with the D455 FOV forwarded.
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_launch),
        launch_arguments={
            'target': 'hw',
            'seed_images_dir': seed_dir,
            'fov_h': str(fov_h),
        }.items(),
    )

    # 3. Result dashboard + report server, with the manual SIFT->report bridge on.
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dashboard_launch),
        launch_arguments={
            'direct_vision_match': 'true',
            'http_port': http_port,
        }.items(),
    )

    return [
        LogInfo(msg=f'[manual_sift_test] D455 HFOV from calib = {fov_h} deg | '
                    f'camera profile {profile} | dashboard http://localhost:{http_port}'),
        realsense,
        vision,
        dashboard,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'width', default_value='640',
            description='RealSense colour width (match the D455 calib: 640).'),
        DeclareLaunchArgument(
            'height', default_value='480',
            description='RealSense colour height (match the D455 calib: 480).'),
        DeclareLaunchArgument(
            'fps', default_value='30',
            description='RealSense colour frame rate.'),
        DeclareLaunchArgument(
            'seed_images_dir',
            default_value=os.path.expanduser('~/ardu_ws/seed_images'),
            description='Directory of reference images the SIFT node matches against.'),
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='Result-dashboard HTTP port.'),
        OpaqueFunction(function=launch_setup),
    ])
