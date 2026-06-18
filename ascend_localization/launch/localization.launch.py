"""
ascend_localization/launch/localization.launch.py

Single-parameter SLAM bring-up. The SAME high-level pipeline runs in Gazebo
simulation and on the Raspberry Pi 5 + RealSense hardware; only the camera
source, calibration file, image topics and clock differ. Those differences are
captured by the `target` (+ `type`) argument and the per-target settings YAML
(the rgbd node reads its mount rotation from `Mount.R` in that YAML).

This is a LOCAL launch file: it brings up only this package's nodes. The master
launch (ascend_bringup/ascend.launch.py) includes it and forwards `target`.

Targets
-------
    target:=sim                 Gazebo SITL (default)
    target:=hw                  Legacy alias for D435i color RGBD
    target:=d435i  type:=n      D435i  COLOR  RGBD   (rolling-shutter RGB + depth->color)
    target:=d435i  type:=i      D435i  INFRA  RGBD   (global-shutter left IR + raw depth)
    target:=d455   type:=n      D455   COLOR  RGBD   (GLOBAL-shutter RGB + depth->color)

`type` selects the stream on a multi-stream camera: 'n' = normal/color,
'i' = infra/IR. It is ignored for sim/hw.

Motion-blur / exposure controls (RealSense driver settings -- they govern blur,
NOT ORB-SLAM). All are launch args so you can A/B test on the bench vs. with
motors running:
    auto_exposure:=true|false   manual exposure only takes effect when false
    exposure:=<microseconds>    shorter -> less motion blur (needs more light/gain)
    gain:=<int>                 raise to compensate for short exposure
    fps:=<int>                  stream frame rate
    width:=<int> height:=<int>  stream resolution (must match the calib YAML intrinsics)
    emitter_enabled:=0|1|2      IR projector: 0=off (clean IR for features),
                                1=on (better depth, dots pollute IR), 2=auto.
                                Only meaningful for type:=i.

The ORB vocabulary ships with ORB-SLAM3 and is read from the install:
    export ORB_SLAM3_ROOT=/path/to/ORB_SLAM3   (defaults to ~/ardu_ws/ORB_SLAM3)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Per-target profile. For multi-stream cameras (d435i/d455) topic + calib are
# chosen by `type` ('n' color, 'i' infra) inside launch_setup via the *_n/*_i keys.
PROFILES = {
    'sim': {
        'calib_n': 'gazebo_rgbd.yaml',
        'rgb_topic_n': '/rgbd_camera/image',
        'depth_topic_n': '/rgbd_camera/depth_image',
        'use_sim_time': True,
        'start_realsense': False,
        'camera_frame': None,          # Gazebo publishes the camera TF itself
    },
    # Legacy alias: D435i color RGBD (kept so existing bringup callers don't break)
    'hw': {
        'calib_n': 'RealSense_D435i.yaml',
        'rgb_topic_n': '/camera/camera/color/image_raw',
        'depth_topic_n': '/camera/camera/aligned_depth_to_color/image_raw',
        'use_sim_time': False,
        'start_realsense': True,
        'camera_frame': 'camera_link',
    },
    'd435i': {
        # normal/color stream (rolling shutter)
        'calib_n': 'RealSense_D435i.yaml',
        'rgb_topic_n': '/camera/camera/color/image_raw',
        'depth_topic_n': '/camera/camera/aligned_depth_to_color/image_raw',
        # infra/IR stream (global shutter); depth is native to the left-IR frame
        # -> use raw depth, no align-to-color.
        'calib_i': 'RealSense_D435i_infra.yaml',
        'rgb_topic_i': '/camera/camera/infra1/image_rect_raw',
        'depth_topic_i': '/camera/camera/depth/image_rect_raw',
        'use_sim_time': False,
        'start_realsense': True,
        'camera_frame': 'camera_link',
    },
    'd455': {
        # color stream is GLOBAL SHUTTER on the D455 (OV9782)
        'calib_n': 'RealSense_D455.yaml',
        'rgb_topic_n': '/camera/camera/color/image_raw',
        'depth_topic_n': '/camera/camera/aligned_depth_to_color/image_raw',
        'use_sim_time': False,
        'start_realsense': True,
        'camera_frame': 'camera_link',
    },
}


def _bool(s):
    return str(s).lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    target = LaunchConfiguration('target').perform(context)
    stype = LaunchConfiguration('type').perform(context).lower()

    if target not in PROFILES:
        raise RuntimeError(
            f"target must be one of {list(PROFILES)}, got '{target}'")
    if stype not in ('n', 'i'):
        raise RuntimeError(f"type must be 'n' (color) or 'i' (infra), got '{stype}'")
    p = PROFILES[target]

    use_infra = (stype == 'i')
    if use_infra and 'calib_i' not in p:
        raise RuntimeError(
            f"target '{target}' has no infra (type:=i) profile; use type:=n")

    suffix = 'i' if use_infra else 'n'
    calib_file = p[f'calib_{suffix}']
    rgb_topic = p[f'rgb_topic_{suffix}']
    depth_topic = p[f'depth_topic_{suffix}']

    # Motion-blur / exposure controls
    auto_exposure = _bool(LaunchConfiguration('auto_exposure').perform(context))
    exposure = int(LaunchConfiguration('exposure').perform(context))
    gain = int(LaunchConfiguration('gain').perform(context))
    fps = int(LaunchConfiguration('fps').perform(context))
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    emitter_enabled = int(LaunchConfiguration('emitter_enabled').perform(context))

    pkg_share = get_package_share_directory('ascend_localization')
    calib = os.path.join(pkg_share, 'config', 'rgbd', calib_file)

    # Vocabulary lives with the ORB-SLAM3 install (portable via env var,
    # mirrors the ORB_SLAM3_ROOT logic in CMakeLists.txt).
    orb_root = os.environ.get(
        'ORB_SLAM3_ROOT', os.path.expanduser('~/ardu_ws/ORB_SLAM3'))
    vocab = os.path.join(orb_root, 'Vocabulary', 'ORBvoc.txt')

    use_sim_time = {'use_sim_time': p['use_sim_time']}
    nodes = []

    # ── Hardware camera driver (Gazebo provides images in sim) ───────────────
    if p['start_realsense']:
        profile = f'{width}x{height}x{fps}'
        cam_params = {

            'enable_sync': True,                # time-sync streams
            # ---- depth module exposure (drives motion blur on the IR/depth) ----
            'depth_module.depth_profile': profile,
            # 'depth_module.enable_auto_exposure': auto_exposure,
            # 'depth_module.exposure': exposure,
            # 'depth_module.gain': gain,
            # 'depth_module.emitter_enabled': emitter_enabled,
        }
        if use_infra:
            # Infra path: depth is native to the left-IR frame; feed raw depth +
            # IR image, do NOT align depth to color, and turn on the IR stream.
            cam_params.update({
                'align_depth.enable': False,
                'enable_infra1': True,
                'depth_module.infra_profile': profile,
            })
        else:
            # Color path: register depth to color and drive the color exposure.
            # NOTE: depth MUST run at the depth module's NATIVE 848x480 here. On the
            # D435i, depth@640x480 + color streaming = 0 depth frames (USB/firmware
            # quirk); depth@848x480 + color streams fine. align_depth resamples the
            # 848x480 depth onto the 640x480 color grid, so the SLAM intrinsics stay
            # 640x480. (Infra path keeps 640 above -- it works there, no color stream.)
            cam_params.update({
                'align_depth.enable': True,
                'depth_module.depth_profile': '848x480x' + str(fps),
                'rgb_camera.color_profile': profile,
                # 'rgb_camera.enable_auto_exposure': auto_exposure,
                # 'rgb_camera.exposure': exposure,
                # 'rgb_camera.gain': gain,
            })

        nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[cam_params],
        ))

    # ── ORB-SLAM3 RGBD visual odometry ───────────────────────────────────────
    #    argv: <vocabulary> <settings.yaml> <doRectify> <CLAHE>
    nodes.append(Node(
        package='ascend_localization',
        executable='rgbd',
        name='rgbd',
        output='screen',
        arguments=[vocab, calib, 'false', 'true'],
        remappings=[
            ('/camera/rgb', rgb_topic),
            ('/camera/depth', depth_topic),
        ],
        parameters=[use_sim_time],
    ))

    # ── SLAM -> ArduPilot relay ──────────────────────────────────────────────
    nodes.append(Node(
        package='ascend_localization',
        executable='slam_to_ap',
        name='slam_to_ap',
        output='screen',
        parameters=[use_sim_time],
    ))

    # ── EKF source manager ───────────────────────────────────────────────────
    nodes.append(Node(
        package='ascend_localization',
        executable='ekf_source_manager',
        name='ekf_source_manager',
        output='screen',
        parameters=[use_sim_time],
    ))

    # ── Static TF base_link -> camera (hardware only) ────────────────────────
    if p['camera_frame'] is not None:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', p['camera_frame']],
            parameters=[use_sim_time],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target', default_value='sim',
            description="Camera target: 'sim', 'hw', 'd435i', 'd455'."),
        DeclareLaunchArgument(
            'type', default_value='n',
            description="Stream: 'n' color/normal, 'i' infra/IR (d435i only)."),
        # ── motion-blur / exposure controls ──
        DeclareLaunchArgument(
            'auto_exposure', default_value='true',
            description="Auto exposure; manual exposure applies only when false."),
        DeclareLaunchArgument(
            'exposure', default_value='8500',
            description="Manual exposure in microseconds (shorter = less blur)."),
        DeclareLaunchArgument(
            'gain', default_value='16',
            description="Sensor gain (raise to compensate short exposure)."),
        DeclareLaunchArgument(
            'fps', default_value='30',
            description="Stream frame rate."),
        DeclareLaunchArgument(
            'width', default_value='640',
            description="Stream width (must match calib YAML)."),
        DeclareLaunchArgument(
            'height', default_value='480',
            description="Stream height (must match calib YAML)."),
        DeclareLaunchArgument(
            'emitter_enabled', default_value='0',
            description="IR projector: 0=off, 1=on, 2=auto (infra only)."),
        OpaqueFunction(function=launch_setup),
    ])
