cmake_minimum_required(VERSION 3.8)
project(ascend_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)


# ---------------------------------------------------------------------------
# ORB-SLAM3 location is machine-specific (sim workstation vs Raspberry Pi 5).
# Set it once per machine:  export ORB_SLAM3_ROOT=/path/to/ORB_SLAM3
# Falls back to the repo-local build present on this workstation.
# ---------------------------------------------------------------------------
if(DEFINED ENV{ORB_SLAM3_ROOT})
  set(ORBSLAM3_ROOT $ENV{ORB_SLAM3_ROOT})
else()
  set(ORBSLAM3_ROOT $ENV{HOME}/ardu_ws/ORB_SLAM3)
endif()
find_package(Pangolin REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(NOT EXISTS "${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so")
  message(FATAL_ERROR
    "libORB_SLAM3.so not found at ${ORBSLAM3_ROOT}/lib/.\n"
    "Set ORB_SLAM3_ROOT or build ORB-SLAM3 first.")
endif()

include_directories(
  include
  ${ORBSLAM3_ROOT}
  ${ORBSLAM3_ROOT}/include
  ${ORBSLAM3_ROOT}/include/CameraModels
  ${ORBSLAM3_ROOT}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

# add_executable(orbslam_mono_node
#   src/orbslam_mono_node.cpp
# )
# target_include_directories(orbslam_mono_node PUBLIC
#   ${cv_bridge_INCLUDE_DIRS}
# )
# target_link_libraries(orbslam_mono_node
#   ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
# )
# ament_target_dependencies(orbslam_mono_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   cv_bridge
# )
# install(TARGETS
#   orbslam_mono_node
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )


# --- debug_camera_node ---
# add_executable(debug_camera_node src/debug_camera_node.cpp)
# target_include_directories(debug_camera_node PRIVATE
#   ${Pangolin_INCLUDE_DIRS}
# )
# target_link_libraries(debug_camera_node
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
# )
# ament_target_dependencies(debug_camera_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   nav_msgs
# )
# install(TARGETS debug_camera_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # --- debug_slam_node ---
# set(ORBSLAM3_ROOT /home/rpi/orb-slam3-root/ORB-SLAM3)
# find_package(Pangolin REQUIRED)
# add_executable(debug_slam_node src/debug_slam_node.cpp)
# target_include_directories(debug_slam_node PRIVATE
#   ${ORBSLAM3_ROOT}
#   ${ORBSLAM3_ROOT}/include
#   ${ORBSLAM3_ROOT}/include/CameraModels
#   ${ORBSLAM3_ROOT}/Thirdparty/Sophus
#   ${OpenCV_INCLUDE_DIRS}
#   ${Pangolin_INCLUDE_DIRS}        # ADD THIS
# )
# target_link_libraries(debug_slam_node
#   ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}           # ADD THIS
# )
# ament_target_dependencies(debug_slam_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
# )
# install(TARGETS debug_slam_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# --- slam_ros_node ---
# add_executable(slam_ros_node src/slam_ros_node.cpp)
# target_include_directories(slam_ros_node PRIVATE
#   ${ORBSLAM3_ROOT}
#   ${ORBSLAM3_ROOT}/include
#   ${ORBSLAM3_ROOT}/include/CameraModels
#   ${ORBSLAM3_ROOT}/Thirdparty/Sophus
#   ${OpenCV_INCLUDE_DIRS}
#   ${Pangolin_INCLUDE_DIRS}
# )
# target_link_libraries(slam_ros_node
#   ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
# )
# ament_target_dependencies(slam_ros_node
#   rclcpp  
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   cv_bridge
# )
# install(TARGETS slam_ros_node
#   DESTINATION lib/${PROJECT_NAME}
# )
# --- slam_ros_node2 ---
#litski0: DISABLED. slam_ros_node2.cpp calls SLAM.GetAtlas(), a custom accessor
#litski0: that exists only in saakshi-v's modified ORB-SLAM3 fork, NOT in the
#litski0: stock ORB-SLAM3 built on this machine -> compile error. The .cpp is
#litski0: kept intact; re-enable once System exposes GetAtlas() (or port the
#litski0: save_point_cloud() helper to the public GetTrackedMapPoints() API).
# add_executable(slam_ros_node2 src/slam_ros_node2.cpp)
# target_include_directories(slam_ros_node2 PRIVATE
#   ${ORBSLAM3_ROOT}
#   ${ORBSLAM3_ROOT}/include
#   ${ORBSLAM3_ROOT}/include/CameraModels
#   ${ORBSLAM3_ROOT}/Thirdparty/Sophus
#   ${OpenCV_INCLUDE_DIRS}
#   ${Pangolin_INCLUDE_DIRS}
# )
# target_link_libraries(slam_ros_node2
#   ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
# )
# ament_target_dependencies(slam_ros_node2
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   cv_bridge
# )
# install(TARGETS slam_ros_node2
#   DESTINATION lib/${PROJECT_NAME}
# )

# --- slam_ros_node3 ---
# add_executable(slam_ros_node3 src/slam_ros_node3.cpp)

# target_include_directories(slam_ros_node3 PRIVATE
#   ${ORBSLAM3_ROOT}
#   ${ORBSLAM3_ROOT}/include
#   ${ORBSLAM3_ROOT}/include/CameraModels
#   ${ORBSLAM3_ROOT}/Thirdparty/Sophus
#   ${OpenCV_INCLUDE_DIRS}
#   ${Pangolin_INCLUDE_DIRS}
# )

# target_link_libraries(slam_ros_node3
#   ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
#   Threads::Threads
# )

# ament_target_dependencies(slam_ros_node3
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   cv_bridge
# )

# install(TARGETS slam_ros_node3
#   DESTINATION lib/${PROJECT_NAME}
# )
# --- stereo_inertial_slam_node ---
# add_executable(stereo_inertial_slam_node src/stereo_inertial_slam_node.cpp)

# target_include_directories(stereo_inertial_slam_node PRIVATE
#   ${ORBSLAM3_ROOT}
#   ${ORBSLAM3_ROOT}/include
#   ${ORBSLAM3_ROOT}/include/CameraModels
#   ${ORBSLAM3_ROOT}/Thirdparty/Sophus
#   ${OpenCV_INCLUDE_DIRS}
#   ${Pangolin_INCLUDE_DIRS}
# )

# target_link_libraries(stereo_inertial_slam_node
#   ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
#   ${OpenCV_LIBS}
#   ${Pangolin_LIBRARIES}
#   Threads::Threads
# )

# ament_target_dependencies(stereo_inertial_slam_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   cv_bridge
# )

# install(TARGETS stereo_inertial_slam_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# --- rgbd_slam_node ---
add_executable(rgbd_slam_node src/rgbd_slam_node.cpp)

target_include_directories(rgbd_slam_node PRIVATE
  ${ORBSLAM3_ROOT}
  ${ORBSLAM3_ROOT}/include
  ${ORBSLAM3_ROOT}/include/CameraModels
  ${ORBSLAM3_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(rgbd_slam_node
  ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Threads::Threads
)

ament_target_dependencies(rgbd_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf2_ros
  tf2_msgs
)

install(TARGETS rgbd_slam_node
  DESTINATION lib/${PROJECT_NAME})
 

# add_executable(
#   orbslam_to_ap_tf
#   src/orbslam_to_ap_tf.cpp
# )

# ament_target_dependencies(
#   orbslam_to_ap_tf
#   rclcpp
#   nav_msgs
#   tf2_msgs
#   geometry_msgs
# )

# install(
#   TARGETS orbslam_to_ap_tf
#   DESTINATION lib/${PROJECT_NAME}
# )


#litski0: =====================================================================
#litski0: Working RGBD odometry + ArduPilot relay ported from
#litski0: ROS2_ORB-SLAM3_Odometry. These are the nodes the modular sim/hw
#litski0: launch uses. New target names (rgbd / slam_to_ap) so they don't
#litski0: collide with the team's existing rgbd_slam_node / orbslam_to_ap_tf.
#litski0: =====================================================================

#litski0: extra deps needed by the AP relay (not find_package'd above)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

#litski0: rgbd -- ORB-SLAM3 RGBD visual odometry (src/rgbd/)
#litski0:   argv: <vocabulary> <settings.yaml> <doRectify> <CLAHE>
add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
target_include_directories(rgbd PRIVATE
  ${ORBSLAM3_ROOT}
  ${ORBSLAM3_ROOT}/include
  ${ORBSLAM3_ROOT}/include/CameraModels
  ${ORBSLAM3_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(rgbd
  ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Threads::Threads
)
ament_target_dependencies(rgbd
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf2_ros
)
install(TARGETS rgbd
  DESTINATION lib/${PROJECT_NAME}
)

#litski0: slam_to_ap -- relay SLAM pose -> ArduPilot DDS (src/ap/tfConverter.cpp)
#litski0:   pure ROS node, no ORB_SLAM3 / OpenCV dependency
add_executable(slam_to_ap
  src/ap/tfConverter.cpp
)
ament_target_dependencies(slam_to_ap
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  builtin_interfaces
)
install(TARGETS slam_to_ap
  DESTINATION lib/${PROJECT_NAME}
)

#litski0: ekf_source_manager -- watch SLAM tracking state, switch ArduPilot EKF
#litski0:   source set (SLAM <-> optical flow) via /ap/joy aux channel, and
#litski0:   publish /ascend/localization/slam_ok for the FSM. Pure ROS node.
find_package(std_msgs REQUIRED)
add_executable(ekf_source_manager
  src/ekf/ekf_source_manager.cpp
)
ament_target_dependencies(ekf_source_manager
  rclcpp
  std_msgs
  sensor_msgs
)
install(TARGETS ekf_source_manager
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
