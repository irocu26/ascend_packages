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

# ADD THIS near the top, after all find_package() calls
set(ORBSLAM3_ROOT /home/saakshi-v/orb-slam3-root/ORB-SLAM3)
find_package(Pangolin REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(
  include

  /home/saakshi-v/orb-slam3-root/ORB-SLAM3
  /home/saakshi-v/orb-slam3-root/ORB-SLAM3/include
  /home/saakshi-v/orb-slam3-root/ORB-SLAM3/include/CameraModels
  /home/saakshi-v/orb-slam3-root/ORB-SLAM3/Thirdparty/Sophus

  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)



# --- debug_camera_node ---
add_executable(debug_camera_node src/debug_camera_node.cpp)
target_include_directories(debug_camera_node PRIVATE
  ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(debug_camera_node
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)
ament_target_dependencies(debug_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  nav_msgs
)
install(TARGETS debug_camera_node
  DESTINATION lib/${PROJECT_NAME}
)


# --- slam_ros_node ---
add_executable(slam_ros_node src/slam_ros_node.cpp)
target_include_directories(slam_ros_node PRIVATE
  ${ORBSLAM3_ROOT}
  ${ORBSLAM3_ROOT}/include
  ${ORBSLAM3_ROOT}/include/CameraModels
  ${ORBSLAM3_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(slam_ros_node
  ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)
ament_target_dependencies(slam_ros_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
)
install(TARGETS slam_ros_node
  DESTINATION lib/${PROJECT_NAME}
)
# --- slam_ros_node2 ---
add_executable(slam_ros_node2 src/slam_ros_node2.cpp)
target_include_directories(slam_ros_node2 PRIVATE
  ${ORBSLAM3_ROOT}
  ${ORBSLAM3_ROOT}/include
  ${ORBSLAM3_ROOT}/include/CameraModels
  ${ORBSLAM3_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(slam_ros_node2
  ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)
ament_target_dependencies(slam_ros_node2
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
)
install(TARGETS slam_ros_node2
  DESTINATION lib/${PROJECT_NAME}
)

# --- slam_ros_node3 ---
add_executable(slam_ros_node3 src/slam_ros_node3.cpp)

target_include_directories(slam_ros_node3 PRIVATE
  ${ORBSLAM3_ROOT}
  ${ORBSLAM3_ROOT}/include
  ${ORBSLAM3_ROOT}/include/CameraModels
  ${ORBSLAM3_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(slam_ros_node3
  ${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Threads::Threads
)

ament_target_dependencies(slam_ros_node3
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
)

install(TARGETS slam_ros_node3
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

ament_package()
