#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.hpp>

#include <string>
#include <Eigen/Core>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class RgbdSlamNode : public rclcpp::Node
{
public:
    RgbdSlamNode(ORB_SLAM3::System* pSLAM, const std::string& settingsFile);

    ~RgbdSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    // Load the camera-mount rotation (optical -> FLU body) from the settings yaml
    // key "Mount.R" (3x3). Falls back to the forward-camera default if absent.
    Eigen::Matrix3f LoadMountRotation(const std::string& settingsFile);
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Dedicated TransformStamped publisher feeding the AP relay (tfConverter).
    // Kept separate from the TF broadcaster: the broadcaster feeds the TF tree/RViz,
    // this topic feeds the ArduPilot relay.
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr ap_tf_pub_;

    // ORB-SLAM3 tracking state (2 = OK) so an orchestration script can switch
    // ALT_HOLD -> GUIDED once tracking is healthy.
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr track_state_pub_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    
    ORB_SLAM3::System* m_SLAM;

    // Camera-mount rotation: maps ORB optical axes -> body FLU. Loaded once from
    // the per-target settings yaml (Mount.R) so sim/hardware just swap configs.
    Eigen::Matrix3f R_cam_to_enu_;

    // Previous camera pose
    Sophus::SE3f prev_pose_;
    bool has_prev_pose_ = false;

    // Accumulated odometry pose
    Sophus::SE3f accumulated_pose_;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
};

#endif
