#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

#include <sophus/se3.hpp>

#include <iostream>

#include <Eigen/Dense>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM, const std::string& settingsFile)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    R_cam_to_enu_ = LoadMountRotation(settingsFile);

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/depth");
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    ap_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/orbslam/tf_to_ap", 10);
    track_state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/orbslam/tracking_state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

Eigen::Matrix3f RgbdSlamNode::LoadMountRotation(const std::string& settingsFile)
{
    // Default: standard optical -> FLU body for a FORWARD-facing camera.
    Eigen::Matrix3f R;
    R << 0,  0, 1,
        -1,  0, 0,
         0, -1, 0;

    cv::FileStorage fs(settingsFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_WARN(this->get_logger(),
            "Could not open settings '%s' for Mount.R; using forward-camera default.",
            settingsFile.c_str());
        return R;
    }

    cv::Mat m;
    fs["Mount.R"] >> m;
    if (m.empty() || m.rows != 3 || m.cols != 3) {
        RCLCPP_WARN(this->get_logger(),
            "No valid 3x3 Mount.R in config; using forward-camera default.");
        return R;
    }

    cv::Mat mf;
    m.convertTo(mf, CV_32F);
    Eigen::Matrix3f Rin;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Rin(i, j) = mf.at<float>(i, j);

    // Sanity-check it's a proper rotation (det ~ +1); warn but still use it.
    float det = Rin.determinant();
    if (std::abs(det - 1.0f) > 1e-2f) {
        RCLCPP_WARN(this->get_logger(),
            "Mount.R determinant = %.4f (expected +1); check the matrix - it may not be a valid rotation.",
            det);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded Mount.R (optical->FLU) from config.");
    return Rin;
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    // // Print translation
    // Eigen::Vector3f t = Tcw.translation();
    // std::cout << "Pose -> x: " << t.x()
    //         << " y: " << t.y()
    //         << " z: " << t.z() << std::endl;

    // // Print quaternion
    // Eigen::Quaternionf q(Tcw.rotationMatrix());
    // std::cout << "Quat -> x: " << q.x()
    //         << " y: " << q.y()
    //         << " z: " << q.z()
    //         << " w: " << q.w() << std::endl;

     int tracking_state = m_SLAM->GetTrackingState();
     std::cout<<"tracking_state: "<<tracking_state<<std::endl;

    // Publish tracking state every frame (incl. invalid-pose frames below) so an
    // orchestration script can watch for OK (==2) to switch ALT_HOLD -> GUIDED.
    {
        std_msgs::msg::Int32 ts_msg;
        ts_msg.data = tracking_state;
        track_state_pub_->publish(ts_msg);
    }

    if (Tcw.matrix().isZero(0)) {
        RCLCPP_WARN(this->get_logger(), "Invalid pose from ORB-SLAM3");
        return;
    }

    // 2. Invert to get camera-in-world
    Sophus::SE3f Twc = Tcw.inverse();

    // --- Compute delta and accumulate ---
    if (!has_prev_pose_) {
        prev_pose_ = Twc;
        has_prev_pose_ = true;
    }

    // Delta between frames
    Sophus::SE3f delta = prev_pose_.inverse() * Twc;
    accumulated_pose_ = accumulated_pose_ * delta;
    prev_pose_ = Twc;

    // 3. Extract translation & rotation
    Eigen::Vector3f t_slam = accumulated_pose_.translation();
    Eigen::Matrix3f R_slam = accumulated_pose_.rotationMatrix();

    // 4. Axis conversion: ORB optical (X=right,Y=down,Z=forward) -> body FLU,
    //    using the per-target mount rotation loaded from the settings yaml.
    Eigen::Vector3f t_ros = R_cam_to_enu_ * t_slam;
    Eigen::Matrix3f R_ros = R_cam_to_enu_ * R_slam * R_cam_to_enu_.transpose();
    

    // 5. Convert to quaternion
    Eigen::Quaternionf q_ros(R_ros);
    q_ros.normalize();

    // 6. Publish Odometry
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "orbslam_odom";
    odom_msg.child_frame_id = "orbslam_body";

    odom_msg.pose.pose.position.x = t_ros.x();
    odom_msg.pose.pose.position.y = t_ros.y();
    odom_msg.pose.pose.position.z = t_ros.z();

    odom_msg.pose.pose.orientation.x = q_ros.x();
    odom_msg.pose.pose.orientation.y = q_ros.y();
    odom_msg.pose.pose.orientation.z = q_ros.z();
    odom_msg.pose.pose.orientation.w = q_ros.w();

    if(tracking_state>1){
        odom_pub_->publish(odom_msg);
    }

    // 7. Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = odom_msg.child_frame_id;

    tf_msg.transform.translation.x = t_ros.x();
    tf_msg.transform.translation.y = t_ros.y();
    tf_msg.transform.translation.z = t_ros.z();

    tf_msg.transform.rotation.x = q_ros.x();
    tf_msg.transform.rotation.y = q_ros.y();
    tf_msg.transform.rotation.z = q_ros.z();
    tf_msg.transform.rotation.w = q_ros.w();

    tf_broadcaster_->sendTransform(tf_msg);

    // 8. Publish the same pose on a dedicated topic for the ArduPilot relay.
    //    Gated on good tracking (state > 1) so the relay only captures its
    //    AP odom->base_link offset once SLAM is actually initialized.
    if(tracking_state>1){
        ap_tf_pub_->publish(tf_msg);
    }
}
