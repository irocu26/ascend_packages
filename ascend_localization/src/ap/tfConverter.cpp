#include "tfConverter.hpp"

SlamRelayNode::SlamRelayNode() : Node("slam_to_ap"), is_aligned_(false)
{
    // 1. Subscribe directly to your ORB-SLAM3 output
    slam_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/orbslam/tf_to_ap", 10, std::bind(&SlamRelayNode::slam_tf_callback, this, std::placeholders::_1));

    // 2. Publish to the ArduPilot DDS intake
    ap_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/ap/tf", 10);

    // 2b. Track ArduPilot's clock. AP runs on epoch time while the SLAM/sim side
    //     runs on sim time (~1.78e9 s apart); AP rejects /ap/tf stamped in sim
    //     time as ancient. AP_DDS publishes /ap/time BEST_EFFORT, so the QoS must
    //     match or no messages arrive.
    auto ap_time_qos = rclcpp::QoS(10).best_effort();
    ap_time_sub_ = this->create_subscription<builtin_interfaces::msg::Time>(
        "/ap/time", ap_time_qos,
        std::bind(&SlamRelayNode::ap_time_callback, this, std::placeholders::_1));

    // 3. Init offset comes from AP's own EKF pose (/ap/pose/filtered), NOT a TF.
    //    AP_DDS publishes this PoseStamped (ENU, body-in-odom) on BOTH sim and
    //    hardware; the odom->base_link TF only existed in sim (Gazebo published
    //    it). Same BEST_EFFORT QoS gotcha as /ap/time.
    auto ap_pose_qos = rclcpp::QoS(10).best_effort();
    ap_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ap/pose/filtered", ap_pose_qos,
        std::bind(&SlamRelayNode::ap_pose_callback, this, std::placeholders::_1));

    // 4. Debug broadcaster for tfConvert_odom -> tfConvert_body
    debug_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 5. Visual-scale correction. 1.0 for metric RGBD / stereo-inertial.
    scale_ = this->declare_parameter<double>("scale", 1);

    RCLCPP_INFO(this->get_logger(),
        "SLAM Relay Initialized (scale=%.3f). Waiting for SLAM + /ap/pose/filtered...",
        scale_);
}

void SlamRelayNode::slam_tf_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    // --- STATE 1: ALIGNMENT ---
    // The trigger is the FIRST SLAM tf. We freeze AP's current EKF body pose
    // (/ap/pose/filtered) as the permanent origin+heading offset, anchoring SLAM's
    // (0,0,0)-at-init to where AP believes the body is.
    if (!is_aligned_) {
        if (!ap_pose_.has_value()) {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "SLAM tf arrived but no /ap/pose/filtered yet - cannot align "
                "(check the topic is up, BEST_EFFORT QoS).");
            return;
        }
        tf2::fromMsg(ap_pose_->pose, t_offset_);
        is_aligned_ = true;
        RCLCPP_INFO(this->get_logger(),
            "First SLAM tf received. Captured AP pose offset. Injecting SLAM into ArduPilot.");
    }

    // --- STATE 2: TRACKING ---
    tf2::Transform t_slam;
    tf2::fromMsg(msg->transform, t_slam);

    // Apply visual-scale correction to the translation only (rotation unscaled).
    t_slam.setOrigin(t_slam.getOrigin() * scale_);

    // Multiply the static offset by the incoming dynamic SLAM matrix
    tf2::Transform t_final = t_offset_ * t_slam;

    // Debug: broadcast the converted pose as tfConvert_odom -> tfConvert_body so it
    // can be inspected/compared in RViz before trusting the /ap/tf injection.
    geometry_msgs::msg::TransformStamped debug_tf;
    debug_tf.header.stamp = msg->header.stamp;
    debug_tf.header.frame_id = "tfConvert_odom";
    debug_tf.child_frame_id = "tfConvert_body";
    debug_tf.transform = tf2::toMsg(t_final);
    debug_broadcaster_->sendTransform(debug_tf);

    // Don't inject until we know AP's clock - a sim-time stamp is ~1.78e9 s in
    // AP's past and the EKF discards it as ancient, so the pose would be ignored.
    if (!ap_time_.has_value()) {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Have SLAM pose but no /ap/time yet - not injecting (check /ap/time is up, BEST_EFFORT QoS).");
        return;
    }

    // Package the single transform exactly as ArduPilot expects it.
    // Stamp with AP's clock (epoch), NOT the SLAM/sim-time stamp.
    geometry_msgs::msg::TransformStamped ap_tf;
    ap_tf.header.stamp = ap_time_.value();
    ap_tf.header.frame_id = "odom";        // Strictly required by ArduPilot
    ap_tf.child_frame_id = "base_link";    // Strictly required by ArduPilot
    ap_tf.transform = tf2::toMsg(t_final);

    // AP_DDS subscribes to /ap/tf as a tf2_msgs/TFMessage (array of transforms),
    // so wrap our single transform in a TFMessage before sending to the DDS agent.
    tf2_msgs::msg::TFMessage ap_msg;
    ap_msg.transforms.push_back(ap_tf);
    ap_pub_->publish(ap_msg);
}

void SlamRelayNode::ap_time_callback(const builtin_interfaces::msg::Time::SharedPtr msg)
{
    ap_time_ = *msg;
}

void SlamRelayNode::ap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    ap_pose_ = *msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamRelayNode>());
    rclcpp::shutdown();
    return 0;
}