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

    // 3. TF Listener to grab the initial GPS/EKF offset from ArduPilot
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. Debug broadcaster for tfConvert_odom -> tfConvert_body
    debug_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 5. Visual-scale correction. Default 0.5 (monocular-style); set to 1.0 for
    //    metric stereo / stereo-inertial via the "scale" ROS parameter.
    scale_ = this->declare_parameter<double>("scale", 1);

    // 6. Frame to align against at init. The SLAM node now outputs true ENU
    //    (downward-mount corrected there), so we align to the level body frame.
    //    Do NOT use the camera frame here or the mount rotation is double-applied.
    offset_frame_ = this->declare_parameter<std::string>("offset_frame", "base_link");

    RCLCPP_INFO(this->get_logger(),
        "SLAM Relay Initialized (scale=%.3f, offset_frame=%s). Waiting for SLAM data...",
        scale_, offset_frame_.c_str());
}

void SlamRelayNode::slam_tf_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    // --- STATE 1: ALIGNMENT ---
    // The trigger is the FIRST SLAM tf. ArduPilot's EKF is already publishing
    // odom->base_link continuously, so at this instant we simply snapshot its
    // latest pose and freeze it as our permanent origin offset.
    if (!is_aligned_) {
        try {
            // Capture odom -> camera-frame: this freezes BOTH the origin offset
            // and the full camera orientation (gimbal pitch included), so SLAM's
            // camera-frame motion is mapped into the correct world axes.
            geometry_msgs::msg::TransformStamped offset_msg = tf_buffer_->lookupTransform(
                "odom", offset_frame_, tf2::TimePointZero);

            tf2::fromMsg(offset_msg.transform, t_offset_);
            is_aligned_ = true;
            RCLCPP_INFO(this->get_logger(),
                "First SLAM tf received. Captured odom->%s offset+rotation. Injecting SLAM into ArduPilot.",
                offset_frame_.c_str());
        } catch (const tf2::TransformException & ex) {
            // ArduPilot should always have a transform ready; if not, skip this
            // frame and try again on the next SLAM tf rather than aligning to a stale origin.
            RCLCPP_ERROR(this->get_logger(),
                "SLAM tf arrived but odom->%s lookup failed: %s", offset_frame_.c_str(), ex.what());
            return;
        }
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamRelayNode>());
    rclcpp::shutdown();
    return 0;
}