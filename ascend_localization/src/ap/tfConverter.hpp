#ifndef TfC_NODE_HPP
#define TfC_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <optional>

class SlamRelayNode : public rclcpp::Node
{
public:
    SlamRelayNode();

private:
    void slam_tf_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    
    void ap_time_callback(const builtin_interfaces::msg::Time::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr slam_sub_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr ap_time_sub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr ap_pub_;

    // Latest ArduPilot clock (epoch). AP only accepts /ap/tf stamped in its own
    // clock; the SLAM stamp is sim time (~1.78e9 s off), so we re-stamp with this.
    std::optional<builtin_interfaces::msg::Time> ap_time_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Debug broadcaster: publishes the converted pose under tfConvert_odom ->
    // tfConvert_body so the result can be compared in RViz before/while it is
    // also sent to ArduPilot.
    std::unique_ptr<tf2_ros::TransformBroadcaster> debug_broadcaster_;

    bool is_aligned_;
    tf2::Transform t_offset_;

    // Visual-scale correction applied to the SLAM translation (rotation unscaled).
    double scale_;

    // Frame whose world pose is captured as the init offset+rotation (the camera
    // frame, e.g. "pitch_link" for the gimbaled rgbd setup).
    std::string offset_frame_;
};

#endif