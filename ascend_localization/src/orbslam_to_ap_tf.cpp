// orbslam_to_ap_tf.cpp

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OrbslamToApTf : public rclcpp::Node
{
public:
    OrbslamToApTf()
    : Node("orbslam_to_ap_tf")
    {
        ap_tf_pub_ =
            create_publisher<tf2_msgs::msg::TFMessage>(
                "/ap/tf", 10);

        odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/orbslam/odom",
                10,
                std::bind(
                    &OrbslamToApTf::odomCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "ORB-SLAM3 -> ArduPilot TF bridge started");
    }

private:
    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf;

        tf.header = msg->header;
        tf.child_frame_id = msg->child_frame_id;

        tf.transform.translation.x =
            msg->pose.pose.position.x;

        tf.transform.translation.y =
            msg->pose.pose.position.y;

        tf.transform.translation.z =
            msg->pose.pose.position.z;

        tf.transform.rotation =
            msg->pose.pose.orientation;

        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(tf);

        ap_tf_pub_->publish(tf_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr
        ap_tf_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<OrbslamToApTf>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}