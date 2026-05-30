#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class DebugCameraNode : public rclcpp::Node
{
public:
    DebugCameraNode()
    : Node("debug_camera_node"),
      frame_count_(0),
      last_time_(this->now())
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/image",
            rclcpp::SensorDataQoS(),
            std::bind(&DebugCameraNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Debug camera node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /rgbd_camera/image");
        RCLCPP_INFO(this->get_logger(),
    "Waiting for camera frames...");
        RCLCPP_INFO(this->get_logger(), "Press ESC in the image window to exit");
    }

private:

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   RCLCPP_INFO(this->get_logger(),
        "FRAME RECEIVED");
        frame_count_++;

        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch(cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // compute fps every 30 frames
        if(frame_count_ % 30 == 0)
        {
            auto now = this->now();
            double dt = (now - last_time_).seconds();
            double fps = 30.0 / dt;
            last_time_ = now;

            RCLCPP_INFO(this->get_logger(),
                "[CAMERA] frame=%d  size=%dx%d  fps=%.1f",
                frame_count_,
                frame.cols,
                frame.rows,
                fps
            );
        }

        cv::imshow("Gazebo Camera Feed", frame);

        char key = cv::waitKey(1);
        if(key == 27)   // ESC
        {
            RCLCPP_INFO(this->get_logger(), "ESC pressed — shutting down");
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    int frame_count_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugCameraNode>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}


