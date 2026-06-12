#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>

#include <System.h>
#include "ImuTypes.h"

#include <deque>
#include <fstream>
#include <mutex>

using std::placeholders::_1;

class SlamRosNode : public rclcpp::Node
{
public:

    SlamRosNode()
    : Node("slam_ros_node")
    {
        std::string vocab =
            "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt";

        std::string settings =
            "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Examples/Monocular-Inertial/Webcam_IMU.yaml";

        slam_ =
            std::make_unique<ORB_SLAM3::System>(
                vocab,
                settings,
                ORB_SLAM3::System::IMU_MONOCULAR,
                true);

        pose_file_.open("slam_coordinates.txt");

        pose_file_
            << "#timestamp x y z\n";

        imu_sub_ =
            create_subscription<
                sensor_msgs::msg::Imu>(
                "/imu",
                1000,
                std::bind(
                    &SlamRosNode::imuCallback,
                    this,
                    _1));

        image_sub_ =
            create_subscription<
                sensor_msgs::msg::Image>(
                "/rgbd_camera/image",
                10,
                std::bind(
                    &SlamRosNode::imageCallback,
                    this,
                    _1));

        RCLCPP_INFO(
            get_logger(),
            "ORB-SLAM3 node started");
    }

    ~SlamRosNode()
    {
        slam_->Shutdown();
        pose_file_.close();
    }

private:

    void imuCallback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        double t =
            msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;

        imu_buffer_.push_back(
            ORB_SLAM3::IMU::Point(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
                t));
    }

    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        
        cv::Mat image;
        try
        {
            image = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        }
        catch(cv_bridge::Exception &e)
        {
            cerr << "cv_bridge: " << e.what() << endl;
            return;
        }

        double tframe =
            msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;

        std::vector<ORB_SLAM3::IMU::Point>
            imu_measurements;

        {
            std::lock_guard<std::mutex>
                lock(imu_mutex_);

            // FIX 2: drain stale IMU data from before the last frame
            while (!imu_buffer_.empty() &&
                   imu_buffer_.front().t < last_image_t_)
            {
                imu_buffer_.pop_front();
            }

            // collect all IMU samples up to this frame
            while (
                !imu_buffer_.empty()
                &&
                imu_buffer_.front().t <= tframe)
            {
                imu_measurements.push_back(
                    imu_buffer_.front());

                imu_buffer_.pop_front();
            }
        }

        // FIX 2: update last image timestamp
        last_image_t_ = tframe;

        Sophus::SE3f pose =
            slam_->TrackMonocular(
                image,
                tframe,
                imu_measurements);

        // FIX 1: guard pose access — only log when tracking is valid
        if (slam_->GetTrackingState() == ORB_SLAM3::Tracking::OK
            && !pose.matrix().hasNaN())
        {
            Eigen::Vector3f t =
                pose.translation();

            pose_file_
                << tframe << " "
                << t.x() << " "
                << t.y() << " "
                << t.z() << "\n";

            pose_file_.flush();

            RCLCPP_INFO(
                get_logger(),
                "x=%f y=%f z=%f",
                t.x(),
                t.y(),
                t.z());
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Tracking not OK, skipping pose.");
        }
    }

private:

    std::unique_ptr<ORB_SLAM3::System>
        slam_;

    rclcpp::Subscription<
        sensor_msgs::msg::Imu>::SharedPtr
        imu_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr
        image_sub_;

    std::deque<
        ORB_SLAM3::IMU::Point>
        imu_buffer_;

    std::mutex imu_mutex_;

    std::ofstream pose_file_;

    double last_image_t_ = 0.0;  // FIX 2: track last image timestamp
};

int main(
    int argc,
    char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<SlamRosNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}