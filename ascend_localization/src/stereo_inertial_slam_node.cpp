/*#include <rclcpp/rclcpp.hpp>

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

class StereoInertialSlamNode : public rclcpp::Node
{
public:

    StereoInertialSlamNode()
    : Node("stereo_inertial_slam_node")
    {
        // ── ORB-SLAM3 paths ───────────────────────────────────────────────
        std::string vocab =
            "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt";

        std::string settings =
            "/home/saakshi-v/ardu_ws/src/ascend_packages/ascend_localization/config/Stereo_IMU.yaml";
        //  ↑ Change this to your Stereo-Inertial YAML path

        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocab,
            settings,
            ORB_SLAM3::System::IMU_STEREO,
            true);   // true = enable Pangolin viewer

        pose_file_.open("stereo_inertial_coordinates.txt");
        pose_file_ << "# timestamp tx ty tz\n";

imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu",
    1000,
    std::bind(&StereoInertialSlamNode::imuCallback, this, _1));

left_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera_left/image",
    10,
    std::bind(&StereoInertialSlamNode::leftImageCallback, this, _1));

right_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera_right/image",
    10,
    std::bind(&StereoInertialSlamNode::rightImageCallback, this, _1));

        RCLCPP_INFO(get_logger(), "Stereo-Inertial ORB-SLAM3 node started");
    }

    ~StereoInertialSlamNode()
    {
        slam_->Shutdown();
        pose_file_.close();
    }

private:

    // ── IMU callback ──────────────────────────────────────────────────────
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {   
       RCLCPP_INFO(this->get_logger(), "IMU callback");
        std::lock_guard<std::mutex> lock(imu_mutex_);

        double t = msg->header.stamp.sec +
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

    // ── Left image callback ───────────────────────────────────────────────
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   {
        RCLCPP_INFO(this->get_logger(), "left callback");
        std::lock_guard<std::mutex> lock(left_mutex_);

        try
        {
            left_image_  = cv_bridge::toCvCopy(msg, "rgb8")->image;
            left_stamp_  = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;
            left_ready_  = true;
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "Failed to decode left image");
            return;
        }
    }
        tryTrack();
    }

    // ── Right image callback ──────────────────────────────────────────────
    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
       { RCLCPP_INFO(this->get_logger(), "right callback");
        std::lock_guard<std::mutex> lock(right_mutex_);

        try
        {
            right_image_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
            right_stamp_ = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;
            right_ready_ = true;
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "Failed to decode right image");
            return;
        }}
    
        tryTrack();
    
    }
    // ── Stereo + IMU tracking ─────────────────────────────────────────────
    //
    // Called from both image callbacks.
    // Only runs when both images have arrived and their timestamps are
    // close enough (within MAX_STEREO_DT seconds).
    // ─────────────────────────────────────────────────────────────────────
    void tryTrack()
    {   RCLCPP_INFO(get_logger(), "TRYTRACK ENTRY");
        RCLCPP_INFO(get_logger(), "TRYTRACK 1");

    if (!left_ready_ || !right_ready_)
        return;

    //RCLCPP_INFO(get_logger(), "TRYTRACK 2");

    double dt = std::abs(left_stamp_ - right_stamp_);

    RCLCPP_INFO(get_logger(), "TRYTRACK 3 dt=%f", dt);
        // Maximum allowed timestamp difference between left and right frames
        constexpr double MAX_STEREO_DT = 0.005; // 5 ms

        // Need both images
        if (!left_ready_ || !right_ready_)
            return;

        // Check stereo sync
        if (dt > MAX_STEREO_DT)
        {
            // Drop the older image and wait for a new pair
            if (left_stamp_ < right_stamp_)
                left_ready_ = false;
            else
                right_ready_ = false;

            RCLCPP_WARN(get_logger(),
                "Stereo timestamp mismatch: %.4f s — dropping older frame", dt);
            return;
        }

        // Use the left image timestamp as the frame timestamp
        double tframe = left_stamp_;
        RCLCPP_INFO(get_logger(), "TRYTRACK 4");
        // ── collect IMU measurements up to this frame ─────────────────────
        std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);

            // Drain stale IMU data from before the last frame
            while (!imu_buffer_.empty() &&
                   imu_buffer_.front().t < last_frame_t_)
            {
                imu_buffer_.pop_front();
            }

            // Collect all IMU samples up to this frame
            while (!imu_buffer_.empty() &&
                   imu_buffer_.front().t <= tframe)
            {
                imu_measurements.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();
            }
        }

        last_frame_t_ = tframe;

        // Take local copies so callbacks can update buffers immediately
        cv::Mat left, right;
        {   RCLCPP_INFO(get_logger(), "BEFORE LEFT LOCK");
            std::lock_guard<std::mutex> ll(left_mutex_);
            std::lock_guard<std::mutex> rl(right_mutex_);
            left  = left_image_.clone();
            right = right_image_.clone();
            left_ready_  = false;
            right_ready_ = false;
        }

        // ── SLAM tracking ─────────────────────────────────────────────────
        Sophus::SE3f pose;
        bool tracking_ok = false;

        try
        {   
            RCLCPP_INFO(get_logger(), "TRYTRACK 5");
            pose = slam_->TrackStereo(left, right, tframe, imu_measurements);
            RCLCPP_INFO(get_logger(), "TRYTRACK 6");

            if (!pose.matrix().hasNaN()
                && !pose.matrix().isZero(0)
                && slam_->GetTrackingState() == ORB_SLAM3::Tracking::OK)
            {
                tracking_ok = true;
            }
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "TrackStereo threw an exception");
            return;
        }

        // ── log pose ──────────────────────────────────────────────────────
        if (tracking_ok)
        {
            Eigen::Vector3f t = pose.translation();

            pose_file_ << tframe   << " "
                       << t.x()   << " "
                       << t.y()   << " "
                       << t.z()   << "\n";
            pose_file_.flush();

            RCLCPP_INFO(get_logger(),
                "x=%.4f  y=%.4f  z=%.4f",
                t.x(), t.y(), t.z());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Tracking not OK — skipping pose");
        }
    }

private:

    // ── ORB-SLAM3 ─────────────────────────────────────────────────────────
    std::unique_ptr<ORB_SLAM3::System> slam_;

    // ── ROS2 subscribers ─────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;

    // ── IMU buffer ────────────────────────────────────────────────────────
    std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex                         imu_mutex_;

    // ── Stereo image buffers ──────────────────────────────────────────────
    cv::Mat    left_image_,  right_image_;
    double     left_stamp_  = 0.0;
    double     right_stamp_ = 0.0;
    bool       left_ready_  = false;
    bool       right_ready_ = false;
    std::mutex left_mutex_,  right_mutex_;

    // ── Timing ───────────────────────────────────────────────────────────
    double last_frame_t_ = 0.0;

    // ── Output ───────────────────────────────────────────────────────────
    std::ofstream pose_file_;
};

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StereoInertialSlamNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}*/

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

class StereoInertialSlamNode : public rclcpp::Node
{
public:

    StereoInertialSlamNode()
    : Node("stereo_inertial_slam_node")
    {
        std::string vocab =
            "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt";

        std::string settings =
            "/home/saakshi-v/ardu_ws/src/ascend_packages/ascend_localization/config/Stereo_IMU.yaml";

        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocab,
            settings,
            ORB_SLAM3::System::IMU_STEREO,
            true);

        pose_file_.open("stereo_inertial_coordinates.txt");
        pose_file_ << "# timestamp tx ty tz\n";

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            1000,
            std::bind(&StereoInertialSlamNode::imuCallback, this, _1));

        left_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera_left/image",
            10,
            std::bind(&StereoInertialSlamNode::leftImageCallback, this, _1));

        right_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera_right/image",
            10,
            std::bind(&StereoInertialSlamNode::rightImageCallback, this, _1));

        RCLCPP_INFO(get_logger(), "Stereo-Inertial ORB-SLAM3 node started");
    }

    ~StereoInertialSlamNode()
    {
        slam_->Shutdown();
        pose_file_.close();
    }

private:

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        double t = msg->header.stamp.sec +
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

    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            left_image_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
            left_stamp_ = msg->header.stamp.sec +
                          msg->header.stamp.nanosec * 1e-9;
            left_ready_ = true;
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "Failed to decode left image");
            return;
        }

        tryTrack();
    }

    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            right_image_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
            right_stamp_ = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;
            right_ready_ = true;
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "Failed to decode right image");
            return;
        }

        tryTrack();
    }

    void tryTrack()
    {
        if (!left_ready_ || !right_ready_)
            return;

        // Allow up to one full frame period for stereo sync
        constexpr double MAX_STEREO_DT = 0.033;

        double dt = std::abs(left_stamp_ - right_stamp_);
        if (dt > MAX_STEREO_DT)
        {
            if (left_stamp_ < right_stamp_)
                left_ready_ = false;
            else
                right_ready_ = false;
            return;
        }

        double tframe = left_stamp_;

        // Collect IMU measurements up to this frame
        std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);

            while (!imu_buffer_.empty() &&
                   imu_buffer_.front().t < last_frame_t_)
                imu_buffer_.pop_front();

            while (!imu_buffer_.empty() &&
                   imu_buffer_.front().t <= tframe)
            {
                imu_measurements.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();
            }
        }

        last_frame_t_ = tframe;

        // Copy images and convert to grayscale
        cv::Mat left, right;
        left  = left_image_.clone();
        right = right_image_.clone();
        left_ready_  = false;
        right_ready_ = false;

        cv::cvtColor(left,  left,  cv::COLOR_RGB2GRAY);
        cv::cvtColor(right, right, cv::COLOR_RGB2GRAY);

        // Track
        Sophus::SE3f pose;
        bool tracking_ok = false;

        try
        {
            pose = slam_->TrackStereo(left, right, tframe, imu_measurements);

            if (!pose.matrix().hasNaN()
                && !pose.matrix().isZero(0)
                && slam_->GetTrackingState() == ORB_SLAM3::Tracking::OK)
            {
                tracking_ok = true;
            }
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "TrackStereo threw an exception");
            return;
        }

        if (tracking_ok)
        {
            Eigen::Vector3f t = pose.translation();

            pose_file_ << tframe << " "
                       << t.x() << " "
                       << t.y() << " "
                       << t.z() << "\n";
            pose_file_.flush();

            RCLCPP_INFO(get_logger(),
                "x=%.4f  y=%.4f  z=%.4f",
                t.x(), t.y(), t.z());
        }
    }

private:

    std::unique_ptr<ORB_SLAM3::System> slam_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;

    std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex                         imu_mutex_;

    cv::Mat left_image_,  right_image_;
    double  left_stamp_  = 0.0;
    double  right_stamp_ = 0.0;
    bool    left_ready_  = false;
    bool    right_ready_ = false;

    double last_frame_t_ = 0.0;

    std::ofstream pose_file_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StereoInertialSlamNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}