// #include <rclcpp/rclcpp.hpp>

// #include <sensor_msgs/msg/image.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>

// #include <cv_bridge/cv_bridge.hpp>
// #include <opencv2/opencv.hpp>

// #include <tf2_ros/transform_broadcaster.h>

// #include <System.h>

// #include <fstream>
// #include <mutex>

// using std::placeholders::_1;

// class RgbdSlamNode : public rclcpp::Node
// {
// public:

//     RgbdSlamNode()
//     : Node("rgbd_slam_node")
//     {
//         std::string vocab =
//             "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt";

//         std::string settings =
//             "/home/saakshi-v/ardu_ws/src/ascend_packages/ascend_localization/config/RGBD.yaml";

//         slam_ = std::make_unique<ORB_SLAM3::System>(
//             vocab,
//             settings,
//             ORB_SLAM3::System::RGBD,
//             true);

//         pose_file_.open("rgbd_coordinates.txt");
//         pose_file_ << "# timestamp tx(m) ty(m) tz(m)\n";

//         odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
//             "/orbslam/odom", 10);

//         tf_broadcaster_ =
//             std::make_unique<tf2_ros::TransformBroadcaster>(this);

//         rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
//             "/rgbd_camera/image",
//             10,
//             std::bind(&RgbdSlamNode::rgbCallback, this, _1));

//         depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
//             "/rgbd_camera/depth_image",
//             10,
//             std::bind(&RgbdSlamNode::depthCallback, this, _1));

//         RCLCPP_INFO(get_logger(), "RGBD ORB-SLAM3 node started");
//         RCLCPP_INFO(get_logger(), "Publishing pose on /orbslam/odom");
//     }

//     ~RgbdSlamNode()
//     {
//         slam_->Shutdown();
//         pose_file_.close();
//     }

// private:

//     void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         try
//         {
//             rgb_image_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
//             rgb_stamp_ = msg->header.stamp.sec +
//                          msg->header.stamp.nanosec * 1e-9;
//             rgb_ready_ = true;
//         }
//         catch (...)
//         {
//             RCLCPP_WARN(get_logger(), "Failed to decode RGB image");
//             return;
//         }

//         tryTrack();
//     }

//     void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         try
//         {
//             depth_image_ = cv_bridge::toCvCopy(msg, "32FC1")->image;
//             depth_stamp_ = msg->header.stamp.sec +
//                            msg->header.stamp.nanosec * 1e-9;
//             depth_ready_ = true;
//         }
//         catch (...)
//         {
//             RCLCPP_WARN(get_logger(), "Failed to decode depth image");
//             return;
//         }

//         tryTrack();
//     }

//     void tryTrack()
//     {
//         if (!rgb_ready_ || !depth_ready_)
//             return;

//         // Allow up to one frame period for sync
//         constexpr double MAX_DT = 0.05;

//         double dt = std::abs(rgb_stamp_ - depth_stamp_);
//         if (dt > MAX_DT)
//         {
//             if (rgb_stamp_ < depth_stamp_)
//                 rgb_ready_ = false;
//             else
//                 depth_ready_ = false;
//             return;
//         }

//         double tframe = rgb_stamp_;

//         cv::Mat rgb   = rgb_image_.clone();
//         cv::Mat depth = depth_image_.clone();
//         rgb_ready_   = false;
//         depth_ready_ = false;

//         // ── SLAM tracking ─────────────────────────────────────────────────
//         Sophus::SE3f Tcw;
//         try
//         {
//             Tcw = slam_->TrackRGBD(rgb, depth, tframe);
//         }
//         catch (...)
//         {
//             RCLCPP_WARN(get_logger(), "TrackRGBD threw an exception");
//             return;
//         }

//         if (Tcw.matrix().isZero(0) || Tcw.matrix().hasNaN())
//             return;

//         if (slam_->GetTrackingState() != ORB_SLAM3::Tracking::OK)
//             return;

//         // ── Tcw → Twc ────────────────────────────────────────────────────
//         Sophus::SE3f Twc = Tcw.inverse();

//         // Set origin on first valid pose
//         if (!has_origin_)
//         {
//             origin_     = Twc;
//             has_origin_ = true;
//         }

//         // Relative pose from start
//         Sophus::SE3f    T_rel = origin_.inverse() * Twc;
//         Eigen::Vector3f t_sl  = T_rel.translation();
//         Eigen::Matrix3f R_sl  = T_rel.rotationMatrix();

//         // ── ORB-SLAM3 → ROS axis conversion ──────────────────────────────
//         // SLAM: X=right, Y=down,    Z=forward
//         // ROS:  X=forward, Y=left,  Z=up
//         Eigen::Matrix3f R_to_ros;
//         R_to_ros <<  0,  0,  1,
//                     -1,  0,  0,
//                      0, -1,  0;

//         Eigen::Vector3f    t_ros = R_to_ros * t_sl;
//         Eigen::Matrix3f    R_ros = R_to_ros * R_sl * R_to_ros.transpose();
//         Eigen::Quaternionf q(R_ros);
//         q.normalize();

//         // ── log ───────────────────────────────────────────────────────────
//         pose_file_ << tframe     << " "
//                    << t_ros.x() << " "
//                    << t_ros.y() << " "
//                    << t_ros.z() << "\n";
//         pose_file_.flush();

//         RCLCPP_INFO(get_logger(),
//             "x=%.3f m  y=%.3f m  z=%.3f m",
//             t_ros.x(), t_ros.y(), t_ros.z());

//         // ── publish odometry ──────────────────────────────────────────────
//         auto now = this->get_clock()->now();

//         nav_msgs::msg::Odometry odom;
//         odom.header.stamp    = now;
//         odom.header.frame_id = "odom";
//         odom.child_frame_id  = "base_link";

//         odom.pose.pose.position.x    = t_ros.x();
//         odom.pose.pose.position.y    = t_ros.y();
//         odom.pose.pose.position.z    = t_ros.z();
//         odom.pose.pose.orientation.x = q.x();
//         odom.pose.pose.orientation.y = q.y();
//         odom.pose.pose.orientation.z = q.z();
//         odom.pose.pose.orientation.w = q.w();

//         odom_pub_->publish(odom);

//         // ── publish TF ────────────────────────────────────────────────────
//         geometry_msgs::msg::TransformStamped tf;
//         tf.header.stamp            = now;
//         tf.header.frame_id         = "odom";
//         tf.child_frame_id          = "base_link";
//         tf.transform.translation.x = t_ros.x();
//         tf.transform.translation.y = t_ros.y();
//         tf.transform.translation.z = t_ros.z();
//         tf.transform.rotation.x    = q.x();
//         tf.transform.rotation.y    = q.y();
//         tf.transform.rotation.z    = q.z();
//         tf.transform.rotation.w    = q.w();

//         tf_broadcaster_->sendTransform(tf);
//     }

// private:

//     std::unique_ptr<ORB_SLAM3::System> slam_;

//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
//     std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;

//     cv::Mat rgb_image_,  depth_image_;
//     double  rgb_stamp_   = 0.0;
//     double  depth_stamp_ = 0.0;
//     bool    rgb_ready_   = false;
//     bool    depth_ready_ = false;

//     bool         has_origin_ = false;
//     Sophus::SE3f origin_;

//     std::ofstream pose_file_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<RgbdSlamNode>();

//     rclcpp::spin(node);

//     rclcpp::shutdown();

//     return 0;
// }

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_broadcaster.h>
// CORRECT
#include <tf2_msgs/msg/tf_message.hpp>
#include <System.h>

#include <fstream>
#include <mutex>

using std::placeholders::_1;

class RgbdSlamNode : public rclcpp::Node
{
public:

    RgbdSlamNode()
    : Node("rgbd_slam_node")
    {
        std::string vocab =
            "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt";

        std::string settings =
            "/home/saakshi-v/ardu_ws/src/ascend_packages/ascend_localization/config/RGBD.yaml";

        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocab,
            settings,
            ORB_SLAM3::System::RGBD,
            true);

        pose_file_.open("rgbd_coordinates.txt");
        pose_file_ << "# timestamp tx(m) ty(m) tz(m)\n";

        // ── publishers ────────────────────────────────────────────────────
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/orbslam/odom", 10);

        // SLAM path — green in RViz
        slam_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/orbslam/path", 10);

        // tf_broadcaster_ =
        //     std::make_unique<tf2_ros::TransformBroadcaster>(this);
        ap_tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
    "/ap/tf", 10);

        // ── subscribers ───────────────────────────────────────────────────
        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/image",
            10,
            std::bind(&RgbdSlamNode::rgbCallback, this, _1));

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/depth_image",
            10,
            std::bind(&RgbdSlamNode::depthCallback, this, _1));

        // Ground truth path from Gazebo EKF odometry — red in RViz
        gt_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry",
            10,
            std::bind(&RgbdSlamNode::gtOdomCallback, this, _1));

        gt_path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/ground_truth/path", 10);

        RCLCPP_INFO(get_logger(), "RGBD ORB-SLAM3 node started");
        RCLCPP_INFO(get_logger(), "SLAM path:         /orbslam/path");
        RCLCPP_INFO(get_logger(), "Ground truth path: /ground_truth/path");
    }

    ~RgbdSlamNode()
    {
        slam_->Shutdown();
        pose_file_.close();
    }

private:

    // ── Ground truth callback ─────────────────────────────────────────────
    void gtOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg->header;
        ps.header.frame_id = "odom";
        ps.pose  = msg->pose.pose;

        gt_path_.header.stamp    = msg->header.stamp;
        gt_path_.header.frame_id = "odom";
        gt_path_.poses.push_back(ps);

        gt_path_pub_->publish(gt_path_);
    }

    // ── RGB callback ──────────────────────────────────────────────────────
    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            rgb_image_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
            rgb_stamp_ = msg->header.stamp.sec +
                         msg->header.stamp.nanosec * 1e-9;
            rgb_ready_ = true;
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "Failed to decode RGB image");
            return;
        }
        tryTrack();
    }

    // ── Depth callback ────────────────────────────────────────────────────
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            depth_image_ = cv_bridge::toCvCopy(msg, "32FC1")->image;
            depth_stamp_ = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;
            depth_ready_ = true;
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "Failed to decode depth image");
            return;
        }
        tryTrack();
    }

    // ── SLAM tracking ─────────────────────────────────────────────────────
    void tryTrack()
    {
        if (!rgb_ready_ || !depth_ready_)
            return;

        constexpr double MAX_DT = 0.05;
        double dt = std::abs(rgb_stamp_ - depth_stamp_);
        if (dt > MAX_DT)
        {
            if (rgb_stamp_ < depth_stamp_)
                rgb_ready_ = false;
            else
                depth_ready_ = false;
            return;
        }

        double tframe = rgb_stamp_;

        cv::Mat rgb   = rgb_image_.clone();
        cv::Mat depth = depth_image_.clone();
        rgb_ready_   = false;
        depth_ready_ = false;

        Sophus::SE3f Tcw;
        try
        {
            Tcw = slam_->TrackRGBD(rgb, depth, tframe);
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(), "TrackRGBD threw an exception");
            return;
        }

        if (Tcw.matrix().isZero(0) || Tcw.matrix().hasNaN())
            return;

        if (slam_->GetTrackingState() != ORB_SLAM3::Tracking::OK)
            return;

        // Tcw → Twc
        Sophus::SE3f Twc = Tcw.inverse();

        if (!has_origin_)
        {
            origin_     = Twc;
            has_origin_ = true;
        }

        Sophus::SE3f    T_rel = origin_.inverse() * Twc;
        Eigen::Vector3f t_sl  = T_rel.translation();

        RCLCPP_INFO(
            get_logger(),
            "RAW SLAM: x=%.3f y=%.3f z=%.3f",
            t_sl.x(),
            t_sl.y(),
            t_sl.z());
        Eigen::Matrix3f R_sl  = T_rel.rotationMatrix();

        // ORB-SLAM3 → ROS axis conversion
        Eigen::Matrix3f R_to_ros;
        R_to_ros <<  0, -1,  0,
                     1,  0,  0,
                     0,  0, -1;

        Eigen::Vector3f    t_ros = R_to_ros * t_sl;
        Eigen::Matrix3f    R_ros = R_to_ros * R_sl * R_to_ros.transpose();
        Eigen::Quaternionf q(R_ros);
        q.normalize();

        // log
        pose_file_ << tframe     << " "
                   << t_ros.x() << " "
                   << t_ros.y() << " "
                   << t_ros.z() << "\n";
        pose_file_.flush();

        RCLCPP_INFO(get_logger(),
            "x=%.3f m  y=%.3f m  z=%.3f m",
            t_ros.x(), t_ros.y(), t_ros.z());

        auto now = this->get_clock()->now();

        // ── odometry ──────────────────────────────────────────────────────
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        odom.pose.pose.position.x    = t_ros.x();
        odom.pose.pose.position.y    = t_ros.y();
        odom.pose.pose.position.z    = t_ros.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom_pub_->publish(odom);

        // ── SLAM path ─────────────────────────────────────────────────────
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom.header;
        ps.pose   = odom.pose.pose;

        slam_path_.header.stamp    = now;
        slam_path_.header.frame_id = "odom";
        slam_path_.poses.push_back(ps);
        slam_path_pub_->publish(slam_path_);

        // ── TF ────────────────────────────────────────────────────────────
        // geometry_msgs::msg::TransformStamped tf;
        // tf.header.stamp            = now;
        // tf.header.frame_id         = "odom";
        // tf.child_frame_id          = "base_link";
        // tf.transform.translation.x = t_ros.x();
        // tf.transform.translation.y = t_ros.y();
        // tf.transform.translation.z = t_ros.z();
        // tf.transform.rotation.x    = q.x();
        // tf.transform.rotation.y    = q.y();
        // tf.transform.rotation.z    = q.z();
        // tf.transform.rotation.w    = q.w();
        // tf_broadcaster_->sendTransform(tf);
        // ── TF to ArduPilot via ap/tf ─────────────────────────────────────
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp            = now;
        tf.header.frame_id         = "odom";      // MUST be odom
        tf.child_frame_id          = "base_link"; // MUST be base_link
        tf.transform.translation.x = t_ros.x();
        tf.transform.translation.y = t_ros.y();
        tf.transform.translation.z = t_ros.z();
        tf.transform.rotation.x    = q.x();
        tf.transform.rotation.y    = q.y();
        tf.transform.rotation.z    = q.z();
        tf.transform.rotation.w    = q.w();

        tf2_msgs::msg::TFMessage tf_msg;
        tf_msg.transforms.push_back(tf);
        ap_tf_pub_->publish(tf_msg);
    }

private:

    std::unique_ptr<ORB_SLAM3::System> slam_;

    // RGB-D subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    // Ground truth subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     slam_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     gt_path_pub_;

    //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr ap_tf_pub_;

    // Image buffers
    cv::Mat rgb_image_,  depth_image_;
    double  rgb_stamp_   = 0.0;
    double  depth_stamp_ = 0.0;
    bool    rgb_ready_   = false;
    bool    depth_ready_ = false;

    // Pose state
    bool         has_origin_ = false;
    Sophus::SE3f origin_;

    // Path accumulators
    nav_msgs::msg::Path slam_path_;
    nav_msgs::msg::Path gt_path_;

    std::ofstream pose_file_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbdSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}