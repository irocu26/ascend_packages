#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <csignal>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"

using namespace std;

// global pointer for signal handler
shared_ptr<rclcpp::Node> g_node = nullptr;

class SlamRosNode : public rclcpp::Node
{
public:
    SlamRosNode()
    : Node("slam_ros_node"),
      // ── exact same SLAM init as mono_webcam.cc ──
      SLAM(
          "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt",
          "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Examples/Monocular/Webcam.yaml",
          ORB_SLAM3::System::MONOCULAR,
          true    // Pangolin on, same as working standalone
      )
    {
        // ── exact same pose file open as mono_webcam.cc ──
        pose_file_.open("/home/saakshi-v/pose_log.txt");
        pose_file_ << "# timestamp tx ty tz qx qy qz qw\n";
        pose_file_.flush();

        // publisher for pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orbslam/pose", 10
        );

        // ── subscriber replaces VideoCapture loop ──
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/image",
            10,
            bind(&SlamRosNode::image_callback, this, placeholders::_1)
        );

        cout << "SlamRosNode started, waiting for images on /rgbd_camera/image ..." << endl;
    }

    void save_and_shutdown()
    {
        cout << "Shutting down SLAM..." << endl;

        pose_file_.flush();
        pose_file_.close();

        SLAM.Shutdown();

        // ── exact same saves as mono_webcam.cc ──
        SLAM.SaveKeyFrameTrajectoryTUM("/home/saakshi-v/KeyFrameTrajectory.txt");
        SLAM.SaveTrajectoryTUM("/home/saakshi-v/FrameTrajectory.txt");

        cout << "Trajectory saved." << endl;
    }

    ~SlamRosNode()
    {
        if(pose_file_.is_open())
        {
            pose_file_.flush();
            pose_file_.close();
        }
    }

private:

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // ── convert ROS image to cv::Mat (replaces cap >> frame) ──
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        }
        
        catch(cv_bridge::Exception &e)
        {
            cerr << "cv_bridge exception: " << e.what() << endl;
            return;
        }

        if(frame.empty())
        {
            cout << "EMPTY FRAME" << endl;
            return;
        }

        cout << "FRAME RECEIVED: " << frame.cols << " x " << frame.rows << endl;

        // ── exact same timestamp as mono_webcam.cc ──
        double timestamp =
            chrono::duration_cast<chrono::duration<double>>(
                chrono::steady_clock::now().time_since_epoch()
            ).count();

        cout << "TRACKING FRAME..." << endl;

        // ── exact same SLAM call as mono_webcam.cc ──
        auto Tcw = SLAM.TrackMonocular(frame, timestamp);

        if(!Tcw.matrix().isZero())
        {
            cout << "POSE ESTIMATED" << endl;

            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Vector3f t = Twc.translation();
            Eigen::Quaternionf q(Twc.rotationMatrix());

            // ── exact same file write as mono_webcam.cc ──
            pose_file_ << fixed << setprecision(6)
                       << timestamp << " "
                       << t.x()     << " "
                       << t.y()     << " "
                       << t.z()     << " "
                       << q.x()     << " "
                       << q.y()     << " "
                       << q.z()     << " "
                       << q.w()     << "\n";
            pose_file_.flush();

            // ── bonus: also publish to ROS topic ──
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.pose.position.x    = t.x();
            pose_msg.pose.position.y    = t.y();
            pose_msg.pose.position.z    = t.z();
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
            pose_pub_->publish(pose_msg);
        }
        else
        {
            cout << "NO POSE YET" << endl;
        }

        cout << "TRACKING DONE" << endl;

        // ── same 1ms sleep as mono_webcam.cc ──
        this_thread::sleep_for(chrono::milliseconds(1));
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    ORB_SLAM3::System SLAM;
    ofstream pose_file_;
};

void signal_handler(int signum)
{
    if(g_node)
    {
        auto node = dynamic_pointer_cast<SlamRosNode>(g_node);
        if(node) node->save_and_shutdown();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    auto node = make_shared<SlamRosNode>();
    g_node = node;

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}