#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <csignal>
#include <thread>
#include <atomic>
#include <cmath>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"

using namespace std;

shared_ptr<rclcpp::Node> g_node = nullptr;
atomic<bool> return_to_origin_flag(false);  // set true when r is pressed
atomic<bool> shutdown_flag(false);

// ── helper: Euclidean distance between two 3D points ──────────────────────
double dist3d(double x1, double y1, double z1,
              double x2, double y2, double z2)
{
    return sqrt((x2-x1)*(x2-x1) +
                (y2-y1)*(y2-y1) +
                (z2-z1)*(z2-z1));
}

class SlamRosNode : public rclcpp::Node
{
public:
    SlamRosNode()
    : Node("slam_ros_node"),
      SLAM(
          "/home/rpi/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt",
          "/home/rpi/orb-slam3-root/ORB-SLAM3/Examples/Monocular/Webcam.yaml",
          ORB_SLAM3::System::MONOCULAR,
          true
      ),
      // EKF pose state
      ekf_origin_set_(false),
      ekf_origin_x_(0), ekf_origin_y_(0), ekf_origin_z_(0),
      ekf_current_x_(0), ekf_current_y_(0), ekf_current_z_(0),
      last_saved_x_(0), last_saved_y_(0), last_saved_z_(0),
      // SLAM scale state
      slam_origin_set_(false),
      slam_origin_x_(0), slam_origin_y_(0), slam_origin_z_(0),
      scale_(1.0),
      image_count_(0),
      returning_(false)
    {
        // ── pose log file ──
        pose_file_.open("/home/rpi/pose_log.txt");
        pose_file_ << "# timestamp  tx_metric  ty_metric  tz_metric  "
                      "tx_slam  ty_slam  tz_slam  scale\n";
        pose_file_.flush();

        // ── image coordinate log ──
        coord_file_.open("/home/rpi/image_coords.txt");
        coord_file_ << "# image_file  ekf_x(m)  ekf_y(m)  ekf_z(m)  "
                       "slam_x  slam_y  slam_z  scale\n";
        coord_file_.flush();

        // ── image save directory ──
        system("mkdir -p /home/rpi/slam_images");

        // ── publishers / subscribers ──
        pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/orbslam/pose", 10);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/ap/v1/cmd_vel", 10);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            bind(&SlamRosNode::image_callback, this, placeholders::_1));

        ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            bind(&SlamRosNode::ekf_callback, this, placeholders::_1));

        // ── return-to-origin timer (runs at 10Hz) ──
        rto_timer_ = this->create_wall_timer(
            chrono::milliseconds(100),
            bind(&SlamRosNode::return_to_origin_step, this));

        cout << "SlamRosNode started" << endl;
        cout << "  Press r + Enter → return to origin" << endl;
        cout << "  Press q + Enter → save and quit" << endl;
    }

    // ── called by keyboard thread ─────────────────────────────────────────
    void trigger_return_to_origin()
    {
        if(!ekf_origin_set_)
        {
            cout << "[RTO] Origin not set yet, fly first." << endl;
            return;
        }
        cout << "[RTO] Returning to origin ("
             << ekf_origin_x_ << ", "
             << ekf_origin_y_ << ", "
             << ekf_origin_z_ << ")" << endl;
        returning_ = true;
        return_to_origin_flag = true;
    }

    void save_and_shutdown()
    {
        cout << "\n======= SHUTDOWN =======" << endl;

        returning_ = false;
        send_zero_velocity();

        if(pose_file_.is_open())  { pose_file_.flush();  pose_file_.close(); }
        if(coord_file_.is_open()) { coord_file_.flush(); coord_file_.close(); }

        SLAM.Shutdown();

        SLAM.SaveKeyFrameTrajectoryTUM("/home/rpi/KeyFrameTrajectory.txt");
        SLAM.SaveTrajectoryTUM("/home/rpi/FrameTrajectory.txt");
        save_point_cloud("/home/rpi/point_cloud.ply");

        cout << "Saved: pose_log.txt, image_coords.txt, KeyFrameTrajectory.txt," << endl;
        cout << "       FrameTrajectory.txt, point_cloud.ply" << endl;
        cout << "Images: /home/rpi/slam_images/" << endl;
        cout << "========================" << endl;
    }

    ~SlamRosNode()
    {
        if(pose_file_.is_open())  { pose_file_.flush();  pose_file_.close(); }
        if(coord_file_.is_open()) { coord_file_.flush(); coord_file_.close(); }
    }

private:

    // ─────────────────────────────────────────────────────────────────────
    // EKF callback — metric position from ArduPilot
    // ─────────────────────────────────────────────────────────────────────
    void ekf_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ekf_current_x_ = msg->pose.pose.position.x;
    ekf_current_y_ = msg->pose.pose.position.y;
    ekf_current_z_ = msg->pose.pose.position.z;

        // set origin on first message
        if(!ekf_origin_set_)
        {
            ekf_origin_x_ = ekf_current_x_;
            ekf_origin_y_ = ekf_current_y_;
            ekf_origin_z_ = ekf_current_z_;
            ekf_origin_set_ = true;
            last_saved_x_  = ekf_current_x_;
            last_saved_y_  = ekf_current_y_;
            last_saved_z_  = ekf_current_z_;
            cout << "[EKF] Origin set: ("
                 << ekf_origin_x_ << ", "
                 << ekf_origin_y_ << ", "
                 << ekf_origin_z_ << ")" << endl;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Image callback — SLAM tracking
    // ─────────────────────────────────────────────────────────────────────
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        }
        catch(cv_bridge::Exception &e)
        {
            cerr << "cv_bridge: " << e.what() << endl;
            return;
        }

        if(frame.empty()) return;

        double timestamp =
            chrono::duration_cast<chrono::duration<double>>(
                chrono::steady_clock::now().time_since_epoch()
            ).count();

        auto Tcw = SLAM.TrackMonocular(frame, timestamp);

        if(Tcw.matrix().isZero()) return;

        // ── get SLAM pose ────────────────────────────────────────────────
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f t = Twc.translation();
        Eigen::Quaternionf q(Twc.rotationMatrix());

        double sx = t.x(), sy = t.y(), sz = t.z();

        // ── set SLAM origin on first valid pose ──────────────────────────
        if(!slam_origin_set_)
        {
            slam_origin_x_ = sx;
            slam_origin_y_ = sy;
            slam_origin_z_ = sz;
            slam_origin_set_ = true;
        }

        // ── compute scale using EKF vs SLAM displacement ─────────────────
        // concept: scale = |EKF moved| / |SLAM moved|
        // we update scale continuously so it improves over time
        if(ekf_origin_set_)
        {
            double ekf_disp = dist3d(
                ekf_origin_x_, ekf_origin_y_, ekf_origin_z_,
                ekf_current_x_, ekf_current_y_, ekf_current_z_);

            double slam_disp = dist3d(
                slam_origin_x_, slam_origin_y_, slam_origin_z_,
                sx, sy, sz);

            // only update scale when drone has moved enough for
            // reliable estimate — avoids division by near-zero
            if(slam_disp > 0.01 && ekf_disp > 0.05)
            {
                double new_scale = ekf_disp / slam_disp;

                // smooth scale with running average (90% old, 10% new)
                // prevents sudden jumps from noisy estimates
                scale_ = 0.9 * scale_ + 0.1 * new_scale;
            }
        }

        // ── metric position = SLAM position × scale ───────────────────────
        double mx = (sx - slam_origin_x_) * scale_;
        double my = (sy - slam_origin_y_) * scale_;
        double mz = (sz - slam_origin_z_) * scale_;

        // ── publish pose ─────────────────────────────────────────────────
        nav_msgs::msg::Odometry pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.pose.position.x = mx;
        pose_msg.pose.pose.position.y = my;
        pose_msg.pose.pose.position.z = mz;
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        pose_pub_->publish(pose_msg);

        // ── write to pose log ────────────────────────────────────────────
        pose_file_ << fixed << setprecision(6)
                   << timestamp << " "
                   << mx << " " << my << " " << mz << " "
                   << sx << " " << sy << " " << sz << " "
                   << scale_ << "\n";
        pose_file_.flush();

        // ── save image + coords every 10cm of EKF movement ───────────────
        double moved_since_save = dist3d(
            last_saved_x_, last_saved_y_, last_saved_z_,
            ekf_current_x_, ekf_current_y_, ekf_current_z_);

        if(moved_since_save >= 0.2 && ekf_origin_set_)
        {
            image_count_++;

            // filename: slam_images/frame_0001.png
            ostringstream fname;
            fname << "/home/rpi/slam_images/frame_"
                  << setw(4) << setfill('0') << image_count_
                  << ".png";

            cv::imwrite(fname.str(), frame);

            // write to coord file
            coord_file_ << fname.str()          << "  "
                        << fixed << setprecision(4)
                        << ekf_current_x_ - ekf_origin_x_ << "  "
                        << ekf_current_y_ - ekf_origin_y_ << "  "
                        << ekf_current_z_ - ekf_origin_z_ << "  "
                        << mx << "  " << my << "  " << mz << "  "
                        << scale_ << "\n";
            coord_file_.flush();

            last_saved_x_ = ekf_current_x_;
            last_saved_y_ = ekf_current_y_;
            last_saved_z_ = ekf_current_z_;

            cout << "[SAVED] frame_" << image_count_
                 << " at EKF=("
                 << ekf_current_x_ - ekf_origin_x_ << ", "
                 << ekf_current_y_ - ekf_origin_y_ << ", "
                 << ekf_current_z_ - ekf_origin_z_ << ") m"
                 << "  SLAM_metric=("
                 << mx << ", " << my << ", " << mz << ")"
                 << "  scale=" << scale_ << endl;
        }

        this_thread::sleep_for(chrono::milliseconds(1));
    }

    // ─────────────────────────────────────────────────────────────────────
    // Return to origin — runs at 10Hz via timer
    // proportional controller: velocity = Kp × position_error
    // ─────────────────────────────────────────────────────────────────────
    void return_to_origin_step()
    {
        if(!returning_) return;
        if(!ekf_origin_set_) return;

        double ex = ekf_origin_x_ - ekf_current_x_;
        double ey = ekf_origin_y_ - ekf_current_y_;
        double ez = ekf_origin_z_ - ekf_current_z_;

        double error = sqrt(ex*ex + ey*ey + ez*ez);

        if(error < 0.15)   // within 15cm — close enough
        {
            cout << "[RTO] Reached origin. Stopping." << endl;
            send_zero_velocity();
            returning_ = false;
            return_to_origin_flag = false;
            return;
        }

        // proportional gain — tune this
        // higher = faster but may overshoot
        double Kp = 0.4;

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = Kp * ex;
        cmd.twist.linear.y = Kp * ey;
        cmd.twist.linear.z = Kp * ez;

        // cap velocity to 0.5 m/s for safety
        double speed = sqrt(
            cmd.twist.linear.x * cmd.twist.linear.x +
            cmd.twist.linear.y * cmd.twist.linear.y +
            cmd.twist.linear.z * cmd.twist.linear.z);

        if(speed > 0.5)
        {
            double scale = 0.5 / speed;
            cmd.twist.linear.x *= scale;
            cmd.twist.linear.y *= scale;
            cmd.twist.linear.z *= scale;
        }

        cmd_vel_pub_->publish(cmd);

        cout << "[RTO] error=" << error << "m  vel=("
             << cmd.twist.linear.x << ", "
             << cmd.twist.linear.y << ", "
             << cmd.twist.linear.z << ")" << endl;
    }

    void send_zero_velocity()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x  = 0;
        cmd.twist.linear.y  = 0;
        cmd.twist.linear.z  = 0;
        cmd.twist.angular.x = 0;
        cmd.twist.angular.y = 0;
        cmd.twist.angular.z = 0;
        cmd_vel_pub_->publish(cmd);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Save point cloud as PLY
    // ─────────────────────────────────────────────────────────────────────
    void save_point_cloud(const string& filename)
    {
        cout << "Saving point cloud..." << endl;

        ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
        if(!pAtlas) { cerr << "Atlas null" << endl; return; }

        vector<ORB_SLAM3::MapPoint*> points = pAtlas->GetAllMapPoints();

        // count valid points first for PLY header
        int valid = 0;
        for(auto p : points)
            if(p && !p->isBad()) valid++;

        ofstream f(filename);
        f << "ply\nformat ascii 1.0\n";
        f << "element vertex " << valid << "\n";
        f << "property float x\nproperty float y\nproperty float z\n";
        f << "end_header\n";

        for(auto p : points)
        {
            if(p && !p->isBad())
            {
                Eigen::Vector3f pos = p->GetWorldPos();
                f << fixed << setprecision(6)
                  << pos.x() << " "
                  << pos.y() << " "
                  << pos.z() << "\n";
            }
        }

        f.close();
        cout << "Point cloud saved: " << valid
             << " points → " << filename << endl;
    }

    // ── members ──────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       image_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr                                    rto_timer_;

    ORB_SLAM3::System SLAM;
    ofstream pose_file_;
    ofstream coord_file_;

    // EKF state
    bool   ekf_origin_set_;
    double ekf_origin_x_, ekf_origin_y_, ekf_origin_z_;
    double ekf_current_x_, ekf_current_y_, ekf_current_z_;
    double last_saved_x_,  last_saved_y_,  last_saved_z_;

    // SLAM scale state
    bool   slam_origin_set_;
    double slam_origin_x_, slam_origin_y_, slam_origin_z_;
    double scale_;

    int  image_count_;
    bool returning_;
};

// ── signal handler ────────────────────────────────────────────────────────
void signal_handler(int)
{
    if(g_node)
    {
        auto node = dynamic_pointer_cast<SlamRosNode>(g_node);
        if(node) node->save_and_shutdown();
    }
    rclcpp::shutdown();
}

// ── keyboard thread ───────────────────────────────────────────────────────
void keyboard_thread()
{
    string input;
    while(!shutdown_flag)
    {
        getline(cin, input);

        if(input == "r" || input == "R")
        {
            if(g_node)
            {
                auto node = dynamic_pointer_cast<SlamRosNode>(g_node);
                if(node) node->trigger_return_to_origin();
            }
        }
        else if(input == "q" || input == "Q")
        {
            cout << "Quit requested..." << endl;
            shutdown_flag = true;
            if(g_node)
            {
                auto node = dynamic_pointer_cast<SlamRosNode>(g_node);
                if(node) node->save_and_shutdown();
            }
            rclcpp::shutdown();
            break;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    auto node = make_shared<SlamRosNode>();
    g_node = node;

    // start keyboard listener in background thread
    thread kb_thread(keyboard_thread);
    kb_thread.detach();

    rclcpp::spin(node);

    shutdown_flag = true;
    rclcpp::shutdown();
    return 0;
}