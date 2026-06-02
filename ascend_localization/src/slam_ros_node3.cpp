/*
 * slam_ros_node_imu.cpp
 *
 * Monocular + IMU ORB-SLAM3 node for ArduPilot / Pixhawk 6C
 * ──────────────────────────────────────────────────────────
 * Subscribes:
 *   /rgbd_camera/image   → sensor_msgs/Image     (camera frames)
 *   /imu                 → sensor_msgs/Imu        (Pixhawk IMU at ~200 Hz)
 *   /odometry            → nav_msgs/Odometry      (ArduPilot EKF — optional,
 *                                                  used ONLY for RTO origin)
 *
 * Publishes:
 *   /orbslam/pose        → nav_msgs/Odometry      (metric VIO pose → feed to
 *                                                  ArduPilot as vision position)
 *   /ap/v1/cmd_vel       → geometry_msgs/TwistStamped  (return-to-origin cmds)
 *
 * Key differences vs. the pure-monocular node:
 *   • Uses ORB_SLAM3::System::IMU_MONOCULAR  — IMU provides metric scale
 *     and gravity-aligned orientation. No external scale estimation needed.
 *   • IMU samples are buffered between frames and passed to
 *     TrackMonocular_IMU() alongside every image.
 *   • The published pose is already in metric units from SLAM — no EKF
 *     scale trick required (EKF is still used to record the takeoff origin
 *     for return-to-origin).
 *   • IMU data synced via a mutex-protected deque.
 *
 * YAML config:
 *   Use a Monocular-Inertial yaml (see ORB-SLAM3 Examples/Monocular-Inertial/).
 *   You MUST fill in your camera intrinsics, distortion, and IMU noise params
 *   (accel/gyro noise density, random walk) calibrated for your sensor pair.
 *   The IMU.T_BS matrix must encode the camera→IMU extrinsic transform.
 *
 * Keyboard (stdin):
 *   r + Enter  →  return to takeoff origin
 *   q + Enter  →  save all logs and shut down
 */

 /*
#include <iostream>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <csignal>
#include <thread>
#include <atomic>
#include <cmath>
#include <sstream>
#include <deque>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <cv_bridge/cv_bridge.hpp>

// ORB-SLAM3 headers
#include "System.h"
#include "ImuTypes.h"   // ORB_SLAM3::IMU::Point

using namespace std;

// ── globals ──────────────────────────────────────────────────────────────────
shared_ptr<rclcpp::Node> g_node = nullptr;
atomic<bool> shutdown_flag(false);

// ── helper: Euclidean distance ────────────────────────────────────────────────
double dist3d(double x1, double y1, double z1,
              double x2, double y2, double z2)
{
    return sqrt((x2-x1)*(x2-x1) +
                (y2-y1)*(y2-y1) +
                (z2-z1)*(z2-z1));
}

// ─────────────────────────────────────────────────────────────────────────────
class SlamImuNode : public rclcpp::Node
{
public:
    SlamImuNode()
    : Node("slam_imu_node"),
      // ── ORB-SLAM3 initialised in IMU_MONOCULAR mode ──────────────────
      //    Change the paths to match your install.
      //    The YAML must be a Monocular-Inertial config (camera + IMU params).
      SLAM(
          "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Vocabulary/ORBvoc.txt",
          "/home/saakshi-v/orb-slam3-root/ORB-SLAM3/Examples/Monocular-Inertial/Webcam_IMU.yaml",
          ORB_SLAM3::System::IMU_MONOCULAR,
          true   // use viewer
      ),
      // EKF / origin state
      ekf_origin_set_(false),
      ekf_origin_x_(0), ekf_origin_y_(0), ekf_origin_z_(0),
      ekf_current_x_(0), ekf_current_y_(0), ekf_current_z_(0),
      // SLAM origin (used for relative pose output)
      slam_origin_set_(false),
      slam_origin_x_(0), slam_origin_y_(0), slam_origin_z_(0),
      // misc
      last_saved_x_(0), last_saved_y_(0), last_saved_z_(0),
      image_count_(0),
      returning_(false)
    {
        // ── log files ────────────────────────────────────────────────────
        pose_file_.open("/home/saakshi-v/pose_log_imu.txt");
        pose_file_ << "# timestamp  tx(m)  ty(m)  tz(m)  "
                      "qx  qy  qz  qw\n";
        pose_file_.flush();

        coord_file_.open("/home/saakshi-v/image_coords_imu.txt");
        coord_file_ << "# image_file  ekf_dx(m)  ekf_dy(m)  ekf_dz(m)  "
                       "slam_x(m)  slam_y(m)  slam_z(m)\n";
        coord_file_.flush();

        system("mkdir -p /home/saakshi-v/slam_images_imu");

        // ── publishers ───────────────────────────────────────────────────
        pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/orbslam/pose", 10);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/ap/v1/cmd_vel", 10);

        // ── subscribers ──────────────────────────────────────────────────

        // IMU — high rate, buffer all samples
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 500,
            bind(&SlamImuNode::imu_callback, this, placeholders::_1));

        // Camera image — triggers SLAM tracking
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/image", 10,
            bind(&SlamImuNode::image_callback, this, placeholders::_1));

        // EKF odometry — only for takeoff-origin RTO
        ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            bind(&SlamImuNode::ekf_callback, this, placeholders::_1));

        // ── return-to-origin timer (10 Hz) ───────────────────────────────
        rto_timer_ = this->create_wall_timer(
            chrono::milliseconds(100),
            bind(&SlamImuNode::return_to_origin_step, this));

        cout << "SlamImuNode (Monocular+IMU) started" << endl;
        cout << "  Press r + Enter → return to takeoff origin" << endl;
        cout << "  Press q + Enter → save logs and quit" << endl;
    }

    // ── public control methods (called from keyboard thread) ──────────────
    void trigger_return_to_origin()
    {
        if(!ekf_origin_set_)
        {
            cout << "[RTO] Origin not set yet — fly first." << endl;
            return;
        }
        cout << "[RTO] Returning to origin ("
             << ekf_origin_x_ << ", "
             << ekf_origin_y_ << ", "
             << ekf_origin_z_ << ")" << endl;
        returning_ = true;
    }

    void save_and_shutdown()
    {
        cout << "\n======= SHUTDOWN =======" << endl;

        returning_ = false;
        send_zero_velocity();

        if(pose_file_.is_open())  { pose_file_.flush();  pose_file_.close(); }
        if(coord_file_.is_open()) { coord_file_.flush(); coord_file_.close(); }

        SLAM.Shutdown();

        SLAM.SaveKeyFrameTrajectoryTUM(
            "/home/saakshi-v/KeyFrameTrajectory_IMU.txt");
        SLAM.SaveTrajectoryTUM(
            "/home/saakshi-v/FrameTrajectory_IMU.txt");
        save_point_cloud("/home/saakshi-v/point_cloud_imu.ply");

        cout << "Saved: pose_log_imu.txt, image_coords_imu.txt," << endl;
        cout << "       KeyFrameTrajectory_IMU.txt, FrameTrajectory_IMU.txt," << endl;
        cout << "       point_cloud_imu.ply" << endl;
        cout << "Images: /home/saakshi-v/slam_images_imu/" << endl;
        cout << "========================" << endl;
    }

    ~SlamImuNode()
    {
        if(pose_file_.is_open())  { pose_file_.flush();  pose_file_.close(); }
        if(coord_file_.is_open()) { coord_file_.flush(); coord_file_.close(); }
    }

private:

    // ─────────────────────────────────────────────────────────────────────
    // IMU callback — buffer every sample with its timestamp
    //
    // These samples are drained and passed to SLAM in image_callback.
    // Using a mutex because IMU and image callbacks run on different
    // executor threads.
    // ─────────────────────────────────────────────────────────────────────
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert ROS timestamp → seconds (double)
        double t = msg->header.stamp.sec +
                   msg->header.stamp.nanosec * 1e-9;

        ORB_SLAM3::IMU::Point pt(
            // linear acceleration  (x, y, z)  in m/s²
            static_cast<float>(msg->linear_acceleration.x),
            static_cast<float>(msg->linear_acceleration.y),
            static_cast<float>(msg->linear_acceleration.z),
            // angular velocity (x, y, z)  in rad/s
            static_cast<float>(msg->angular_velocity.x),
            static_cast<float>(msg->angular_velocity.y),
            static_cast<float>(msg->angular_velocity.z),
            t
        );

        lock_guard<mutex> lk(imu_mutex_);
        imu_buf_.push_back(pt);
    }

    // ─────────────────────────────────────────────────────────────────────
    // EKF callback — metric position from ArduPilot (origin + RTO only)
    // ─────────────────────────────────────────────────────────────────────
    void ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ekf_current_x_ = msg->pose.pose.position.x;
        ekf_current_y_ = msg->pose.pose.position.y;
        ekf_current_z_ = msg->pose.pose.position.z;

        if(!ekf_origin_set_)
        {
            ekf_origin_x_ = ekf_current_x_;
            ekf_origin_y_ = ekf_current_y_;
            ekf_origin_z_ = ekf_current_z_;
            ekf_origin_set_ = true;
            last_saved_x_  = ekf_current_x_;
            last_saved_y_  = ekf_current_y_;
            last_saved_z_  = ekf_current_z_;
            cout << "[EKF] Takeoff origin locked: ("
                 << ekf_origin_x_ << ", "
                 << ekf_origin_y_ << ", "
                 << ekf_origin_z_ << ")" << endl;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Image callback — SLAM tracking with IMU
    //
    // Every time a frame arrives:
    //   1. Drain the IMU buffer of all samples older than this frame.
    //   2. Call TrackMonocular_IMU() with the frame + IMU vector.
    //   3. The pose returned is already metric (IMU provides scale).
    //   4. Publish as nav_msgs/Odometry for ArduPilot (vision position).
    // ─────────────────────────────────────────────────────────────────────
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // ── decode image ─────────────────────────────────────────────────
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

        // ── image timestamp ──────────────────────────────────────────────
        double img_t = msg->header.stamp.sec +
                       msg->header.stamp.nanosec * 1e-9;

        // ── drain IMU buffer: collect all samples with t ≤ img_t ─────────
        //
        // ORB-SLAM3 needs all IMU samples that arrived BEFORE this frame.
        // Samples that arrive AFTER the frame's timestamp are left in the
        // buffer for the next image.
        vector<ORB_SLAM3::IMU::Point> vImu;
        {
            lock_guard<mutex> lk(imu_mutex_);
            while(!imu_buf_.empty() && imu_buf_.front().t <= img_t)
            {
                vImu.push_back(imu_buf_.front());
                imu_buf_.pop_front();
            }
        }

        // ── SLAM tracking ────────────────────────────────────────────────
        // TrackMonocular_IMU returns Tcw (camera-to-world transform).
        // Returns identity / zero on tracking failure.
        auto Tcw = SLAM.TrackMonocular(frame, img_t, vImu);

        if(Tcw.matrix().isZero(0)) return;  // tracking lost / not init'd yet

        // ── extract metric pose ──────────────────────────────────────────
        // Twc = world-to-camera inverse = camera position in world frame
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f t_wc = Twc.translation();
        Eigen::Quaternionf q(Twc.rotationMatrix());

        // ── set SLAM origin on first valid pose ──────────────────────────
        // Keeps output as displacement from startup rather than absolute.
        if(!slam_origin_set_)
        {
            slam_origin_x_ = t_wc.x();
            slam_origin_y_ = t_wc.y();
            slam_origin_z_ = t_wc.z();
            slam_origin_set_ = true;
            cout << "[SLAM] VIO initialised. First metric pose captured." << endl;
        }

        // Relative metric displacement from SLAM start
        // IMU_MONOCULAR mode → these are already in metres (no external scale needed)
        double mx = t_wc.x() - slam_origin_x_;
        double my = t_wc.y() - slam_origin_y_;
        double mz = t_wc.z() - slam_origin_z_;

        // ── publish pose for ArduPilot ───────────────────────────────────
        //
        // ArduPilot expects vision position on a topic it can consume.
        // If you are using the VISION_POSITION_ESTIMATE MAVLink path,
        // you may need a separate bridge node. Alternatively, configure
        // ArduCopter's EKF3 to fuse this topic directly via ROS MAVLink.
        //
        // Frame note: ORB-SLAM3 uses a right-handed frame where
        //   +X = right, +Y = down, +Z = forward  (camera optical).
        // ArduPilot NED expects +X = North, +Y = East, +Z = Down.
        // You may need a frame rotation depending on camera mounting.
        // Add a tf2 transform or rotate mx/my/mz here if needed.

        nav_msgs::msg::Odometry pose_msg;
        pose_msg.header.stamp    = msg->header.stamp;
        pose_msg.header.frame_id = "odom";
        pose_msg.child_frame_id  = "base_link";

        pose_msg.pose.pose.position.x    = mx;
        pose_msg.pose.pose.position.y    = my;
        pose_msg.pose.pose.position.z    = mz;
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        pose_pub_->publish(pose_msg);

        // ── pose log ─────────────────────────────────────────────────────
        pose_file_ << fixed << setprecision(6)
                   << img_t << "  "
                   << mx << "  " << my << "  " << mz << "  "
                   << q.x() << "  " << q.y() << "  "
                   << q.z() << "  " << q.w() << "\n";
        pose_file_.flush();

        // ── save image + coord every 10 cm of EKF movement ───────────────
        if(ekf_origin_set_)
        {
            double moved = dist3d(
                last_saved_x_, last_saved_y_, last_saved_z_,
                ekf_current_x_, ekf_current_y_, ekf_current_z_);

            if(moved >= 0.10)
            {
                image_count_++;

                ostringstream fname;
                fname << "/home/saakshi-v/slam_images_imu/frame_"
                      << setw(4) << setfill('0') << image_count_
                      << ".png";
                cv::imwrite(fname.str(), frame);

                coord_file_ << fname.str()   << "  "
                            << fixed << setprecision(4)
                            << ekf_current_x_ - ekf_origin_x_ << "  "
                            << ekf_current_y_ - ekf_origin_y_ << "  "
                            << ekf_current_z_ - ekf_origin_z_ << "  "
                            << mx << "  " << my << "  " << mz << "\n";
                coord_file_.flush();

                last_saved_x_ = ekf_current_x_;
                last_saved_y_ = ekf_current_y_;
                last_saved_z_ = ekf_current_z_;

                cout << "[SAVED] frame_" << image_count_
                     << "  EKF_delta=("
                     << ekf_current_x_ - ekf_origin_x_ << ", "
                     << ekf_current_y_ - ekf_origin_y_ << ", "
                     << ekf_current_z_ - ekf_origin_z_ << ") m"
                     << "  SLAM=(" << mx << ", " << my << ", " << mz << ") m"
                     << endl;
            }
        }

        this_thread::sleep_for(chrono::milliseconds(1));
    }

    // ─────────────────────────────────────────────────────────────────────
    // Return-to-origin — 10Hz proportional controller (same as original)
    // Uses EKF position for closed-loop control.
    // ─────────────────────────────────────────────────────────────────────
    void return_to_origin_step()
    {
        if(!returning_)        return;
        if(!ekf_origin_set_)   return;

        double ex = ekf_origin_x_ - ekf_current_x_;
        double ey = ekf_origin_y_ - ekf_current_y_;
        double ez = ekf_origin_z_ - ekf_current_z_;
        double error = sqrt(ex*ex + ey*ey + ez*ez);

        if(error < 0.15)   // within 15 cm — done
        {
            cout << "[RTO] Reached origin. Hovering." << endl;
            send_zero_velocity();
            returning_ = false;
            return;
        }

        // Proportional gain — increase for faster return, decrease to reduce overshoot
        const double Kp = 0.4;

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp    = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x  = Kp * ex;
        cmd.twist.linear.y  = Kp * ey;
        cmd.twist.linear.z  = Kp * ez;

        // Cap at 0.5 m/s for safety
        double speed = sqrt(
            cmd.twist.linear.x * cmd.twist.linear.x +
            cmd.twist.linear.y * cmd.twist.linear.y +
            cmd.twist.linear.z * cmd.twist.linear.z);

        if(speed > 0.5)
        {
            double sc = 0.5 / speed;
            cmd.twist.linear.x *= sc;
            cmd.twist.linear.y *= sc;
            cmd.twist.linear.z *= sc;
        }

        cmd_vel_pub_->publish(cmd);

        cout << "[RTO] error=" << error << " m  vel=("
             << cmd.twist.linear.x << ", "
             << cmd.twist.linear.y << ", "
             << cmd.twist.linear.z << ")" << endl;
    }

    void send_zero_velocity()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp    = this->now();
        cmd.header.frame_id = "base_link";
        // all fields default to 0
        cmd_vel_pub_->publish(cmd);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Save map point cloud as PLY (same as original)
    // ─────────────────────────────────────────────────────────────────────
    void save_point_cloud(const string& filename)
    {
        cout << "Saving point cloud…" << endl;

        ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
        if(!pAtlas) { cerr << "Atlas null" << endl; return; }

        vector<ORB_SLAM3::MapPoint*> points = pAtlas->GetAllMapPoints();

        int valid = 0;
        for(auto p : points)
            if(p && !p->isBad()) valid++;

        ofstream f(filename);
        f << "ply\nformat ascii 1.0\n"
          << "element vertex " << valid << "\n"
          << "property float x\nproperty float y\nproperty float z\n"
          << "end_header\n";

        for(auto p : points)
        {
            if(p && !p->isBad())
            {
                Eigen::Vector3f pos = p->GetWorldPos();
                f << fixed << setprecision(6)
                  << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
            }
        }

        f.close();
        cout << "Point cloud: " << valid << " points → " << filename << endl;
    }

    // ── ROS interfaces ────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr    image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr      imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    ekf_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr                                rto_timer_;

    // ── ORB-SLAM3 ─────────────────────────────────────────────────────────
    ORB_SLAM3::System SLAM;

    // ── IMU buffer ────────────────────────────────────────────────────────
    deque<ORB_SLAM3::IMU::Point> imu_buf_;
    mutex                         imu_mutex_;

    // ── EKF state ─────────────────────────────────────────────────────────
    bool   ekf_origin_set_;
    double ekf_origin_x_, ekf_origin_y_, ekf_origin_z_;
    double ekf_current_x_, ekf_current_y_, ekf_current_z_;
    double last_saved_x_,  last_saved_y_,  last_saved_z_;

    // ── SLAM origin ───────────────────────────────────────────────────────
    bool   slam_origin_set_;
    double slam_origin_x_, slam_origin_y_, slam_origin_z_;

    // ── misc ──────────────────────────────────────────────────────────────
    int  image_count_;
    bool returning_;

    ofstream pose_file_;
    ofstream coord_file_;
};

// ── signal handler ────────────────────────────────────────────────────────────
void signal_handler(int)
{
    if(g_node)
    {
        auto node = dynamic_pointer_cast<SlamImuNode>(g_node);
        if(node) node->save_and_shutdown();
    }
    rclcpp::shutdown();
}

// ── keyboard thread ───────────────────────────────────────────────────────────
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
                auto node = dynamic_pointer_cast<SlamImuNode>(g_node);
                if(node) node->trigger_return_to_origin();
            }
        }
        else if(input == "q" || input == "Q")
        {
            cout << "Quit requested…" << endl;
            shutdown_flag = true;
            if(g_node)
            {
                auto node = dynamic_pointer_cast<SlamImuNode>(g_node);
                if(node) node->save_and_shutdown();
            }
            rclcpp::shutdown();
            break;
        }
    }
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    auto node = make_shared<SlamImuNode>();
    g_node = node;

    // keyboard listener in background thread
    thread kb_thread(keyboard_thread);
    kb_thread.detach();

    rclcpp::spin(node);

    shutdown_flag = true;
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
            image =
                cv_bridge::toCvCopy(
                    msg,
                    "mono8")->image;
        }
        catch(...)
        {
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

            while(
                !imu_buffer_.empty()
                &&
                imu_buffer_.front().t
                    <= tframe)
            {
                imu_measurements.push_back(
                    imu_buffer_.front());

                imu_buffer_.pop_front();
            }
        }

        Sophus::SE3f pose =
            slam_->TrackMonocular(
                image,
                tframe,
                imu_measurements);

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