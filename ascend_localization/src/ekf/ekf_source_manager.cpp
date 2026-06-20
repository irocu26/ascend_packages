// ekf_source_manager.cpp
//
// Watches ORB-SLAM3 tracking state and selects the ArduPilot EKF source set on
// the fly, so the vehicle falls back from SLAM (external nav) to optical-flow +
// rangefinder when SLAM tracking is lost, and switches back when it recovers.
//
// Mechanism (no MAVROS): ArduPilot's "EKF Source Set" is a 3-position aux
// function (RCx_OPTION = 90). AP_DDS lets a companion drive RC channels via a
// sensor_msgs/Joy on /ap/joy:
//   - axes[i] maps to RC channel (i+1), normalised [-1..+1] -> [RCmin..RCmax]
//   - axes[i] = NaN releases that channel back to the real RC
//   - overrides time out, so we must republish on a timer
// This deployment configures EK3 source set 1 = optical flow + rangefinder and
// source set 2 = external nav (SLAM). So we hold the aux channel (default RC8)
// MID (0.0 -> source set 2 = SLAM) when tracking is healthy and LOW (-1.0 ->
// source set 1 = optical flow + rangefinder) when lost.
//
// Also publishes /ascend/localization/slam_ok (Bool) for the mission-control
// FSM, which uses it to enter/leave its NAV_DEGRADED hold.
//
// Hysteresis avoids flapping: SLAM must be OK for `ok_frames_to_recover`
// consecutive frames to (re)acquire, and lost for `lost_frames_to_degrade`
// consecutive frames to degrade. Degradation is only allowed AFTER SLAM has
// been acquired at least once, so normal SLAM initialisation on the ground
// (state != OK) does not trip a false fallback.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class EkfSourceManager : public rclcpp::Node
{
public:
  EkfSourceManager()
  : Node("ekf_source_manager")
  {
    // ORB-SLAM3 Tracking::eTrackingState: OK == 2 (1=NOT_INITIALIZED, 3=RECENTLY_LOST, 4=LOST)
    ok_state_       = this->declare_parameter<int>("slam_ok_state", 2);
    ok_to_recover_  = this->declare_parameter<int>("ok_frames_to_recover", 5);
    lost_to_degrade_= this->declare_parameter<int>("lost_frames_to_degrade", 10);
    aux_channel_    = this->declare_parameter<int>("aux_rc_channel", 8);   // RC channel with RCx_OPTION=90
    publish_hz_     = this->declare_parameter<double>("joy_publish_hz", 10.0);
    axis_slam_      = this->declare_parameter<double>("axis_value_slam", 0.0);   // MID -> source set 2 (external nav / SLAM)
    axis_flow_      = this->declare_parameter<double>("axis_value_flow", -1.0);  // LOW -> source set 1 (optical flow + rangefinder)

    if (aux_channel_ < 1 || aux_channel_ > 8) {
      RCLCPP_WARN(get_logger(),
        "aux_rc_channel=%d out of AP_DDS range [1..8]; clamping to 8.", aux_channel_);
      aux_channel_ = std::min(std::max(aux_channel_, 1), 8);
    }

    // Start assuming SLAM (source set 2) is good. We do NOT degrade until SLAM
    // has been acquired at least once, so SLAM startup is not mistaken for a
    // tracking loss.
    slam_ok_       = true;
    ever_acquired_ = false;
    ok_run_        = 0;
    lost_run_      = 0;

    state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/orbslam/tracking_state", 10,
      std::bind(&EkfSourceManager::onState, this, std::placeholders::_1));

    ok_pub_  = this->create_publisher<std_msgs::msg::Bool>("/ascend/localization/slam_ok", 10);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/ap/joy", 10);

    const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, publish_hz_)));
    timer_ = this->create_wall_timer(period, std::bind(&EkfSourceManager::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "ekf_source_manager up: aux RC%d, SLAM axis %.1f / FLOW axis %.1f, "
      "recover>=%d ok-frames, degrade>=%d lost-frames, %.0f Hz.",
      aux_channel_, axis_slam_, axis_flow_, ok_to_recover_, lost_to_degrade_, publish_hz_);
  }

private:
  void onState(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const bool frame_ok = (msg->data == ok_state_);
    if (frame_ok) { ok_run_++;   lost_run_ = 0; }
    else          { lost_run_++; ok_run_   = 0; }

    if (!slam_ok_ && ok_run_ >= ok_to_recover_) {
      slam_ok_ = true;
      RCLCPP_INFO(get_logger(),
        "SLAM reacquired (%d consecutive OK) -> EKF source set 2 (external nav / SLAM).", ok_run_);
    } else if (slam_ok_ && ever_acquired_ && lost_run_ >= lost_to_degrade_) {
      slam_ok_ = false;
      RCLCPP_WARN(get_logger(),
        "SLAM tracking lost (%d consecutive non-OK) -> EKF source set 1 (optical flow + rangefinder).",
        lost_run_);
    }

    if (ok_run_ >= ok_to_recover_) {
      ever_acquired_ = true;   // first acquisition gates future degradation
    }
  }

  void onTimer()
  {
    // 1) Tell the FSM whether SLAM nav is trustworthy.
    std_msgs::msg::Bool ok;
    ok.data = slam_ok_;
    ok_pub_->publish(ok);

    // 2) Hold the EKF-source aux channel; release all other channels (NaN).
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = this->now();
    const int n = std::max(4, aux_channel_);  // AP_DDS ignores Joy with <4 axes
    joy.axes.assign(n, std::numeric_limits<float>::quiet_NaN());
    const int idx = aux_channel_ - 1;          // RC channel -> 0-based axis index
    joy.axes[idx] = static_cast<float>(slam_ok_ ? axis_slam_ : axis_flow_);
    joy_pub_->publish(joy);
  }

  // Parameters
  int    ok_state_;
  int    ok_to_recover_;
  int    lost_to_degrade_;
  int    aux_channel_;
  double publish_hz_;
  double axis_slam_;
  double axis_flow_;

  // State
  bool slam_ok_;
  bool ever_acquired_;
  int  ok_run_;
  int  lost_run_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      ok_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr    joy_pub_;
  rclcpp::TimerBase::SharedPtr                           timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfSourceManager>());
  rclcpp::shutdown();
  return 0;
}
