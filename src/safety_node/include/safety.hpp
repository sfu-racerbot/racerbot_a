#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode();
  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);

  /// @brief Callback invoked on new odometry data; updates the current longitudinal speed used for iTTC.
  /// @param msg Shared pointer to the incoming Odometry message.
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// @brief Callback invoked on each new lidar scan; computes per-beam iTTC and
  /// triggers an emergency brake if any beam is below the threshold.
  /// @param scan_msg Shared pointer to the incoming LaserScan message.
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  /// @brief Callback called every 50ms to update the state of the safety_node and publish to the safety brake topic
  void safety_update();

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_brake_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  int deadman_button_;
  double message_timeout_duration_ = 50000000.0;
  bool enable_ttc_;
  double max_ttc_angle_deg_;
  float ttc_threshold_;

  bool last_deadman_state;
  bool joystick_active_;
  bool deadman_held_;
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;

  // iTTC state
  float speed_ = 0.0f;
  bool estop_active_ = false;
};