#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode();

private:

    /// @brief Callback invoked on new odometry data; updates the current longitudinal speed used for iTTC.
    /// @param msg Shared pointer to the incoming Odometry message.
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    /// @brief Callback invoked on each new lidar scan; computes per-beam iTTC and 
    /// triggers an emergency brake if any beam is below the threshold.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief Callback invoked on new joystick input; updates deadman button state and last-seen time.
    /// @param joy_msg Shared pointer to the incoming Joy message.
    void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);

    /// @brief Callback invoked when a controller node publishes a drive command; forwards it to /drive 
    /// unless gated by estop or deadman.
    /// @param drive_msg Shared pointer to the incoming raw AckermannDriveStamped message.
    void drive_raw_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg);

    /// @brief Checks whether the deadman button is currently held and its signal is still fresh.
    /// @return True if driving is currently permitted by the deadman gate, false otherwise.
    bool deadman_engaged();

    /// @brief Publishes a drive command with the given speed and steering angle to /drive.
    /// @param speed Speed to command, in m/s.
    /// @param steering_angle Steering angle to command, in radians.
    void publish_drive(float speed, float steering_angle);

    // Subscriptions / publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_raw_sub_;

    // iTTC state
    float speed_ = 0.0f;
    bool estop_active_ = false;

    // Deadman state
    bool deadman_held_ = false;
    bool joy_received_ = false;
    rclcpp::Time last_joy_time_;

    // Params
    float ttc_threshold_;
    int deadman_button_;
    double joy_timeout_sec_;
    bool enable_deadman_;
    double max_ttc_angle_deg_;
};
