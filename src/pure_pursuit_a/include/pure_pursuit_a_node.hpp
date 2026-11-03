#pragma once

#include <string>
#include <vector>
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

struct Waypoint
{
    double x;
    double y;
};

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_brake_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
    std::vector<Waypoint> waypoints_;
    bool use_pf_;                 // false = sim ground truth, true = particle filter (irl)
    std::string odom_topic_;
    double current_speed_ = 0.0;
    bool enable_safety_;
    bool safety_brake_engaged_;
    double safety_bubble_scale_;
    double lookahead_base_;        // base lookahead L0 (m)
    double lookahead_min_;         // lower clamp (m)
    double lookahead_max_;         // upper clamp (m)
    double lookahead_speed_gain_;  // extra lookahead (m) per m/s of speed
    size_t last_goal_index_ = 0;   // index of last chosen goal
    double wheel_base_;            // front-to-rear axle distance (m)
    double max_steering_;          // steering angle limit (rad)
    double max_speed_;             // speed on straights (m/s)
    double min_speed_;             // speed at full steering lock (m/s)
    bool initialized_ = false;     // false until first odom, then closest waypoint is found once

    void speed_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);

    /// @brief Runs everytime something is published to the safety brake topic
    void safety_brake_callback(const std_msgs::msg::Bool::ConstSharedPtr safety_brake_msg);

    /// @brief Loads the waypoint path from a file into waypoints_.
    /// @param path Path to the waypoint file (e.g. a CSV with x, y per row).
    void load_waypoints(const std::string& path);

    /// @brief Computes the speed-scaled lookahead distance.
    /// @param v Current forward speed of the car (m/s).
    /// @return Lookahead distance in meters.
    double currentLookahead(double v) const;

    /// @brief Computes the pure pursuit steering angle and target speed for a goal
    ///        point and publishes the drive command.
    /// @param local_x Goal point distance ahead of the car (m), in the car frame.
    /// @param local_y Goal point lateral offset (m), in the car frame. Positive is left.
    void drive_to_goal(double local_x, double local_y);

    /// @brief Odometry callback: finds the goal waypoint at the current lookahead
    ///        distance in front of the car, and computes the pure pursuit steering.
    /// @param msg Odometry message with the car's pose and velocity.
    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
};
