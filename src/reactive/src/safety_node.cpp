#include "safety_node.hpp"
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

SafetyNode::SafetyNode() : Node("safety_node")
{
    RCLCPP_INFO(this->get_logger(), "Safety node started");

    this->declare_parameter("ttc_threshold", 0.3);
    this->declare_parameter("deadman_button", 4);
    this->declare_parameter("joy_timeout_sec", 0.5);
    this->declare_parameter("enable_deadman", true);
    this->declare_parameter("max_ttc_angle_deg", 90.0);

    ttc_threshold_ = static_cast<float>(this->get_parameter("ttc_threshold").as_double());
    deadman_button_ = this->get_parameter("deadman_button").as_int();
    joy_timeout_sec_ = this->get_parameter("joy_timeout_sec").as_double();
    enable_deadman_ = this->get_parameter("enable_deadman").as_bool();
    max_ttc_angle_deg_ = this->get_parameter("max_ttc_angle_deg").as_double() * M_PI / 180.0;

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 1);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 1, std::bind(&SafetyNode::odometry_callback, this, _1));

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 1, std::bind(&SafetyNode::scan_callback, this, _1));

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&SafetyNode::joy_callback, this, _1));

    drive_raw_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive_raw", 10, std::bind(&SafetyNode::drive_raw_callback, this, _1));
}

void SafetyNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    speed_ = static_cast<float>(msg->twist.twist.linear.x);
}

void SafetyNode::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
    last_joy_time_ = this->now();
    joy_received_ = true;

    if (static_cast<int>(joy_msg->buttons.size()) > deadman_button_) {
        deadman_held_ = joy_msg->buttons[deadman_button_] != 0;
    } else {
        deadman_held_ = false;
    }
}

bool SafetyNode::deadman_engaged()
{
    if (!enable_deadman_) return true;
    if (!deadman_held_ || !joy_received_) return false;

    double age_sec = (this->now() - last_joy_time_).seconds();
    return age_sec < joy_timeout_sec_;
}

void SafetyNode::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    // TTC = r / (+rdot), where range rate is positive when getting closer
    estop_active_ = false;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        float range = scan_msg->ranges[i];
        if (!std::isfinite(range)) continue; // skip if no object in proximity

        float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        if (std::abs(angle) > max_ttc_angle_deg_) continue;

        float range_rate = speed_ * std::cos(angle);
        float closing_rate = std::max(range_rate, 0.0f);
        if (closing_rate <= 1e-3f) continue; // skip if object is not approaching

        float iTTC = range / closing_rate;

        if (iTTC <= ttc_threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "Brake! iTTC=%.2f", iTTC);
            estop_active_ = true;
            publish_drive(0.0f, 0.0f);
            break;
        }
    }
}

void SafetyNode::drive_raw_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg)
{
    if (estop_active_ || !deadman_engaged())
    {
        publish_drive(0.0f, 0.0f);
        return;
    }

    publish_drive(
        static_cast<float>(drive_msg->drive.speed),
        static_cast<float>(drive_msg->drive.steering_angle));
}

void SafetyNode::publish_drive(float speed, float steering_angle)
{
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;
    drive_pub_->publish(drive_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}
