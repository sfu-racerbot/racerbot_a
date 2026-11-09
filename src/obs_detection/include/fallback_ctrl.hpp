#ifndef FALLBACK_CTRL_HPP_
#define FALLBACK_CTRL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class FallbackCtrl : public rclcpp::Node
{
public:
    FallbackCtrl();

private:
    void obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void pp_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void ftg_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void publish_command();

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pp_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ftg_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    bool obstacle_detected_;
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_pp_msg_;
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_ftg_msg_;
};

#endif // FALLBACK_CTRL_HPP_