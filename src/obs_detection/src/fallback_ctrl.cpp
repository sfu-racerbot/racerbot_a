#include "obs_detection/fallback_ctrl.hpp"

FallbackCtrl::FallbackCtrl() : Node("fallback_ctrl_node"), obstacle_detected_(false)
{
    // Subscriptions
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "obstacle_ahead", 10,
        std::bind(&FallbackCtrl::obstacle_callback, this, std::placeholders::_1));

    pp_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/pure_pursuit/drive", 10,
        std::bind(&FallbackCtrl::pp_callback, this, std::placeholders::_1));

    ftg_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ftg/drive", 10,
        std::bind(&FallbackCtrl::ftg_callback, this, std::placeholders::_1));

    // Final multiplexed drive output to the robot/simulator
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    RCLCPP_INFO(this->get_logger(), "Fallback Ctrl Node initialized.");
}

void FallbackCtrl::obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    obstacle_detected_ = msg->data;
}

void FallbackCtrl::pp_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    latest_pp_msg_ = msg;
    if (!obstacle_detected_) {
        publish_command();
    }
}

void FallbackCtrl::ftg_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    latest_ftg_msg_ = msg;
    if (obstacle_detected_) {
        publish_command();
    }
}

void FallbackCtrl::publish_command()
{
    if (obstacle_detected_ && latest_ftg_msg_) {
        // Switch to Follow The Gap when obstacle is present
        drive_pub_->publish(*latest_ftg_msg_);
    } else if (!obstacle_detected_ && latest_pp_msg_) {
        // Default to Pure Pursuit for waypoint tracking
        drive_pub_->publish(*latest_pp_msg_);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FallbackCtrl>());
    rclcpp::shutdown();
    return 0;
}