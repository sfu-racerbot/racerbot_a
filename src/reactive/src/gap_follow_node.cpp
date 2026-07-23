#include "gap_follow_node.hpp"

GapFollowNode::GapFollowNode() : Node("gap_follow_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Follow node started");

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    gap_sub_ = this->create_subscription<reactive::msg::Gap>(
    "gap", 10,
    std::bind(&GapFollowNode::gap_callback, this, std::placeholders::_1));
}

void GapFollowNode::gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
    drive_best_point(gap_msg);
}

void GapFollowNode::drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
    float abs_angle = std::abs(gap_msg->target_angle);
    float velocity;
    if (abs_angle < 10.0f * M_PI / 180.0f) velocity = 2.0f;
    else if (abs_angle < 20.0f * M_PI / 180.0f) velocity = 1.5f;
    else velocity = 0.5f;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = std::clamp(gap_msg->target_angle, -0.4189f, 0.4189f);
    drive_msg.drive.speed = velocity;
    drive_pub_->publish(drive_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollowNode>());
    rclcpp::shutdown();
    return 0;
}