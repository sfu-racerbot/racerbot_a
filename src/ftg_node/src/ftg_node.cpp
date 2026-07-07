#include "ftg_node.hpp"

FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node")
{
    RCLCPP_INFO(this->get_logger(), "Follow the gap node started");

    this->drive_msg_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}