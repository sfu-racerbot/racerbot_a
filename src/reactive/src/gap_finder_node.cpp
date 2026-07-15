#include "gap_finder_node.hpp"

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");
    
    this->drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&GapFinderNode::lidar_callback, this, std::placeholders::_1));

}

void GapFinderNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    
}

std::vector<float> GapFinderNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFinderNode>());
    rclcpp::shutdown();
    return 0;
}
