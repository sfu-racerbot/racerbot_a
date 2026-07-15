#include "gap_finder_node.hpp"

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");

    // Declare with a default value
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("car_width", 0.3);
    this->declare_parameter("bubble_radius", 0.35);
    this->declare_parameter("disparity_threshold", 2.0);
    this->declare_parameter("min_angle", -M_PI/2.0); // -90 degrees
    this->declare_parameter("max_angle", M_PI/2.0); // 90 degrees

    // Read into member variables
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    car_width_ = this->get_parameter("car_width").as_double();
    bubble_radius_ = this->get_parameter("bubble_radius").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    min_angle_ = this->get_parameter("min_angle").as_double();
    max_angle_ = this->get_parameter("max_angle").as_double();
    
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
