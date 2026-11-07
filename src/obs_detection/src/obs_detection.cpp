#include "../include/obs_detection.hpp"

ObsDetectionNode::ObsDetectionNode() : Node("obs_detection_node")
{
    RCLCPP_INFO(this->get_logger(), "Obs_detection node started");

    // declare node parameters // placeholder setup stuff
    this->declare_parameter("obstacle_distance", 2.0);
    this->declare_parameter("obstacle_edge_jump", 0.5);
    this->declare_parameter("car_width_extended", 0.55);

    obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
    obstacle_edge_jump_ = this->get_parameter("obstacle_edge_jump").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ObsDetectionNode::lidar_callback, this, std::placeholders::_1));

    obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_ahead", 10);
}

void ObsDetectionNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    // obstacle detection core logic structure
    bool obstacle_detected = false; // placeholder result for now // just a placeholder

    // TODO: implement lane bounding, forward_min proximity, and edge jump checks here
    // placeholder logic loop // totally unfinished placeholder code here

    std_msgs::msg::Bool msg;
    msg.data = obstacle_detected;
    obstacle_pub_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObsDetectionNode>());
    rclcpp::shutdown();
    return 0;
}