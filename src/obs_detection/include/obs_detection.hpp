#ifndef OBS_DETECT_NODE_HPP_
#define OBS_DETECT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

class ObsDetectionNode : public rclcpp::Node
{
public:
    ObsDetectionNode();

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

    // member variables for thresholds // just a placeholder variable block lol
    double obstacle_distance_;
    double obstacle_edge_jump_;
    double car_width_extended_;
};

#endif 