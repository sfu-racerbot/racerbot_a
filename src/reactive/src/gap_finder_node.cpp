#include "gap_finder_node.hpp"
using std::vector;

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");

    // Declare with a default value
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("car_width", 0.3);
    this->declare_parameter("bubble_radius", 0.35);
    this->declare_parameter("disparity_threshold", 2.0);
    this->declare_parameter("fov_half_angle", M_PI/2.0); // 90 degrees

    // Read into member variables
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    car_width_ = this->get_parameter("car_width").as_double();
    bubble_radius_ = this->get_parameter("bubble_radius").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle").as_double();
    
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&GapFinderNode::lidar_callback, this, std::placeholders::_1));
}

void GapFinderNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    extend_obstacles(scan_msg, ranges);
    int gap_index = find_furthest_gap(scan_msg, ranges);
}

vector<float> GapFinderNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    vector<float> ranges = scan_msg->ranges;

    // Capping values past max_lidar_range, and rejecting values past fov angle range
    for (size_t i = 0; i < ranges.size(); ++i) {
        if (std::abs(scan_msg->angle_min + scan_msg->angle_increment * i) > fov_half_angle_) {
            ranges[i] = 0;
        }
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }

    return ranges;
}

void GapFinderNode::extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    // TODO
}

int GapFinderNode::find_furthest_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    // TODO
    return 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFinderNode>());
    rclcpp::shutdown();
    return 0;
}
