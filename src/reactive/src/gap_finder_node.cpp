#include "gap_finder_node.hpp"
using std::vector;

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");

    // Declare with a default value
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("car_width_extended", 0.35);
    this->declare_parameter("disparity_threshold", 2.0);
    this->declare_parameter("fov_half_angle_deg", 90.0);
    this->declare_parameter("minimum_gap_threshold", 1.0);

    // Read into member variables
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();
    
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&GapFinderNode::lidar_callback, this, std::placeholders::_1));

    gap_pub_ = this->create_publisher<reactive::msg::Gap>("gap", 10);
}

void GapFinderNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    extend_obstacles(scan_msg, ranges);
    int gap_index = find_furthest_gap(scan_msg, ranges);
    
    if (gap_index == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid gap found");
        return;
    }

    reactive::msg::Gap gap_msg;
    gap_msg.angles.reserve(ranges.size());
    gap_msg.ranges.reserve(ranges.size());

    for (size_t i = 0; i < ranges.size(); ++i) {
        gap_msg.angles.push_back(scan_msg->angle_min + scan_msg->angle_increment * i);
        gap_msg.ranges.push_back(ranges[i]);
    }

    gap_msg.target_angle = scan_msg->angle_min + scan_msg->angle_increment * gap_index;
    gap_msg.target_range = ranges[gap_index];

    gap_pub_->publish(gap_msg);
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
    if (ranges.empty()) return;

    float prev_reading = ranges[0];
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        float current_reading = ranges[i];

        if (std::abs(prev_reading - current_reading) > disparity_threshold_)
        {
            float closer_range = std::min(prev_reading, current_reading);
            
            // Safety bubble: arc length s = r * theta -> theta = car_width_extended_ / r
            double theta = car_width_extended_ / closer_range;
            // Index increment: Used on each side so divide by 2
            size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);
            
            size_t start = (i > index_increment) ? i - index_increment : 0;
            size_t end = std::min(i + index_increment + 1, ranges.size());
            
            for (size_t j = start; j < end; ++j) {
                ranges[j] = std::min(ranges[j], closer_range); // don't overwrite an already-closer point
            }
        }
        prev_reading = ranges[i];
    }
}

int GapFinderNode::find_furthest_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    int best_index = -1;
    float furthest_range = 0.0;

    size_t i = 0;
    while (i < ranges.size())
    {
        // Skip points that aren't part of a "free" gap
        if (ranges[i] <= minimum_gap_threshold_)
        {
            ++i;
            continue;
        }

        // Found the start of a gap - walk forward to find its end
        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_)
        {
            ++i;
        }
        size_t gap_end = i; // exclusive

        // Check if the gap is wide enough for the car to fit through,
        // using the closest range within the gap as the worst-case radius
        float min_range_in_gap = *std::min_element(ranges.begin() + gap_start, ranges.begin() + gap_end);
        double theta = car_width_extended_ / min_range_in_gap;
        size_t min_index_width = static_cast<size_t>(theta / scan_msg->angle_increment);

        if ((gap_end - gap_start) < min_index_width)
        {
            continue; // gap too narrow for the car, skip it
        }

        // Gap is valid - find the furthest point within it
        for (size_t j = gap_start; j < gap_end; ++j)
        {
            if (ranges[j] > furthest_range)
            {
                furthest_range = ranges[j];
                best_index = static_cast<int>(j);
            }
        }
    }

    return best_index;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFinderNode>());
    rclcpp::shutdown();
    return 0;
}
