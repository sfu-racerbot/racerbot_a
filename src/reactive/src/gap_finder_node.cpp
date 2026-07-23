#include "gap_finder_node.hpp"
using std::vector;

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");

    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("car_width_extended", 0.35);
    this->declare_parameter("disparity_threshold", 0.4);
    this->declare_parameter("fov_half_angle", M_PI/2.0);
    this->declare_parameter("minimum_gap_threshold", 1.0);

    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle").as_double();
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

    for (size_t i = 0; i < ranges.size(); ++i) {
        if (std::abs(scan_msg->angle_min + scan_msg->angle_increment * i) > fov_half_angle_) {
            ranges[i] = 0;
        }
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }


    for (size_t i = 1; i < ranges.size() - 1; ++i) {
        ranges[i] = (ranges[i-1] + ranges[i] + ranges[i+1]) / 3.0f;
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
            
            double theta = car_width_extended_ / closer_range;
            size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);
            
            size_t start = (i > index_increment) ? i - index_increment : 0;
            size_t end = std::min(i + index_increment + 1, ranges.size());
            
            for (size_t j = start; j < end; ++j) {
                ranges[j] = std::min(ranges[j], closer_range);
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
        if (ranges[i] <= minimum_gap_threshold_)
        {
            ++i;
            continue;
        }

        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_)
        {
            ++i;
        }
        size_t gap_end = i;

        float sum = 0.0f;
        for (size_t j = gap_start; j < gap_end; ++j) sum += ranges[j];
        float avg_range_in_gap = sum / (gap_end - gap_start);
        double theta = car_width_extended_ / avg_range_in_gap;
        size_t min_index_width = static_cast<size_t>(theta / scan_msg->angle_increment);

        if ((gap_end - gap_start) < min_index_width)
        {
            continue;
        }

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