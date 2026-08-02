#include "ftg_node.hpp"
using std::vector;

FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node")
{
    RCLCPP_INFO(this->get_logger(), "Follow the gap node started");

    // ROS2 parameters — tune at launch without recompiling
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("fov_half_angle_deg", 90.0);
    this->declare_parameter("car_width", 0.35);
    this->declare_parameter("minimum_gap_threshold", 1.0);

    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    car_width_ = this->get_parameter("car_width").as_double();
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();

    this->drive_msg_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
}

vector<float> FollowTheGapNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    vector<float> ranges = scan_msg->ranges;

    for (size_t i = 0; i < ranges.size(); ++i) {
        float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        if (std::abs(angle) > fov_half_angle_) ranges[i] = 0.0f;
        if (!std::isfinite(ranges[i]) || ranges[i] < scan_msg->range_min) ranges[i] = 0.0f;
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }

    // 3-point moving average to reduce noise
    for (size_t i = 1; i < ranges.size() - 1; ++i) {
        ranges[i] = (ranges[i-1] + ranges[i] + ranges[i+1]) / 3.0f;
    }

    return ranges;
}

void FollowTheGapNode::draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    // find closest point, skipping zeroed beams from FOV clipping
    bool found_closest = false;
    size_t closest_index = 0;
    for (size_t i = 0; i < ranges.size(); ++i) {
        if (ranges[i] == 0.0f) continue;
        if (!found_closest || ranges[i] < ranges[closest_index]) {
            closest_index = i;
            found_closest = true;
        }
    }

    if (!found_closest) return;

    // exact angular half-width using atan2 — more accurate than small angle approx at close range
    double theta = 2.0 * std::atan2(car_width_ / 2.0, static_cast<double>(ranges[closest_index]));
    size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);

    size_t start = (closest_index > index_increment) ? closest_index - index_increment : 0;
    size_t end = std::min(closest_index + index_increment + 1, ranges.size());

    for (size_t i = start; i < end; ++i) {
        ranges[i] = 0.0f;
    }
}

vector<std::pair<int, int>> FollowTheGapNode::find_all_gaps(const vector<float>& ranges)
{
    vector<std::pair<int, int>> gaps;
    size_t i = 0;

    while (i < ranges.size()) {
        if (ranges[i] <= minimum_gap_threshold_) { ++i; continue; }

        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_) ++i;
        size_t gap_end = i;

        gaps.push_back({static_cast<int>(gap_start), static_cast<int>(gap_end)});
    }

    return gaps;
}

std::pair<int, int> FollowTheGapNode::pick_best_gap(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const vector<float>& ranges,
    const vector<std::pair<int, int>>& gaps)
{
    std::pair<int, int> best_gap = {-1, -1};
    float best_score = -1.0f;

    for (const auto& gap : gaps) {
        int gap_start = gap.first;
        int gap_end = gap.second;

        float sum = 0.0f;
        for (int j = gap_start; j < gap_end; ++j) sum += ranges[j];
        float avg_depth = sum / (gap_end - gap_start);

        // exact angular width check using atan2 instead of small angle approx
        double theta = 2.0 * std::atan2(car_width_ / 2.0, static_cast<double>(avg_depth));
        size_t min_width = static_cast<size_t>(theta / scan_msg->angle_increment);
        if (static_cast<size_t>(gap_end - gap_start) < min_width) continue;

        // width x depth scoring
        float score = (gap_end - gap_start) * avg_depth;

        if (score > best_score) {
            best_score = score;
            best_gap = gap;
        }
    }

    return best_gap;
}

int FollowTheGapNode::pick_best_point(const std::pair<int, int>& best_gap)
{
    if (best_gap.first == -1 || best_gap.second == -1) return -1;

    // pick center of the gap
    return best_gap.first + (best_gap.second - best_gap.first) / 2;
}

void FollowTheGapNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    draw_safety_bubble(scan_msg, ranges);
    auto gaps = find_all_gaps(ranges);
    auto best_gap = pick_best_gap(scan_msg, ranges, gaps);
    int best_idx = pick_best_point(best_gap);
                                            //skibidi milad
    if (best_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid gap found");
        return;
    }

    float steering_angle = scan_msg->angle_min + scan_msg->angle_increment * best_idx;
    // clamp to F1Tenth hardware limits
    steering_angle = std::clamp(steering_angle, -0.4189f, 0.4189f);

    float abs_angle = std::abs(steering_angle);
    float speed;
    if (abs_angle < 10.0f * M_PI / 180.0f)      speed = 2.0f;
    else if (abs_angle < 20.0f * M_PI / 180.0f) speed = 1.5f;
    else                                          speed = 0.5f;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = scan_msg->header.stamp;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    drive_msg_publisher->publish(drive_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}