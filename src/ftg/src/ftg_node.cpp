#include "ftg_node.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
using std::vector;

FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node") {
    RCLCPP_INFO(this->get_logger(), "Follow the gap node started");

    // ROS2 parameters
    this->declare_parameter("max_speed", 4.5);
    this->declare_parameter("min_speed", 0.7);
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("fov_half_angle_deg", 100.0);
    this->declare_parameter("car_width", 0.4);
    this->declare_parameter("minimum_gap_threshold", 1.0);
    this->declare_parameter("enable_safety", true);
    this->declare_parameter("safety_bubble_scale", 0.5);
    this->declare_parameter("prev_angle_adjuster", 0.6);
    this->declare_parameter("bubble_lookahead", 3.0);
    this->declare_parameter("bubble_half_width", 0.45);
    this->declare_parameter("wheelbase", 0.33);
    this->declare_parameter("lookahead_max", 1.8);
    this->declare_parameter("steering_deadzone",0.7);
    this->declare_parameter("steering_angle_max", 0.349);
    this->declare_parameter("max_lateral_acc",3.5);

    max_speed_ = this->get_parameter("max_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    car_width_ = this->get_parameter("car_width").as_double();
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();
    enable_safety_ = this->get_parameter("enable_safety").as_bool();
    safety_bubble_scale_ = this->get_parameter("safety_bubble_scale").as_double();
    prev_angle_adjuster_ = this->get_parameter("prev_angle_adjuster").as_double();
    bubble_lookahead_ = this->get_parameter("bubble_lookahead").as_double();
    bubble_half_width_ = this->get_parameter("bubble_half_width").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    lookahead_max_ = this->get_parameter("lookahead_max").as_double();
    steering_deadzone_ = this->get_parameter("steering_deadzone").as_double() * M_PI / 180.0;
    steering_angle_max_ = this->get_parameter("steering_angle_max").as_double();
    max_lateral_acc_ = this->get_parameter("max_lateral_acc").as_double();
    prev_angle_ = 0;

    this->drive_msg_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
        std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
    safety_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>("/safety/brake", 10, 
        std::bind(&FollowTheGapNode::safety_brake_callback, this, std::placeholders::_1));
}

vector<float> FollowTheGapNode::preprocess_lidar(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {

    const auto &ranges = scan_msg->ranges;
    const int n = ranges.size();

    // lambda: invalid or too-close readings become 0, others capped at max_lidar_range_
    auto clean = [&](int i) {
        float r = ranges[i];
        if (!std::isfinite(r) || r < scan_msg->range_min) {
            return 0.0f;
        }
        return std::min(r, static_cast<float>(max_lidar_range_));
    };

    // Find the first and last ranges in the considered FOV via fov_half_angle_
    const int first = std::max<int>(0,
        std::ceil((-fov_half_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));
    const int last = std::min<int>(n - 1,
        std::floor((fov_half_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));

    // Create zerod out output vector
    vector<float> processed(n, 0.0f);

    // loop over valid ranges cleaning them and applying them to output with smoothing
    for (int i = first; i <= last; i++) {
        if (i == 0 || i == n - 1) {
            processed[i] = clean(i);
            continue;
        }
        processed[i] = (clean(i - 1) + clean(i) + clean(i + 1)) / 3.0f;
    }

    return processed;
}

void FollowTheGapNode::draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float> &ranges) {
    // Find closest obstacle
    bool found_closest = false;
    size_t closest_index;
    for (size_t i = 1; i < ranges.size(); ++i) {
        if (ranges[i] == 0) {
            continue; //ignore 0s
        }

        // Only consider points inside a box ahead of the car (bubble_lookahead_ x bubble_half_width_)
        float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        float ahead = ranges[i] * std::cos(angle);
        float beside = ranges[i] * std::sin(angle);
        if (ahead <= 0.0f || ahead > bubble_lookahead_ ||
            std::abs(beside) > bubble_half_width_) {
            continue;
        }

        if (!found_closest || ranges[i] < ranges[closest_index]) {
            found_closest = true;
            closest_index = i;
        }
    }

    if (!found_closest)
    return;

    // Draw safety bubble by calculating theta for the arc made by r = closest
    // point with s = car width

    const double n = static_cast<double>(ranges.size());
    const double offset = std::abs(static_cast<double>(closest_index) - n / 2.0);

    // Adjust size of bubble based on the angle between the closest obstacle and
    // the car's facing direction
    double direction_adjustment = std::max(1.0 - 2.0 * offset / n, 0.0);

    // Adjust based on distance
    double distance_adjustment = std::min(0.1 / ranges[closest_index], 1.0);

    double theta = 2.0 * std::atan2(car_width_ * safety_bubble_scale_ * 
        std::max(direction_adjustment, distance_adjustment), ranges[closest_index]);

    size_t index_increment = (theta / scan_msg->angle_increment) / 2; // Used on each side so divide by 2

    size_t start = (closest_index > index_increment) ? closest_index - index_increment : 0;

    size_t end = std::min(closest_index + index_increment, ranges.size());

    for (size_t i = start; i < end; ++i) {
        ranges[i] = 0;
    }
}

vector<std::pair<int, int>> FollowTheGapNode::find_all_gaps(const vector<float> &ranges) {
    vector<std::pair<int, int>> gaps;
    size_t i = 0;

    while (i < ranges.size()) {
        if (ranges[i] <= minimum_gap_threshold_) {
            ++i;
            continue;
        }

        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_) {
            ++i;
        }
        size_t gap_end = i;

        gaps.push_back({static_cast<int>(gap_start), static_cast<int>(gap_end)});
    }

    return gaps;
}

std::pair<int, int> FollowTheGapNode::pick_best_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const vector<float> &ranges, const std::vector<std::pair<int, int>> &gaps) {
    std::pair<int, int> best_gap = {-1, -1};
    float best_score = -1.0f;

    for (const auto &gap : gaps) {
        int gap_start = gap.first;
        int gap_end = gap.second;

        // Calculate average depth of the gap
        float sum = 0.0f;
        for (int j = gap_start; j < gap_end; ++j) {
            sum += ranges[j];
        }
        float avg_depth = sum / (gap_end - gap_start);

        // Ensure the gap is wide enough for the car
        size_t min_width = static_cast<size_t>((car_width_ / avg_depth) / scan_msg->angle_increment);
        if (static_cast<size_t>(gap_end - gap_start) < min_width) continue;

        // Score to find best gap by favoring gaps that are both wide and deep
        float score = 0.0f;
        for (int j = gap_start; j < gap_end; j++) {
          score += ranges[j] - minimum_gap_threshold_;
        }

        if (score > best_score) {
            best_score = score;
            best_gap = gap;
        }
    }

    return best_gap;
}

int FollowTheGapNode::pick_best_point(const std::pair<int, int> &best_gap, const vector<float> &ranges) {
    if (best_gap.first == -1 || best_gap.second == -1) {
        return -1; // Invalid gap
    }

    // Range-weighted average index, so the point leans toward the deepest part of the gap
    double weight_sum = 0.0;
    double index_sum = 0.0;
    for (int i = best_gap.first; i < best_gap.second; i++) {
        weight_sum += ranges[i];
        index_sum += ranges[i] * i;
    }

    return static_cast<int>(std::round(index_sum / weight_sum));
}

void FollowTheGapNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // Safety
    int safety_publisher_count = this->safety_brake_sub_->get_publisher_count();
    if (enable_safety_ && (safety_brake_engaged_ || safety_publisher_count == 0)) {
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = 0.0;
        drive_msg_publisher->publish(drive_msg);
        return;
    }

    auto ranges = preprocess_lidar(scan_msg);
    draw_safety_bubble(scan_msg, ranges);
    auto gaps = find_all_gaps(ranges);
    auto best_gap = pick_best_gap(scan_msg, ranges, gaps);
    int best_idx = pick_best_point(best_gap, ranges);

    if (best_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid gap found");
        return;
    }

    float current_angle = scan_msg->angle_min + scan_msg->angle_increment * best_idx;

    // Use the target range as the lookahead, clamped so distant targets don't make steering sluggish
    double target_range = ranges[best_idx];
    double lookahead = std::clamp(target_range, 0.5, lookahead_max_);

    // target angle arcing through target point
    float target_angle = static_cast<float>(std::atan2(2.0 * wheelbase_ * std::sin(current_angle), lookahead));

    //If the point we need to steer to is hard to the left or right, steer as hard to it
    if (abs(current_angle) > 50 * M_PI / 180.0) {
      if (current_angle >= 0) {
        target_angle = steering_angle_max_; // Steering max
      } else {
        target_angle = -1.0f * 0.41f;
      }
    }

    // Smooth with previous steering_angle
    float steering_angle = prev_angle_ * prev_angle_adjuster_ + target_angle * (1 - prev_angle_adjuster_);
    prev_angle_ = steering_angle;

    // Deadzone: zero out small steering angles to prevent wobble on straights
    if (std::abs(steering_angle) < steering_deadzone_) {
        steering_angle = 0.0f;
    }

    float abs_angle = std::abs(steering_angle);

    // Limit 1: slow down for sharp turns (v = sqrt(a * R), R = wheelbase / tan(steer))
    float abs_tan = std::tan(abs_angle);
    float v_turn = max_speed_;
    if (abs_tan > 1e-3f)
    {
        v_turn = std::sqrt(max_lateral_acc_ * wheelbase_ / abs_tan);
    }

    // Limit 2: slow down when little distance is left to turn
    float v_range = std::sqrt(max_lateral_acc_ * target_range);

    float speed = std::min({static_cast<float>(max_speed_), v_turn, v_range});
    speed = std::clamp(speed, static_cast<float>(min_speed_), static_cast<float>(max_speed_));

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = scan_msg->header.stamp;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    drive_msg_publisher->publish(drive_msg);
}

void FollowTheGapNode::safety_brake_callback(const std_msgs::msg::Bool::ConstSharedPtr safety_brake_msg) {
    safety_brake_engaged_ = safety_brake_msg->data;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}
