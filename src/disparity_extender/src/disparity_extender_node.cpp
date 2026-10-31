#include "disparity_extender_node.hpp"
using std::vector;

DisparityExtenderNode::DisparityExtenderNode() : Node("disparity_extender_node")
{
    RCLCPP_INFO(this->get_logger(), "Disparity Extender node started");

    // Declare with a default value
    this->declare_parameter("enable_stop", true);
    this->declare_parameter("max_steering_angle", 0.349);
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("car_width", 0.4);
    this->declare_parameter("car_width_extended", 0.85);
    this->declare_parameter("disparity_threshold", 1.0);
    this->declare_parameter("fov_half_angle_deg", 90.0);
    this->declare_parameter("minimum_gap_threshold", 0.1);
    this->declare_parameter("max_speed", 5.5);
    this->declare_parameter("min_speed", 2.0);
    this->declare_parameter("alpha", 0.8);
    this->declare_parameter("wheelbase", 0.33);
    this->declare_parameter("max_lateral_acc", 3.5);
    this->declare_parameter("enable_safety", true);
    // Add enable_stop param that stops the car if no valid gap found

    // Read into member variables
    enable_stop_ = this->get_parameter("enable_stop").as_bool();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    car_width_ = this->get_parameter("car_width").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    alpha_ = this->get_parameter("alpha").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    max_lateral_acc_ = this->get_parameter("max_lateral_acc").as_double();
    enable_safety_ = this->get_parameter("enable_safety").as_bool();

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&DisparityExtenderNode::lidar_callback, this, std::placeholders::_1));
    safety_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/safety/brake", 10,
        std::bind(&DisparityExtenderNode::safety_brake_callback, this, std::placeholders::_1));
}

void DisparityExtenderNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    int safety_publisher_count = this->safety_brake_sub_->get_publisher_count();
    if (enable_safety_ && (safety_brake_engaged_ || safety_publisher_count == 0)) {
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = 0.0;
        drive_pub_->publish(drive_msg);
        return;
    }

    auto ranges = preprocess_lidar(scan_msg);
    extend_obstacles(scan_msg, ranges);
    draw_safety_bubble(scan_msg, ranges);

    auto gap = find_furthest_gap(scan_msg, ranges);

    if (gap.first == -1)
    {
        if (enable_stop_)
        {
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = this->now();
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed = 0.0;
            drive_pub_->publish(drive_msg);
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No valid gap found");
        return;
    }

    int furthest_point = find_furthest_point(ranges, gap);

    float target_angle = scan_msg->angle_min + scan_msg->angle_increment * furthest_point;
    float target_range = ranges[furthest_point];

    drive_best_point(target_range, target_angle);
}

vector<float> DisparityExtenderNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    vector<float> ranges = scan_msg->ranges;

    // Capping values past max_lidar_range, and rejecting values past fov angle range
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        if (std::abs(scan_msg->angle_min + scan_msg->angle_increment * i) > fov_half_angle_)
        {
            ranges[i] = 0;
        }
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }

    return ranges;
}

void DisparityExtenderNode::extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    if (ranges.empty()) return;

    const vector<float> original = ranges; // raw values for disparity comparisons

    float prev_reading = original[0];
    for (size_t i = 0; i < original.size(); ++i)
    {
        float current_reading = original[i];

        if (std::abs(prev_reading - current_reading) > disparity_threshold_)
        {
            float closer_range = std::min(prev_reading, current_reading);

            if (closer_range > 1e-3f) // avoid divide-by-zero / near-zero blowup
            {
                double theta = 2.0 * std::atan2(car_width_extended_ / 2.0, closer_range);
                theta = std::min(theta, M_PI); // also cap max bubble angle as a safety net

                size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);

                size_t start = (i > index_increment) ? i - index_increment : 0;
                size_t end = std::min(i + index_increment + 1, ranges.size());

                for (size_t j = start; j < end; ++j) {
                    ranges[j] = std::min(ranges[j], closer_range);
                }
            }
        }

        prev_reading = current_reading; // compare against raw values, not mutated output
    }
}

void DisparityExtenderNode::draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
     // Find closest obstacle (ignoring 0 ranges since 0 will already be avoided)
    bool found_closest = false;
    size_t closest_index;
    for (size_t i = 1; i < ranges.size(); ++i) {
        if (ranges[i] != 0 && !found_closest) {
            found_closest = true;
            closest_index = i;
        }

        if (found_closest && ranges[i] != 0 && ranges[i] < ranges[closest_index]) {
            closest_index = i;
        }
    }

    if (!found_closest) return; // handle case where all ranges are 0

    // Draw safety buble by calculating theta for the arc made by r = closest point with s = car width

    double theta = 2.0 * std::atan2(car_width_ / 2.0, ranges[closest_index]);

    size_t index_increment = (theta / scan_msg->angle_increment) / 2; // Used on each side so divide by 2

    size_t start = (closest_index > index_increment) ? closest_index - index_increment : 0;

    size_t end = std::min(closest_index + index_increment, ranges.size());

    for (size_t i = start; i < end; ++i) {
        ranges[i] = 0;
    }
}

std::pair<int, int> DisparityExtenderNode::find_furthest_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                                                     const vector<float> &ranges)
{
    int best_start = -1;
    int best_end = -1;
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
        double theta = 2.0 * std::atan2(car_width_extended_ / 2.0, min_range_in_gap);
        size_t min_index_width = static_cast<size_t>(theta / scan_msg->angle_increment);

        if ((gap_end - gap_start) < min_index_width)
        {
            continue; // gap too narrow for the car, skip it
        }

        // Gap is valid - check if it contains a further point than the best gap found so far
        for (size_t j = gap_start; j < gap_end; ++j)
        {
            if (ranges[j] > furthest_range)
            {
                furthest_range = ranges[j];
                best_start = static_cast<int>(gap_start);
                best_end = static_cast<int>(gap_end);
            }
        }
    }

    return {best_start, best_end};
}

int DisparityExtenderNode::find_furthest_point(vector<float> &ranges, const std::pair<int, int> &gap)
{
    if (gap.first == -1) return -1;

    int furthest = gap.first;
    for (int i = gap.first; i < gap.second; ++i) {
        if (ranges[i] > ranges[furthest])
        {
            furthest = i;
        }
    }

    return furthest;
}

void DisparityExtenderNode::drive_best_point(const float range, const float steering_angle)
{
    float steer = static_cast<float>(alpha_) * steering_angle + (1.0f - alpha_) * prev_steer_;
    prev_steer_ = steer;
    steer = std::clamp(steer, static_cast<float>(-max_steering_angle_), static_cast<float>(max_steering_angle_));
    // Limit 1: slow down for sharp turns (v = sqrt(a * R), R = wheelbase / tan(steer))
    float abs_tan = std::tan(std::abs(steer));
    float v_turn = max_speed_;
    if (abs_tan > 1e-3f)
    {
        v_turn = std::sqrt(max_lateral_acc_ * wheelbase_ / abs_tan);
    }

    // Limit 2: slow down when little distance is left to turn
    float v_range = std::sqrt(max_lateral_acc_ * range);

    float velocity = std::min({static_cast<float>(max_speed_), v_turn, v_range});
    velocity = std::clamp(velocity, static_cast<float>(min_speed_), static_cast<float>(max_speed_));

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = steer;
    drive_msg.drive.speed = velocity;
    drive_pub_->publish(drive_msg);
}

void DisparityExtenderNode::safety_brake_callback(const std_msgs::msg::Bool::ConstSharedPtr safety_brake_msg) {
    safety_brake_engaged_ = safety_brake_msg->data;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisparityExtenderNode>());
    rclcpp::shutdown();
    return 0;
}
