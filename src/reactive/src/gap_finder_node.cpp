#include "gap_finder_node.hpp"

using std::vector;

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");

    // ROS2 parameters
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("fov_half_angle_deg", 100.0);
    this->declare_parameter("car_width", 0.4);
    this->declare_parameter("minimum_gap_threshold", 0.1);
    this->declare_parameter("safety_bubble_scale", 0.5);
    this->declare_parameter("bubble_lookahead", 3.0);
    this->declare_parameter("bubble_half_width", 0.45);
    this->declare_parameter("full_bubble_threshold", 0.25);
    this->declare_parameter("disparity_threshold", 1.5);
    this->declare_parameter("obstacle_distance", 2.0);
    this->declare_parameter("obstacle_edge_jump", 0.5);
    this->declare_parameter("obstacle_max_width", 0.8);
    this->declare_parameter("obstacle_min_target_angle_deg", 0.0);
    this->declare_parameter("obstacle_hold_frames", 8);
    this->declare_parameter("obstacle_detection", true);

    // Read into member variables
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    car_width_ = this->get_parameter("car_width").as_double();
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();
    safety_bubble_scale_ = this->get_parameter("safety_bubble_scale").as_double();
    bubble_lookahead_ = this->get_parameter("bubble_lookahead").as_double();
    bubble_half_width_ = this->get_parameter("bubble_half_width").as_double();
    full_bubble_threshold_ = this->get_parameter("full_bubble_threshold").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
    obstacle_edge_jump_ = this->get_parameter("obstacle_edge_jump").as_double();
    obstacle_max_width_ = this->get_parameter("obstacle_max_width").as_double();
    obstacle_min_target_angle_ = this->get_parameter("obstacle_min_target_angle_deg").as_double() * M_PI / 180.0;
    obstacle_hold_frames_ = this->get_parameter("obstacle_hold_frames").as_int();
    obstacle_detection_ = this->get_parameter("obstacle_detection").as_bool();

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&GapFinderNode::lidar_callback, this, std::placeholders::_1));

    gap_pub_ = this->create_publisher<reactive::msg::Gap>("gap", 10);
}

void GapFinderNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    draw_safety_bubble(scan_msg, ranges);

    const vector<float> raw_ranges = ranges; // before obstacle extension

    if (obstacle_detection_)
        extend_obstacles(scan_msg, ranges);

    auto gaps = find_all_gaps(ranges);
    auto gap = pick_best_gap(scan_msg, ranges, gaps);
    int best_point = pick_best_point(gap, ranges);

    if (best_point == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid gaps found");

        return;
    }

    reactive::msg::Gap gap_msg;
    gap_msg.angles.reserve(gap.second - gap.first);
    gap_msg.ranges.reserve(gap.second - gap.first);

    for (size_t i = gap.first; i < static_cast<size_t>(gap.second); ++i)
    {
        gap_msg.angles.push_back(scan_msg->angle_min + scan_msg->angle_increment * i);
        gap_msg.ranges.push_back(ranges[i]);
    }

    gap_msg.target_angle = scan_msg->angle_min + scan_msg->angle_increment * best_point;
    gap_msg.target_range = ranges[best_point];

    gap_msg.obstacle_ahead = obstacle_detection_ ? update_obstacle_state(
                                                       detect_obstacle(scan_msg, raw_ranges, gap_msg.target_angle))
                                                 : false;

    gap_pub_->publish(gap_msg);
}

vector<float> GapFinderNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    const auto& ranges = scan_msg->ranges;
    vector<float> processed(ranges.size(), 0.0f);

    auto clean = [&](int i) {
        const float r = ranges[i];

        // NAN values are set to 0, anything below scan_msg->range min to 0,
        // and everything above max_lidar_range_ is clamped

        if (!std::isfinite(r) || r < scan_msg->range_min) {
            return 0.0f;
        }

        // clamp values past max_lidar_range
        return std::min(r, static_cast<float>(max_lidar_range_));
    };

    // Find the first and last ranges in the FOV using fov_half_angle_
    const int first = std::max(0.0, std::ceil((-fov_half_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));
    const int last = std::max(0.0, std::ceil((fov_half_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));

    for (int i = first; i <= last; i++) {
        // out of bounds for averaging, clean and continue
        if (i == 0 || i == static_cast<int>(processed.size()) - 1) {
            processed[i] = clean(i);

            continue;
        }

        // average scans, may not be better
        processed[i] = (clean(i - 1) + clean(i) + clean(i + 1)) / 3.0f;
    }

    return ranges;
}

void GapFinderNode::draw_safety_bubble(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float> &ranges
) {
    bool found_closest = false;
    size_t closest_index = 0;

    for (size_t i = 1; i < ranges.size(); ++i) {
        if (ranges[i] == 0) {
            continue; //ignore 0s
        }

        // only consider points inside a box ahead of the car (bubble_lookahead_ x bubble_half_width_)
        float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        float ahead = ranges[i] * std::cos(angle);
        float beside = ranges[i] * std::sin(angle);

        if (ahead <= 0.0f || ahead > bubble_lookahead_ || std::abs(beside) > bubble_half_width_) {
            continue;
        }

        if (!found_closest || ranges[i] < ranges[closest_index]) {
            found_closest = true;
            closest_index = i;
        }
    }

    if (!found_closest)
        return;

    //Figure out how deep into the bubble box the closest point is 
    float d_angle = scan_msg->angle_min + scan_msg->angle_increment * closest_index;
    double d_beside = ranges[closest_index] * std::sin(d_angle);
    double depth = std::clamp((bubble_half_width_ - std::abs(d_beside)) / full_bubble_threshold_, 0.0, 1.0);

    // Draw safety bubble by calculating theta for the arc made by r = closest
    // point with s = car width

    const double n = static_cast<double>(ranges.size());
    const double offset = std::abs(static_cast<double>(closest_index) - n / 2.0);

    // Adjust size of bubble based on the angle between the closest obstacle and
    // the car's facing direction
    double direction_adjustment = std::max(1.0 - 2.0 * offset / n, 0.0);

    // Adjust based on distance
    double distance_adjustment = std::min(0.1 / ranges[closest_index], 1.0);

    double theta = 2.0 * std::atan2(
        car_width_ * safety_bubble_scale_ * std::max(direction_adjustment, distance_adjustment),
        ranges[closest_index]
    );

    size_t index_increment = (theta / scan_msg->angle_increment) / 2; // Used on each side so divide by 2

    size_t start = (closest_index > index_increment) ? closest_index - index_increment : 0;
    size_t end = std::min(closest_index + index_increment, static_cast<size_t>(n));
    
    double depth_smoothed = std::clamp((depth - 0.2) / 0.8, 0.0, 1.0);

    for (size_t i = start; i < end; i++) {
      ranges[i] *= (1 - depth_smoothed);
    }
}

vector<std::pair<int, int>> GapFinderNode::find_all_gaps(const vector<float>& ranges) {
    vector<std::pair<int, int>> gaps;

    size_t i = 0;

    while (i < ranges.size()) {
        // skip points that aren't part of a "free" gap
        if (ranges[i] <= minimum_gap_threshold_) {
            i++;
            continue;
        }

        // Found the start of a gap, walk forward to its end
        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_) {
            i++;
        }

        size_t gap_end = i; // exclusive

        gaps.push_back({static_cast<int>(gap_start), static_cast<int>(gap_end)});
    }

    return gaps;
}

std::pair<int, int> GapFinderNode::pick_best_gap(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const vector<float>& ranges,
    const vector<std::pair<int, int>>& gaps
) {
    std::pair<int, int> best_gap = {-1, -1};
    float best_score = -1.0f;

    for (const auto &gap : gaps) {
        int gap_start = gap.first;
        int gap_end = gap.second;

        // Calculate average depth of the gap
        float sum = 0.0f;

        for (int i = gap_start; i < gap_end; ++i) {
            sum += ranges[i];
        }
        float avg_depth = sum / (gap_end - gap_start);

        // Ensure the gap is wide enough for the car
        size_t min_width = static_cast<size_t>((car_width_ / avg_depth) / scan_msg->angle_increment);
        if (static_cast<size_t>(gap_end - gap_start) < min_width)
            continue;

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

int GapFinderNode::pick_best_point(const std::pair<int, int>& gap, const vector<float>& ranges)
{
    if (gap.first == -1 || gap.second == -1) {
        return -1; // invalid gap
    }

    // Range-weighted average index, so the point leans toward the deepest part of the gap
    double weight_sum = 0.0;
    double index_sum = 0.0;

    for (int i = gap.first; i < gap.second; i++) {
        weight_sum += ranges[i];
        index_sum += ranges[i] * i;
    }

    return static_cast<int>(std::round(index_sum / weight_sum));
}

void GapFinderNode::extend_obstacles(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float> &ranges
) {
    const vector<float> original = ranges; // raw values for disparity comparisons

    float prev_reading = original[0];

    for (size_t i = 0; i < original.size(); ++i)
    {
        float current_reading = original[i];

        if (std::abs(prev_reading - current_reading) <= disparity_threshold_)
            continue;
        
        float closer_range = std::min(prev_reading, current_reading);

        if (closer_range > 1e-3f) // avoid divide-by-zero / near-zero blowup
        {
            double dynamic_width = car_width_;
            if (closer_range < 1.0f)
            {
                dynamic_width = std::max(car_width_* closer_range, 0.2);
            }

            double theta = 2.0 * std::atan2(dynamic_width / 2.0, closer_range);
            theta = std::min(theta, M_PI / 2.0);

            size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);
            size_t start = (i > index_increment) ? i - index_increment : 0;
            size_t end = std::min(i + index_increment + 1, ranges.size());

            for (size_t j = start; j < end; ++j)
            {
                ranges[j] = std::min(ranges[j], closer_range);
            }
        }
        

        prev_reading = current_reading; // compare against raw values, not mutated output
    }
}

bool GapFinderNode::detect_obstacle(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                                    const vector<float> &raw, double target_angle)
{
    const int n = static_cast<int>(raw.size());
    if (n == 0)
        return false;

    auto range_at = [&](int i) -> float
    {
        const float r = raw[i];
        return (std::isfinite(r) && r > 0.05f) ? r : static_cast<float>(max_lidar_range_);
    };
    auto angle_at = [&](int i) -> double
    {
        return scan_msg->angle_min + scan_msg->angle_increment * i;
    };
    const double scan_limit = std::min(fov_half_angle_, M_PI / 2.0);

    // Closest reading inside the car's lane
    const double half_width = car_width_ / 2.0;
    int closest = -1;
    float forward_min = std::numeric_limits<float>::max();
    for (int i = 0; i < n; ++i)
    {
        const double a = angle_at(i);
        if (std::abs(a) >= scan_limit)
            continue;
        const float r = range_at(i);
        if (std::abs(r * std::sin(a)) > half_width)
            continue;
        if (r < forward_min)
        {
            forward_min = r;
            closest = i;
        }
    }
    if (closest < 0 || forward_min > obstacle_distance_)
        return false;

    // Walk outward until the range jumps away; no jump before the scan ends means a wall
    const int confirm_beams = 3;
    auto find_edge = [&](int dir) -> int
    {
        int j = closest;
        while (true)
        {
            const int next = j + dir;
            if (next < 0 || next >= n || std::abs(angle_at(next)) >= scan_limit)
                return -1;
            if (range_at(next) - range_at(j) > obstacle_edge_jump_)
            {
                bool persists = true;
                for (int k = 0; k < confirm_beams; ++k)
                {
                    const int m = next + dir * k;
                    if (m < 0 || m >= n || range_at(m) - range_at(j) <= obstacle_edge_jump_)
                    {
                        persists = false;
                        break;
                    }
                }
                if (persists)
                    return j;
            }
            j = next;
        }
    };

    const int left_edge = find_edge(+1);
    const int right_edge = find_edge(-1);
    if (left_edge < 0 || right_edge < 0)
        return false;

    const double y_left = range_at(left_edge) * std::sin(angle_at(left_edge));
    const double y_right = range_at(right_edge) * std::sin(angle_at(right_edge));
    if (y_left - y_right > obstacle_max_width_)
        return false;

    return std::abs(target_angle) >= obstacle_min_target_angle_;
}

bool GapFinderNode::update_obstacle_state(bool detected)
{
    bool prev_state = (obstacle_hold_counter_ > 0);

    if (detected)
    {
        obstacle_hold_counter_ = obstacle_hold_frames_;
    }
    else if (obstacle_hold_counter_ > 0)
    {
        --obstacle_hold_counter_;
    }

    bool current_state = (obstacle_hold_counter_ > 0);

    // Log only on state transitions to avoid spamming the console
    if (!prev_state && current_state)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected ahead! Engaging braking override.");
    }
    else if (prev_state && !current_state)
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle cleared. Resuming normal gap following speed.");
    }

    return current_state;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFinderNode>());
    rclcpp::shutdown();
    return 0;
}
