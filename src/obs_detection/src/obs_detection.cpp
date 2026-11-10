#include "../include/obs_detection.hpp"

ObsDetectionNode::ObsDetectionNode() : Node("obs_detection_node")
{
    RCLCPP_INFO(this->get_logger(), "Obs_detection node started");

    // declare node parameters (detection)
    this->declare_parameter("max_obstacle_distance", 10.0);
    this->declare_parameter("obstacle_edge_jump", 0.5);
    this->declare_parameter("car_width_extended", 0.55);
    this->declare_parameter("fov_half_angle_deg", 135.0);
    this->declare_parameter("obstacle_min_target_angle_deg", 0.0);
    this->declare_parameter("obstacle_max_width", 0.8);
    this->declare_parameter("obstacle_hold_frames", 8);

    // declare node parameters (follow-the-gap fallback)
    this->declare_parameter("max_speed", 4.5);
    this->declare_parameter("min_speed", 0.5);
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("ftg_fov_half_angle_deg", 70.0);
    this->declare_parameter("car_width", 0.4);
    this->declare_parameter("minimum_gap_threshold", 1.0);
    this->declare_parameter("safety_bubble_scale", 0.5);
    this->declare_parameter("prev_angle_adjuster", 0.6);
    this->declare_parameter("bubble_lookahead", 3.0);
    this->declare_parameter("bubble_half_width", 0.45);
    this->declare_parameter("wheelbase", 0.33);
    this->declare_parameter("lookahead_max", 1.8);
    this->declare_parameter("steering_deadzone", 0.7);

    max_obstacle_distance_ = this->get_parameter("max_obstacle_distance").as_double();
    obstacle_edge_jump_ = this->get_parameter("obstacle_edge_jump").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();
    obstacle_max_width_ = this->get_parameter("obstacle_max_width").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    obstacle_min_target_angle_ = this->get_parameter("obstacle_min_target_angle_deg").as_double() * M_PI / 180.0;
    obstacle_hold_frames_ = this->get_parameter("obstacle_hold_frames").as_int();

    max_speed_ = this->get_parameter("max_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    ftg_fov_half_angle_ = this->get_parameter("ftg_fov_half_angle_deg").as_double() * M_PI / 180.0;
    car_width_ = this->get_parameter("car_width").as_double();
    minimum_gap_threshold_ = this->get_parameter("minimum_gap_threshold").as_double();
    safety_bubble_scale_ = this->get_parameter("safety_bubble_scale").as_double();
    prev_angle_adjuster_ = this->get_parameter("prev_angle_adjuster").as_double();
    bubble_lookahead_ = this->get_parameter("bubble_lookahead").as_double();
    bubble_half_width_ = this->get_parameter("bubble_half_width").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    lookahead_max_ = this->get_parameter("lookahead_max").as_double();
    steering_deadzone_ = this->get_parameter("steering_deadzone").as_double() * M_PI / 180.0;

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ObsDetectionNode::lidar_callback, this, std::placeholders::_1));

    obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_ahead", 10);
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
}

float ObsDetectionNode::range_at(int i, const std::vector<float> &processed_ranges)
{
    const float r = processed_ranges[i];
    return (std::isfinite(r) && r > 0.05f) ? r : static_cast<float>(max_obstacle_distance_);
};

double ObsDetectionNode::angle_at(int i, ConstScanMessagePtr scan_msg)
{
    return scan_msg->angle_min + scan_msg->angle_increment * i;
};

std::vector<float> ObsDetectionNode::preprocess_lidar(ConstScanMessagePtr scan_msg)
{
    auto ranges = scan_msg->ranges;

    for (size_t i = 0; i < ranges.size(); i++)
    {
        float range = ranges[i];
        if (!std::isfinite(range))
        {
            range = (std::isnan(range) || range < 0) ? 0.0f : max_obstacle_distance_;
        }
        ranges[i] = std::clamp(range, 0.0f, static_cast<float>(max_obstacle_distance_));
    }
    return ranges;
}

void ObsDetectionNode::lidar_callback(ConstScanMessagePtr scan_msg)
{
    if (scan_msg->ranges.empty())
        return;

    std::vector<float> processed_ranges = preprocess_lidar(scan_msg);

    auto message = detectObstacleAhead(scan_msg, processed_ranges);

    obstacle_pub_->publish(message);

    // message.data is the hysteresis-held state, so the fallback stays active
    // for obstacle_hold_frames after the obstacle disappears from view.
    if (message.data)
    {
        drive_pub_->publish(ftg_compute_drive(scan_msg));
    }
    else
    {
        // Don't carry stale steering smoothing into the next obstacle event
        prev_angle_ = 0.0;
    }
}

std_msgs::msg::Bool ObsDetectionNode::detectObstacleAhead(ConstScanMessagePtr scan_msg,
                                                          const std::vector<float> &processed_ranges)
{
    std_msgs::msg::Bool msg;
    msg.data = false;

    // first find our candidate (chooses the laser with closest point in our lane currently)
    auto candidate = find_closest_candidate(scan_msg, processed_ranges);

    if (!candidate)
    {
        msg.data = update_obstacle_state(false, *candidate);
        return msg;
    }
    /*
        see if the candidate has left and right edges (the jumps "(a, b)"
        in distance for a reading such as say 8 7 (9 2) 2 (2 9) 9 8
        should indicate some form of shape which means higher likely hood of there being an obstacle)
    */

    auto edges = find_obstacle_edges(scan_msg, processed_ranges, *candidate);

    if (!edges)
    {
        msg.data = update_obstacle_state(false, *candidate);
        return msg;
    }

    if (!is_obstacle(scan_msg, processed_ranges, *edges))
    {
        msg.data = update_obstacle_state(false, *candidate);
        return msg;
    };

    update_obstacle_state(true, *candidate);

    msg.data = true;

    return msg;
}

std::optional<Candidate> ObsDetectionNode::find_closest_candidate(
    ConstScanMessagePtr scan_msg, const std::vector<float> &processed_ranges)
{
    const float scan_limit = std::min(fov_half_angle_, M_PI / 2.0);
    int closest = -1;
    float forward_min = std::numeric_limits<float>::max();

    auto within_lane = [&](int i, float forward_min) -> bool
    {
        const float r = range_at(i, processed_ranges);
        const float angle = angle_at(i, scan_msg);
        if (
            std::abs(angle) < scan_limit &&
            std::abs(r * std::sin(angle)) <= (car_width_extended_ / 2.0) && r < forward_min)
        {
            return true;
        }
        return false;
    };

    // candidate searching loop
    for (size_t i = 0; i < processed_ranges.size(); i++)
    {
        if (within_lane(i, forward_min))
        {
            forward_min = range_at(i, processed_ranges);
            closest = i;
        }
    }
    if (closest == -1 || forward_min == std::numeric_limits<float>::max())
    {
        return std::nullopt;
    }
    Candidate candidate{closest, range_at(closest, processed_ranges), angle_at(closest, scan_msg)};
    return candidate;
}

std::optional<Edges> ObsDetectionNode::find_obstacle_edges(ConstScanMessagePtr scan_msg, const std::vector<float> &processed_ranges, Candidate candidate)
{
    const int confirm_beams = 3;
    const int n = processed_ranges.size();
    const double scan_limit = std::min(fov_half_angle_, M_PI / 2.0);

    auto jump_is_persistent = [&](int i, int next, int dir) -> bool
    {
        for (int k = 0; k < confirm_beams; ++k)
        {
            if (next < 0 || next >= n || std::abs(angle_at(next, scan_msg)) >= scan_limit)
                return false;
            const int m = next + dir * k;
            if (m < 0 || m >= n || range_at(m, processed_ranges) - range_at(i, processed_ranges) <= obstacle_edge_jump_)
            {
                return false;
            }
        }
        return true;
    };

    auto in_scan_bounds = [&](int index) -> bool
    {
        if (index < 0 || index >= static_cast<int>(processed_ranges.size()))
        {
            return false;
        }
        return true;
    };

    auto find_edge_to_the = [&](std::string direction) -> int
    {
        int dir = (direction == "right" ? -1 : 1);

        for (int i = candidate.index; in_scan_bounds(i + dir); i += dir)
        {
            const int next = i + dir;
            if (range_at(next, processed_ranges) - range_at(i, processed_ranges) > obstacle_edge_jump_)
            {
                if (jump_is_persistent(i, next, dir))
                    return i;
            }
        }
        return -1;
    };

    int left = find_edge_to_the("left");
    int right = find_edge_to_the("right");
    if (left < 0 || right < 0)
        return std::nullopt;

    return Edges{left, right};
}

bool ObsDetectionNode::is_obstacle(ConstScanMessagePtr scan_msg, const std::vector<float> &processed_ranges, Edges edges)
{
    const double y_left = range_at(edges.left, processed_ranges) * std::sin(angle_at(edges.left, scan_msg));
    const double y_right = range_at(edges.right, processed_ranges) * std::sin(angle_at(edges.right, scan_msg));
    if (y_left - y_right > obstacle_max_width_)
        return false;
    return true;
}

bool ObsDetectionNode::update_obstacle_state(bool detected, const Candidate &candidate)
{
    bool prev_state = (obstacle_hold_counter_ > 0);

    if (detected)
    {

        Vector3 current_obstacle_position = Vector3{candidate.range * std::cos(candidate.angle), candidate.range * std::sin(candidate.angle), 0.0};
        if (last_obstacle_position_.has_value())
        {
            Vector3 delta = last_obstacle_position_.value() - current_obstacle_position;
            if (std::abs(delta.x) < vector_difference_threshold && std::abs(delta.y) < vector_difference_threshold)
            {
                last_obstacle_position_ = current_obstacle_position;
                obstacle_hold_counter_ = obstacle_hold_frames_;
            }
            else
            {
                last_obstacle_position_ = std::nullopt;
            }
        }
        else
        {
            obstacle_hold_counter_ = obstacle_hold_frames_;
            last_obstacle_position_ = current_obstacle_position;
        }
    }
    else if (obstacle_hold_counter_ > 0)
    {
        --obstacle_hold_counter_;
    }
    else
    {
        last_obstacle_position_ = std::nullopt;
    }

    bool current_state = (obstacle_hold_counter_ > 0);

    // Log only on state transitions to avoid spamming the console
    if (!prev_state && current_state)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected ahead! Follow-the-gap fallback engaged.");
    }
    else if (prev_state && !current_state)
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle cleared. Follow-the-gap fallback released.");
    }

    return current_state;
}

// Follow-the-gap fallback

std::vector<float> ObsDetectionNode::ftg_preprocess(ConstScanMessagePtr scan_msg)
{
    const auto &ranges = scan_msg->ranges;
    const int n = ranges.size();

    // invalid or too-close readings become 0, others capped at max_lidar_range_
    auto clean = [&](int i)
    {
        float r = ranges[i];
        if (!std::isfinite(r) || r < scan_msg->range_min)
        {
            return 0.0f;
        }
        return std::min(r, static_cast<float>(max_lidar_range_));
    };

    // first/last beams inside the FTG field of view
    const int first = std::max<int>(0,
                                    std::ceil((-ftg_fov_half_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));
    const int last = std::min<int>(n - 1,
                                   std::floor((ftg_fov_half_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));

    std::vector<float> processed(n, 0.0f);

    for (int i = first; i <= last; i++)
    {
        if (i == 0 || i == n - 1)
        {
            processed[i] = clean(i);
            continue;
        }
        processed[i] = (clean(i - 1) + clean(i) + clean(i + 1)) / 3.0f;
    }

    return processed;
}

void ObsDetectionNode::ftg_draw_safety_bubble(ConstScanMessagePtr scan_msg, std::vector<float> &ranges)
{
    bool found_closest = false;
    size_t closest_index = 0;
    for (size_t i = 1; i < ranges.size(); ++i)
    {
        if (ranges[i] == 0)
        {
            continue;
        }

        // Only consider points inside a box ahead of the car (bubble_lookahead_ x bubble_half_width_)
        float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        float ahead = ranges[i] * std::cos(angle);
        float beside = ranges[i] * std::sin(angle);
        if (ahead <= 0.0f || ahead > bubble_lookahead_ ||
            std::abs(beside) > bubble_half_width_)
        {
            continue;
        }

        if (!found_closest || ranges[i] < ranges[closest_index])
        {
            found_closest = true;
            closest_index = i;
        }
    }

    if (!found_closest)
        return;

    const double n = static_cast<double>(ranges.size());
    const double offset = std::abs(static_cast<double>(closest_index) - n / 2.0);

    // Adjust bubble size by the angle between the closest obstacle and the car's heading
    double direction_adjustment = std::max(1.0 - 2.0 * offset / n, 0.0);

    // Adjust based on distance
    double distance_adjustment = std::min(0.1 / ranges[closest_index], 1.0);

    double theta = 2.0 * std::atan2(car_width_ * safety_bubble_scale_ *
                                        std::max(direction_adjustment, distance_adjustment),
                                    ranges[closest_index]);

    size_t index_increment = (theta / scan_msg->angle_increment) / 2; // per side

    size_t start = (closest_index > index_increment) ? closest_index - index_increment : 0;
    size_t end = std::min(closest_index + index_increment, ranges.size());

    for (size_t i = start; i < end; ++i)
    {
        ranges[i] = 0;
    }
}

std::vector<std::pair<int, int>> ObsDetectionNode::ftg_find_all_gaps(const std::vector<float> &ranges)
{
    std::vector<std::pair<int, int>> gaps;
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

        gaps.push_back({static_cast<int>(gap_start), static_cast<int>(gap_end)});
    }

    return gaps;
}

std::pair<int, int> ObsDetectionNode::ftg_pick_best_gap(ConstScanMessagePtr scan_msg,
                                                        const std::vector<float> &ranges, const std::vector<std::pair<int, int>> &gaps)
{
    std::pair<int, int> best_gap = {-1, -1};
    float best_score = -1.0f;

    for (const auto &gap : gaps)
    {
        int gap_start = gap.first;
        int gap_end = gap.second;

        float sum = 0.0f;
        for (int j = gap_start; j < gap_end; ++j)
        {
            sum += ranges[j];
        }
        float avg_depth = sum / (gap_end - gap_start);

        // Ensure the gap is wide enough for the car
        size_t min_width = static_cast<size_t>((car_width_ / avg_depth) / scan_msg->angle_increment);
        if (static_cast<size_t>(gap_end - gap_start) < min_width)
            continue;

        // Score favors gaps that are both wide and deep
        float score = (gap_end - gap_start) * avg_depth;

        if (score > best_score)
        {
            best_score = score;
            best_gap = gap;
        }
    }

    return best_gap;
}

int ObsDetectionNode::ftg_pick_best_point(const std::pair<int, int> &best_gap, const std::vector<float> &ranges)
{
    if (best_gap.first == -1 || best_gap.second == -1)
    {
        return -1;
    }

    // Range-weighted average index, so the point leans toward the deepest part of the gap
    double weight_sum = 0.0;
    double index_sum = 0.0;
    for (int i = best_gap.first; i < best_gap.second; i++)
    {
        weight_sum += ranges[i];
        index_sum += ranges[i] * i;
    }

    return static_cast<int>(std::round(index_sum / weight_sum));
}

ackermann_msgs::msg::AckermannDriveStamped ObsDetectionNode::ftg_compute_drive(ConstScanMessagePtr scan_msg)
{
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = scan_msg->header.stamp;

    auto ranges = ftg_preprocess(scan_msg);
    ftg_draw_safety_bubble(scan_msg, ranges);
    auto gaps = ftg_find_all_gaps(ranges);
    auto best_gap = ftg_pick_best_gap(scan_msg, ranges, gaps);
    int best_idx = ftg_pick_best_point(best_gap, ranges);

    // Obstacle ahead and nowhere to go: stop rather than let the normal driver run into it
    if (best_idx == -1)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Obstacle fallback: no valid gap found, stopping");
        drive_msg.drive.speed = 0.0;
        return drive_msg;
    }

    float current_angle = scan_msg->angle_min + scan_msg->angle_increment * best_idx;

    // Target range as lookahead, clamped so distant targets don't make steering sluggish
    double target_range = ranges[best_idx];
    double lookahead = std::clamp(target_range, 0.5, lookahead_max_);

    // pure-pursuit style angle arcing through the target point
    float target_angle = static_cast<float>(std::atan2(2.0 * wheelbase_ * std::sin(current_angle), lookahead));

    // smooth with previous
    float steering_angle = prev_angle_ * prev_angle_adjuster_ + target_angle * (1 - prev_angle_adjuster_);
    prev_angle_ = steering_angle;

    // Deadzone: zero out small steering angles to prevent wobble on straights
    if (std::abs(steering_angle) < steering_deadzone_)
    {
        steering_angle = 0.0f;
    }

    float abs_angle = std::abs(steering_angle);

    // Angle-based speed factor
    double max_angle = 14.0 * M_PI / 180.0;
    double t_angle = std::clamp(abs_angle / max_angle, 0.0, 1.0);
    double angle_speed = max_speed_ - t_angle * (max_speed_ - min_speed_);

    // Range-based speed factor
    double min_range = 0.5;
    double max_range = 5.0;
    double t_range = std::clamp((target_range - min_range) / (max_range - min_range), 0.0, 1.0);
    double range_speed = min_speed_ + t_range * (max_speed_ - min_speed_);

    // Take the more conservative (lower) of the two
    float speed = static_cast<float>(std::min(angle_speed, range_speed) * 1.1);

    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    return drive_msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObsDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
