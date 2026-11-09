#include "../include/obs_detection.hpp"

ObsDetectionNode::ObsDetectionNode() : Node("obs_detection_node")
{
    RCLCPP_INFO(this->get_logger(), "Obs_detection node started");

    // declare node parameters 
    this->declare_parameter("max_obstacle_distance", 2.0);
    this->declare_parameter("obstacle_edge_jump", 0.5);
    this->declare_parameter("car_width_extended", 0.55);
    this->declare_parameter("fov_half_angle_deg", 135.0);
    this->declare_parameter("obstacle_min_target_angle_deg", 0.0);
    this->declare_parameter("obstacle_max_width", 0.8);
    this->declare_parameter("obstacle_hold_frames", 8);

    max_obstacle_distance_ = this->get_parameter("max_obstacle_distance").as_double();
    obstacle_edge_jump_ = this->get_parameter("obstacle_edge_jump").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();
    obstacle_max_width_ = this->get_parameter("obstacle_max_width").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle_deg").as_double() * M_PI / 180.0;
    obstacle_min_target_angle_ = this->get_parameter("obstacle_min_target_angle_deg").as_double() * M_PI / 180.0;
    obstacle_hold_frames_ = this->get_parameter("obstacle_hold_frames").as_int();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ObsDetectionNode::lidar_callback, this, std::placeholders::_1));

    obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_ahead", 10);
}


float ObsDetectionNode::range_at(int i, const std::vector<float>& processed_ranges){
    const float r = processed_ranges[i];
    return (std::isfinite(r) && r > 0.05f) ? r : static_cast<float>(max_obstacle_distance_);
};

double ObsDetectionNode::angle_at(int i, ConstScanMessagePtr scan_msg){
    return scan_msg->angle_min + scan_msg->angle_increment * i;
};



std::vector<float> ObsDetectionNode::preprocess_lidar(ConstScanMessagePtr scan_msg)
{
    auto ranges = scan_msg->ranges;

    for(size_t i = 0; i < ranges.size(); i++){
        float range = ranges[i];
        if(!std::isfinite(range)){
            range = (std::isnan(range) || range < 0) ? 0.0f : max_obstacle_distance_;
        }
        ranges[i] = std::clamp(range, 0.0f, static_cast<float>(max_obstacle_distance_));
    }
    return ranges;
}

void ObsDetectionNode::lidar_callback(ConstScanMessagePtr scan_msg)
{
    if(scan_msg->ranges.empty())
        return;

    std::vector<float> processed_ranges = preprocess_lidar(scan_msg);

    auto message = detectObstacleAhead(scan_msg, processed_ranges);
    
    obstacle_pub_->publish(message);

}

std_msgs::msg::Bool ObsDetectionNode::detectObstacleAhead(ConstScanMessagePtr scan_msg, 
const std::vector<float>& processed_ranges){
    std_msgs::msg::Bool msg;
    msg.data = false;

    // first find our candidate (chooses the laser with closest point in our lane currently)
    auto candidate = find_closest_candidate(scan_msg, processed_ranges);

    if (!candidate){
        msg.data = update_obstacle_state(false);
        return msg;
    }
    /* 
        see if the candidate has left and right edges (the jumps "(a, b)" 
        in distance for a reading such as say 8 7 (9 2) 2 (2 9) 9 8 
        should indicate some form of shape which means higher likely hood of there being an obstacle)
    */

    auto edges = find_obstacle_edges(scan_msg, processed_ranges, *candidate);

    if (!edges){
        msg.data = update_obstacle_state(false);
        return msg;
    }

    
    if(!is_obstacle(scan_msg, processed_ranges, *edges)){
        msg.data = update_obstacle_state(false);
        return msg;
    };

    
    update_obstacle_state(true);
   
    msg.data = true;

    return msg;
}


 std::optional<Candidate> ObsDetectionNode::find_closest_candidate(
    ConstScanMessagePtr scan_msg, const std::vector<float>& processed_ranges){
        const float scan_limit = std::min(fov_half_angle_, M_PI / 2.0);
        int closest = -1;
        float forward_min = std::numeric_limits<float>::max();
        
        auto within_lane = [&](int i, float forward_min) -> bool
        {
            const float r = range_at(i, processed_ranges);
            const float angle = angle_at(i, scan_msg);
            if (
                std::abs(angle) < scan_limit &&
                std::abs(r * std::sin(angle)) <= (car_width_extended_ / 2.0)
                && r < forward_min
            ){
                return true;
            }
            return false;
        };

        
        // candidate searching loop
        for(size_t i = 0; i < processed_ranges.size(); i++){
            if(within_lane(i, forward_min)){
                forward_min = range_at(i, processed_ranges);
                closest = i;
            }
        }
        if(closest == -1 || forward_min == std::numeric_limits<float>::max()){
            return std::nullopt;
        }
        Candidate candidate{closest, range_at(closest, processed_ranges), angle_at(closest, scan_msg)};
        return candidate;
 }


std::optional<Edges>  ObsDetectionNode::find_obstacle_edges(ConstScanMessagePtr scan_msg, const std::vector<float>& processed_ranges, Candidate candidate){
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
        if(index < 0 || index >= static_cast<int>(processed_ranges.size())){
            return false;
        }
        return true;
    };

    auto find_edge_to_the = [&](std::string direction) -> int
    {
        int dir = (direction == "right" ? -1 : 1);

        for(int i = candidate.index; in_scan_bounds(i + dir); i+=dir){
            const int next = i + dir;
            if (range_at(next, processed_ranges) - range_at(i, processed_ranges) > obstacle_edge_jump_){
                if(jump_is_persistent(i, next, dir))
                    return i;
            }
        }
        return -1;
    };

    int left = find_edge_to_the("left");
    int right = find_edge_to_the("right");
    if(left < 0 || right < 0)
        return std::nullopt;

    return Edges{left, right};

}

bool 
ObsDetectionNode::is_obstacle(ConstScanMessagePtr scan_msg, const std::vector<float>& processed_ranges, Edges edges){
    const double y_left = range_at(edges.left, processed_ranges) * std::sin(angle_at(edges.left, scan_msg));
    const double y_right = range_at(edges.right, processed_ranges) * std::sin(angle_at(edges.right, scan_msg));
    if (y_left - y_right > obstacle_max_width_)
        return false;
    return true;
}

bool ObsDetectionNode::update_obstacle_state(bool detected)
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
    rclcpp::spin(std::make_shared<ObsDetectionNode>());
    rclcpp::shutdown();
    return 0;
}