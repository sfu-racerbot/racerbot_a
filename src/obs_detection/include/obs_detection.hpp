#ifndef OBS_DETECT_NODE_HPP_
#define OBS_DETECT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <string>

struct Candidate{
    int index;
    float range;
    double angle;
};

struct Edges{
    int left;
    int right;
};

using ConstScanMessagePtr = sensor_msgs::msg::LaserScan::ConstSharedPtr;

class ObsDetectionNode : public rclcpp::Node
{
public:
    ObsDetectionNode();

private:
    void lidar_callback(ConstScanMessagePtr scan_msg);
    std::vector<float> preprocess_lidar(ConstScanMessagePtr scan_msg);
    std_msgs::msg::Bool detectObstacleAhead(ConstScanMessagePtr scan_msg, const std::vector<float>& processed_ranges);

    std::optional<Candidate> find_closest_candidate(ConstScanMessagePtr scan_msg, const std::vector<float>& processed_ranges);
    std::optional<Edges> find_obstacle_edges(ConstScanMessagePtr scan_msg, const std::vector<float>& processed_ranges, Candidate candidate);
    bool is_obstacle(ConstScanMessagePtr scan_msg, const std::vector<float>& processed_ranges, Edges edges);

    //This uses hysterisis for determining whether or not there's still an obstacle
    bool update_obstacle_state(bool detected);
    float range_at(int i, const std::vector<float>& processed_ranges);
    double angle_at(int i, ConstScanMessagePtr scan_msg);
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    
    // member variables for thresholds 
    double obstacle_distance_;
    double obstacle_edge_jump_;
    double car_width_extended_;
    double obstacle_max_width_ = 0.8;
    int obstacle_hold_frames_;
    int obstacle_hold_counter_ = 0;

    //member variables for LiDAR processing
    double max_lidar_range_;
    double fov_half_angle_;
    double obstacle_min_target_angle_;
    
};

#endif 