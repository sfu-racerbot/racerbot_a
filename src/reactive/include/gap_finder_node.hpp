#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "reactive/msg/gap.hpp"
#include <vector>

class GapFinderNode : public rclcpp::Node
{
public:
    GapFinderNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<reactive::msg::Gap>::SharedPtr gap_pub_;
    double max_lidar_range_;
    double fov_half_angle_;
    double car_width_;
    double minimum_gap_threshold_;
    double safety_bubble_scale_;
    double bubble_lookahead_;
    double bubble_half_width_;
    double full_bubble_threshold_;
    double disparity_threshold_;
    double obstacle_distance_;
    double obstacle_edge_jump_;
    double obstacle_max_width_;
    double obstacle_min_target_angle_;
    int obstacle_hold_frames_;
    bool obstacle_detection_;

    int obstacle_hold_counter_ = 0;

    /// @brief Callback invoked each time the lidar completes a new scan.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief Preprocesses a new lidar scan into a cleaned ranges array.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    /// @return Output vector to be filled with the preprocessed range values.
    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief creates a safety bubble around closest obstacle
    /// @param ranges preprocessed range vector to modify in place
    /// @param scan_msg the scan data from the lidar
    void draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, std::vector<float> &ranges);

    /// @brief Inflates (extends) obstacles by zeroing out ranges within a safety
    ///        bubble around points closer than a threshold, so the car doesn't clip corners.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    /// @param ranges Preprocessed range values to mutate in place, applying obstacle bubbles.
    void extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, std::vector<float> &ranges);

    /// @brief find the index of the best point in the furthest valid gap
    /// @brief Find all gaps in the lidar ranges that are deeper than the minimum threshold
    /// @param ranges preprocessed and extended range vector
    /// @return a vector of start and end indices for each gap
    std::vector<std::pair<int, int>> find_all_gaps(const std::vector<float>& ranges);

    /// @brief Evaluates all found gaps and picks the best one based on width * depth, returns -1 if not foudn
    /// @param scan_msg The scan data from the lidar
    /// @param ranges preprocessed and extended range vector
    /// @param gaps a vector of start and end indices for each gap
    /// @return a pair representing the start and end index of the best gap
    std::pair<int, int> pick_best_gap(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
        const std::vector<float>& ranges,
        const std::vector<std::pair<int, int>>& gaps
    );

    /// @brief Finds the exact index (center) to drive towards within the best gap
    /// @param best_gap a pair representing the start and end index of the best gap
    /// @return index of the best point to target, or -1 if no valid gap
    int pick_best_point(const std::pair<int, int>& best_gap, const std::vector<float>& ranges);

    /// @brief Updates the node's obstacle state and the obstacle frame hold
    /// @param detected Whether or not an obstacle was detected
    /// @return If the obstacle hold counter is greater than zero, meaning an obstacle is still detected
    bool update_obstacle_state(bool detected);

    /// @brief Detects whether or not there is an obstacle in front of the car
    /// @param scan_msg The LiDAR scan message
    /// @param raw The raw LiDAR scan ranges
    /// @param target_angle The target angle determined by the gap finder
    /// @return Whether or not an obstacle was detected
    bool detect_obstacle(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, const std::vector<float> &raw, double target_angle);
};
