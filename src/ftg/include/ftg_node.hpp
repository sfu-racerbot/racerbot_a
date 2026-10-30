#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
using std::vector;
/// @brief The ROS node responsible for controlling the car with Follow The Gap
class FollowTheGapNode : public rclcpp::Node
{
public:
    FollowTheGapNode();

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_msg_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_brake_sub_;
    double max_speed_;
    double min_speed_;
    double max_lidar_range_;
    double fov_half_angle_;
    double car_width_;
    double minimum_gap_threshold_;
    bool enable_safety_;
    bool safety_brake_engaged_;
    double safety_bubble_scale_;
    double prev_angle_adjuster_;  // scale weight of prev angle when smoothing
    double prev_angle_;
    double bubble_lookahead_;     // how far ahead an obj can be and get a bubble
    double bubble_half_width_;    // how far either side of the car's centreline a obj can be and get bubble
    double wheelbase_;            // distance between front and rear axles
    double lookahead_max_;        // cap on the target distance used for steering
    double steering_deadzone_;    // steering angles below this (rad) are zeroed to stop wobble
    double steering_angle_max_;    //Max steering angle in radians

    /// @brief clean lidar scan
    /// @param scan_msg The scan data from the lidar
    /// @return preprocessed range vector
    vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief extend obstacles outward to account for car width
    /// @param scan_msg the scan data from the lidar
    /// @param ranges preprocessed range vector to modify in place
    void extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, vector<float>& ranges);

    /// @brief creates a safety bubble around closest obstacle
    /// @param ranges preprocessed range vector to modify in place
    /// @param scan_msg the scan data from the lidar
    void draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, std::vector<float>& ranges);

    /// @brief find the index of the best point in the furthest valid gap
    /// @brief Find all gaps in the lidar ranges that are deeper than the minimum threshold
    /// @param ranges preprocessed and extended range vector
    /// @return a vector of start and end indices for each gap
    vector<std::pair<int, int>> find_all_gaps(const vector<float>& ranges);

    /// @brief Evaluates all found gaps and picks the best one based on width * depth
    /// @param scan_msg The scan data from the lidar
    /// @param ranges preprocessed and extended range vector
    /// @param gaps a vector of start and end indices for each gap
    /// @return a pair representing the start and end index of the best gap
    std::pair<int, int> pick_best_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
        const vector<float>& ranges,
        const vector<std::pair<int, int>>& gaps);

    /// @brief Finds the exact index (center) to drive towards within the best gap
    /// @param best_gap a pair representing the start and end index of the best gap
    /// @return index of the best point to target, or -1 if no valid gap
    int pick_best_point(const std::pair<int, int>& best_gap, const vector<float>& ranges);

    /// @brief func to be called when the lidar completes a scan
    /// @param scan_msg The scan data from the lidar
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief Runs everytime something is published to the safety brake topic
    void safety_brake_callback(const std_msgs::msg::Bool::ConstSharedPtr safety_brake_msg);
};
