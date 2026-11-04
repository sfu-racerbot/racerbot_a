#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class DisparityExtenderNode : public rclcpp::Node
{
public:
    DisparityExtenderNode();

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_brake_sub_;
    
    bool enable_safety_;
    bool enable_stop_;
    float prev_steer_ = 0.0f;
    double max_steering_angle_; // max angle that the car can steer (rad)
    double max_lidar_range_;
    double car_width_;
    double car_width_extended_;
    double disparity_threshold_;
    double fov_half_angle_;
    double minimum_gap_threshold_;
    double max_speed_;
    double min_speed_;
    double alpha_;
    double wheelbase_;
    double max_lateral_acc_;
    bool enable_safety_;

    bool safety_brake_engaged_;

    /// @brief Callback invoked each time the lidar completes a new scan.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief Preprocesses a new lidar scan into a cleaned ranges array.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    /// @return Output vector to be filled with the preprocessed range values.
    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief Extends obstacles by decreasing the ranges in a bubble
    ///        of the closer points in a disparity, so the car doesn't clip corners.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    /// @param ranges Preprocessed range values to mutate in place, applying disparity extension.
    void extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, std::vector<float> &ranges);

    /// @brief creates a safety bubble around closest obstacle
    /// @param ranges preprocessed range vector to modify in place
    /// @param scan_msg the scan data from the lidar
    void draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, 
                        std::vector<float>& ranges);

    /// @brief Finds the indices of the furthest gap of the ranges array to steer toward, returns -1 if not found.
    /// @param scan_msg Shared pointer to the incoming LaserScan message.
    /// @param ranges Ranges array (after obstacle extension) to search for the best gap.
    /// @return Indices into ranges corresponding to the furthest gap.
    std::pair<int, int> find_furthest_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, const std::vector<float> &ranges);

    /// @brief Find the index of the furthest gap given, returns -1 if not found.
    /// @param ranges Ranges array (after obstacle extension) to search for the best gap.
    /// @param gap Pair of indices into ranges corresponding to the gap.
    /// @return Range and steering angle corresponding to the best point.
    int find_furthest_point(std::vector<float> &ranges, const std::pair<int, int> &gap);

    /// @brief Publishes to drive, driving to the given gap.
    /// @param range Target point distance from the vehicle
    /// @param steering_angle Steering angle toward the target point
    void drive_best_point(const float range, const float steering_angle);

    /// @brief Runs everytime something is published to the safety brake topic
    void safety_brake_callback(const std_msgs::msg::Bool::ConstSharedPtr safety_brake_msg);
};
