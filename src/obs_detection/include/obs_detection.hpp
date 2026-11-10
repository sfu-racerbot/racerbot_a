#ifndef OBS_DETECT_NODE_HPP_
#define OBS_DETECT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <utility>

struct Candidate
{
    int index;
    float range;
    double angle;
};

struct Edges
{
    int left;
    int right;
};

struct Vector3
{
    double x{};
    double y{};
    double z{};

    constexpr Vector3 &operator-=(const Vector3 &rhs) noexcept
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr Vector3 operator-() const noexcept
    {
        return Vector3{-x, -y, -z};
    }
};

constexpr Vector3 operator-(Vector3 a, const Vector3 &b) noexcept
{
    a -= b;
    return a;
}

using ConstScanMessagePtr = sensor_msgs::msg::LaserScan::ConstSharedPtr;

class ObsDetectionNode : public rclcpp::Node
{
public:
    ObsDetectionNode();

private:
    void lidar_callback(ConstScanMessagePtr scan_msg);
    std::vector<float> preprocess_lidar(ConstScanMessagePtr scan_msg);
    std_msgs::msg::Bool detectObstacleAhead(ConstScanMessagePtr scan_msg, const std::vector<float> &processed_ranges);

    std::optional<Candidate> find_closest_candidate(ConstScanMessagePtr scan_msg, const std::vector<float> &processed_ranges);
    std::optional<Edges> find_obstacle_edges(ConstScanMessagePtr scan_msg, const std::vector<float> &processed_ranges, Candidate candidate);
    bool is_obstacle(ConstScanMessagePtr scan_msg, const std::vector<float> &processed_ranges, Edges edges);

    // This uses hysterisis for determining whether or not there's still an obstacle
    bool update_obstacle_state(bool detected, const Candidate &candidate);
    float range_at(int i, const std::vector<float> &processed_ranges);
    double angle_at(int i, ConstScanMessagePtr scan_msg);

    // Follow-the-gap fallback
    // Only runs (and only publishes to the drive topic) while an obstacle is held.
    ackermann_msgs::msg::AckermannDriveStamped ftg_compute_drive(ConstScanMessagePtr scan_msg);
    std::vector<float> ftg_preprocess(ConstScanMessagePtr scan_msg);
    void ftg_draw_safety_bubble(ConstScanMessagePtr scan_msg, std::vector<float> &ranges);
    std::vector<std::pair<int, int>> ftg_find_all_gaps(const std::vector<float> &ranges);
    std::pair<int, int> ftg_pick_best_gap(ConstScanMessagePtr scan_msg,
                                          const std::vector<float> &ranges, const std::vector<std::pair<int, int>> &gaps);
    int ftg_pick_best_point(const std::pair<int, int> &best_gap, const std::vector<float> &ranges);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // member variables for thresholds
    double max_obstacle_distance_;
    double obstacle_edge_jump_;
    double car_width_extended_;
    double obstacle_max_width_ = 0.8;
    int obstacle_hold_frames_;
    int obstacle_hold_counter_ = 0;
    double vector_difference_threshold = 0.1;

    // member variables for LiDAR processing
    double fov_half_angle_;
    double obstacle_min_target_angle_;

    // follow-the-gap parameters
    double max_speed_;
    double min_speed_;
    double max_lidar_range_;
    double ftg_fov_half_angle_;
    double car_width_;
    double minimum_gap_threshold_;
    double safety_bubble_scale_;
    double prev_angle_adjuster_; // weight of previous steering angle when smoothing
    double bubble_lookahead_;    // how far ahead an obj can be and get a bubble
    double bubble_half_width_;   // how far either side of centreline an obj can be and get a bubble
    double wheelbase_;
    double lookahead_max_;
    double steering_deadzone_; // radians, steering below this is zeroed
    double prev_angle_ = 0.0;

    // state variables
    std::optional<Vector3> last_obstacle_position_;
};

#endif
