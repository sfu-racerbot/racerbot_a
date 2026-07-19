#include "rclcpp/rclcpp.hpp"
#include <vector>

/// @brief Represents a scan from the LiDAR, includes the scan's angle and range (distance)
struct LidarPoint
{
    float angle;
    float range;
};

/// @brief Message struct to communicate between gap_finder and gap_follow
struct GapMessage
{
    long timestamp;
    std::vector<LidarPoint> points;
};