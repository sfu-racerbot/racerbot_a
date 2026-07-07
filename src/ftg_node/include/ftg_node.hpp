#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

/// @brief The ROS node responsible for controlling the car with Follow The Gap
class FollowTheGapNode : public rclcpp::Node
{
public:
    FollowTheGapNode();

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_msg_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;

    /// @brief Function to be called when the lidar completes a scan
    /// @param scan_msg The scan data from the lidar
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};