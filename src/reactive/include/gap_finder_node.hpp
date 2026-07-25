#include <algorithm>
#include <cmath>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reactive/msg/gap.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class GapFinderNode : public rclcpp::Node
{
public:
  GapFinderNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<reactive::msg::Gap>::SharedPtr gap_pub_;
  double max_lidar_range_;
  double car_width_extended_;
  double disparity_threshold_;
  double fov_half_angle_;
  double minimum_gap_threshold_;

  /// @brief Callback invoked each time the lidar completes a new scan.
  /// @param scan_msg Shared pointer to the incoming LaserScan message.
  void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  /// @brief Preprocesses a new lidar scan into a cleaned ranges array.
  /// @param scan_msg Shared pointer to the incoming LaserScan message.
  /// @return Output vector to be filled with the preprocessed range values.
  std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  /// @brief Inflates (extends) obstacles by zeroing out ranges within a safety
  ///        bubble around points closer than a threshold, so the car doesn't clip corners.
  /// @param scan_msg Shared pointer to the incoming LaserScan message.
  /// @param ranges Preprocessed range values to mutate in place, applying obstacle bubbles.
  void extend_obstacles(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, std::vector<float> & ranges);

  /// @brief Finds the indices of the furthest gap of the ranges array to steer toward.
  /// @param scan_msg Shared pointer to the incoming LaserScan message.
  /// @param ranges Ranges array (after obstacle extension) to search for the best gap.
  /// @return Indices into ranges corresponding to the furthest gap.
  std::pair<int, int> find_furthest_gap(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, std::vector<float> & ranges);
};
