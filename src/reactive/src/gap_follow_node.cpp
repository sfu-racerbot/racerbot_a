#include "gap_follow_node.hpp"

GapFollowNode::GapFollowNode() : Node("gap_follow_node")
{
  RCLCPP_INFO(this->get_logger(), "Gap Follow node started");

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

  gap_sub_ = this->create_subscription<reactive::msg::Gap>(
    "gap", 10, std::bind(&GapFollowNode::gap_callback, this, std::placeholders::_1));

  this->declare_parameter("use_fallback_method", false);
  this->declare_parameter("degree", 2);
  this->declare_parameter("steering_gain", 1.0);
  this->declare_parameter("lookahead_distance", 1.5);

  use_fallback_method = this->get_parameter("use_fallback_method").as_bool();
  RCLCPP_INFO(
    this->get_logger(), "Using follow method: '%s'",
    use_fallback_method ? "drive_best_point" : "least_squares");

  degree = this->get_parameter("degree").as_int();
  steering_gain = this->get_parameter("steering_gain").as_double();
  lookahead_distance = this->get_parameter("lookahead_distance").as_double();
}

void GapFollowNode::gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
  if (use_fallback_method) {
    drive_best_point(gap_msg);
  } else {
    least_squares_pathfinding(gap_msg);
  }
}

void GapFollowNode::drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
  // Speed depends on target point range
  float velocity;
  float target_range = gap_msg->target_range;
  if (target_range > 2)
    velocity = 3.0f;
  else if (target_range > 1)
    velocity = 2.0f;
  else if (target_range > 0.5)
    velocity = 1.0f;
  else
    velocity = 0.5f;

  // Publishing to drive
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->now();
  drive_msg.drive.steering_angle = gap_msg->target_angle;
  drive_msg.drive.speed = velocity;
  drive_pub_->publish(drive_msg);
}

void GapFollowNode::least_squares_pathfinding(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
  if (static_cast<int>(gap_msg->ranges.size()) < degree + 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "Not enough gap points to fit degree-%d polynomial. Falling back to drive_best_point",
      degree);
    drive_best_point(gap_msg);
    return;
  }

  // Create coordinate vectors for least squares to use
  Eigen::VectorXd x(gap_msg->ranges.size());
  Eigen::VectorXd y(gap_msg->ranges.size());

  for (int i = 0; i < gap_msg->ranges.size(); i++) {
    // Convert polar coordinates to cartesian coordinates
    std::pair<double, double> coordinate =
      polar_to_cartesian(gap_msg->ranges[i], gap_msg->angles[i]);
    x(i) = coordinate.first;
    y(i) = coordinate.second;
  }

  // make sure we get the furthest point in the LiDAR gap to clamp our lookahead value to
  double max_lookahead = std::max(x.maxCoeff(), 0.1);

  // Determine polynomial coefficients
  Eigen::VectorXd coefficients = fit_polynomial(x, y);

  double lookahead = std::clamp(lookahead_distance, 0.1, max_lookahead);
  double steering_angle = compute_steering_angle_simple(coefficients, lookahead);
  double absolute_angle = std::abs(steering_angle);

  double velocity = angle_to_speed_function(absolute_angle);

  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->now();
  drive_msg.drive.steering_angle = steering_angle;
  drive_msg.drive.speed = velocity;
  drive_pub_->publish(drive_msg);
}

std::pair<double, double> GapFollowNode::polar_to_cartesian(double r, double theta)
{
  return std::pair<double, double>(r * std::cos(theta), r * std::sin(theta));
}

Eigen::VectorXd GapFollowNode::fit_polynomial(const Eigen::VectorXd & x, const Eigen::VectorXd & y)
{
  int n = x.size();
  Eigen::MatrixXd A(n, degree + 1);

  // Build Vandermonde matrix
  for (int i = 0; i < n; ++i) {
    double val = 1.0;
    for (int j = 0; j <= degree; ++j) {
      A(i, j) = val;
      val *= x(i);
    }
  }

  // Solve A * coeffs = y
  Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(y);

  return coeffs;
}

double GapFollowNode::get_curve_output(double x, Eigen::VectorXd coefficients)
{
  double y = 0.0;
  for (int i = 0; i < degree + 1; i++)  // make sure to factor in constant term
  {
    y += coefficients[i] * (std::pow(x, i));
  }
  return y;
}

double GapFollowNode::compute_steering_angle_simple(Eigen::VectorXd coefficients, double target_x)
{
  double target_y = get_curve_output(target_x, coefficients);
  double alpha = std::atan2(target_y, target_x);

  double steering_angle = steering_gain * alpha;
  return std::clamp(steering_angle, -max_steering_angle, max_steering_angle);
}

double GapFollowNode::angle_to_speed_function(double angle)
{
  return -1.6 * std::log(angle + 0.3) + 2.4;  // current formula from Desmos tinkering
                                              // <https://www.desmos.com/calculator/ijolz4pnpy>
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GapFollowNode>());
  rclcpp::shutdown();
  return 0;
}
