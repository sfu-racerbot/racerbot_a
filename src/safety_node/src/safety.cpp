#include "safety.hpp"

constexpr double millisecond_to_nanosecond = 1000000.0;

SafetyNode::SafetyNode() : rclcpp::Node("safety_node")
{
  this->safety_brake_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/safety/brake", 10);
  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&SafetyNode::joy_callback, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 1, std::bind(&SafetyNode::odometry_callback, this, std::placeholders::_1));

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1, std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));

  this->declare_parameter("deadman_button", 4);
  this->declare_parameter("message_timeout_duration", 50.0);
  this->declare_parameter("ttc_threshold", 0.3);
  this->declare_parameter("enable_ttc", true);
  this->declare_parameter("max_ttc_angle_deg", 90.0);

  deadman_button_ = this->get_parameter("deadman_button").as_int();
  message_timeout_duration_ = this->get_parameter("message_timeout_duration").as_double() * millisecond_to_nanosecond;
  ttc_threshold_ = static_cast<float>(this->get_parameter("ttc_threshold").as_double());
  enable_ttc_ = this->get_parameter("enable_ttc").as_bool();
  max_ttc_angle_deg_ = this->get_parameter("max_ttc_angle_deg").as_double() * M_PI / 180.0;

  last_deadman_state = false;

  timer_ = create_wall_timer(std::chrono::milliseconds(50),
                             std::bind(&SafetyNode::safety_update, this));

  RCLCPP_INFO(this->get_logger(), "Safety node launched successfully. Engaging auto deadman hold until /joy connects...");
}

void SafetyNode::safety_update()
{
  auto age = (now() - last_msg_time_).nanoseconds();
  std_msgs::msg::Bool msg;

  // ensures that the deadman is held when messages stop being sent from /joy
  if (age >= message_timeout_duration_ && joystick_active_)
  {
    RCLCPP_INFO(this->get_logger(), "/joy message timeout exceeded, engaging auto deadman hold...");
    joystick_active_ = false;
  }

  msg.data = !joystick_active_ || estop_active_ || !deadman_held_;
  this->safety_brake_pub_->publish(msg);
}

void SafetyNode::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  last_msg_time_ = now();
  if (!joystick_active_)
  {
    RCLCPP_INFO(this->get_logger(), "/joy connected, disengaging auto deadman hold...");
  }
  joystick_active_ = true;

  if (static_cast<int>(joy_msg->buttons.size()) > deadman_button_)
  {
    if (joy_msg->buttons[deadman_button_] != 0)
    {
      deadman_held_ = true;
    }
    else
    {
      deadman_held_ = false;
    }
  }
  else
  {
    deadman_held_ = false;
  }

  if (last_deadman_state != deadman_held_)
  {
    if (deadman_held_ == true)
    {
      RCLCPP_INFO(this->get_logger(), "Deadman now engaged");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Deadman now disengaged");
    }
  }
  last_deadman_state = deadman_held_;
}

void SafetyNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  speed_ = static_cast<float>(msg->twist.twist.linear.x);
}

void SafetyNode::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  // TTC = r / (+rdot), where range rate is positive when getting closer

  if (!enable_ttc_)
  {
    estop_active_ = false;
    return;
  }

  for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
  {
    float range = scan_msg->ranges[i];
    if (!std::isfinite(range))
      continue; // skip if no object in proximity

    float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
    if (std::abs(angle) > max_ttc_angle_deg_)
      continue;

    float range_rate = speed_ * std::cos(angle);
    float closing_rate = std::max(range_rate, 0.0f);
    if (closing_rate <= 1e-3f)
      continue; // skip if object is not approaching

    float iTTC = range / closing_rate;

    if (iTTC <= ttc_threshold_)
    {
      if (!estop_active_)
      {
        RCLCPP_INFO(this->get_logger(), "Brake! iTTC=%.2f", iTTC);
      }
      estop_active_ = true;
      return;
    }
  }
  // only if nothing triggers the e-brake should we reset it's state
  estop_active_ = false;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}