#include "safety.hpp"

constexpr double millisecond_to_nanosecond = 1000000.0;

SafetyNode::SafetyNode() : rclcpp::Node("safety_node") {
  this->deadman_held_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/safety/deadman", 10);
  this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&SafetyNode::joy_callback, this, std::placeholders::_1));
  
  this->declare_parameter("deadman_button", 4);
  this->declare_parameter("message_timeout_duration", 50.0);

  deadman_button_ = this->get_parameter("deadman_button").as_int();
  message_timeout_duration_ = this->get_parameter("message_timeout_duration").as_double() * millisecond_to_nanosecond;

  last_deadman_state = false;

  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      auto age = (now() - last_msg_time_).nanoseconds();

      // ensures that the deadman is held when messages stop being sent from /joy
      if (age >= message_timeout_duration_ && joystick_active_) {
        RCLCPP_INFO(this->get_logger(), "/joy message timeout exceeded, engaging auto deadman hold...");
        joystick_active_ = false;
      }

      if (!joystick_active_) {
        std_msgs::msg::Bool msg;
        msg.data = false;
        this->deadman_held_pub_->publish(msg);
      }
  });

  RCLCPP_INFO(this->get_logger(), "Safety node launched successfully. Engaging auto deadman hold until /joy connects...");
}

void SafetyNode::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg) {
  last_msg_time_ = now();
  if (!joystick_active_) {
    RCLCPP_INFO(this->get_logger(), "/joy connected, disengaging auto deadman hold...");
  }
  joystick_active_ = true;

  std_msgs::msg::Bool msg;
   if (static_cast<int>(joy_msg->buttons.size()) > deadman_button_) {
        if (joy_msg->buttons[deadman_button_] != 0) {
          msg.data = true;
        } else {
          msg.data = false;
        }
    } else {
        msg.data = false;
    }

    this->deadman_held_pub_->publish(msg);

    if (last_deadman_state != msg.data) {
      if (msg.data == true) {
        RCLCPP_INFO(this->get_logger(), "Deadman now engaged");
      } else {
        RCLCPP_INFO(this->get_logger(), "Deadman now disengaged");
      }
    }
    last_deadman_state = msg.data;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}