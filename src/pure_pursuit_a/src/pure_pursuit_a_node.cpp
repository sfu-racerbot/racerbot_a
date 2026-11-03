#include "pure_pursuit_a_node.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_a_node")
{
    // ROS2 parameters
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("use_pf", true);
    this->declare_parameter("enable_safety", true);
    this->declare_parameter("track", "levine");
    this->declare_parameter("lookahead_base", 1.0);
    this->declare_parameter("lookahead_min", 0.8);
    this->declare_parameter("lookahead_max", 3.0);
    this->declare_parameter("lookahead_speed_gain", 0.1);
    this->declare_parameter("wheelbase", 0.33);
    this->declare_parameter("max_steering", 0.4189);  // about 24 deg
    this->declare_parameter("max_speed", 4.0);
    this->declare_parameter("min_speed", 1.5);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    use_pf_ = this->get_parameter("use_pf").as_bool();
    enable_safety_ = this->get_parameter("enable_safety").as_bool();
    const std::string track = this->get_parameter("track").as_string();
    lookahead_base_ = this->get_parameter("lookahead_base").as_double();
    lookahead_min_ = this->get_parameter("lookahead_min").as_double();
    lookahead_max_ = this->get_parameter("lookahead_max").as_double();
    lookahead_speed_gain_ = this->get_parameter("lookahead_speed_gain").as_double();
    wheel_base_ = this->get_parameter("wheelbase").as_double();
    max_steering_ = this->get_parameter("max_steering").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();

    // Safety
    safety_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/safety/brake", 10,
        std::bind(&PurePursuit::safety_brake_callback, this, std::placeholders::_1));

    // loading waypoints from the package share directory
    const std::string path =
        ament_index_cpp::get_package_share_directory("pure_pursuit_a") +
        "/waypoints/" + track + ".csv";
    load_waypoints(path);

    // localization
    // pose source: PF output on the real car, ground truth in sim
    const std::string pose_topic = use_pf_ ? "/pf/pose/odom" : odom_topic_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 10,
        std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

    // PF usually doesn't fill in twist, so take speed from wheel odom
    if (use_pf_) {
        speed_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&PurePursuit::speed_callback, this, std::placeholders::_1));
    }

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints for %s", waypoints_.size(), track.c_str());
}

void PurePursuit::safety_brake_callback(const std_msgs::msg::Bool::ConstSharedPtr safety_brake_msg) {
    safety_brake_engaged_ = safety_brake_msg->data;
}

void PurePursuit::speed_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    current_speed_ = msg->twist.twist.linear.x;
}

void PurePursuit::load_waypoints(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open waypoint file: " + path);
    }

    std::string line;
    while (std::getline(file, line)) {
        // ignore empty lines or any starting wth # (comments)
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string x_str, y_str;
        if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str, ',')) {
            continue;
        }

        try {
            Waypoint wp{std::stod(x_str), std::stod(y_str)};
            if (std::isfinite(wp.x) && std::isfinite(wp.y)) {
                waypoints_.push_back(wp);
            }
        } catch (const std::exception&) {
            // ignore rows that arent numbers
        }
    }

    if (waypoints_.empty()) {
        throw std::runtime_error("No valid waypoints found");
    }
}

double PurePursuit::currentLookahead(double v) const
{
    const double L = lookahead_base_ + lookahead_speed_gain_ * std::abs(v);
    return std::clamp(L, lookahead_min_, lookahead_max_);
}

void PurePursuit::drive_to_goal(double local_x, double local_y)
{
    // pure pursuit: curvature of the arc to the goal point
    // local_x^2 + local_y^2 is the squared distance to the goal, so this uses the
    // actual distance to the waypoint rather than L
    const double dist_sq = local_x * local_x + local_y * local_y;
    const double curvature = 2.0 * local_y / dist_sq; // k = 2y/d^2

    // bicycle model: curvature -> steering angle
    double steering = std::atan(wheel_base_ * curvature);
    steering = std::clamp(steering, -max_steering_, max_steering_);

    // slow down in turns: max_speed_ when straight, min_speed_ at full lock
    const double turn_ratio = std::abs(steering) / max_steering_;
    const double target_speed =
        max_speed_ - (max_speed_ - min_speed_) * turn_ratio;

    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = now();
    drive.header.frame_id = "base_link";
    drive.drive.steering_angle = steering;
    drive.drive.speed = target_speed;
    drive_pub_->publish(drive);
}

void PurePursuit::odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{

    int safety_publisher_count = this->safety_brake_sub_->get_publisher_count();
    if (enable_safety_ && (safety_brake_engaged_ || safety_publisher_count == 0)) {
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = 0.0;
        drive_pub_->publish(drive_msg);
        return;
    }

    // current position and orientation of car
    double car_x = msg->pose.pose.position.x;
    double car_y = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // forward speed and speed-scaled lookahead
    const double speed = use_pf_ ? current_speed_ : msg->twist.twist.linear.x;
    const double L = currentLookahead(speed);

    // First odom msg finds the closest waypoint to give the last_goal_index_ to start finding the waypoint from
    if (!initialized_) { // TODO: Handle when to reset
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < waypoints_.size(); i++) {
            double dx = waypoints_[i].x - car_x;
            double dy = waypoints_[i].y - car_y;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < best_dist) {
                best_dist = d;
                last_goal_index_ = i;
            }
        }
        initialized_ = true;
    }

    // searching forward through the waypoints
    for (size_t step = 0; step < waypoints_.size(); ++step) {
        size_t i = (last_goal_index_ + step) % waypoints_.size();
        double dx = waypoints_[i].x - car_x;
        double dy = waypoints_[i].y - car_y;

        // transforming world point to the car frame
        double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy; // how far ahead of car
        double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy; // how far to the left (positive) or right (negative)
        double distance = std::sqrt(dx * dx + dy * dy);

        // waypoint must be INFRONT of the car and ATLEAST L away.
        if (local_x > 0.0 && distance >= L) { // TODO: Interpolate between 2 points possibly
            last_goal_index_ = i;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Goal %zu: world(%.2f, %.2f) local(%.2f, %.2f)",
                i, waypoints_[i].x, waypoints_[i].y, local_x, local_y);

            drive_to_goal(local_x, local_y);

            return;
        }
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No suitable forward waypoint found"); // TODO: Handle failure
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
