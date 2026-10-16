#include "gap_follow_node.hpp"

GapFollowNode::GapFollowNode() : Node("gap_follow_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Follow node started");

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    gap_sub_ = this->create_subscription<reactive::msg::Gap>(
        "gap", 10,
        std::bind(&GapFollowNode::gap_callback, this, std::placeholders::_1));
    
    deadman_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/safety/deadman", 10,
        std::bind(&GapFollowNode::deadman_callback, this, std::placeholders::_1));

    this->declare_parameter("use_fallback_method", false);
    this->declare_parameter("degree", 2);
    this->declare_parameter("steering_gain", 1.0);
    this->declare_parameter("k_samples", 200);
    this->declare_parameter("max_steering_angle", 45.0);
    this->declare_parameter("max_speed", 4.0);
    this->declare_parameter("min_speed", 0.5);
    this->declare_parameter("hysteresis_alpha", 0.3);
    this->declare_parameter("speed_curve_scale", 1.0);
    this->declare_parameter("max_slew_rate", 170.0);
    this->declare_parameter("enable_deadman", true);

    use_fallback_method_ = this->get_parameter("use_fallback_method").as_bool();
    RCLCPP_INFO(this->get_logger(), "Using follow method: '%s'", use_fallback_method_ ? "drive_best_point" : "least_squares");

    enable_deadman_ = this->get_parameter("enable_deadman").as_bool();

    degree_ = this->get_parameter("degree").as_int();
    steering_gain_ = this->get_parameter("steering_gain").as_double();
    k_samples_ = this->get_parameter("k_samples").as_int();
    max_steering_angle_ = DEG2RAD(this->get_parameter("max_steering_angle").as_double());
    max_speed_ = this->get_parameter("max_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    hysteresis_alpha_ = this->get_parameter("hysteresis_alpha").as_double();
    speed_curve_scale_ = this->get_parameter("speed_curve_scale").as_double();
    max_slew_rate_ = this->get_parameter("max_slew_rate").as_double();
}

void GapFollowNode::gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
    int safety_publisher_count = this->deadman_sub_->get_publisher_count();
    if (enable_deadman_ && (!deadman_engaged_ || safety_publisher_count == 0)) {
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = commanded_steering_angle_;
        drive_msg.drive.speed = 0.0;
        drive_pub_->publish(drive_msg);
        return;
    }

    if (use_fallback_method_)
    {
        drive_best_point(gap_msg);
    }
    else
    {
        least_squares_pathfinding(gap_msg);
    }
}

void GapFollowNode::drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
    // Speed depends on target point range
    float velocity;
    float target_range = gap_msg->target_range;

    /* Slower speed at small range
    double min_range = 0.5;
    double max_range = 4.0;

    double t = (target_range - min_range) / (max_range - min_range);
    t = std::clamp(t, 0.0, 1.0);

    velocity = static_cast<float>(min_speed_ + (t * t) * (max_speed_ - min_speed_));
    */

    // Step linear
    if (target_range > 4.0f)
        velocity = max_speed_;
    else if (target_range > 3.0f)
        velocity = 4.0f;
    else if (target_range > 2.0f)
        velocity = 3.0f;
    else if (target_range > 1.0f)
        velocity = 2.0f;
    else if (target_range > 0.5)
        velocity = 1.0f;
    else
        velocity = min_speed_;

    // Publishing to drive
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = gap_msg->target_angle;
    drive_msg.drive.speed = velocity;
    drive_pub_->publish(drive_msg);
}

void GapFollowNode::least_squares_pathfinding(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{

    if (static_cast<int>(gap_msg->ranges.size()) < degree_ + 1)
    {
        RCLCPP_WARN(this->get_logger(), "Not enough gap points to fit degree-%d polynomial. Falling back to drive_best_point", degree_);
        drive_best_point(gap_msg);
        return;
    }

    if (gap_msg->ranges.size() != gap_msg->angles.size())
    {
        RCLCPP_WARN(this->get_logger(), "Ranges and angles have different sizes. Falling back to drive_best_point");
        drive_best_point(gap_msg);
        return;
    }

    // Create coordinate vectors for least squares to use
    Eigen::VectorXd theta(gap_msg->ranges.size());
    Eigen::VectorXd r(gap_msg->ranges.size());

    for (size_t i = 0; i < gap_msg->ranges.size(); i++)
    {
        theta(i) = gap_msg->angles[i];
        r(i) = gap_msg->ranges[i];
    }

    // make sure we get the furthest point in the LiDAR gap to clamp our lookahead value to
    double max_lookahead = std::max(r.maxCoeff(), 0.1);

    // Determine polynomial coefficients
    Eigen::VectorXd coefficients = fit_polynomial(theta, r);

    double steering_angle = compute_steering_angle(coefficients, theta, max_lookahead);

    double now_sec = this->now().seconds();
    double dt = (last_callback_time_ > 0.0) ? (now_sec - last_callback_time_) : 0.02;
    last_callback_time_ = now_sec;

    filtered_steering_angle_ = hysteresis_alpha_ * steering_angle + (1 - hysteresis_alpha_) * filtered_steering_angle_;

    double max_delta = max_slew_rate_ * dt; // new param, rad/sec
    double delta = std::clamp(filtered_steering_angle_ - commanded_steering_angle_, -max_delta, max_delta);
    commanded_steering_angle_ = std::clamp(commanded_steering_angle_ + delta, -max_steering_angle_, max_steering_angle_);

    double velocity = angle_to_speed_function(filtered_steering_angle_);
    commanded_steering_angle_ = steering_gain_ * filtered_steering_angle_;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = commanded_steering_angle_;
    drive_msg.drive.speed = velocity;
    drive_pub_->publish(drive_msg);
}

Eigen::VectorXd GapFollowNode::fit_polynomial(const Eigen::VectorXd &x, const Eigen::VectorXd &y)
{
    int n = x.size();
    Eigen::MatrixXd A(n, degree_ + 1);

    // Build Vandermonde matrix
    for (int i = 0; i < n; ++i)
    {
        double val = 1.0;
        for (int j = 0; j <= degree_; ++j)
        {
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
    for (int i = 0; i < degree_ + 1; i++) // make sure to factor in constant term
    {
        y += coefficients[i] * (std::pow(x, i));
    }
    return y;
}

double GapFollowNode::compute_steering_angle(Eigen::VectorXd coefficients, Eigen::VectorXd theta, double max_lookahead)
{
    double theta_min = std::max(theta.minCoeff(), -max_steering_angle_);
    double theta_max = std::min(theta.maxCoeff(), max_steering_angle_);

    double best_theta = theta_min;
    double best_range = -std::numeric_limits<double>::max();

    for (int i = 0; i <= k_samples_; i++)
    {
        double t = theta_min + (theta_max - theta_min) * i / k_samples_;
        double predicted_r = get_curve_output(t, coefficients);
        if (predicted_r >= max_lookahead) continue;

        if (predicted_r > best_range)
        {
            best_range = predicted_r;
            best_theta = t;
        }
    }
    return best_theta;
}

double GapFollowNode::angle_to_speed_function(double angle)
{

    const double a = -1.6;
    const double b = 1.0;
    const double c = 3.6;

    double y = speed_curve_scale_ * (a * std::log(std::abs(angle) + b) + c); // current formula from Desmos tinkering <https://www.desmos.com/calculator/yeyvbi21nf>
    return std::clamp(y, min_speed_, max_speed_);
}

void GapFollowNode::deadman_callback(const std_msgs::msg::Bool::ConstSharedPtr deadman_msg) {
    deadman_engaged_ = deadman_msg->data;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollowNode>());
    rclcpp::shutdown();
    return 0;
}
