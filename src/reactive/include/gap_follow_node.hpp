#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "reactive/msg/gap.hpp"

#include <Eigen/Dense>
#include <limits>

#define DEG2RAD(x) x *(M_PI / 180.0)

class GapFollowNode : public rclcpp::Node
{
public:
    GapFollowNode();

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<reactive::msg::Gap>::SharedPtr gap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;

    // ROS2 parameters
    bool use_fallback_method_ = false;
    bool enable_deadman_ = true;
    int degree_ = 2;
    double steering_gain_ = 1.0;
    double k_samples_ = 200;
    double max_steering_angle_ = DEG2RAD(45.0f);
    double max_speed_ = 3.5;
    double min_speed_ = 0.25;
    double hysteresis_alpha_ = 0.3;
    double speed_curve_scale_ = 1.0;
    double max_slew_rate_ = 0.1;

    double filtered_steering_angle_ = 0.0;
    double commanded_steering_angle_ = 0.0;
    double last_callback_time_ = 0.0;

    bool deadman_engaged_ = false;


    /// @brief Callback invoked each time we find a valid gap from the laser scan.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Baseline method that publishes to drive, driving to the best gap found in the gap_msg.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Uses the method of least squares to determine a curve the car should follow based on the gap it recieves.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void least_squares_pathfinding(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Determine coefficients for the best fitting curve, given a set of inputs (X coordinates) and outputs (Y coordinates)
    /// @param x Vector of X coordinates
    /// @param y Vector of Y coordinates
    /// @returns Vector of coefficients for the polynomial, ordered from lowest degree to highest degree
    Eigen::VectorXd fit_polynomial(const Eigen::VectorXd &x, const Eigen::VectorXd &y);

    /// @brief Given coefficients and an input value, returns the output value. Basically y = f(x).
    /// @param x Input value
    /// @param coefficients Coefficients of the function, ordered from lowest degree to highest degree
    /// @returns The output value 'y'
    double get_curve_output(double x, Eigen::VectorXd coefficients);

    /// @brief Determine steering angle based on coefficients that were calculated
    /// @param coefficients Coefficients of the function, ordered from lowest degree to highest degree
    /// @param theta Vector of angles in the gap
    /// @param max_lookahead The maximum lookahead distance that the value should be clamped too
    /// @returns The angle the car should steer
    double compute_steering_angle(Eigen::VectorXd coefficients, Eigen::VectorXd theta, double max_lookahead);

    /// @brief The mathematical function that determines the car's speed based on the target_angle
    /// @param angle The target angle
    /// @return The speed the car should drive at
    double angle_to_speed_function(double angle);

    /// @brief Runs everytime something is published to the deadman topic
    void deadman_callback(const std_msgs::msg::Bool::ConstSharedPtr deadman_msg);
};
