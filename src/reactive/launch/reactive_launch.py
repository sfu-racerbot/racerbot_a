from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    enable_safety_arg = DeclareLaunchArgument(
        "enable_safety",
        default_value="true",
        description="Whether the safety brake from the safety node is enabled",
    )

    use_fallback_follow_method = DeclareLaunchArgument(
        "use_fallback_follow_method",
        default_value="false",
        description="Whether the gap follow node should use the fallback point-following method",
    )

    least_squares_degree = DeclareLaunchArgument(
        "least_squares_degree",
        default_value="2",
        description="Polynomial degree used by the least-squares gap follower",
    )

    steering_gain = DeclareLaunchArgument(
        "steering_gain",
        default_value="2.0",
        description="Gain applied to steering toward the selected gap curve",
    )

    k_samples = DeclareLaunchArgument(
        "k_samples",
        default_value="200",
        description="Number of samples used to find the best steering angle",
    )

    max_steering_angle = DeclareLaunchArgument(
        "max_steering_angle",
        default_value="20.0",
        description="Maximum steering angle allowed for the follower in degrees",
    )

    max_speed = DeclareLaunchArgument(
        "max_speed",
        default_value="4.0",
        description="Maximum forward speed allowed for the gap follower in meters per second",
    )

    min_speed = DeclareLaunchArgument(
        "min_speed",
        default_value="0.5",
        description="Minimum forward speed allowed for the gap follower in meters per second",
    )

    hysteresis_alpha = DeclareLaunchArgument(
        "hysteresis_alpha",
        default_value="0.3",
        description="Smoothing factor for steering-angle hysteresis",
    )

    speed_curve_scale = DeclareLaunchArgument(
        "speed_curve_scale",
        default_value="1.0",
        description="Scale factor for the steering-to-speed curve",
    )
    max_slew_rate = DeclareLaunchArgument(
        "max_slew_rate",
        default_value="170.0",
        description="Maximum rate of change of commanded steering angle, in degrees/sec.",
    )

    opponent_car = LaunchConfiguration("opponent_car")

    opponent_car_arg = DeclareLaunchArgument(
        "opponent_car",
        default_value="false",
        choices=["true", "false"],
        description="If true, remap /drive -> /opp_drive and /scan -> /opp_scan",
    )

    drive_topic = IfElseSubstitution(opponent_car, "/opp_drive", "/drive")
    scan_topic = IfElseSubstitution(opponent_car, "/opp_scan", "/scan")

    obstacle_detection = DeclareLaunchArgument(
        "obstacle_detection",
        default_value="true",
        description="Enables the obstacle detection",
    )

    max_lidar_range_arg = DeclareLaunchArgument(
        "max_lidar_range",
        default_value="10.0",
        description="Maximum LiDAR range considered (meters).",
    )
    car_width_extended_arg = DeclareLaunchArgument(
        "car_width_extended",
        default_value="0.55",
        description="Extended car width used for gap/bubble computation (meters).",
    )
    disparity_threshold_arg = DeclareLaunchArgument(
        "disparity_threshold",
        default_value="1.5",
        description="Range difference between adjacent beams that triggers an obstacle bubble (meters).",
    )
    fov_half_angle_deg_arg = DeclareLaunchArgument(
        "fov_half_angle_deg",
        default_value="90.0",
        description="Half field-of-view angle considered for gap finding (degrees).",
    )
    minimum_gap_threshold_arg = DeclareLaunchArgument(
        "minimum_gap_threshold",
        default_value="0.1",
        description="Minimum range for a LiDAR point to be considered part of a free gap (meters).",
    )
    obstacle_distance_arg = DeclareLaunchArgument(
        "obstacle_distance",
        default_value="2.0",
        description="Maximum distance for an obstacle to be considered ahead (meters).",
    )
    obstacle_edge_jump_arg = DeclareLaunchArgument(
        "obstacle_edge_jump",
        default_value="0.5",
        description="Range jump required to confirm an obstacle edge (meters).",
    )
    obstacle_max_width_arg = DeclareLaunchArgument(
        "obstacle_max_width",
        default_value="0.8",
        description="Maximum obstacle width allowed (meters).",
    )
    obstacle_min_target_angle_deg_arg = DeclareLaunchArgument(
        "obstacle_min_target_angle_deg",
        default_value="0.0",
        description="Minimum target angle for obstacle detection (degrees).",
    )
    obstacle_hold_frames_arg = DeclareLaunchArgument(
        "obstacle_hold_frames",
        default_value="8",
        description="Number of frames to hold obstacle state after detection.",
    )

    hard_turn_threshold_deg = DeclareLaunchArgument(
        "hard_turn_threshold_deg",
        default_value="16.0",
        description="What angle should the car consider to be a 'hard turn'",
    )

    return LaunchDescription(
        [
            enable_safety_arg,
            use_fallback_follow_method,
            least_squares_degree,
            steering_gain,
            k_samples,
            max_steering_angle,
            max_speed,
            min_speed,
            hysteresis_alpha,
            speed_curve_scale,
            max_slew_rate,
            opponent_car_arg,
            obstacle_detection,
            max_lidar_range_arg,
            car_width_extended_arg,
            disparity_threshold_arg,
            fov_half_angle_deg_arg,
            minimum_gap_threshold_arg,
            obstacle_distance_arg,
            obstacle_edge_jump_arg,
            obstacle_max_width_arg,
            obstacle_min_target_angle_deg_arg,
            obstacle_hold_frames_arg,
            hard_turn_threshold_deg,
            # Launch gap_finder_node
            Node(
                package="reactive",
                executable="gap_finder_node",
                name="gap_finder_node",
                output="screen",
                remappings=[("/drive", drive_topic), ("/scan", scan_topic)],
                parameters=[
                    {"obstacle_detection": LaunchConfiguration("obstacle_detection")},
                    {"max_lidar_range": LaunchConfiguration("max_lidar_range")},
                    {"car_width_extended": LaunchConfiguration("car_width_extended")},
                    {"disparity_threshold": LaunchConfiguration("disparity_threshold")},
                    {"fov_half_angle_deg": LaunchConfiguration("fov_half_angle_deg")},
                    {
                        "minimum_gap_threshold": LaunchConfiguration(
                            "minimum_gap_threshold"
                        )
                    },
                    {"obstacle_distance": LaunchConfiguration("obstacle_distance")},
                    {"obstacle_edge_jump": LaunchConfiguration("obstacle_edge_jump")},
                    {"obstacle_max_width": LaunchConfiguration("obstacle_max_width")},
                    {
                        "obstacle_min_target_angle_deg": LaunchConfiguration(
                            "obstacle_min_target_angle_deg"
                        )
                    },
                    {
                        "obstacle_hold_frames": LaunchConfiguration(
                            "obstacle_hold_frames"
                        )
                    },
                ],
            ),
            # Launch gap_follower_node
            Node(
                package="reactive",
                executable="gap_follow_node",
                name="gap_follow_node",
                output="screen",
                parameters=[
                    {
                        "use_fallback_method": LaunchConfiguration(
                            "use_fallback_follow_method"
                        ),
                        "degree": LaunchConfiguration("least_squares_degree"),
                        "steering_gain": LaunchConfiguration("steering_gain"),
                        "k_samples": LaunchConfiguration("k_samples"),
                        "max_steering_angle": LaunchConfiguration("max_steering_angle"),
                        "max_speed": LaunchConfiguration("max_speed"),
                        "min_speed": LaunchConfiguration("min_speed"),
                        "hysteresis_alpha": LaunchConfiguration("hysteresis_alpha"),
                        "speed_curve_scale": LaunchConfiguration("speed_curve_scale"),
                        "max_slew_rate": LaunchConfiguration("max_slew_rate"),
                        "enable_safety": LaunchConfiguration("enable_safety"),
                        "hard_turn_threshold_deg": LaunchConfiguration(
                            "hard_turn_threshold_deg"
                        ),
                    }
                ],
                remappings=[("/drive", drive_topic), ("/scan", scan_topic)],
            ),
        ]
    )
