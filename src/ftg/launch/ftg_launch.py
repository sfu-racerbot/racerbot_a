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

    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        default_value="4.5",
        description="Maximum forward speed allowed in meters per second",
    )

    min_speed_arg = DeclareLaunchArgument(
        "min_speed",
        default_value="0.7",
        description="Minimum forward speed allowed in meters per second",
    )

    max_lidar_range_arg = DeclareLaunchArgument(
        "max_lidar_range",
        default_value="10.0",
        description="Maximum LiDAR range considered (meters)",
    )

    fov_half_angle_deg_arg = DeclareLaunchArgument(
        "fov_half_angle_deg",
        default_value="100.0",
        description="Half field-of-view angle considered for processing (degrees)",
    )

    car_width_arg = DeclareLaunchArgument(
        "car_width",
        default_value="0.4",
        description="Car width used for safety bubble computation (meters)",
    )

    minimum_gap_threshold_arg = DeclareLaunchArgument(
        "minimum_gap_threshold",
        default_value="1.0",
        description="Minimum range for a LiDAR point to be considered part of a free gap (meters)",
    )

    safety_bubble_scale_arg = DeclareLaunchArgument(
        "safety_bubble_scale",
        default_value="0.5",
        description="Scale factor for the safety bubble size",
    )

    prev_angle_adjuster_arg = DeclareLaunchArgument(
        "prev_angle_adjuster",
        default_value="0.6",
        description="Weight applied to previous steering angle for smoothing",
    )

    bubble_lookahead_arg = DeclareLaunchArgument(
        "bubble_lookahead",
        default_value="3.0",
        description="Maximum lookahead distance for obstacle bubble (meters)",
    )

    bubble_half_width_arg = DeclareLaunchArgument(
        "bubble_half_width",
        default_value="0.45",
        description="Half width of the obstacle bubble region (meters)",
    )

    wheelbase_arg = DeclareLaunchArgument(
        "wheelbase",
        default_value="0.33",
        description="Vehicle wheelbase in meters",
    )

    lookahead_max_arg = DeclareLaunchArgument(
        "lookahead_max",
        default_value="1.8",
        description="Maximum lookahead distance for target point selection (meters)",
    )

    steering_deadzone_arg = DeclareLaunchArgument(
        "steering_deadzone",
        default_value="0.7",
        description="Deadzone angle in radians below which steering is zeroed out",
    )

    steering_max_angle_arg = DeclareLaunchArgument(
        "steering_angle_max",
        default_value="0.349",
        description="Maximum turning angle in radians."
    )

    max_lateral_acc_arg = DeclareLaunchArgument(
        "max_lateral_acc",
        default_value="3.5",
        description="Maximum lateral acceleration in m/s^2",
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

    return LaunchDescription(
        [
            enable_safety_arg,
            opponent_car_arg,
            max_speed_arg,
            min_speed_arg,
            max_lidar_range_arg,
            fov_half_angle_deg_arg,
            car_width_arg,
            minimum_gap_threshold_arg,
            safety_bubble_scale_arg,
            prev_angle_adjuster_arg,
            bubble_lookahead_arg,
            bubble_half_width_arg,
            wheelbase_arg,
            lookahead_max_arg,
            steering_deadzone_arg,
            steering_max_angle_arg,
            max_lateral_acc_arg,
            # Launch follow the gap
            Node(
                package="ftg",
                executable="ftg_node",
                name="ftg_node",
                output="screen",
                parameters=[
                    {"enable_safety": LaunchConfiguration("enable_safety")},
                    {"max_speed": LaunchConfiguration("max_speed")},
                    {"min_speed": LaunchConfiguration("min_speed")},
                    {"max_lidar_range": LaunchConfiguration("max_lidar_range")},
                    {"fov_half_angle_deg": LaunchConfiguration("fov_half_angle_deg")},
                    {"car_width": LaunchConfiguration("car_width")},
                    {"minimum_gap_threshold": LaunchConfiguration("minimum_gap_threshold")},
                    {"safety_bubble_scale": LaunchConfiguration("safety_bubble_scale")},
                    {"prev_angle_adjuster": LaunchConfiguration("prev_angle_adjuster")},
                    {"bubble_lookahead": LaunchConfiguration("bubble_lookahead")},
                    {"bubble_half_width": LaunchConfiguration("bubble_half_width")},
                    {"wheelbase": LaunchConfiguration("wheelbase")},
                    {"lookahead_max": LaunchConfiguration("lookahead_max")},
                    {"steering_deadzone": LaunchConfiguration("steering_deadzone")},
                    {"steering_angle_max": LaunchConfiguration("steering_angle_max")},
                    {"max_lateral_acc": LaunchConfiguration("max_lateral_acc")},
                ],
                remappings=[("/drive", drive_topic), ("/scan", scan_topic)],
            ),
        ]
    )
