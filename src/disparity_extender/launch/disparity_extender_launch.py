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

    max_lidar_range_arg = DeclareLaunchArgument(
        "max_lidar_range",
        default_value="10.0",
        description="Maximum LiDAR range considered (meters)",
    )

    car_width_arg = DeclareLaunchArgument(
        "car_width",
        default_value="0.4",
        description="Car width used for safety bubble computation (meters)",
    )

    car_width_extended_arg = DeclareLaunchArgument(
        "car_width_extended",
        default_value="0.85",
        description="Extended car width used for gap/bubble computation (meters)",
    )

    disparity_threshold_arg = DeclareLaunchArgument(
        "disparity_threshold",
        default_value="1.0",
        description="Range difference between adjacent beams that triggers an obstacle bubble (meters)",
    )

    fov_half_angle_deg_arg = DeclareLaunchArgument(
        "fov_half_angle_deg",
        default_value="90.0",
        description="Half field-of-view angle considered for processing (degrees)",
    )

    minimum_gap_threshold_arg = DeclareLaunchArgument(
        "minimum_gap_threshold",
        default_value="0.1",
        description="Minimum range for a LiDAR point to be considered part of a free gap (meters)",
    )

    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        default_value="5.5",
        description="Maximum forward speed allowed in meters per second",
    )

    min_speed_arg = DeclareLaunchArgument(
        "min_speed",
        default_value="2.0",
        description="Minimum forward speed allowed in meters per second",
    )

    alpha_arg = DeclareLaunchArgument(
        "alpha",
        default_value="0.8",
        description="Smoothing factor for steering angle",
    )

    wheelbase_arg = DeclareLaunchArgument(
        "wheelbase",
        default_value="0.33",
        description="Vehicle wheelbase in meters",
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

    max_steering_angle_arg = DeclareLaunchArgument(
        "max_steering_angle",
        default_value="0.349",
        description="Maximum steering angle in radians",
    )

    drive_topic = IfElseSubstitution(opponent_car, "/opp_drive", "/drive")
    scan_topic = IfElseSubstitution(opponent_car, "/opp_scan", "/scan")

    return LaunchDescription(
        [
            enable_safety_arg,
            opponent_car_arg,
            max_lidar_range_arg,
            car_width_arg,
            car_width_extended_arg,
            disparity_threshold_arg,
            fov_half_angle_deg_arg,
            minimum_gap_threshold_arg,
            max_speed_arg,
            min_speed_arg,
            alpha_arg,
            wheelbase_arg,
            max_lateral_acc_arg,
            max_steering_angle_arg,
            # Launch disparity_extender_node
            Node(
                package="disparity_extender",
                executable="disparity_extender_node",
                name="disparity_extender_node",
                output="screen",
                parameters=[
                    {"enable_safety": LaunchConfiguration("enable_safety")},
                    {"max_lidar_range": LaunchConfiguration("max_lidar_range")},
                    {"car_width": LaunchConfiguration("car_width")},
                    {"car_width_extended": LaunchConfiguration("car_width_extended")},
                    {"disparity_threshold": LaunchConfiguration("disparity_threshold")},
                    {"fov_half_angle_deg": LaunchConfiguration("fov_half_angle_deg")},
                    {"minimum_gap_threshold": LaunchConfiguration("minimum_gap_threshold")},
                    {"max_speed": LaunchConfiguration("max_speed")},
                    {"min_speed": LaunchConfiguration("min_speed")},
                    {"alpha": LaunchConfiguration("alpha")},
                    {"wheelbase": LaunchConfiguration("wheelbase")},
                    {"max_lateral_acc": LaunchConfiguration("max_lateral_acc")},
                    {"max_steering_angle": LaunchConfiguration("max_steering_angle")},
                ],
                remappings=[("/drive", drive_topic), ("/scan", scan_topic)],
            ),
        ]
    )
