from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    deadman_button = DeclareLaunchArgument(
        "deadman_button",
        default_value="4",
        description="Which button should be considered the deadman switch",
    )

    message_timeout_duration = DeclareLaunchArgument(
        "message_timeout_duration",
        default_value="50.0",
        description="How many milliseconds the node should wait for a /joy message before it assumes that no /joy is connected",
    )

    enable_ttc = DeclareLaunchArgument(
        "enable_ttc",
        default_value="true",
        description="Whether the ttc braking is enforced by safety_node",
    )

    ttc_threshold = DeclareLaunchArgument(
        "ttc_threshold",
        default_value="0.3",
        description="Time-to-collision threshold in seconds for emergency braking",
    )

    max_ttc_angle_deg = DeclareLaunchArgument(
        "max_ttc_angle_deg",
        default_value="90.0",
        description="Maximum angle from center (degrees) considered for TTC braking",
    )

    opponent_car = LaunchConfiguration("opponent_car")

    opponent_car_arg = DeclareLaunchArgument(
        "opponent_car",
        default_value="false",
        choices=["true", "false"],
        description="If true, remap /ego_racecar/odom -> /ego_racecar/opp_odom and /scan -> /opp_scan",
    )

    odom_topic = IfElseSubstitution(
        opponent_car, "/ego_racecar/opp_odom", "/ego_racecar/odom"
    )
    scan_topic = IfElseSubstitution(opponent_car, "/opp_scan", "/scan")

    return LaunchDescription(
        [
            deadman_button,
            message_timeout_duration,
            enable_ttc,
            ttc_threshold,
            max_ttc_angle_deg,
            opponent_car_arg,
            Node(
                package="safety_node",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[
                    {"deadman_button": LaunchConfiguration("deadman_button")},
                    {
                        "message_timeout_duration": LaunchConfiguration(
                            "message_timeout_duration"
                        ),
                        "enable_ttc": LaunchConfiguration("enable_ttc"),
                        "ttc_threshold": LaunchConfiguration("ttc_threshold"),
                        "max_ttc_angle_deg": LaunchConfiguration("max_ttc_angle_deg"),
                    },
                ],
                remappings=[("/ego_racecar/odom", odom_topic), ("/scan", scan_topic)],
            ),
        ]
    )
