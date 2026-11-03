from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="Odometry topic (pose source in sim, wheel odom for speed when use_pf is true)",
    )

    use_pf_arg = DeclareLaunchArgument(
        "use_pf",
        default_value="true",
        description="true = pose from particle filter (irl), false = pose from odom_topic (sim ground truth)",
    )

    enable_safety_arg = DeclareLaunchArgument(
        "enable_safety",
        default_value="true",
        description="Whether the safety brake from the safety node is enabled",
    )

    track_arg = DeclareLaunchArgument(
        "track",
        default_value="levine",
        description="Name of the waypoint CSV (without .csv) in the waypoints folder",
    )

    lookahead_base_arg = DeclareLaunchArgument(
        "lookahead_base",
        default_value="1.0",
        description="Base lookahead distance in meters",
    )

    lookahead_min_arg = DeclareLaunchArgument(
        "lookahead_min",
        default_value="0.8",
        description="Minimum lookahead distance in meters",
    )

    lookahead_max_arg = DeclareLaunchArgument(
        "lookahead_max",
        default_value="3.0",
        description="Maximum lookahead distance in meters",
    )

    lookahead_speed_gain_arg = DeclareLaunchArgument(
        "lookahead_speed_gain",
        default_value="0.1",
        description="Speed gain applied to lookahead distance",
    )

    wheelbase_arg = DeclareLaunchArgument(
        "wheelbase",
        default_value="0.33",
        description="Vehicle wheelbase in meters",
    )

    max_steering_arg = DeclareLaunchArgument(
        "max_steering",
        default_value="0.4189",
        description="Maximum steering angle in radians",
    )

    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        default_value="4.0",
        description="Maximum forward speed in m/s",
    )

    min_speed_arg = DeclareLaunchArgument(
        "min_speed",
        default_value="1.5",
        description="Minimum forward speed in m/s",
    )

    return LaunchDescription(
        [
            odom_topic_arg,
            use_pf_arg,
            enable_safety_arg,
            track_arg,
            lookahead_base_arg,
            lookahead_min_arg,
            lookahead_max_arg,
            lookahead_speed_gain_arg,
            wheelbase_arg,
            max_steering_arg,
            max_speed_arg,
            min_speed_arg,
            # Launch pure pursuit
            Node(
                package="pure_pursuit_a",
                executable="pure_pursuit_a_node",
                name="pure_pursuit_a_node",
                output="screen",
                parameters=[
                    {
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "use_pf": LaunchConfiguration("use_pf"),
                        "enable_safety": LaunchConfiguration("enable_safety"),
                        "track": LaunchConfiguration("track"),
                        "lookahead_base": LaunchConfiguration("lookahead_base"),
                        "lookahead_min": LaunchConfiguration("lookahead_min"),
                        "lookahead_max": LaunchConfiguration("lookahead_max"),
                        "lookahead_speed_gain": LaunchConfiguration(
                            "lookahead_speed_gain"
                        ),
                        "wheelbase": LaunchConfiguration("wheelbase"),
                        "max_steering": LaunchConfiguration("max_steering"),
                        "max_speed": LaunchConfiguration("max_speed"),
                        "min_speed": LaunchConfiguration("min_speed"),
                    }
                ],
            ),
        ]
    )
