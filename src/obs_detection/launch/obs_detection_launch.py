from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        # obstacle detection
        ("max_obstacle_distance", "10.0", "Range cap applied to LiDAR readings in the obstacle detector (m)."),
        ("obstacle_edge_jump", "0.5", "Minimum range jump required to identify an obstacle edge (m)."),
        ("car_width_extended", "0.55", "Effective vehicle width used when searching for obstacles (m)."),
        ("fov_half_angle_deg", "135.0", "Half of the LiDAR field of view used by the detector (deg)."),
        ("obstacle_min_target_angle_deg", "0.0", "Minimum target angle used for obstacle detection (deg)."),
        ("obstacle_max_width", "0.8", "Maximum lateral obstacle width allowed (m)."),
        ("obstacle_hold_frames", "8", "Frames the obstacle state (and FTG fallback) is held after last detection."),
        # follow-the-gap fallback
        ("max_speed", "4.5", "Maximum fallback speed (m/s)."),
        ("min_speed", "0.5", "Minimum fallback speed (m/s)."),
        ("max_lidar_range", "10.0", "Maximum LiDAR range considered by follow-the-gap (m)."),
        ("ftg_fov_half_angle_deg", "70.0", "Half field-of-view used by follow-the-gap (deg)."),
        ("car_width", "0.4", "Car width used for the safety bubble and gap width check (m)."),
        ("minimum_gap_threshold", "1.0", "Minimum range for a point to count as part of a free gap (m)."),
        ("safety_bubble_scale", "0.5", "Scale factor for the safety bubble size."),
        ("prev_angle_adjuster", "0.6", "Weight of previous steering angle for smoothing."),
        ("bubble_lookahead", "3.0", "Maximum lookahead distance for the obstacle bubble (m)."),
        ("bubble_half_width", "0.45", "Half width of the obstacle bubble region (m)."),
        ("wheelbase", "0.33", "Vehicle wheelbase (m)."),
        ("lookahead_max", "1.8", "Maximum lookahead distance for target point selection (m)."),
        ("steering_deadzone", "0.7", "Steering deadzone (deg) below which steering is zeroed."),
    ]

    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in args
    ]

    return LaunchDescription(
        declarations
        + [
            Node(
                package="obs_detection",
                executable="obs_detection",
                name="obs_detection_node",
                output="screen",
                parameters=[{name: LaunchConfiguration(name) for name, _, _ in args}],
            ),
        ]
    )
