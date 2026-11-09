from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    max_obstacle_distance_arg = DeclareLaunchArgument(
        "max_obstacle_distance",
        default_value="10.0",
        description="Maximum distance considered by the detector.",
    )

    obstacle_edge_jump_arg = DeclareLaunchArgument(
        "obstacle_edge_jump",
        default_value="0.5",
        description="Minimum range jump required to identify an obstacle edge.",
    )

    car_width_extended_arg = DeclareLaunchArgument(
        "car_width_extended",
        default_value="0.55",
        description="Effective vehicle width used when searching for obstacles.",
    )

    fov_half_angle_deg_arg = DeclareLaunchArgument(
        "fov_half_angle_deg",
        default_value="135.0",
        description="Half of the LiDAR field of view considered, in degrees.",
    )

    obstacle_min_target_angle_deg_arg = DeclareLaunchArgument(
        "obstacle_min_target_angle_deg",
        default_value="0.0",
        description="Minimum target angle used for obstacle detection, in degrees.",
    )

    obstacle_max_width_arg = DeclareLaunchArgument(
        "obstacle_max_width",
        default_value="0.8",
        description="Maximum lateral obstacle width allowed, in meters.",
    )

    obstacle_hold_frames_arg = DeclareLaunchArgument(
        "obstacle_hold_frames",
        default_value="8",
        description="Number of frames to hold obstacle state for hysteresis.",
    )

    # 1. Obstacle Detection Node
    obs_detection_node = Node(
        package="obs_detection",
        executable="obs_detection",
        name="obs_detection_node",
        output="screen",
        parameters=[{
            "max_obstacle_distance": LaunchConfiguration("max_obstacle_distance"),
            "obstacle_edge_jump": LaunchConfiguration("obstacle_edge_jump"),
            "car_width_extended": LaunchConfiguration("car_width_extended"),
            "fov_half_angle_deg": LaunchConfiguration("fov_half_angle_deg"),
            "obstacle_min_target_angle_deg": LaunchConfiguration("obstacle_min_target_angle_deg"),
            "obstacle_max_width": LaunchConfiguration("obstacle_max_width"),
            "obstacle_hold_frames": LaunchConfiguration("obstacle_hold_frames"),
        }],
    )

    # 2. Fallback Control Multiplexer Node
    fallback_ctrl_node = Node(
        package="obs_detection",
        executable="fallback_ctrl",
        name="fallback_ctrl_node",
        output="screen",
    )

    return LaunchDescription([
        max_obstacle_distance_arg,
        obstacle_edge_jump_arg,
        car_width_extended_arg,
        fov_half_angle_deg_arg,
        obstacle_min_target_angle_deg_arg,
        obstacle_max_width_arg,
        obstacle_hold_frames_arg,
        obs_detection_node,
        fallback_ctrl_node,
    ])