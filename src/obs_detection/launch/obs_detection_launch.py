from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    obstacle_distance_arg = DeclareLaunchArgument(
        "obstacle_distance",
        default_value="2.0",
        description="Maximum distance at which an obstacle is considered ahead.",
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

    max_lidar_range_arg = DeclareLaunchArgument(
        "max_lidar_range",
        default_value="10.0",
        description="Maximum LiDAR range considered by the detector.",
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

    # 1. Obstacle Detection Node
    obs_detection_node = Node(
        package="obs_detection",
        executable="obs_detection",
        name="obs_detection_node",
        output="screen",
        parameters=[{
            "obstacle_distance": LaunchConfiguration("obstacle_distance"),
            "obstacle_edge_jump": LaunchConfiguration("obstacle_edge_jump"),
            "car_width_extended": LaunchConfiguration("car_width_extended"),
            "max_lidar_range": LaunchConfiguration("max_lidar_range"),
            "fov_half_angle_deg": LaunchConfiguration("fov_half_angle_deg"),
            "obstacle_min_target_angle_deg": LaunchConfiguration(
                "obstacle_min_target_angle_deg"
            ),
            "obstacle_max_width": LaunchConfiguration("obstacle_max_width"),
        }],
    )

    # 2. Fallback Control Multiplexer Node (Arbitrates between PP and FTG)
    fallback_ctrl_node = Node(
        package="obs_detection",
        executable="fallback_ctrl",
        name="fallback_ctrl_node",
        output="screen",
    )

    # 3. Pure Pursuit Node (Make sure package/executable names match your workspace)
    pure_pursuit_node = Node(
        package="pure_pursuit_a",
        executable="pure_pursuit_a_node", # Adjust if your executable name differs
        name="pure_pursuit_node",
        output="screen",
        # Remap its output so it feeds into our fallback controller's sandbox topic
        remappings=[
            ('/drive', '/pure_pursuit/drive')
        ]
    )

    # 4. Follow The Gap / Reactive Node
    ftg_node = Node(
        package="ftg", # Adjust package name if different
        executable="ftg_node", # Adjust executable name if different
        name="ftg_node",
        output="screen",
        # Remap its output so it feeds into our fallback controller's sandbox topic
        remappings=[
            ('/drive', '/ftg/drive')
        ]
    )

    return LaunchDescription([
        obstacle_distance_arg,
        obstacle_edge_jump_arg,
        car_width_extended_arg,
        max_lidar_range_arg,
        fov_half_angle_deg_arg,
        obstacle_min_target_angle_deg_arg,
        obstacle_max_width_arg,
        obs_detection_node,
        fallback_ctrl_node,
        pure_pursuit_node,
        ftg_node,
    ])