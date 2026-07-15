from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch gap_finder_node
        Node(
            package='reactive',
            executable='gap_finder_node',
            name='gap_finder_node',
            output='screen',
        ),

        # Launch gap_follower_node
        Node(
            package='reactive',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
        )
    ])
