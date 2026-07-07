from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch follow the gap
        Node(
            package='ftg_node',
            executable='ftg_node',
            name='ftg_node',
            output='screen',
            parameters=[]
        ),
    ])