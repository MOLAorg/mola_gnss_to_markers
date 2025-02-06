from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mola_gnss_to_markers',
            executable='mola_gnss_to_marker_node',
            name='mola_gnss_to_marker_node',
            output='screen',
        ),
    ])
