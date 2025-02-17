from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic_gps',
            default_value='/gps'
        ),
        DeclareLaunchArgument(
            'input_topic_georef_metadata',
            default_value='/lidar_odometry/geo_ref_metadata'
        ),
        DeclareLaunchArgument(
            'output_topic_marker',
            default_value='/gnss_georef_marker'
        ),
        DeclareLaunchArgument(
            'output_marker_line_width',
            default_value='0.3'
        ),
        DeclareLaunchArgument(
            'output_marker_color',
            default_value='[0.0, 1.0, 0.0, 0.6]'
        ),
        Node(
            package='mola_gnss_to_markers',
            executable='mola_gnss_to_marker_node',
            name='mola_gnss_to_marker_node',
            output='screen',
            parameters=[
                {'input_topic_gps': LaunchConfiguration('input_topic_gps')},
                {'input_topic_georef_metadata': LaunchConfiguration('input_topic_georef_metadata')},
                {'output_topic_marker': LaunchConfiguration('output_topic_marker')},
                {'output_marker_line_width': LaunchConfiguration('output_marker_line_width')},
                {'output_marker_color': LaunchConfiguration('output_marker_color')}
            ],
        ),
    ])
