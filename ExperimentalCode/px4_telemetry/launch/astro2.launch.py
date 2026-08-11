import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='px4_telemetry',
            executable='px4_telemetry_node',
            name='px4_telemetry_node',
            namespace='astro2',
            parameters=[os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml')],
            output='screen'
        )
    ])
