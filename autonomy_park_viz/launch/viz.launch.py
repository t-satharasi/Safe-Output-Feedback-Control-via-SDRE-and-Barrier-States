import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='autonomy_park_viz',
            executable='autonomy_park_viz_node',
            name='autonomy_park_viz_node',
            namespace='autonomy_park_viz',
            parameters=[os.path.join(get_package_share_directory('autonomy_park_viz'), 'param', 'park_geometry.yaml')],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('autonomy_park_viz'), 'rviz', 'autonomy_park.rviz')]
        )
    ])
