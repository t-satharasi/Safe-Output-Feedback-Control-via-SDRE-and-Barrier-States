import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='astro_sim',
            parameters=[os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'joy_config.yaml')],
            output='screen'
        ),
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
        ),
        Node(
            package='px4_telemetry',
            executable='px4_telemetry_node',
            name='px4_telemetry_node',
            namespace='astro_sim',
            parameters=[
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'button_config.yaml'),
                {'sim_mode': True}
            ],
            output='screen'
        ),
        Node(
            package='px4_teleop',
            executable='px4_teleop_node',
            name='px4_teleop_node',
            namespace='astro_sim',
            parameters=[
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'teleop_config.yaml'),
                os.path.join(get_package_share_directory('px4_safety_lib'), 'param', 'safety_config.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'sim_obstacles.yaml')
            ],
            output='screen'
        ),
        Node(
            package='virtual_obstacle',
            executable='virtual_obstacle_node',
            name='virtual_obstacle_node',
            parameters=[
                os.path.join(get_package_share_directory('virtual_obstacle'), 'param', 'obstacles.yaml')
            ],
            output='screen'
        )
    ])
