import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():   
    return LaunchDescription([
        Node(
            package='px4_telemetry',
            executable='px4_telemetry_node',
            name='px4_telemetry_node1',
            namespace='agent_1',
            parameters=[
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'button_config.yaml'),
                {'sim_mode': True},
            ],
            output='screen'
        ),
        Node(
            package='px4_teleop',
            executable='px4_teleop_node',
            name='px4_teleop_node1',
            namespace='agent_1',
            parameters=[
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'teleop_config.yaml'),
                os.path.join(get_package_share_directory('px4_safety_lib'), 'param', 'safety_config.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'sim_obstacles.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'button_config.yaml'),
                {"agent_ids": ['agent_2', 'agent_3']}
            ],
            output='screen'
        ),
        Node(
            package='px4_telemetry',
            executable='px4_telemetry_node',
            name='px4_telemetry_node2',
            namespace='agent_2',
            parameters=[
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'button_config.yaml'),
                {'sim_mode': True},
            ],
            output='screen'
        ),
        Node(
            package='px4_teleop',
            executable='px4_teleop_node',
            name='px4_teleop_node2',
            namespace='agent_2',
            parameters=[
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'teleop_config.yaml'),
                os.path.join(get_package_share_directory('px4_safety_lib'), 'param', 'safety_config.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'sim_obstacles.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'button_config.yaml'),
                {"agent_ids": ['agent_1', 'agent_3']}

            ],
            output='screen'
        ),
        Node(
            package='px4_telemetry',
            executable='px4_telemetry_node',
            name='px4_telemetry_node3',
            namespace='agent_3',
            parameters=[
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'button_config.yaml'),
                {'sim_mode': True},
            ],
            output='screen'
        ),
        Node(
            package='px4_teleop',
            executable='px4_teleop_node',
            name='px4_teleop_node3',
            namespace='agent_3',
            parameters=[
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'teleop_config.yaml'),
                os.path.join(get_package_share_directory('px4_safety_lib'), 'param', 'safety_config.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'sim_obstacles.yaml'),
                os.path.join(get_package_share_directory('px4_telemetry'), 'param', 'park_coordinates.yaml'),
                os.path.join(get_package_share_directory('px4_teleop'), 'param', 'button_config.yaml'),
                {"agent_ids": ['agent_1', 'agent_2']}

            ],
            output='screen'
        ),
        Node(package='fleet_manager',
             executable='fleet_manager_node',
             name='fleet_manager_node',
             namespace='fleet_manager',
             parameters=[os.path.join(get_package_share_directory('fleet_manager'), 'params', 'experiment_params.yaml')],
             output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='multi_agent',
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
        )
    ])
