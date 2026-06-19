import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('safe_bayesian_optimization'),
            'config',
            'safe_bayesian_optimization.yaml'
        ]),
        description='Path to the configuration file'
    )

    reactive_planner_config_arg = DeclareLaunchArgument(
        'reactive_planner_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('safe_bayesian_optimization'),
            'config',
            'reactive_planner.yaml'
        ]),
        description='Path to the reactive planner configuration file'
    )

    yaml_dir = get_package_share_directory('safe_bayesian_optimization')
    config_file = os.path.join(yaml_dir, 'config/lpsc.yaml')

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    visualizer_params = config.get('visualizer', {}).get('ros__parameters', {})
    mapping_params = config.get('mapping_node', {}).get('ros__parameters', {})
    data_collector_params = config.get(
        'data_collector', {}).get('ros__parameters', {})

    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        ])
    )

    return LaunchDescription([
        config_file_arg,
        reactive_planner_config_arg,
        Node(
            package='safe_bayesian_optimization',
            executable='safe_bayesian_optimization_node',
            name='safe_bayesian_optimization_node',
            parameters=[
                LaunchConfiguration('config_file'),
                LaunchConfiguration('reactive_planner_config'),
            ],
            output='screen'
        ),
        Node(
            package='safe_bayesian_optimization',
            executable='goal_point_publisher',
            name='goal_point_publisher',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),
        Node(
            package='safe_bayesian_optimization',
            executable='reactive_navigation_node',
            name='reactive_navigation_node',
            parameters=[LaunchConfiguration('reactive_planner_config')],
            output='screen'
        ),
        Node(
            package='foxglove_visualization',
            executable='visualizer',
            name='visualizer',
            output='screen',
            parameters=[visualizer_params]
        ),
        Node(
            package='foxglove_visualization',
            executable='leg_measurements_publisher',
            name='leg_measurements_publisher',
            output='screen'
        ),
        Node(
            package='foxglove_visualization',
            executable='fake_data_publisher',
            name='fake_data_publisher',
            output='screen'
        ),
        Node(
            package='mapping_collector',
            executable='data_collector',
            name='data_collector',
            output='screen',
            parameters=[data_collector_params]
        ),
        Node(
            package='mapping_package',
            executable='terrain_mapping_node',
            name='mapping_node',
            output='screen',
            parameters=[mapping_params]
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
            parameters=[{
                'background_r': 255,
                'background_g': 255,
                'background_b': 255,
            }]
        ),
        Node(
            package='safe_bayesian_optimization',
            executable='turtlesim_spatial_publisher.py',
            name='spirit_spatial_publisher',
            output='screen'
        ),
        foxglove_bridge_launch,
    ])
