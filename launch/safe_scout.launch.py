from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    ])
