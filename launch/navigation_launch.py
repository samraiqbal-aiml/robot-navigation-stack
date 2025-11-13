from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_bot',
            executable='simple_navigator',
            name='simple_navigator',
            output='screen',
            parameters=[{
                'linear_speed': 0.2,
                'angular_speed': 0.5,
                'goal_tolerance': 0.3
            }]
        ),
        Node(
            package='navigation_bot',
            executable='path_planner',
            name='path_planner',
            output='screen'
        )
    ])
