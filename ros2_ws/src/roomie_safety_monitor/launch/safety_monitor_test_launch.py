#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Safety Monitor 테스트용 Launch 파일"""
    
    # Launch arguments
    obstacle_topic_arg = DeclareLaunchArgument(
        'obstacle_topic',
        default_value='/vs/obstacle',
        description='Topic for obstacle detection messages'
    )
    
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic',
        default_value='/goal_pose',
        description='Topic for navigation goals'
    )
    
    # Safety Monitor Node
    safety_monitor_node = Node(
        package='roomie_safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{
            'obstacle_topic': LaunchConfiguration('obstacle_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
        }],
        remappings=[
            ('/vs/obstacle', LaunchConfiguration('obstacle_topic')),
            ('/goal_pose', LaunchConfiguration('goal_topic')),
        ]
    )
    
    # Obstacle Simulator Node
    obstacle_simulator_node = Node(
        package='roomie_safety_monitor',
        executable='obstacle_simulator',
        name='obstacle_simulator',
        output='screen',
        remappings=[
            ('/vs/obstacle', LaunchConfiguration('obstacle_topic')),
        ]
    )
    
    return LaunchDescription([
        obstacle_topic_arg,
        goal_topic_arg,
        safety_monitor_node,
        obstacle_simulator_node,
    ])
