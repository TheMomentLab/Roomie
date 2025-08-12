#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roomie_safety_monitor',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
            parameters=[{
                # 필요한 파라미터들을 여기에 추가
            }]
        )
    ])
