from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roomie_ac',
            executable='ac_node',
            name='ac_node',
            output='screen'
        ),
        Node(
            package='roomie_ac',
            executable='yolo_vision_service',
            name='yolo_vision_service',
            output='screen'
        )
    ])