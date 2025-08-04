from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 메인 엘리베이터 컨트롤러 노드만 실행
        Node(
            package='roomie_ec',
            executable='roomie_ec_node',
            name='roomie_ec_node',
            output='screen',
        ),
    ]) 