from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 테스트 통신 노드 (모의 VS + GUI 이벤트 수신)
        Node(
            package='roomie_ec',
            executable='test_communication_node',
            name='test_communication_node',
            output='screen',
        ),
        
        # 토픽 모니터링 노드
        Node(
            package='roomie_ec',
            executable='topic_monitor_node',
            name='topic_monitor_node',
            output='screen',
        ),
        
        # 실제 엘리베이터 컨트롤러 노드
        Node(
            package='roomie_ec',
            executable='roomie_ec_node',
            name='roomie_ec_node',
            output='screen',
        ),
    ]) 