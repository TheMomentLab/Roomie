from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    # ===== Launch Arguments =====
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Debug mode for testing (true: simulation, false: real)'
    )
    
    start_state_arg = DeclareLaunchArgument(
        'start_state',
        default_value='INIT',
        description='Starting state for scenario testing'
    )
    
    return LaunchDescription([
        # Launch arguments
        debug_mode_arg,
        start_state_arg,
        
        # 테스트 통신 노드 (모의 VS) - debug_mode가 true일 때만 실행
        Node(
            package='roomie_ec',
            executable='test_communication_node',
            name='test_communication_node',
            output='screen',
            condition=LaunchConfigurationEquals('debug_mode', 'true'),
            parameters=[{
                'start_state': LaunchConfiguration('start_state')
            }]
        ),
        
        # Simple Navigator 2 (후진 PID 제어) - 실제 모드일 때만 실행
        ExecuteProcess(
            cmd=['ros2', 'run', 'roomie_simple_navigator', 'simple_navigator2'],
            output='screen',
            name='simple_navigator_reverse',
            condition=LaunchConfigurationEquals('debug_mode', 'false'),
        ),
        
        # 실제 엘리베이터 컨트롤러 노드
        Node(
            package='roomie_ec',
            executable='roomie_ec_node',
            name='roomie_ec_node',
            output='screen',
            parameters=[{
                'debug_mode': LaunchConfiguration('debug_mode'),
                'start_state': LaunchConfiguration('start_state')
            }]
        ),
    ]) 