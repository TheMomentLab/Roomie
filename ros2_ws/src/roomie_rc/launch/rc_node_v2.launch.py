from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments 선언
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='디버그 모드 활성화 여부'
    )
    
    start_state_arg = DeclareLaunchArgument(
        'start_state',
        default_value='idle',
        description='시작 상태 설정'
    )
    
    start_robot_state_arg = DeclareLaunchArgument(
        'start_robot_state',
        default_value='2',
        description='시작 로봇 상태 설정'
    )

    # Node 설정
    rc_node = Node(
        package='roomie_rc',
        executable='rc_node_v2',
        name='rc_node_v2',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode'),
            'start_state': LaunchConfiguration('start_state'),
            'start_robot_state': LaunchConfiguration('start_robot_state')
        }]
    )

    return LaunchDescription([
        debug_mode_arg,
        start_state_arg,
        start_robot_state_arg,
        rc_node
    ])