from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode', default_value='true', description='디버그 모드 여부'
    )
    start_state_arg = DeclareLaunchArgument(
        'start_state', default_value='waiting_dest_input', description='시작 상태'
    )

    gc_node = Node(
        package='roomie_gc',
        executable='roomie_gc_node',
        name='roomie_gc_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode'),
            'start_state': LaunchConfiguration('start_state'),
        }]
    )

    return LaunchDescription([
        debug_mode_arg,
        start_state_arg,
        gc_node,
    ])



