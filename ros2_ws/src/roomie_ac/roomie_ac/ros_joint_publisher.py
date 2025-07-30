# roomie_arm_control/ros_joint_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np # numpy는 관절 각도 처리에 사용될 수 있습니다.

# DEBUG 상수는 실제 프로젝트에서는 설정 파일 등에서 관리하는 것이 좋습니다.
DEBUG = True

class ROSJointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        # /joint_states 토픽에 JointState 메시지를 발행하는 퍼블리셔를 생성합니다.
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('ROSJointPublisher 노드 초기화됨.')

    def publish(self, joint_names, joint_positions_rad):
        # JointState 메시지 객체를 생성합니다.
        msg = JointState()
        # 메시지 헤더의 타임스탬프를 현재 시간으로 설정합니다.
        msg.header.stamp = self.get_clock().now().to_msg()
        # 관절 이름을 설정합니다. (예: ['joint_1', 'joint_2', 'joint_3', 'joint_4'])
        msg.name = joint_names
        # 관절 위치를 설정합니다. (라디안 단위, 파이썬 리스트로 변환)
        msg.position = joint_positions_rad.tolist()
        
        # 메시지를 발행합니다.
        self.publisher_.publish(msg)
        
        if DEBUG:
            self.get_logger().info(f"  [ROS2 TX] -> /joint_states Published: {np.round(joint_positions_rad, 4)}")