# roomie_ac/test/dummy_vision_service.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from roomie_msgs.srv import ButtonStatus # 올바른 메시지 임포트
from builtin_interfaces.msg import Time
import time

class DummyVisionService(Node):
    """
    Vision Service의 더미 구현.
    VisionServiceClient가 올바르게 작동하는지 테스트하기 위한 용도.
    """
    def __init__(self):
        super().__init__('dummy_vision_service')
        
        # QoS 설정 (클라이언트의 QoS와 일치시키는 것이 중요)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # /vs/command/button_status 서비스 서버 생성
        self.srv = self.create_service(ButtonStatus, '/vs/command/button_status', self.handle_button_status, qos_profile=qos_profile)
        self.get_logger().info('DummyVisionService 준비 완료: 서비스 [/vs/command/button_status] 제공 중.')

    def handle_button_status(self, request, response):
        """
        ButtonStatus 서비스 요청을 처리하고 더미 응답을 반환합니다.
        """
        self.get_logger().info(f'더미 VS 서버: 요청 수신 - 로봇 ID: {request.robot_id}, 버튼 ID: {request.button_ids}')

        response.robot_id = request.robot_id
        response.success = True # 항상 성공으로 응답

        # 요청된 button_ids에 맞춰 더미 데이터 생성
        response.xs = []
        response.ys = []
        response.depths = []
        response.is_pressed = []
        response.timestamp = []

        current_time_msg = self.get_clock().now().to_msg()

        for btn_id in request.button_ids:
            # 실제 시나리오를 반영하는 더미 데이터 생성 (예시)
            if btn_id == 1: # 1층 버튼
                response.xs.append(0.1)
                response.ys.append(0.05)
                response.depths.append(0.3)
                response.is_pressed.append(False)
            elif btn_id == 102: # 열기 버튼
                response.xs.append(0.0)
                response.ys.append(0.0)
                response.depths.append(0.25)
                response.is_pressed.append(False)
            else: # 그 외 버튼
                response.xs.append(0.0)
                response.ys.append(0.0)
                response.depths.append(0.0)
                response.is_pressed.append(False) # 기본값
            response.timestamp.append(current_time_msg)

        self.get_logger().info(f'더미 VS 서버: 응답 전송 - Success: {response.success}, Xs: {response.xs}, Depths: {response.depths}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DummyVisionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()