# roomie_arm_control/vision_service_client.py

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from roomie_msgs.srv import ButtonStatus
from . import config
from .config import ROBOT_ID

class VisionServiceClient(Node):
    """
    Vision Service(VS)와 통신하여 버튼의 위치 및 상태 정보를 요청하는 ROS 2 서비스 클라이언트.
    """
    def __init__(self):
        super().__init__('vision_service_client')
        
        # QoS 설정 (서비스 호출의 안정성을 위해 설정 권장)
        #qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, # 신뢰성 있는 전송 보장history=HistoryPolicy.KEEP_LAST,depth=1,
        #    durability=DurabilityPolicy.VOLATILE)

        # /vs/command/button_status 서비스 클라이언트 생성
        self.cli = self.create_client(ButtonStatus, '/vs/command/button_status') # 원본 주석 처리
        #self.cli = self.create_client(ButtonStatus2, '/vs/command/button_status2') # 새로운 서비스로 생성
        self.get_logger().info('VisionServiceClient 노드 초기화됨. VS 서비스 대기 중...')

        # 서비스가 사용 가능해질 때까지 기다립니다.
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('VS 서비스가 사용 가능해질 때까지 기다리는 중...')
        self.get_logger().info('VS 서비스가 사용 가능합니다.')

    async def request_button_status(self, robot_id: int, button_id: int):
        """
        Vision Service에 특정 버튼의 상태 정보를 요청합니다.
        """
        if robot_id != ROBOT_ID:
            self.get_logger().warn(f"요청된 robot_id({robot_id})가 현재 로봇 ID({ROBOT_ID})와 일치하지 않아 VS 요청을 무시합니다.")
            return None
        if config.DEBUG:
            self.get_logger().info(f"VS에 버튼 상태 요청 중: 로봇 ID={robot_id}, 버튼 ID={button_id}")

        request = ButtonStatus.Request()
        request.robot_id = robot_id
        request.button_id = button_id

        try:
            future = self.cli.call_async(request)
            response = await future
            
            if response is not None:
                if config.DEBUG:
                    self.get_logger().info(f"VS로부터 응답 수신: success={response.success}")
                    if response.success:
                        # ✨ [핵심 수정] 로그에서 points 필드 대신 x, y, size를 사용합니다.
                        self.get_logger().info(
                            f"  버튼 {button_id}: x={response.x:.3f}, y={response.y:.3f}, size={response.size:.3f}, pressed={response.is_pressed}"
                        )
                    else:
                        self.get_logger().warn("VS 요청이 실패했습니다 (success=False).")
                return response

        except Exception as e:
            self.get_logger().error(f'VS 서비스 호출 예외 발생: {e}')
            return None