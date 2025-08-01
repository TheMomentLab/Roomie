# roomie_arm_control/vision_service_client.py

import rclpy
from rclpy.node import Node
from rclpy.client import SrvClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from roomie_msgs.srv import ButtonStatus # ButtonStatus.srv 파일이 정의된 패키지

from . import config # config.py 임포트

class VisionServiceClient(Node):
    """
    Vision Service(VS)와 통신하여 버튼의 위치 및 상태 정보를 요청하는 ROS 2 서비스 클라이언트.
    """
    def __init__(self):
        super().__init__('vision_service_client')
        
        # QoS 설정 (서비스 호출의 안정성을 위해 설정 권장)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # 신뢰성 있는 전송 보장
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # /vs/command/button_status 서비스 클라이언트 생성
        self.cli = self.create_client(ButtonStatus, '/vs/command/button_status', qos_profile=qos_profile)
        self.get_logger().info('VisionServiceClient 노드 초기화됨. VS 서비스 대기 중...')

        # 서비스가 사용 가능해질 때까지 기다립니다.
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('VS 서비스가 사용 가능해질 때까지 기다리는 중...')
        self.get_logger().info('VS 서비스가 사용 가능합니다.')

    async def request_button_status(self, robot_id, button_ids):
        """
        Vision Service에 특정 버튼의 상태 정보를 요청합니다.
        robot_id: 요청을 보내는 로봇의 ID
        button_ids: 상태를 요청할 버튼 ID들의 리스트 (예: [102, 103] for 열기/닫기)
        
        반환: ButtonStatus.Response 객체 또는 오류 시 None
        """
        if config.DEBUG: # config.DEBUG 사용
            self.get_logger().info(f"VS에 버튼 상태 요청 중: 로봇 ID={robot_id}, 버튼 ID={button_ids}")

        request = ButtonStatus.Request()
        request.robot_id = robot_id
        request.button_ids = button_ids

        try:
            # 비동기적으로 서비스 호출
            future = self.cli.call_async(request)
            # future가 완료될 때까지 기다림
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                if config.DEBUG: # config.DEBUG 사용
                    self.get_logger().info(f"VS로부터 응답 수신: success={response.success}")
                    if response.success:
                        for i, btn_id in enumerate(button_ids):
                            self.get_logger().info(f"  버튼 {btn_id}: xs={response.xs[i]:.2f}, ys={response.ys[i]:.2f}, sizes={response.depths[i]:.2f}, pressed={response.is_pressed[i]}")
                    else:
                        self.get_logger().warn("VS 요청이 실패했습니다.")
                return response
            else:
                self.get_logger().error('VS 서비스 호출 실패: 응답 없음')
                return None
        except Exception as e:
            self.get_logger().error(f'VS 서비스 호출 예외 발생: {e}')
            return None