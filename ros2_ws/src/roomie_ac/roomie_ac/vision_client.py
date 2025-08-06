# roomie_arm_control/vision_service_client.py

import rclpy
from rclpy.node import Node
from roomie_msgs.srv import ButtonStatus
from . import config
from .config import ROBOT_ID
from rclpy.callback_groups import ReentrantCallbackGroup 

class VisionServiceClient(Node):
    """
    Vision Service(VS)와 통신하여 버튼의 위치 및 상태 정보를 요청하는 ROS 2 서비스 클라이언트.
    """
    def __init__(self, callback_group: ReentrantCallbackGroup):
        super().__init__('vision_service_client_node')

        self.cli = self.create_client(ButtonStatus, '/vs/command/button_status', callback_group=callback_group)
        self.get_logger().info('VisionServiceClient 노드 초기화됨. VS 서비스 대기 중...')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('VS 서비스가 사용 가능해질 때까지 기다리는 중...')
        self.get_logger().info('VS 서비스가 사용 가능합니다.')

    def request_button_status(self, robot_id: int, button_id: int):
        """
        [수정됨] Vision Service에 버튼 상태 정보를 요청하고, 결과에 따라 선별적으로 로그를 출력합니다.
        """
        if robot_id != ROBOT_ID:
            self.get_logger().warn(f"요청된 robot_id({robot_id})가 현재 로봇 ID({ROBOT_ID})와 일치하지 않아 VS 요청을 무시합니다.")
            return None

        request = ButtonStatus.Request()
        request.robot_id = robot_id
        request.button_id = button_id

        # 요청 로그는 그대로 유지하여 어떤 요청이 보내졌는지 알 수 있게 합니다.
        if config.DEBUG:
            self.get_logger().info(f"VS에 버튼 상태 요청 중 (blocking): {request}")

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            try:
                response = future.result()
                # ======================= [핵심 수정 시작] =======================
                if response.success:
                    # '진짜' 성공했을 때만 상세 로그 출력
                    if config.DEBUG:
                        self.get_logger().info(f"✅✅ VS로부터 유효한 버튼 정보 수신: {response} ✅✅")
                else:
                    # 서비스는 성공했지만, 버튼을 찾지 못했을 경우 간결한 경고 출력
                    if config.DEBUG:
                        self.get_logger().warn(f"🟡 VS 응답: {response}성공. 하지만 요청한 버튼(id:{button_id})을 찾지 못했습니다.")
                # ======================== [핵심 수정 끝] ========================
                return response
            except Exception as e:
                self.get_logger().error(f'서비스 결과 처리 중 예외 발생: {e}')
                return None
        else:
            self.get_logger().error('VS 서비스 응답 시간 초과 (10초). Vision Service 노드가 멈췄거나 응답을 못 보내는지 확인하세요.')
            return None