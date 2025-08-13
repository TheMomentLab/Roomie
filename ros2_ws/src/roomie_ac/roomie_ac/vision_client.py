
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import asyncio
from . import config



# [수정] ButtonStatus 외에 새로 만든 ButtonStatus2도 임포트합니다.
from roomie_msgs.srv import ButtonStatus, ButtonStatus2

class VisionServiceClient(Node):
    """
    [수정됨] 두 가지 Vision Service('normal', 'corner')와 통신하는 클라이언트.
    config 설정에 따라 적절한 서비스를 호출합니다.
    """
    def __init__(self, callback_group: ReentrantCallbackGroup):
        super().__init__('vision_service_client_node')

        # [수정] 두 종류의 서비스 클라이언트를 각각 생성합니다.
        self.normal_mode_client = self.create_client(
            ButtonStatus,
            '/vs/command/button_status',  # 일반 모드 서비스 이름
            callback_group=callback_group
        )
        self.corner_mode_client = self.create_client(
            ButtonStatus2,
            '/vs/command/button_status2', # 모서리 모드 서비스 이름
            callback_group=callback_group
        )
        self.get_logger().info('VisionServiceClient 노드 초기화됨 (일반/모서리 모드 클라이언트 생성).')

    # [수정] 요청 메서드를 하나로 통합하고, mode에 따라 분기합니다.
    async def request_status(self, mode: str, robot_id: int, button_id: int):
        """
        설정된 모드에 따라 적절한 Vision Service를 호출합니다.
        """
        client = None
        request = None

        if mode == 'corner':
            client = self.corner_mode_client
            request = ButtonStatus2.Request()
            service_name = client.srv_name
        elif mode == 'normal':
            client = self.normal_mode_client
            request = ButtonStatus.Request()
            service_name = client.srv_name
        else:
            self.get_logger().error(f"지원하지 않는 모드입니다: {mode}")
            return None

        # 서비스가 준비되었는지 확인
        if not client.service_is_ready():
            self.get_logger().error(f"Vision Service '{service_name}'가 사용 불가능합니다.")
            return None

        # 요청 메시지 채우기
        request.robot_id = robot_id
        request.button_id = button_id

        if config.DEBUG:
            self.get_logger().info(f"VS에 '{mode}' 모드로 상태 요청: {request}")

        future = client.call_async(request)

        try:
            response = await asyncio.wait_for(future, timeout=10.0)
            if config.DEBUG:
                self.get_logger().info(f"✅ VS로부터 응답 수신: {response}")
            return response
        except asyncio.TimeoutError:
            self.get_logger().error(f"'{service_name}' 서비스 응답 시간 초과 (10초).")
            return None
        except Exception as e:
            self.get_logger().error(f"서비스 결과 처리 중 예외 발생: {e}")
            return None