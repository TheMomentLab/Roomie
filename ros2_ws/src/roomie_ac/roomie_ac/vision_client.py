# roomie_arm_control/vision_service_client.py

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup 
from roomie_msgs.srv import ButtonStatus
import asyncio
from . import config

class VisionServiceClient(Node):
    """
    Vision Service(VS)와 통신하여 버튼의 위치 및 상태 정보를 요청하는 ROS 2 서비스 클라이언트.
    """
    def __init__(self, callback_group: ReentrantCallbackGroup):
        super().__init__('vision_service_client_node')
        self.cli = self.create_client(ButtonStatus, '/vs/command/button_status', callback_group=callback_group)
        # [수정] 시작 시 서비스 연결을 기다리지 않습니다.
        self.get_logger().info('VisionServiceClient 노드 초기화됨.')

    async def request_button_status(self, robot_id: int, button_id: int):
        # [수정] 서비스 요청 직전에 서비스가 사용 가능한지 확인합니다.
        if not self.cli.service_is_ready():
            self.get_logger().error('Vision Service가 사용 불가능합니다. 서비스 노드가 실행 중인지 확인하세요.')
            return None


        request = ButtonStatus.Request()
        request.robot_id = robot_id
        request.button_id = button_id
        
        if config.DEBUG:
            self.get_logger().info(f"VS에 버튼 상태 요청 중 (non-blocking): {request}")

        future = self.cli.call_async(request)
        
        try:
            # rclpy.spin_until_future_complete 대신 await로 future가 완료되기를 기다립니다.
            response = await asyncio.wait_for(future, timeout=10.0)
            if config.DEBUG:
                self.get_logger().info(f"✅✅VS로부터 응답 수신: {response}✅✅")
            return response
        except asyncio.TimeoutError:
            self.get_logger().error('VS 서비스 응답 시간 초과 (10초). Vision Service 노드가 멈췄거나 응답을 못 보내는지 확인하세요.')
            return None
        except Exception as e:
            self.get_logger().error(f'서비스 결과 처리 중 예외 발생: {e}')
            return None