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
        self.get_logger().info('VisionServiceClient 노드 초기화됨. VS 서비스 대기 중...')

        # 서비스가 준비될 때까지 기다리는 로직은 그대로 유지합니다.
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('VS 서비스가 사용 가능해질 때까지 기다리는 중...')
        self.get_logger().info('VS 서비스가 사용 가능합니다.')

    async def request_button_status(self, robot_id: int, button_id: int):
        """
        [최종 수정] Vision Service에 특정 버튼의 상태 정보를 비동기(asynchronous) 방식으로 요청합니다.
        """
        if robot_id != config.ROBOT_ID:
            self.get_logger().warn(f"요청된 robot_id({robot_id})가 현재 로봇 ID({config.ROBOT_ID})와 일치하지 않아 VS 요청을 무시합니다.")
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