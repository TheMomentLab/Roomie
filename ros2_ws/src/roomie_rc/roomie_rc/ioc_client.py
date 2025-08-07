#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# ROS2 메시지 import
from roomie_msgs.srv import ControlLock
from roomie_msgs.srv import CheckDoorState, CheckItemLoaded


class IOCClient:
    """
    IOC(IO Controller)와의 통신을 담당하는 클라이언트
    """
    
    def __init__(self, parent_node: Node):
        self.node = parent_node
        
        # IO Controller 클라이언트 초기화
        self._init_ioc_clients()
        
        self.node.get_logger().info('IOC Client 초기화 완료')
    
    def _init_ioc_clients(self):
        """IO Controller 클라이언트 초기화"""
        # 서랍 잠금 제어 Service Client (RC → IOC)
        self.control_lock_client = self.node.create_client(
            ControlLock,
            '/ioc/control_lock'
        )
        
        # 서랍문 상태 확인 Service Client (RC → IOC)
        self.check_door_client = self.node.create_client(
            CheckDoorState,
            '/ioc/check_door_state'
        )
        
        # 물품 적재 확인 Service Client (RC → IOC)
        self.check_item_client = self.node.create_client(
            CheckItemLoaded,
            '/ioc/check_item_loaded'
        )
        
        self.node.get_logger().info('IOC Service Clients 초기화 완료')
    
    def control_lock(self, robot_id: int, locked: bool, callback):
        """
        서랍 잠금 제어 (콜백 방식)
        Args:
            robot_id (int): 로봇 ID
            locked (bool): 잠금 여부
            callback (callable): 결과 처리 콜백 함수 (success: bool, message: str)
        """
        if not self.control_lock_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('IOC ControlLock 서비스를 찾을 수 없습니다')
            callback(False, "서비스 연결 실패")
            return
        
        request = ControlLock.Request()
        request.robot_id = robot_id
        request.locked = locked
        
        action = "잠금" if locked else "잠금 해제"
        self.node.get_logger().info(f'IOC 서랍 {action} 요청')
        
        try:
            future = self.control_lock_client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_control_lock_response(f, callback)
            )
            self.node.get_logger().info(f'IOC 서랍 {action} 요청 전송 완료')
                
        except Exception as e:
            self.node.get_logger().error(f'IOC 서랍 제어 요청 실패: {e}')
            callback(False, str(e))
    
    def _handle_control_lock_response(self, future, callback):
        """서랍 잠금 제어 응답 처리"""
        try:
            response = future.result()
            self.node.get_logger().info(f'IOC 서랍 제어 응답: success={response.success}')
            callback(response.success, "성공" if response.success else "실패")
        except Exception as e:
            self.node.get_logger().error(f'IOC 서랍 제어 응답 처리 중 오류: {e}')
            callback(False, str(e))
    
    def control_drawer_lock(self, locked: bool):
        """서랍 잠금 제어 (동기 방식 - 이전 버전과의 호환성 유지)"""
        if not self.control_lock_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('IOC ControlLock 서비스를 찾을 수 없습니다')
            return True  # 시뮬레이션에서는 성공으로 처리
        
        request = ControlLock.Request()
        request.robot_id = self.node.robot_id
        request.locked = locked
        
        action = "잠금" if locked else "잠금 해제"
        self.node.get_logger().info(f'IOC 서랍 {action} 요청')
        
        try:
            future = self.control_lock_client.call_async(request)
            self.node.get_logger().info(f'IOC 서랍 {action} 요청 전송 완료')
            return True
                
        except Exception as e:
            self.node.get_logger().error(f'IOC 서랍 제어 요청 실패: {e}')
            return False
    

    def check_door_state(self, robot_id: int, callback):
        """
        서랍문 상태 확인 (콜백 방식)
        Args:
            robot_id (int): 로봇 ID
            callback (callable): 결과 처리 콜백 함수 (is_opened: bool)
        """
        if not self.check_door_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('IOC CheckDoorState 서비스를 찾을 수 없습니다')
            callback(False)
            return
        
        request = CheckDoorState.Request()
        request.robot_id = robot_id
        
        self.node.get_logger().info('IOC 서랍문 상태 확인')
        
        try:
            future = self.check_door_client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_door_state_response(f, callback)
            )
            self.node.get_logger().info('IOC 서랍문 상태 확인 요청 전송 완료')
                
        except Exception as e:
            self.node.get_logger().error(f'IOC 서랍문 상태 확인 실패: {e}')
            callback(False)
    
    def _handle_door_state_response(self, future, callback):
        """서랍문 상태 확인 응답 처리"""
        try:
            response = future.result()
            self.node.get_logger().info(f'IOC 서랍문 상태 응답: is_opened={response.is_opened}')
            callback(response.is_opened)
        except Exception as e:
            self.node.get_logger().error(f'IOC 서랍문 상태 응답 처리 중 오류: {e}')
            callback(False)
    
    def check_item_loaded(self, robot_id: int, callback):
        """
        물품 적재 확인 (콜백 방식)
        Args:
            robot_id (int): 로봇 ID
            callback (callable): 결과 처리 콜백 함수 (item_loaded: bool)
        """
        if not self.check_item_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('IOC CheckItemLoaded 서비스를 찾을 수 없습니다')
            callback(False)
            return
        
        request = CheckItemLoaded.Request()
        request.robot_id = robot_id
        
        self.node.get_logger().info('IOC 물품 적재 확인')
        
        try:
            future = self.check_item_client.call_async(request)
            future.add_done_callback(
                lambda f: self._handle_item_loaded_response(f, callback)
            )
            self.node.get_logger().info('IOC 물품 적재 확인 요청 전송 완료')
                
        except Exception as e:
            self.node.get_logger().error(f'IOC 물품 적재 확인 실패: {e}')
            callback(False)
    
    def _handle_item_loaded_response(self, future, callback):
        """물품 적재 확인 응답 처리"""
        try:
            response = future.result()
            self.node.get_logger().info(f'IOC 물품 적재 응답: item_loaded={response.item_loaded}')
            callback(response.item_loaded)
        except Exception as e:
            self.node.get_logger().error(f'IOC 물품 적재 응답 처리 중 오류: {e}')
            callback(False)
    
    # 편의 메서드들
    
    def unlock_drawer(self):
        """서랍 열기 (편의 메서드)"""
        return self.control_drawer_lock(False)
    
    def lock_drawer(self):
        """서랍 잠그기 (편의 메서드)"""
        return self.control_drawer_lock(True)
    
    def is_drawer_open(self):
        """서랍이 열려있는지 확인 (편의 메서드)"""
        return self.check_door_state()

    def has_item(self):
        """물품이 있는지 확인 (편의 메서드)"""
        return self.check_item_loaded() 