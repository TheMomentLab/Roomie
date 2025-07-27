#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from roomie_msgs.srv import ControlLock, CheckDoorState, CheckItemLoaded


class TestIOCNode(Node):
    """IOC(IO Controller) 테스트를 위한 더미 노드"""
    
    def __init__(self):
        super().__init__('test_ioc_node')
        
        # IOC 서비스 서버들
        self.ioc_control_lock_server = self.create_service(
            ControlLock,
            '/ioc/control_lock',
            self.ioc_control_lock_callback
        )
        
        self.ioc_check_door_server = self.create_service(
            CheckDoorState,
            '/ioc/check_door_state',
            self.ioc_check_door_callback
        )
        
        self.ioc_check_item_server = self.create_service(
            CheckItemLoaded,
            '/ioc/check_item_loaded',
            self.ioc_check_item_callback
        )
        
        # 시뮬레이션 상태들
        self.drawer_is_locked = True     # 서랍 잠금 상태 (기본값: 잠김)
        self.drawer_is_open = False      # 서랍 열림 상태 (기본값: 닫힘)
        self.item_loaded = False         # 물품 적재 상태 (기본값: 없음)
        
        self.get_logger().info('🔧 테스트 IOC 노드 시작됨')
        self.get_logger().info('📡 IOC 서비스 서버들:')
        self.get_logger().info('   - /ioc/control_lock (서랍 잠금 제어)')
        self.get_logger().info('   - /ioc/check_door_state (서랍문 상태 확인)')
        self.get_logger().info('   - /ioc/check_item_loaded (물품 적재 확인)')
        self.get_logger().info('🔒 초기 상태:')
        self.get_logger().info(f'   - 서랍 잠금: {self.drawer_is_locked}')
        self.get_logger().info(f'   - 서랍 열림: {self.drawer_is_open}')
        self.get_logger().info(f'   - 물품 적재: {self.item_loaded}')
        
        # 상태 변경을 위한 명령어 안내
        self.get_logger().info('💡 상태 변경 명령어:')
        self.get_logger().info('   - 서랍 잠금/해제: ros2 service call /ioc/control_lock roomie_msgs/srv/ControlLock "{robot_id: 1, locked: true/false}"')
        self.get_logger().info('   - 서랍 상태 확인: ros2 service call /ioc/check_door_state roomie_msgs/srv/CheckDoorState "{robot_id: 1}"')
        self.get_logger().info('   - 물품 상태 확인: ros2 service call /ioc/check_item_loaded roomie_msgs/srv/CheckItemLoaded "{robot_id: 1}"')
    
    def ioc_control_lock_callback(self, request, response):
        """서랍 잠금 제어 서비스 콜백"""
        self.get_logger().info(f'🔒 서랍 잠금 제어 요청: robot_id={request.robot_id}, locked={request.locked}')
        
        # 시뮬레이션: 잠금 상태 변경
        self.drawer_is_locked = request.locked
        
        # 잠금 상태에 따른 서랍 열림 상태도 변경
        if request.locked:
            self.drawer_is_open = False  # 잠그면 닫힘
            self.get_logger().info('🔒 서랍 잠금 완료 (서랍 닫힘)')
        else:
            self.drawer_is_open = True   # 해제하면 열림
            self.get_logger().info('🔓 서랍 잠금 해제 완료 (서랍 열림)')
        
        response.success = True
        
        self.get_logger().info(f'✅ 응답: success={response.success}')
        return response
    
    def ioc_check_door_callback(self, request, response):
        """서랍문 상태 확인 서비스 콜백"""
        self.get_logger().info(f'🚪 서랍문 상태 확인 요청: robot_id={request.robot_id}')
        
        # 시뮬레이션: 현재 서랍 열림 상태 반환
        response.is_opened = self.drawer_is_open
        
        self.get_logger().info(f'✅ 응답: is_opened={response.is_opened}')
        return response
    
    def ioc_check_item_callback(self, request, response):
        """물품 적재 확인 서비스 콜백"""
        self.get_logger().info(f'📦 물품 적재 확인 요청: robot_id={request.robot_id}')
        
        # 시뮬레이션: 현재 물품 적재 상태 반환
        response.item_loaded = self.item_loaded
        
        self.get_logger().info(f'✅ 응답: item_loaded={response.item_loaded}')
        return response
    
    def set_item_loaded(self, loaded: bool):
        """물품 적재 상태 설정 (테스트용)"""
        self.item_loaded = loaded
        status = "적재" if loaded else "제거"
        self.get_logger().info(f'📦 물품 {status} 상태로 변경됨')
    
    def toggle_drawer_state(self):
        """서랍 상태 토글 (테스트용)"""
        self.drawer_is_open = not self.drawer_is_open
        status = "열림" if self.drawer_is_open else "닫힘"
        self.get_logger().info(f'🚪 서랍 상태 토글: {status}')
        
        # 열림 상태면 잠금 해제, 닫힘 상태면 잠금
        self.drawer_is_locked = not self.drawer_is_open


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_ioc_node = TestIOCNode()
        rclpy.spin(test_ioc_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'test_ioc_node' in locals():
            test_ioc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 