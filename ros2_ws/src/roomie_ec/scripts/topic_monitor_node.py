#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

# Message imports
from roomie_msgs.msg import RobotGuiEvent

# Local imports
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'roomie_ec'))
from config import *

class TopicMonitorNode(Node):
    def __init__(self):
        super().__init__('topic_monitor_node')
        
        # ===== Callback Groups =====
        self.callback_group = ReentrantCallbackGroup()
        
        # ===== Subscribers =====
        # GUI 이벤트 구독자
        self.gui_event_sub = self.create_subscription(
            RobotGuiEvent,
            GUI_EVENT_TOPIC,
            self.gui_event_callback,
            10,
            callback_group=self.callback_group
        )
        
        # ===== Monitoring State =====
        self.last_gui_event_time = None
        self.gui_event_count = 0
        
        # ===== Timers =====
        # 서비스 상태 확인 타이머 (3초마다)
        self.service_check_timer = self.create_timer(
            3.0,
            self.check_services,
            callback_group=self.callback_group
        )
        
        # 토픽 상태 확인 타이머 (2초마다)
        self.topic_check_timer = self.create_timer(
            2.0,
            self.check_topics,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('=== 토픽 모니터링 노드가 시작되었습니다 ===')
        self.get_logger().info('서비스와 토픽 상태를 실시간으로 모니터링합니다.')
    
    def gui_event_callback(self, msg):
        """GUI 이벤트 메시지 수신"""
        self.gui_event_count += 1
        self.last_gui_event_time = time.time()
        
        self.get_logger().info(f'📱 GUI 이벤트 수신 #{self.gui_event_count}')
        self.get_logger().info(f'   rgui_event_id: {msg.rgui_event_id}')
        self.get_logger().info(f'   robot_id: {msg.robot_id}')
        self.get_logger().info(f'   task_id: {msg.task_id}')
        self.get_logger().info(f'   detail: {msg.detail}')
    
    def check_services(self):
        """서비스 상태 확인"""
        try:
            # Location 서비스 확인
            location_clients = self.get_service_names_and_types()
            location_available = any(
                VS_LOCATION_SERVICE in client[0] 
                for client in location_clients
            )
            
            # SetVSMode 서비스 확인
            set_mode_available = any(
                VS_SET_MODE_SERVICE in client[0] 
                for client in location_clients
            )
            
            self.get_logger().info('🔍 서비스 상태:')
            self.get_logger().info(f'   {VS_LOCATION_SERVICE}: {"✅" if location_available else "❌"}')
            self.get_logger().info(f'   {VS_SET_MODE_SERVICE}: {"✅" if set_mode_available else "❌"}')
            
        except Exception as e:
            self.get_logger().error(f'서비스 확인 중 오류: {str(e)}')
    
    def check_topics(self):
        """토픽 상태 확인"""
        try:
            # GUI 이벤트 토픽 확인
            topic_names_and_types = self.get_topic_names_and_types()
            gui_topic_available = any(
                GUI_EVENT_TOPIC in topic[0] 
                for topic in topic_names_and_types
            )
            
            self.get_logger().info('📡 토픽 상태:')
            self.get_logger().info(f'   {GUI_EVENT_TOPIC}: {"✅" if gui_topic_available else "❌"}')
            
            # GUI 이벤트 수신 상태
            if self.last_gui_event_time:
                time_since_last = time.time() - self.last_gui_event_time
                self.get_logger().info(f'   마지막 GUI 이벤트: {time_since_last:.1f}초 전')
            else:
                self.get_logger().info('   마지막 GUI 이벤트: 없음')
            
            self.get_logger().info(f'   총 GUI 이벤트 수신: {self.gui_event_count}회')
            
        except Exception as e:
            self.get_logger().error(f'토픽 확인 중 오류: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # 멀티스레드 실행자 생성
    executor = MultiThreadedExecutor()
    
    # 노드 생성 및 실행자에 추가
    node = TopicMonitorNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 