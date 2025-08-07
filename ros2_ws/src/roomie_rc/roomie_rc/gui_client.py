#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient

# ROS2 메시지 import
from roomie_msgs.msg import RobotGuiEvent
from roomie_msgs.action import StartCountdown, ReturnCountdown



class GUIClient:
    """
    RGUI(Robot GUI)와의 통신을 담당하는 클라이언트
    """
    
    def __init__(self, parent_node: Node):
        self.node = parent_node
        
        # QoS 프로파일 설정
        self.qos_profile = QoSProfile(depth=10)
        
        # Publishers, Subscribers, Action Clients 초기화
        self._init_publishers()
        self._init_subscribers()
        self._init_action_clients()
        
        self.node.get_logger().info('GUI Client 초기화 완료')
    
    def _init_publishers(self):
        """Publisher 초기화"""
        # GUI 이벤트 Publisher (RC → RGUI)
        self.gui_event_pub = self.node.create_publisher(
            RobotGuiEvent,
            '/robot_gui/event',
            self.qos_profile
        )
        self.node.get_logger().info('GUI Event Publisher 초기화 완료')
    
    def _init_subscribers(self):
        """Subscriber 초기화"""
        # GUI 이벤트 Subscriber (RGUI → RC)
        self.gui_event_sub = self.node.create_subscription(
            RobotGuiEvent,
            '/robot_gui/event',
            self._gui_event_callback,
            self.qos_profile
        )
        self.node.get_logger().info('GUI Event Subscriber 초기화 완료')
    
    def _init_action_clients(self):
        """Action Client 초기화"""
        # 출발 카운트다운 Action Client (RC → RGUI)
        self.start_countdown_client = ActionClient(
            self.node,
            StartCountdown,
            '/robot_gui/action/start_countdown'
        )
        
        # 복귀 카운트다운 Action Client (RC → RGUI)
        self.return_countdown_client = ActionClient(
            self.node,
            ReturnCountdown,
            '/robot_gui/action/return_countdown'
        )
        
        self.node.get_logger().info('GUI Action Clients 초기화 완료')
    
    def _gui_event_callback(self, msg):
        """GUI 이벤트 콜백 (RGUI → RC)"""
        self.node.get_logger().info(f'GUI 이벤트 수신: robot_id={msg.robot_id}, event_id={msg.rgui_event_id}')
        self.node.get_logger().info(f'이벤트 내용: {msg.detail}')
        
        # 등록된 콜백이 있으면 호출
        if hasattr(self, '_event_callback'):
            self._event_callback(msg)
    
    def subscribe_events(self, callback):
        """GUI 이벤트 구독 콜백 등록"""
        self._event_callback = callback
    
    def send_gui_event(self, event_id, detail="", task_id=0):
        """GUI 이벤트 전송 (RC → RGUI)"""
        msg = RobotGuiEvent()
        msg.robot_id = self.node.robot_id
        msg.rgui_event_id = event_id
        msg.task_id = task_id
        msg.timestamp = self.node.get_clock().now().to_msg()
        msg.detail = detail
        
        self.gui_event_pub.publish(msg)
        self.node.get_logger().info(f'GUI에게 이벤트 전송: event_id={event_id}, detail="{detail}"')
    
    def start_countdown(self, robot_id, task_id, task_type_id, callback=None):
        """출발 카운트다운 시작 (RC → RGUI)"""
        if not self.start_countdown_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn('StartCountdown 액션 서버를 찾을 수 없습니다')
            return False
        
        goal_msg = StartCountdown.Goal()
        goal_msg.robot_id = robot_id
        goal_msg.task_id = task_id
        goal_msg.task_type_id = task_type_id
        
        self.node.get_logger().info(f'GUI 카운트다운 시작 요청: robot_id={robot_id}, task_id={task_id}, task_type_id={task_type_id}')
        
        try:
            send_goal_future = self.start_countdown_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self.node.get_logger().info(f'카운트다운 피드백: {feedback.feedback.remaining_time}초')
            )
            
            if callback:
                send_goal_future.add_done_callback(
                    lambda future: self._handle_countdown_response(future, callback)
                )
            
            self.node.get_logger().info('GUI 카운트다운 요청 전송 완료')
            return True
                
        except Exception as e:
            self.node.get_logger().error(f'GUI 카운트다운 요청 실패: {e}')
            return False
            
    def _handle_countdown_response(self, future, callback):
        """카운트다운 응답 처리"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('카운트다운 목표가 거부됨')
            callback(None)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: callback(future.result())
        )
    
    def start_return_countdown(self, callback=None):
        """복귀 카운트다운 시작 (RC → RGUI)"""
        if not self.return_countdown_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn('ReturnCountdown 액션 서버를 찾을 수 없습니다')
            return False
        
        goal_msg = ReturnCountdown.Goal()
        goal_msg.robot_id = self.node.robot_id
        
        self.node.get_logger().info('GUI 복귀 카운트다운 시작 요청')
        
        try:
            send_goal_future = self.return_countdown_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self.node.get_logger().info(f'복귀 카운트다운 피드백: {feedback.feedback.remaining_time}초')
            )
            
            if callback:
                send_goal_future.add_done_callback(
                    lambda future: self._handle_countdown_response(future, callback)
                )
            
            self.node.get_logger().info('GUI 복귀 카운트다운 요청 전송 완료')
            return True
        except Exception as e:
            self.node.get_logger().error(f'GUI 복귀 카운트다운 요청 실패: {e}')
            return False 