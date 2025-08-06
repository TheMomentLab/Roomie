#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from datetime import datetime

# Service imports
from roomie_msgs.srv import Location, SetVSMode, ButtonStatus, ElevatorStatus, DoorStatus

# Message imports
from roomie_msgs.msg import RobotGuiEvent
from geometry_msgs.msg import PoseStamped

# Action imports
from roomie_msgs.action import ClickButton, SetPose
from rclpy.action import ActionServer

# Built-in message imports
from builtin_interfaces.msg import Time

# Local imports
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'roomie_ec'))
from config import *

class TestCommunicationNode(Node):
    def __init__(self):
        super().__init__('test_communication_node')
        
        # ===== ROS Parameters =====
        self.declare_parameter('start_state', START_STATE)
        self.start_state = self.get_parameter('start_state').value
        
        # ===== Callback Groups =====
        self.callback_group = ReentrantCallbackGroup()
        
        # ===== Service Servers (모의 VS) =====
        # Location 서비스 서버
        self.location_server = self.create_service(
            Location,
            VS_LOCATION_SERVICE,
            self.location_callback,
            callback_group=self.callback_group
        )
        
        # SetVSMode 서비스 서버
        self.set_vs_mode_server = self.create_service(
            SetVSMode,
            VS_SET_MODE_SERVICE,
            self.set_vs_mode_callback,
            callback_group=self.callback_group
        )
        
        # ButtonStatus 서비스 서버
        self.button_status_server = self.create_service(
            ButtonStatus,
            VS_BUTTON_STATUS_SERVICE,
            self.button_status_callback,
            callback_group=self.callback_group
        )
        
        # ElevatorStatus 서비스 서버 (시나리오 2용)
        self.elevator_status_server = self.create_service(
            ElevatorStatus,
            VS_ELEVATOR_STATUS_SERVICE,
            self.elevator_status_callback,
            callback_group=self.callback_group
        )
        
        # DoorStatus 서비스 서버 (시나리오 2용)
        self.door_status_server = self.create_service(
            DoorStatus,
            VS_DOOR_STATUS_SERVICE,
            self.door_status_callback,
            callback_group=self.callback_group
        )
        
        # ===== Action Servers =====
        # ClickButton 액션 서버
        self.click_button_server = ActionServer(
            self,
            ClickButton,
            ARM_CLICK_BUTTON_ACTION,
            self.click_button_callback,
            callback_group=self.callback_group
        )
        
        # SetPose 액션 서버
        self.set_pose_server = ActionServer(
            self,
            SetPose,
            ARM_SET_POSE_ACTION,
            self.set_pose_callback,
            callback_group=self.callback_group
        )
        
        # ===== Subscribers =====
        # GUI 이벤트 구독자
        self.gui_event_sub = self.create_subscription(
            RobotGuiEvent,
            GUI_EVENT_TOPIC,
            self.gui_event_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Simple Navigator 목표 구독자
        self.simple_goal_sub = self.create_subscription(
            PoseStamped,
            SIMPLE_GOAL_TOPIC,
            self.simple_goal_callback,
            10,
            callback_group=self.callback_group
        )
        
        # ===== Test State =====
        self.current_vs_mode = 0   # 초기 모드 (대기모드)
        self.gui_event_count = 0
        self.location_request_count = 0  # 아루코 마커 인식 시뮬레이션용
        self.button_status_count = 0     # 버튼 상태 요청 카운터
        self.elevator_status_count = 0   # 엘리베이터 상태 요청 카운터 (시나리오 2용)
        self.door_status_count = 0       # 문 상태 요청 카운터 (시나리오 2용)
        self.elevator_arrival_count = 0  # 엘리베이터 도착 요청 카운터 (시나리오 4용)
        self.door_status_exit_count = 0  # 문 상태 요청 카운터 (시나리오 4용)
        
        # ===== Timers =====
        # 상태 출력 타이머 (5초마다)
        self.status_timer = self.create_timer(
            5.0,
            self.print_status,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('=== 테스트 노드 시작 ===')
        self.get_logger().info(f'시작 상태: {self.start_state}')
        self.get_logger().info('아루코 마커 인식 시뮬레이션: 처음 4번 실패, 5번째 성공')
        if self.start_state in ["SCENARIO_3_READY", "SETTING_ELEVATOR_INTERIOR_MODE"]:
            self.get_logger().info('시나리오 3 모드: 엘리베이터 내부 버튼 클릭 시뮬레이션')
    
    def location_callback(self, request, response):
        """Location 서비스 요청 처리 (모의 VS)"""
        self.location_request_count += 1
        
        # 시작 상태에 따른 응답 조정
        if self.start_state in ["INIT", "CHECKING_LOCATION"]:
            # 아루코 마커 인식 시뮬레이션: 처음 4번 실패, 5번째 성공
            if self.location_request_count <= 4:
                # 처음 4번은 아루코 마커 인식 실패
                self.get_logger().info(f'📍 Location 요청 #{self.location_request_count}: 아루코 마커 인식 실패')
                response.robot_id = request.robot_id
                response.success = False
                response.location_id = 0
            else:
                # 5번째부터 아루코 마커 인식 성공 (목적지 5)
                self.get_logger().info(f'📍 Location 요청 #{self.location_request_count}: 아루코 마커 인식 성공 (위치 5)')
                response.robot_id = request.robot_id
                response.success = True
                response.location_id = 5
        else:
            # 다른 상태에서는 바로 성공
            self.get_logger().info(f'📍 Location 요청 #{self.location_request_count}: 바로 성공 (위치 5)')
            response.robot_id = request.robot_id
            response.success = True
            response.location_id = 5
        
        return response
    
    def set_vs_mode_callback(self, request, response):
        """SetVSMode 서비스 요청 처리 (모의 VS)"""
        old_mode = self.current_vs_mode
        self.current_vs_mode = request.mode_id
        
        self.get_logger().info(f'🎯 SetVSMode: {old_mode}→{self.current_vs_mode}')
        
        response.robot_id = request.robot_id
        response.success = True
        
        return response
    
    def gui_event_callback(self, msg):
        """GUI 이벤트 메시지 수신"""
        self.gui_event_count += 1
        
        self.get_logger().info(f'📱 GUI 이벤트: {msg.rgui_event_id} - {msg.detail}')
    
    def simple_goal_callback(self, msg):
        """Simple Navigator 목표 수신"""
        self.get_logger().info(f'🎯 Simple Navigator 목표 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.get_logger().info(f'   방향: yaw={msg.pose.orientation.z:.2f}')
    
    def button_status_callback(self, request, response):
        """ButtonStatus 서비스 요청 처리 (모의 VS)"""
        self.button_status_count += 1
        
        # 시작 상태에 따른 응답 조정
        if self.start_state in ["TRACKING_BUTTON", "BUTTON_REACHED", "CLICKING_BUTTON", "BUTTON_CLICKED", "RETURNING_ARM", "ARM_RETURNED"]:
            # 버튼 추적 시뮬레이션: 점진적으로 목표에 접근
            if self.button_status_count <= 10:
                # 처음 10번은 점진적으로 접근
                progress = self.button_status_count / 10.0
                x = 0.3 + (0.5 - 0.3) * progress  # 0.3 → 0.5
                depth = 0.03 + (0.06 - 0.03) * progress  # 0.03 → 0.06
            else:
                # 10번 이후 목표 위치 도달
                x = 0.5
                depth = 0.06
        else:
            # 다른 상태에서는 바로 목표 위치
            x = 0.5
            depth = 0.06
        
        response.robot_id = request.robot_id
        response.button_id = request.button_id
        response.success = True
        response.x = x
        response.y = 0.5  # y는 고정
        response.size = depth  # depth를 size로 사용
        response.is_pressed = False
        response.timestamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'🔘 ButtonStatus 요청 #{self.button_status_count}: x={x:.3f}, size={depth:.3f}')
        
        return response
    
    def elevator_status_callback(self, request, response):
        """ElevatorStatus 서비스 요청 처리 (모의 VS)"""
        self.elevator_status_count += 1
        
        # 시나리오 2 시뮬레이션: 점진적으로 조건 만족
        if self.elevator_status_count <= 5:
            # 처음 5번은 조건 불만족 (다른 층이거나 방향 불일치)
            response.robot_id = request.robot_id
            response.success = True
            response.direction = 1  # downward
            response.position = 4   # 다른 층
        elif self.elevator_status_count <= 8:
            # 6-8번: 시나리오 2 조건 만족 (현재 층과 올바른 방향)
            response.robot_id = request.robot_id
            response.success = True
            response.direction = 0  # upward (5층→6층이므로)
            response.position = 5   # 현재 층
        else:
            # 9번 이후: 시나리오 4 시뮬레이션 (6층 도착)
            response.robot_id = request.robot_id
            response.success = True
            response.direction = 0  # upward
            response.position = 6   # 목적지 6층
        
        self.get_logger().info(f'🛗 ElevatorStatus 요청 #{self.elevator_status_count}: direction={response.direction}, position={response.position}')
        
        return response
    

    
    def door_status_callback(self, request, response):
        """DoorStatus 서비스 요청 처리 (모의 VS)"""
        self.door_status_count += 1
        
        # 시나리오 2 시뮬레이션: 점진적으로 문 열림
        if self.door_status_count <= 3:
            # 처음 3번은 문이 닫힌 상태
            response.robot_id = request.robot_id
            response.success = True
            response.door_opened = False
        elif self.door_status_count <= 8:
            # 4-8번: 시나리오 2 문 열림
            response.robot_id = request.robot_id
            response.success = True
            response.door_opened = True
        else:
            # 9번 이후: 시나리오 4 시뮬레이션 (6층에서 문 열림)
            if self.door_status_count <= 12:
                # 9-12번: 문이 닫힌 상태
                response.robot_id = request.robot_id
                response.success = True
                response.door_opened = False
            else:
                # 13번 이후: 문이 열린 상태
                response.robot_id = request.robot_id
                response.success = True
                response.door_opened = True
        
        self.get_logger().info(f'🚪 DoorStatus 요청 #{self.door_status_count}: door_opened={response.door_opened}')
        
        return response
    

    
    def click_button_callback(self, goal_handle):
        """ClickButton 액션 요청 처리 (모의 팔)"""
        self.get_logger().info(f'🎯 ClickButton 액션 시작: button_id={goal_handle.request.button_id}')
        
        # 시작 상태에 따른 처리 시간 조정
        if self.start_state in ["CLICKING_BUTTON", "BUTTON_CLICKED", "RETURNING_ARM", "ARM_RETURNED"]:
            # 정상적인 시뮬레이션: 10초
            time.sleep(10)
        elif self.start_state in ["SCENARIO_3_READY", "SETTING_ELEVATOR_INTERIOR_MODE"]:
            # 시나리오 3 시뮬레이션: 5초 (내부 버튼 클릭은 더 빠름)
            time.sleep(5)
        else:
            # 다른 상태에서는 빠르게 완료
            time.sleep(1)
        
        result = ClickButton.Result()
        result.robot_id = goal_handle.request.robot_id
        result.success = True
        result.message = "버튼 클릭 완료"
        
        goal_handle.succeed()
        
        self.get_logger().info('✅ ClickButton 액션 완료')
        return result
    
    def set_pose_callback(self, goal_handle):
        """SetPose 액션 요청 처리 (모의 팔)"""
        self.get_logger().info(f'🤖 SetPose 액션 시작: pose_id={goal_handle.request.pose_id}')
        
        # pose_id에 따른 처리 시간 조정
        if goal_handle.request.pose_id == 3:  # 팔 정면 설정
            # 팔 정면 설정: 3초
            time.sleep(3)
        elif goal_handle.request.pose_id == 4:  # 팔 상향 설정 (시나리오 3)
            # 시작 상태에 따른 처리 시간 조정
            if self.start_state in ["SCENARIO_3_READY", "SETTING_ELEVATOR_INTERIOR_MODE"]:
                # 정상적인 시뮬레이션: 3초
                time.sleep(3)
            else:
                # 다른 상태에서는 빠르게 완료
                time.sleep(1)
        elif goal_handle.request.pose_id == 0:  # 팔 원위치
            # 시작 상태에 따른 처리 시간 조정
            if self.start_state in ["RETURNING_ARM", "ARM_RETURNED"]:
                # 정상적인 시뮬레이션: 5초
                time.sleep(5)
            else:
                # 다른 상태에서는 빠르게 완료
                time.sleep(1)
        else:
            # 다른 pose_id: 1초
            time.sleep(1)
        
        result = SetPose.Result()
        result.robot_id = goal_handle.request.robot_id
        result.success = True
        
        goal_handle.succeed()
        
        self.get_logger().info('✅ SetPose 액션 완료')
        return result
    
    def print_status(self):
        """현재 상태 출력"""
        self.get_logger().info(f'📊 상태: 모드={self.current_vs_mode}, 위치요청={self.location_request_count}, 버튼요청={self.button_status_count}, 이벤트={self.gui_event_count}, 엘리베이터={self.elevator_status_count}, 문={self.door_status_count}')

def main(args=None):
    rclpy.init(args=args)
    
    # 멀티스레드 실행자 생성
    executor = MultiThreadedExecutor()
    
    # 노드 생성 및 실행자에 추가
    node = TestCommunicationNode()
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