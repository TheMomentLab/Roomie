#!/usr/bin/env python3
"""
Roomie Elevator Controller Node

이 노드는 로봇이 엘리베이터를 이용한 층간 이동을 수행하는 메인 컨트롤러입니다.

=== 현재 구현 상태 ===
✅ 시나리오 1: 1층 → 엘리베이터 도착 (완료)
❌ 시나리오 2: 엘리베이터 탑승 (미구현)

=== 주요 기능 ===
- VS 서비스/액션 클라이언트 (위치 확인, 버튼 클릭, 팔 제어)
- 후진 이동 로직 (시간 기반 거리 계산)
- simple_navigator2 모니터링 및 동기화
- 디버그 모드 지원 (통신 테스트 vs 실제 주행)
- 부분 테스트 지원 (start_state 파라미터)

=== 해결된 문제들 ===
✅ cmd_vel 충돌: simple_navigator2가 idle 상태일 때 cmd_vel 발행하지 않음
✅ 완료 신호 수신: simple_navigator2에서 즉시 "completed" 메시지 발행
✅ 후진 이동: TF 대신 시간 기반 간단한 거리 계산 사용
"""

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
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

# Action imports
from roomie_msgs.action import ClickButton, SetPose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Built-in message imports
from builtin_interfaces.msg import Time

# TF imports (for quaternion conversion only)
import tf_transformations

# Local imports
from .config import *

class RoomieECNode(Node):
    def __init__(self):
        super().__init__('roomie_ec_node')
        
        # ===== ROS Parameters =====
        self.declare_parameter('debug_mode', DEBUG_MODE)
        self.declare_parameter('start_state', START_STATE)
        
        self.debug_mode = self.get_parameter('debug_mode').value
        self.start_state = self.get_parameter('start_state').value
        
        # ===== Configuration Variables =====
        # 로봇 설정
        self.robot_id = ROBOT_ID
        self.location_id = LOCATION_ID  # 도착지 (ELE_1)
        self.task_id = TASK_ID
        self.current_floor = CURRENT_FLOOR
        self.target_floor = TARGET_FLOOR
        self.scenario_id = SCENARIO_ID
        
        # 타이머 설정
        self.location_check_timeout = LOCATION_CHECK_TIMEOUT  # 5초 타임아웃
        
        # ===== Node State =====
        self.current_state = self.start_state  # 파라미터로 시작 상태 설정
        self.location_check_start_time = time.time()
        self.location_check_count = 0
        
        # ===== Post-Scenario 1 State =====
        self.backup_start_time = time.time()
        self.backup_distance_moved = 0.0
        self.initial_position = None
        self.backup_started = False  # 후진 이동 시작 플래그
        
        # ===== Backup Movement State =====
        self.backup_start_time = None  # 후진 시작 시간
        
        # ===== Scenario 2 State =====
        self.elevator_status_start_time = time.time()
        self.elevator_status_count = 0
        self.door_status_start_time = time.time()
        self.door_status_count = 0
        
        # ===== Scenario 4 State =====
        self.elevator_arrival_start_time = time.time()
        self.elevator_arrival_count = 0
        self.door_status_exit_start_time = time.time()
        self.door_status_exit_count = 0
        
        # ===== Simple Navigator Monitoring =====
        self.simple_nav_start_time = time.time()
        self.simple_nav_timeout = 60.0  # 60초 타임아웃
        
        # ===== Callback Groups =====
        self.callback_group = ReentrantCallbackGroup()
        
        # ===== Service Clients =====
        # VS 서비스 클라이언트들
        self.location_client = self.create_client(
            Location, 
            VS_LOCATION_SERVICE,
            callback_group=self.callback_group
        )
        
        self.set_vs_mode_client = self.create_client(
            SetVSMode,
            VS_SET_MODE_SERVICE,
            callback_group=self.callback_group
        )
        
        self.button_status_client = self.create_client(
            ButtonStatus,
            VS_BUTTON_STATUS_SERVICE,
            callback_group=self.callback_group
        )
        
        # 시나리오 2용 서비스 클라이언트들
        self.elevator_status_client = self.create_client(
            ElevatorStatus,
            VS_ELEVATOR_STATUS_SERVICE,
            callback_group=self.callback_group
        )
        
        self.door_status_client = self.create_client(
            DoorStatus,
            VS_DOOR_STATUS_SERVICE,
            callback_group=self.callback_group
        )
        
        # ===== Action Clients =====
        # set_pose.py와 동일한 방식으로 ActionClient 생성 (callback_group 제거)
        self.click_button_client = ActionClient(
            self,
            ClickButton,
            ARM_CLICK_BUTTON_ACTION
        )
        
        self.set_pose_client = ActionClient(
            self,
            SetPose,
            ARM_SET_POSE_ACTION
        )
        
        # Nav2 Action Client (시나리오 2용)
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # ===== Publishers =====
        # GUI 이벤트 발행자
        self.gui_event_pub = self.create_publisher(
            RobotGuiEvent,
            GUI_EVENT_TOPIC,
            10
        )
        
        # 로봇 이동 명령 발행자
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10
        )
        
        # Simple Navigator 목표 발행자
        self.simple_goal_pub = self.create_publisher(
            PoseStamped,
            SIMPLE_GOAL_TOPIC,
            10
        )
        
        # Simple Navigator 상태 구독자
        self.simple_nav_status_sub = self.create_subscription(
            String,
            '/simple_nav/status',
            self.simple_nav_status_callback,
            10,
            callback_group=self.callback_group
        )
        
        # ===== Timers =====
        # 메인 시나리오 타이머 (1Hz)
        self.scenario_timer = self.create_timer(
            1.0 / SCENARIO_TIMER_HZ,
            self.scenario_step,
            callback_group=self.callback_group
        )
        
        # 위치 확인 타이머 (1Hz)
        self.location_timer = self.create_timer(
            1.0 / LOCATION_TIMER_HZ,
            self.check_location,
            callback_group=self.callback_group
        )
        
        # 버튼 추적 타이머 (10Hz)
        self.button_tracking_timer = self.create_timer(
            1.0 / BUTTON_TRACKING_TIMER_HZ,
            self.track_button,
            callback_group=self.callback_group
        )
        
        # 시나리오 2용 타이머들 (필요할 때 생성)
        # 엘리베이터 상태 확인 타이머 (1Hz)
        self.elevator_status_timer = None
        
        # 문 상태 확인 타이머 (1Hz)
        self.door_status_timer = None
        
        # 후진 이동 타이머 (필요할 때 생성)
        self.backup_timer = None
        
        # Simple Navigator 모니터링 타이머 (필요할 때 생성)
        self.simple_nav_monitor_timer = None
        
        # 시나리오 2용 타이머들 (필요할 때 생성)
        self.elevator_status_timer = None
        self.door_status_timer = None
        
        # 시나리오 4용 타이머들 (필요할 때 생성)
        self.elevator_arrival_timer = None
        self.door_status_exit_timer = None
        
        self.get_logger().info('=== EC 노드 시작 ===')
        self.get_logger().info(f'시나리오: {self.current_floor}층→{self.target_floor}층')
        self.get_logger().info(f'디버그 모드: {self.debug_mode}')
        self.get_logger().info(f'시작 상태: {self.start_state}')
        
        # ActionClient 연결 상태 확인
        self.get_logger().info(f'🔗 SetPose 액션 서버 주소: {ARM_SET_POSE_ACTION}')
        self.get_logger().info(f'🔗 ClickButton 액션 서버 주소: {ARM_CLICK_BUTTON_ACTION}')
        
        # ActionClient 연결 대기
        self.get_logger().info('⏳ ActionClient 연결 대기 중...')
        
        # ActionClient 연결 대기 (set_pose.py와 동일한 방식)
        self.get_logger().info('🔍 SetPose 액션 서버 연결 대기 중...')
        self.set_pose_client.wait_for_server()
        self.get_logger().info('✅ SetPose 액션 서버 연결됨!')
        
        self.get_logger().info('🔍 ClickButton 액션 서버 연결 대기 중...')
        self.click_button_client.wait_for_server()
        self.get_logger().info('✅ ClickButton 액션 서버 연결됨!')
        
        # VS 서비스 연결 상태 확인
        self.get_logger().info(f'🔗 VS Location 서비스 주소: {VS_LOCATION_SERVICE}')
        self.get_logger().info(f'🔗 VS SetVSMode 서비스 주소: {VS_SET_MODE_SERVICE}')
        self.get_logger().info(f'🔗 VS ButtonStatus 서비스 주소: {VS_BUTTON_STATUS_SERVICE}')
        
        # VS 서비스 연결 대기
        self.get_logger().info('⏳ VS 서비스 연결 대기 중...')
        self.location_client.wait_for_service()
        self.get_logger().info('✅ VS Location 서비스 연결됨!')
        self.set_vs_mode_client.wait_for_service()
        self.get_logger().info('✅ VS SetVSMode 서비스 연결됨!')
        self.button_status_client.wait_for_service()
        self.get_logger().info('✅ VS ButtonStatus 서비스 연결됨!')
    
    def scenario_step(self):
        """메인 시나리오 단계별 실행"""
        if self.current_state == "INIT":
            self.current_state = "SETTING_ARM_FORWARD"
            self.get_logger().info("🚀 시나리오 시작 - 팔 정면 설정")
            self.set_arm_forward()
        

        
        elif self.current_state == "LOCATION_CONFIRMED":
            self.current_state = "SETTING_ELEVATOR_MODE"
            self.get_logger().info("✅ 위치 확인 완료")
            self.set_elevator_mode()
            
        elif self.current_state == "ELEVATOR_MODE_SET":
            self.current_state = "SENDING_GUI_WARNING"
            self.get_logger().info("🎯 모드 설정 완료")
            self.send_gui_warning()
            
        elif self.current_state == "SENDING_GUI_WARNING":
            self.current_state = "TRACKING_BUTTON"
            self.get_logger().info("📱 GUI 이벤트 발송 완료")
            self.get_logger().info("🚗 버튼 추적 시작")
            
        elif self.current_state == "BUTTON_REACHED":
            self.current_state = "CLICKING_BUTTON"
            self.get_logger().info("🎯 목표 위치 도달")
            self.click_button()
            
        elif self.current_state == "BUTTON_CLICKED":
            self.current_state = "RETURNING_ARM"
            self.get_logger().info("✅ 버튼 클릭 완료")
            self.return_arm()
            
        elif self.current_state == "ARM_RETURNED":
            self.current_state = "SENDING_MOVEMENT_EVENT"
            self.get_logger().info("🤖 팔 원위치 완료")
            self.send_movement_event()
            
        elif self.current_state == "SENDING_MOVEMENT_EVENT":
            self.current_state = "BACKING_UP"
            self.get_logger().info("📱 이동 이벤트 발송 완료")
            self.get_logger().info("🚗 후진 이동 시작 (전광판 인식 준비)")
            self.start_backup_movement()
            
        elif self.current_state == "BACKING_UP":
            # 후진 이동이 아직 시작되지 않았으면 시작
            if not self.backup_started:
                self.get_logger().info("🚗 후진 이동 시작 (전광판 인식 준비)")
                self.start_backup_movement()
                self.backup_started = True
            
        elif self.current_state == "BACKUP_COMPLETED":
            self.current_state = "MOVING_TO_ELEVATOR_CENTER"
            self.get_logger().info("✅ 후진 이동 완료")
            self.get_logger().info("🎯 엘리베이터 중앙으로 이동 시작")
            self.move_to_elevator_center()
            
        elif self.current_state == "ELEVATOR_CENTER_REACHED":
            self.current_state = "SCENARIO_2_READY"
            # self.current_state = "COMPLETED"  # 강제 완료
            self.get_logger().info("✅ 엘리베이터 중앙 도달")
            self.get_logger().info("🎉 시나리오 2 준비 완료 - 엘리베이터 도착 판단 대기")
            
        elif self.current_state == "ERROR":
            # 에러 상태 - 타이머 정지
            self.scenario_timer.cancel()
            self.location_timer.cancel()
            self.button_tracking_timer.cancel()
            if self.backup_timer:
                self.backup_timer.cancel()
                self.backup_timer = None
            if self.elevator_status_timer:
                self.elevator_status_timer.cancel()
                self.elevator_status_timer = None
            if self.door_status_timer:
                self.door_status_timer.cancel()
                self.door_status_timer = None
            if self.simple_nav_monitor_timer:
                self.simple_nav_monitor_timer.cancel()
                self.simple_nav_monitor_timer = None
            self.get_logger().error("❌ 시나리오 실행 중 오류 발생 - 종료")
            
        elif self.current_state == "SCENARIO_2_READY":
            self.current_state = "CHECKING_ELEVATOR_STATUS"
            self.get_logger().info("🚀 시나리오 2 시작")
            self.get_logger().info("🛗 엘리베이터 도착 판단 시작")
            self.elevator_status_start_time = time.time()
            
            # 엘리베이터 상태 확인 타이머 시작
            if self.elevator_status_timer is None:
                self.elevator_status_timer = self.create_timer(
                    1.0,  # 1Hz
                    self.check_elevator_status,
                    callback_group=self.callback_group
                )
            
        elif self.current_state == "ELEVATOR_ARRIVED":
            self.current_state = "WAITING_FOR_DOOR_OPEN"
            self.get_logger().info("✅ 엘리베이터 도착 확인")
            self.get_logger().info("🚪 문 열림 대기 시작")
            self.door_status_start_time = time.time()
            
            # 엘리베이터 상태 확인 타이머 정지
            if self.elevator_status_timer:
                self.elevator_status_timer.cancel()
                self.elevator_status_timer = None
            
            # 문 상태 확인 타이머 시작
            if self.door_status_timer is None:
                self.door_status_timer = self.create_timer(
                    1.0,  # 1Hz
                    self.wait_for_door_open,
                    callback_group=self.callback_group
                )
            
        elif self.current_state == "DOOR_OPENED":
            self.get_logger().info("✅ 문 열림 확인")
            self.send_boarding_event()  # 이 함수에서 상태를 BOARDING_EVENT_SENT로 변경
            
        elif self.current_state == "BOARDING_EVENT_SENT":
            self.current_state = "MOVING_TO_ELEVATOR_INTERIOR"
            self.get_logger().info("📱 탑승 이벤트 발송 완료")
            self.get_logger().info("🚶 엘리베이터 내부로 이동 시작")
            self.move_to_elevator_interior()
            
        elif self.current_state == "ELEVATOR_INTERIOR_REACHED":
            self.current_state = "SCENARIO_3_READY"
            self.get_logger().info("✅ 엘리베이터 내부 도달")
            self.get_logger().info("🎉 시나리오 3 준비 완료 - 엘리베이터 내부 버튼 조작 대기")
            
        elif self.current_state == "SCENARIO_3_READY":
            self.current_state = "SETTING_ELEVATOR_INTERIOR_MODE"
            self.get_logger().info("🚀 시나리오 3 시작 - 엘리베이터 내부 모드 설정")
            self.set_elevator_interior_mode()
            
        elif self.current_state == "ELEVATOR_INTERIOR_MODE_SET":
            self.current_state = "SENDING_INTERIOR_GUI_WARNING"
            self.get_logger().info("✅ 엘리베이터 내부 모드 설정 완료")
            self.send_interior_gui_warning()
            
        elif self.current_state == "INTERIOR_GUI_WARNING_SENT":
            self.current_state = "CLICKING_INTERIOR_BUTTON"
            self.get_logger().info("📱 내부 GUI 경고 발송 완료")
            self.click_interior_button()
            
        elif self.current_state == "INTERIOR_BUTTON_CLICKED":
            self.current_state = "SETTING_ARM_UPWARD"
            self.get_logger().info("✅ 내부 버튼 클릭 완료")
            self.set_arm_upward()
            
        elif self.current_state == "ARM_UPWARD_SET":
            self.current_state = "SENDING_INTERIOR_MOVEMENT_EVENT"
            self.get_logger().info("✅ 팔 상향 설정 완료")
            self.send_interior_movement_event()
            
        elif self.current_state == "INTERIOR_MOVEMENT_EVENT_SENT":
            self.current_state = "SCENARIO_4_READY"
            self.get_logger().info("📱 내부 이동 이벤트 발송 완료")
            self.get_logger().info("🎉 시나리오 3 완료 및 시나리오 4 준비 완료")
            
        elif self.current_state == "SCENARIO_4_READY":
            self.current_state = "CHECKING_ELEVATOR_ARRIVAL"
            self.get_logger().info("🚀 시나리오 4 시작 - 엘리베이터 도착 확인")
            self.elevator_arrival_start_time = time.time()
            
            # 엘리베이터 도착 확인 타이머 시작
            if self.elevator_arrival_timer is None:
                self.elevator_arrival_timer = self.create_timer(
                    1.0,  # 1Hz
                    self.check_elevator_arrival,
                    callback_group=self.callback_group
                )
            
        elif self.current_state == "ELEVATOR_ARRIVED_EXIT":
            self.current_state = "WAITING_FOR_DOOR_OPEN_EXIT"
            self.get_logger().info("✅ 엘리베이터 도착 확인 (6층)")
            self.get_logger().info("🚪 문 열림 대기 시작")
            self.door_status_exit_start_time = time.time()
            
            # 엘리베이터 도착 확인 타이머 정지
            if self.elevator_arrival_timer:
                self.elevator_arrival_timer.cancel()
                self.elevator_arrival_timer = None
            
            # 문 상태 확인 타이머 시작
            if self.door_status_exit_timer is None:
                self.door_status_exit_timer = self.create_timer(
                    1.0,  # 1Hz
                    self.wait_for_door_open_exit,
                    callback_group=self.callback_group
                )
            
        elif self.current_state == "WAITING_FOR_DOOR_OPEN_EXIT":
            # 타이머가 취소되었거나 상태가 변경된 경우 처리하지 않음
            if self.door_status_exit_timer is None:
                return
            # 타이머 안전성 체크 추가
            if self.current_state != "WAITING_FOR_DOOR_OPEN_EXIT":
                return
            self.wait_for_door_open_exit()
            
        elif self.current_state == "DOOR_OPENED_EXIT":
            self.current_state = "SENDING_EXIT_EVENT"
            
            # 문 상태 확인 타이머 정지
            if self.door_status_exit_timer:
                self.door_status_exit_timer.cancel()
                self.door_status_exit_timer = None
            
            self.send_exit_event()
            
        elif self.current_state == "EXIT_EVENT_SENT":
            self.current_state = "EXITING_ELEVATOR"
            self.get_logger().info("🚶 엘리베이터 외부로 이동 시작")
            self.exit_elevator()
            
        elif self.current_state == "ELEVATOR_EXIT_COMPLETED":
            self.current_state = "RESTORING_NORMAL_MODE"
            self.get_logger().info("✅ 엘리베이터 하차 완료")
            self.get_logger().info("🔄 VS 모드 복원 시작")
            self.restore_normal_mode()
            
        elif self.current_state == "NORMAL_MODE_RESTORED":
            self.current_state = "COMPLETED"
            self.get_logger().info("🎉 전체 시나리오 완료")
            
            # 모든 타이머 정지
            self.scenario_timer.cancel()
            self.location_timer.cancel()
            self.button_tracking_timer.cancel()
            if self.backup_timer:
                self.backup_timer.cancel()
                self.backup_timer = None
            if self.elevator_status_timer:
                self.elevator_status_timer.cancel()
                self.elevator_status_timer = None
            if self.door_status_timer:
                self.door_status_timer.cancel()
                self.door_status_timer = None
            if self.elevator_arrival_timer:
                self.elevator_arrival_timer.cancel()
                self.elevator_arrival_timer = None
            if self.door_status_exit_timer:
                self.door_status_exit_timer.cancel()
                self.door_status_exit_timer = None
            
        elif self.current_state == "COMPLETED":
            # 시나리오 완료 - 타이머 정지
            self.scenario_timer.cancel()
            self.location_timer.cancel()
            self.button_tracking_timer.cancel()
            self.get_logger().info("🎉 시나리오 완료")
    
    def set_arm_forward(self):
        """팔 정면 설정 액션 실행"""
        goal = SetPose.Goal()
        goal.robot_id = self.robot_id
        goal.pose_id = 4  # 정면 바라보기
        
        self.get_logger().info(f"🤖 팔 정면 설정 액션 시작 - robot_id: {goal.robot_id}, pose_id: {goal.pose_id}")
        self.get_logger().info(f"🔗 액션 서버 주소: {ARM_SET_POSE_ACTION}")
        
        try:
            self.set_pose_client.send_goal_async(goal).add_done_callback(self.set_arm_forward_callback)
            self.get_logger().info("📤 액션 goal 전송 완료")
        except Exception as e:
            self.get_logger().error(f"❌ 액션 goal 전송 실패: {e}")
            self.current_state = "ERROR"
    
    def set_arm_forward_callback(self, future):
        """팔 정면 설정 액션 완료 처리"""
        try:
            goal_handle = future.result()
            self.get_logger().info(f"📥 액션 응답 수신 - accepted: {goal_handle.accepted}")
            
            if goal_handle.accepted:
                self.get_logger().info("✅ 팔 정면 설정 액션 수락됨")
                self.get_logger().info("⏳ 액션 실행 중...")
                goal_handle.get_result_async().add_done_callback(self.set_arm_forward_result_callback)
            else:
                self.get_logger().error("❌ 팔 정면 설정 액션 거부됨")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"❌ 팔 정면 설정 액션 오류: {e}")
            self.current_state = "ERROR"
    
    def set_arm_forward_result_callback(self, future):
        """팔 정면 설정 결과 처리"""
        try:
            result_response = future.result()
            result = result_response.result
            self.get_logger().info(f"📋 액션 결과 수신 - robot_id: {result.robot_id}, success: {result.success}")
            
            if result.success:
                self.get_logger().info("✅ 팔 정면 설정 완료 - 위치 확인 시작")
                self.current_state = "CHECKING_LOCATION"
                self.location_check_start_time = time.time()
            else:
                self.get_logger().error("❌ 팔 정면 설정 실패")
                self.get_logger().error("🔍 액션 서버에서 실패 응답을 받았습니다. 서버 로그를 확인해주세요.")
                # 실패해도 계속 진행 (디버그 모드에서는 실패해도 테스트 계속)
                if self.debug_mode:
                    self.get_logger().info("🔧 디버그 모드: 실패해도 계속 진행")
                    self.current_state = "CHECKING_LOCATION"
                    self.location_check_start_time = time.time()
                else:
                    self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"❌ 팔 정면 설정 결과 처리 오류: {e}")
            self.current_state = "ERROR"
    
    def check_location(self):
        """VS로부터 현재 위치 확인"""
        try:
            if self.current_state != "CHECKING_LOCATION":
                return
                
            # 타임아웃 체크
            elapsed_time = time.time() - self.location_check_start_time
            if elapsed_time > self.location_check_timeout:
                self.get_logger().error(f"❌ 위치 확인 타임아웃 ({self.location_check_timeout}초)")
                self.current_state = "ERROR"
                return
                
            # VS 서비스 연결 상태 확인
            service_ready = self.location_client.service_is_ready()
            if not service_ready:
                self.get_logger().error(f"❌ VS Location 서비스가 준비되지 않음")
                return
                
            # Location 서비스 요청
            request = Location.Request()
            request.robot_id = self.robot_id
            
            self.get_logger().info(f"📍 위치 확인 #{self.location_check_count + 1} - robot_id: {request.robot_id}")
            
            future = self.location_client.call_async(request)
            future.add_done_callback(self.location_callback)
            
        except Exception as e:
            self.get_logger().error(f"❌ check_location 오류: {e}")
            self.current_state = "ERROR"
            return
        
        self.location_check_count += 1
    
    def location_callback(self, future):
        """위치 확인 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                if response.location_id == self.location_id:
                    self.get_logger().info(f"✅ 도착지 도착: {response.location_id}")
                    self.current_state = "LOCATION_CONFIRMED"
                else:
                    self.get_logger().info(f"📍 현재 위치: {response.location_id} (목표: {self.location_id})")
            else:
                self.get_logger().info("📍 아루코 마커 인식 실패")
                
        except Exception as e:
            self.get_logger().error(f"❌ location_callback 오류: {e}")
    
    def set_elevator_mode(self):
        """VS를 엘리베이터 모드로 설정"""
        request = SetVSMode.Request()
        request.robot_id = self.robot_id
        request.mode_id = ELEVATOR_EXTERNAL_MODE  # 엘리베이터 외부 모드
        
        self.get_logger().info("🎯 엘리베이터 모드 설정 요청")
        
        future = self.set_vs_mode_client.call_async(request)
        future.add_done_callback(self.set_elevator_mode_callback)
    
    def set_elevator_mode_callback(self, future):
        """엘리베이터 모드 설정 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info("✅ 모드 설정 완료")
                self.current_state = "ELEVATOR_MODE_SET"
            else:
                self.get_logger().error("❌ 모드 설정 실패")
                self.current_state = "ERROR"
                
        except Exception as e:
            self.get_logger().error(f"엘리베이터 모드 설정 중 오류: {str(e)}")
            self.current_state = "ERROR"
    
    def send_gui_warning(self):
        """GUI에 엘리베이터 버튼 조작 시작 이벤트 발송"""
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = GUI_EVENT_BUTTON_OPERATION_START  # 엘리베이터 버튼 조작 시작
        msg.task_id = self.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = f"엘리베이터 버튼 조작 시작 - {self.current_floor}층 → {self.target_floor}층"
        
        self.gui_event_pub.publish(msg)
        self.get_logger().info("📱 GUI 이벤트 발송 완료")
    
    def track_button(self):
        """버튼 추적 및 주행 제어"""
        if self.current_state != "TRACKING_BUTTON":
            return
            
        # ButtonStatus 서비스 요청
        request = ButtonStatus.Request()
        request.robot_id = self.robot_id
        request.button_id = TARGET_BUTTON_ID
        
        future = self.button_status_client.call_async(request)
        future.add_done_callback(self.button_status_callback)
    
    def button_status_callback(self, future):
        """버튼 상태 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                x = response.x
                y = response.y
                size = response.size
                is_pressed = response.is_pressed
                
                # 버튼이 감지되었고 유효한 경우
                if size > 0.0001:  # 최소 크기 체크
                    # 주행 제어 명령 계산
                    cmd_vel = self.calculate_control_command(x, y, size)
                    
                    # 목표 달성 확인
                    if self.is_target_reached(x, y, size):
                        self.get_logger().info(f"🎯 목표 위치 도달: x={x:.3f}, size={size:.3f}")
                        self.stop_robot()
                        self.current_state = "BUTTON_REACHED"
                        # 버튼 추적 타이머 정지
                        if self.button_tracking_timer:
                            self.button_tracking_timer.cancel()
                            self.button_tracking_timer = None
                        return
                    
                    # 제어 명령 발행
                    self.cmd_vel_pub.publish(cmd_vel)
                else:
                    self.stop_robot()
            else:
                self.stop_robot()
                
        except Exception as e:
            self.get_logger().error(f"버튼 상태 처리 오류: {e}")
            self.stop_robot()
    
    def calculate_control_command(self, x, y, size):
        """주행 제어 명령 계산"""
        cmd_vel = Twist()
        
        # 크기 기반 전진/후진 제어
        size_error = TARGET_BUTTON_SIZE - size
        linear_speed = KP_SIZE * size_error
        
        # x좌표 기반 회전 제어
        x_error = TARGET_BUTTON_X - x
        angular_speed = KP_X * x_error
        
        # 속도 제한
        if linear_speed > 0:  # 전진할 때
            linear_speed = max(MIN_LINEAR_SPEED, min(linear_speed, MAX_LINEAR_SPEED))
        else:  # 후진할 때
            linear_speed = max(-MAX_LINEAR_SPEED, min(linear_speed, -MIN_LINEAR_SPEED))
        
        angular_speed = max(-MAX_ANGULAR_SPEED, min(angular_speed, MAX_ANGULAR_SPEED))
        
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        
        self.get_logger().info(f'버튼: x={x:.3f}, size={size:.3f} | 제어: linear={linear_speed:.3f}, angular={angular_speed:.3f}')
        
        return cmd_vel
    
    def is_target_reached(self, x, y, size):
        """목표 위치 도달 확인"""
        size_error = abs(TARGET_BUTTON_SIZE - size)
        x_error = abs(TARGET_BUTTON_X - x)
        
        return size_error < TARGET_SIZE_ERROR and x_error < TARGET_X_ERROR
    
    def stop_robot(self):
        """로봇 정지"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def click_button(self):
        """버튼 클릭 액션 실행"""
        goal = ClickButton.Goal()
        goal.robot_id = self.robot_id
        goal.button_id = TARGET_BUTTON_ID
        
        self.get_logger().info(f"🎯 버튼 클릭 액션 시작: {TARGET_BUTTON_ID}")
        
        self.click_button_client.send_goal_async(goal).add_done_callback(self.click_button_callback)
    
    def click_button_callback(self, future):
        """버튼 클릭 액션 완료 처리"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("✅ 버튼 클릭 액션 수락됨")
                goal_handle.get_result_async().add_done_callback(self.click_button_result_callback)
            else:
                self.get_logger().error("❌ 버튼 클릭 액션 거부됨")
        except Exception as e:
            self.get_logger().error(f"버튼 클릭 액션 오류: {e}")
    
    def click_button_result_callback(self, future):
        """버튼 클릭 결과 처리"""
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(f"✅ 버튼 클릭 완료: {result.message}")
                self.current_state = "BUTTON_CLICKED"
            else:
                self.get_logger().error(f"❌ 버튼 클릭 실패: {result.message}")
                # 실패 시에도 디버그 모드에서는 계속 진행
                if self.debug_mode:
                    self.get_logger().info("🔧 디버그 모드: 실패해도 계속 진행")
                    self.current_state = "BUTTON_CLICKED"
                else:
                    self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"버튼 클릭 결과 처리 오류: {e}")
            self.current_state = "ERROR"
    
    def return_arm(self):
        """팔 원위치 액션 실행"""
        goal = SetPose.Goal()
        goal.robot_id = self.robot_id
        goal.pose_id = 5  # 전광판 상향 
        
        self.get_logger().info("🤖 팔 원위치 액션 시작")
        
        self.set_pose_client.send_goal_async(goal).add_done_callback(self.return_arm_callback)
    
    def return_arm_callback(self, future):
        """팔 원위치 액션 완료 처리"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("✅ 팔 원위치 액션 수락됨")
                goal_handle.get_result_async().add_done_callback(self.return_arm_result_callback)
            else:
                self.get_logger().error("❌ 팔 원위치 액션 거부됨")
        except Exception as e:
            self.get_logger().error(f"팔 원위치 액션 오류: {e}")
    
    def return_arm_result_callback(self, future):
        """팔 원위치 결과 처리"""
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info("✅ 팔 원위치 완료")
                self.current_state = "ARM_RETURNED"
            else:
                self.get_logger().error("❌ 팔 원위치 실패")
        except Exception as e:
            self.get_logger().error(f"팔 원위치 결과 처리 오류: {e}")
    
    def send_movement_event(self):
        """GUI에 이동 시작 이벤트 발송"""
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = GUI_EVENT_MOVEMENT_START  # 이동 시작
        msg.task_id = self.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = f"이동 시작 - {self.current_floor}층 → {self.target_floor}층"
        
        self.gui_event_pub.publish(msg)
        self.get_logger().info("📱 이동 이벤트 발송 완료")
    
    def start_backup_movement(self):
        """후진 이동 시작"""
        self.backup_start_time = time.time()
        
        # 후진 이동 타이머 생성 (10Hz)
        if self.backup_timer is None:
            self.backup_timer = self.create_timer(
                0.1,  # 10Hz
                self.backup_movement_loop,
                callback_group=self.callback_group
            )
        
        self.get_logger().info(f"🚗 후진 이동 시작: 목표 거리 {BACKUP_DISTANCE}m")
    
    def backup_movement_loop(self):
        """후진 이동 제어 루프"""
        if self.current_state != "BACKING_UP":
            return
            
        # 디버그 모드일 때: 바로 완료
        if self.debug_mode:
            self.get_logger().info("🔧 디버그 모드: 후진 이동 시뮬레이션 완료")
            self.stop_robot()
            self.current_state = "BACKUP_COMPLETED"
            # 타이머 정지
            if self.backup_timer:
                self.backup_timer.cancel()
                self.backup_timer = None
            return
            
        # 실제 주행 모드일 때: 시간 기반 거리 측정
        # 타임아웃 체크 (10초 후 다음 단계로 진행)
        if time.time() - self.backup_start_time > BACKUP_TIMEOUT:
            self.get_logger().info(f"⏰ 후진 이동 타임아웃 ({BACKUP_TIMEOUT}초) - 다음 단계로 진행")
            self.stop_robot()
            self.current_state = "BACKUP_COMPLETED"
            # 타이머 정지
            if self.backup_timer:
                self.backup_timer.cancel()
                self.backup_timer = None
            return
            
        # 시간 기반 거리 계산
        elapsed_time = time.time() - self.backup_start_time
        current_distance = BACKUP_SPEED * elapsed_time
        
        if current_distance >= BACKUP_DISTANCE:
            # 목표 거리 도달
            self.stop_robot()
            self.get_logger().info(f"✅ 후진 이동 완료: {current_distance:.3f}m")
            self.current_state = "BACKUP_COMPLETED"
            # self.current_state = "COMPLETED"  # 강제 완료
            # 타이머 정지
            if self.backup_timer:
                self.backup_timer.cancel()
                self.backup_timer = None
        else:
            # 후진 명령 발행
            cmd_vel = Twist()
            cmd_vel.linear.x = -BACKUP_SPEED  # 후진
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
    
    def move_to_elevator_center(self):
        """엘리베이터 중앙으로 이동 (simple_navigator2 사용)"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 엘리베이터 중앙 위치 설정
        goal_pose.pose.position.x = ELEVATOR_CENTER_X
        goal_pose.pose.position.y = ELEVATOR_CENTER_Y
        goal_pose.pose.position.z = 0.0
        
        # 방향 설정 (쿼터니언으로 변환)
        q = tf_transformations.quaternion_from_euler(0, 0, ELEVATOR_CENTER_YAW)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        
        # simple_navigator2에 목표 전송
        self.simple_goal_pub.publish(goal_pose)
        self.get_logger().info(f"🎯 엘리베이터 중앙 목표 전송: ({ELEVATOR_CENTER_X}, {ELEVATOR_CENTER_Y})")
        
        # Simple Navigator 모니터링 시작
        self.simple_nav_start_time = time.time()
        if self.simple_nav_monitor_timer is None:
            self.simple_nav_monitor_timer = self.create_timer(
                1.0,  # 1Hz로 모니터링
                self.monitor_simple_navigator,
                callback_group=self.callback_group
            )
    
    def simple_nav_status_callback(self, msg):
        """Simple Navigator 상태 콜백"""
        if self.current_state == "MOVING_TO_ELEVATOR_CENTER":
            if msg.data == "completed":
                self.get_logger().info("✅ Simple Navigator 완료 신호 수신")
                self.current_state = "ELEVATOR_CENTER_REACHED"
                # 타이머 정지
                if self.simple_nav_monitor_timer:
                    self.simple_nav_monitor_timer.cancel()
                    self.simple_nav_monitor_timer = None
            else:
                self.get_logger().info(f"🎯 Simple Navigator 상태: {msg.data}")
    
    def elevator_center_reached_callback(self):
        """엘리베이터 중앙 도달 콜백"""
        self.get_logger().info("✅ 엘리베이터 중앙 도달 완료")

        self.current_state = "ELEVATOR_CENTER_REACHED"
        # self.current_state = "COMPLETED"
    
    def monitor_simple_navigator(self):
        """Simple Navigator 모니터링 타이머 (타임아웃 처리용)"""
        if self.current_state != "MOVING_TO_ELEVATOR_CENTER":
            return
            
        # 디버그 모드일 때: 바로 완료
        if self.debug_mode:
            self.get_logger().info("🔧 디버그 모드: Simple Navigator 시뮬레이션 완료")
            self.current_state = "ELEVATOR_CENTER_REACHED"
            # 타이머 정지
            if self.simple_nav_monitor_timer:
                self.simple_nav_monitor_timer.cancel()
                self.simple_nav_monitor_timer = None
            return
            
        # 실제 주행 모드일 때: 타임아웃만 체크 (완료는 콜백에서 처리)
        if time.time() - self.simple_nav_start_time > self.simple_nav_timeout:
            self.get_logger().error(f"Simple Navigator 타임아웃 ({self.simple_nav_timeout}초)")
            self.current_state = "ERROR"
            # 타이머 정지
            if self.simple_nav_monitor_timer:
                self.simple_nav_monitor_timer.cancel()
                self.simple_nav_monitor_timer = None
    
    # ===== Scenario 2 Functions =====
    
    def check_elevator_status(self):
        """엘리베이터 상태 확인 (도착 판단)"""
        # 타이머가 취소되었거나 상태가 변경된 경우 처리하지 않음
        if self.current_state != "CHECKING_ELEVATOR_STATUS" or self.elevator_status_timer is None:
            return
            
        # 타임아웃 체크
        if time.time() - self.elevator_status_start_time > ELEVATOR_STATUS_TIMEOUT:
            self.get_logger().error(f"엘리베이터 상태 확인 타임아웃 ({ELEVATOR_STATUS_TIMEOUT}초)")
            self.current_state = "ERROR"
            return
            
        # ElevatorStatus 서비스 요청
        request = ElevatorStatus.Request()
        request.robot_id = self.robot_id
        
        self.get_logger().info(f"🛗 엘리베이터 상태 확인 요청 #{self.elevator_status_count + 1}")
        
        future = self.elevator_status_client.call_async(request)
        future.add_done_callback(self.elevator_status_callback)
        
        self.elevator_status_count += 1
    
    def elevator_status_callback(self, future):
        """엘리베이터 상태 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                # 현재 층과 엘리베이터 위치 비교
                if response.position == self.current_floor:
                    # 방향 확인 (5층→6층이므로 upward=0이어야 함)
                    expected_direction = ELEVATOR_DIRECTION_UPWARD if self.target_floor > self.current_floor else ELEVATOR_DIRECTION_DOWNWARD
                    
                    if response.direction == expected_direction:
                        self.get_logger().info(f"✅ 엘리베이터 도착 확인: 층={response.position}, 방향={response.direction}")
                        self.current_state = "ELEVATOR_ARRIVED"
                    else:
                        self.get_logger().info(f"🛗 엘리베이터 위치 일치, 방향 불일치: 층={response.position}, 방향={response.direction}")
                        # 계속 확인 (타이머가 다시 호출)
                else:
                    self.get_logger().info(f"🛗 엘리베이터 위치 불일치: 현재층={self.current_floor}, 엘리베이터층={response.position}")
                    # 계속 확인 (타이머가 다시 호출)
            else:
                self.get_logger().info("🛗 엘리베이터 상태 확인 실패")
                # 계속 확인 (타이머가 다시 호출)
                
        except Exception as e:
            self.get_logger().error(f"엘리베이터 상태 확인 중 오류: {str(e)}")
    
    def wait_for_door_open(self):
        """문 열림 대기"""
        # 타이머가 취소되었거나 상태가 변경된 경우 처리하지 않음
        if self.current_state != "WAITING_FOR_DOOR_OPEN" or self.door_status_timer is None:
            return
            
        # 타임아웃 체크
        if time.time() - self.door_status_start_time > DOOR_STATUS_TIMEOUT:
            self.get_logger().error(f"문 열림 대기 타임아웃 ({DOOR_STATUS_TIMEOUT}초)")
            self.current_state = "ERROR"
            return
            
        # DoorStatus 서비스 요청
        request = DoorStatus.Request()
        request.robot_id = self.robot_id
        
        self.get_logger().info(f"🚪 문 상태 확인 요청 #{self.door_status_count + 1}")
        
        future = self.door_status_client.call_async(request)
        future.add_done_callback(self.door_status_callback)
        
        self.door_status_count += 1
    
    def door_status_callback(self, future):
        """문 상태 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                if response.door_opened:
                    self.get_logger().info("✅ 문 열림 확인")
                    self.current_state = "DOOR_OPENED"
                else:
                    self.get_logger().info("🚪 문이 아직 닫힌 상태")
                    # 계속 확인 (타이머가 다시 호출)
            else:
                self.get_logger().info("🚪 문 상태 확인 실패")
                # 계속 확인 (타이머가 다시 호출)
                
        except Exception as e:
            self.get_logger().error(f"문 상태 확인 중 오류: {str(e)}")
    
    def send_boarding_event(self):
        """GUI에 엘리베이터 탑승 시작 이벤트 발송"""
        """GUI에 엘리베이터 탑승 시작 이벤트 발송"""
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = GUI_EVENT_ELEVATOR_BOARDING_START  # 엘리베이터 탑승 시작
        msg.task_id = self.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = f"엘리베이터 탑승 시작 - {self.current_floor}층 → {self.target_floor}층"
        
        self.gui_event_pub.publish(msg)
        self.get_logger().info("📱 탑승 이벤트 발송 완료")
        self.current_state = "BOARDING_EVENT_SENT"
    
    def move_to_elevator_interior(self):
        """엘리베이터 내부로 이동 (Nav2 사용)"""
        # 디버그 모드일 때: 바로 완료
        if self.debug_mode:
            self.get_logger().info("🔧 디버그 모드: Nav2 시뮬레이션 완료")
            self.current_state = "ELEVATOR_INTERIOR_REACHED"
            return
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 엘리베이터 내부 위치 설정
        goal.pose.pose.position.x = ELEVATOR_INTERIOR_X
        goal.pose.pose.position.y = ELEVATOR_INTERIOR_Y
        goal.pose.pose.position.z = 0.0
        
        # 방향 설정 (쿼터니언으로 변환)
        q = tf_transformations.quaternion_from_euler(0, 0, ELEVATOR_INTERIOR_YAW)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        
        # Nav2 Action 전송
        self.get_logger().info(f"🎯 엘리베이터 내부 Nav2 목표 전송: ({ELEVATOR_INTERIOR_X}, {ELEVATOR_INTERIOR_Y})")
        
        self.nav2_client.send_goal_async(goal).add_done_callback(self.nav2_goal_callback)
    
    def nav2_goal_callback(self, future):
        """Nav2 목표 전송 완료 콜백"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("✅ Nav2 목표 수락됨")
                goal_handle.get_result_async().add_done_callback(self.nav2_result_callback)
            else:
                self.get_logger().error("❌ Nav2 목표 거부됨")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"Nav2 목표 전송 오류: {e}")
            self.current_state = "ERROR"
    
    def nav2_result_callback(self, future):
        """Nav2 결과 처리 콜백"""
        try:
            result = future.result().result
            if result.error_code == 0:  # SUCCESS
                self.get_logger().info("✅ Nav2 이동 완료")
                self.current_state = "ELEVATOR_INTERIOR_REACHED"
            else:
                self.get_logger().error(f"❌ Nav2 이동 실패: error_code={result.error_code}")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"Nav2 결과 처리 오류: {e}")
            self.current_state = "ERROR"
    
    def set_elevator_interior_mode(self):
        """엘리베이터 내부 모드 설정"""
        request = SetVSMode.Request()
        request.robot_id = self.robot_id
        request.mode_id = 4  # 엘리베이터 내부 모드
        
        self.get_logger().info("🎯 엘리베이터 내부 모드 설정 요청")
        
        future = self.set_vs_mode_client.call_async(request)
        future.add_done_callback(self.set_elevator_interior_mode_callback)
    
    def set_elevator_interior_mode_callback(self, future):
        """엘리베이터 내부 모드 설정 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.current_state = "ELEVATOR_INTERIOR_MODE_SET"
            else:
                self.get_logger().error("❌ 엘리베이터 내부 모드 설정 실패")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"엘리베이터 내부 모드 설정 중 오류: {str(e)}")
            self.current_state = "ERROR"
    
    def send_interior_gui_warning(self):
        """GUI에 엘리베이터 내부 버튼 조작 시작 이벤트 발송"""
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = GUI_EVENT_BUTTON_OPERATION_START  # 엘리베이터 버튼 조작 시작
        msg.task_id = self.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = f"엘리베이터 내부 버튼 조작 시작 - {self.current_floor}층 → {self.target_floor}층"
        
        self.gui_event_pub.publish(msg)
        self.current_state = "INTERIOR_GUI_WARNING_SENT"
    
    def click_interior_button(self):
        """엘리베이터 내부 버튼 클릭 (추적 없이)"""
        goal = ClickButton.Goal()
        goal.robot_id = self.robot_id
        goal.button_id = 6  # 6층 버튼
        
        self.get_logger().info("🎯 엘리베이터 내부 버튼 클릭 액션 시작")
        
        self.click_button_client.send_goal_async(goal).add_done_callback(self.click_interior_button_callback)
    
    def click_interior_button_callback(self, future):
        """엘리베이터 내부 버튼 클릭 액션 완료 처리"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("✅ 엘리베이터 내부 버튼 클릭 액션 수락됨")
                goal_handle.get_result_async().add_done_callback(self.click_interior_button_result_callback)
            else:
                self.get_logger().error("❌ 엘리베이터 내부 버튼 클릭 액션 거부됨")
        except Exception as e:
            self.get_logger().error(f"엘리베이터 내부 버튼 클릭 액션 오류: {e}")
    
    def click_interior_button_result_callback(self, future):
        """엘리베이터 내부 버튼 클릭 결과 처리"""
        try:
            result = future.result().result
            if result.success:
                self.current_state = "INTERIOR_BUTTON_CLICKED"
            else:
                self.get_logger().error("❌ 엘리베이터 내부 버튼 클릭 실패")
        except Exception as e:
            self.get_logger().error(f"엘리베이터 내부 버튼 클릭 결과 처리 오류: {e}")
    
    def set_arm_upward(self):
        """팔 상향 설정 액션 실행"""
        goal = SetPose.Goal()
        goal.robot_id = self.robot_id
        goal.pose_id = 5  # 상향 바라보기 (전광판 인식용)
        
        self.get_logger().info("🤖 팔 상향 설정 액션 시작")
        
        self.set_pose_client.send_goal_async(goal).add_done_callback(self.set_arm_upward_callback)
    
    def set_arm_upward_callback(self, future):
        """팔 상향 설정 액션 완료 처리"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("✅ 팔 상향 설정 액션 수락됨")
                goal_handle.get_result_async().add_done_callback(self.set_arm_upward_result_callback)
            else:
                self.get_logger().error("❌ 팔 상향 설정 액션 거부됨")
        except Exception as e:
            self.get_logger().error(f"팔 상향 설정 액션 오류: {e}")
    
    def set_arm_upward_result_callback(self, future):
        """팔 상향 설정 결과 처리"""
        try:
            result = future.result().result
            if result.success:
                self.current_state = "ARM_UPWARD_SET"
            else:
                self.get_logger().error("❌ 팔 상향 설정 실패")
        except Exception as e:
            self.get_logger().error(f"팔 상향 설정 결과 처리 오류: {e}")
    
    def send_interior_movement_event(self):
        """GUI에 엘리베이터 내부 이동 시작 이벤트 발송"""
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = GUI_EVENT_MOVEMENT_START  # 이동 시작
        msg.task_id = self.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = f"엘리베이터 내부 이동 시작 - {self.current_floor}층 → {self.target_floor}층"
        
        self.gui_event_pub.publish(msg)
        self.current_state = "INTERIOR_MOVEMENT_EVENT_SENT"
    
    # ===== 시나리오 4 메서드들 =====
    
    def check_elevator_arrival(self):
        """엘리베이터 도착 확인 (6층 도착 판단)"""
        # 타이머가 취소되었거나 상태가 변경된 경우 처리하지 않음
        if self.current_state != "CHECKING_ELEVATOR_ARRIVAL" or self.elevator_arrival_timer is None:
            return
            
        # 타임아웃 체크
        if time.time() - self.elevator_arrival_start_time > ELEVATOR_ARRIVAL_TIMEOUT:
            self.get_logger().error(f"엘리베이터 도착 확인 타임아웃 ({ELEVATOR_ARRIVAL_TIMEOUT}초)")
            self.current_state = "ERROR"
            return
            
        # ElevatorStatus 서비스 요청
        request = ElevatorStatus.Request()
        request.robot_id = self.robot_id
        
        self.get_logger().info(f"🛗 엘리베이터 도착 확인 요청 #{self.elevator_arrival_count + 1}")
        
        future = self.elevator_status_client.call_async(request)
        future.add_done_callback(self.elevator_arrival_callback)
        
        self.elevator_arrival_count += 1
    
    def elevator_arrival_callback(self, future):
        """엘리베이터 도착 확인 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                # 목적지 층(6층)과 엘리베이터 위치 비교
                if response.position == self.target_floor:
                    self.get_logger().info(f"✅ 엘리베이터 도착 확인: 목적지 {self.target_floor}층 도착")
                    self.current_state = "ELEVATOR_ARRIVED_EXIT"
                else:
                    self.get_logger().info(f"🛗 엘리베이터 위치 불일치: 목적지={self.target_floor}층, 현재={response.position}층")
                    # 계속 확인 (타이머가 다시 호출)
            else:
                self.get_logger().info("🛗 엘리베이터 도착 확인 실패")
                # 계속 확인 (타이머가 다시 호출)
                
        except Exception as e:
            self.get_logger().error(f"엘리베이터 도착 확인 중 오류: {str(e)}")
    
    def wait_for_door_open_exit(self):
        """문 열림 대기 (6층에서)"""
        # 타이머가 취소되었거나 상태가 변경된 경우 처리하지 않음
        if self.current_state != "WAITING_FOR_DOOR_OPEN_EXIT" or self.door_status_exit_timer is None:
            return
            
        # 타임아웃 체크
        if time.time() - self.door_status_exit_start_time > DOOR_STATUS_TIMEOUT:
            self.get_logger().error(f"문 열림 대기 타임아웃 ({DOOR_STATUS_TIMEOUT}초)")
            self.current_state = "ERROR"
            return
            
        # DoorStatus 서비스 요청
        request = DoorStatus.Request()
        request.robot_id = self.robot_id
        
        self.get_logger().info(f"🚪 문 상태 확인 요청 #{self.door_status_exit_count + 1}")
        
        future = self.door_status_client.call_async(request)
        future.add_done_callback(self.door_status_exit_callback)
        
        self.door_status_exit_count += 1
    
    def door_status_exit_callback(self, future):
        """문 상태 응답 처리 (6층에서)"""
        try:
            response = future.result()
            
            if response.success:
                if response.door_opened:
                    self.get_logger().info("✅ 문 열림 확인 (6층)")
                    self.current_state = "DOOR_OPENED_EXIT"
                else:
                    self.get_logger().info("🚪 문이 아직 닫힌 상태 (6층)")
                    # 계속 확인 (타이머가 다시 호출)
            else:
                self.get_logger().info("🚪 문 상태 확인 실패 (6층)")
                # 계속 확인 (타이머가 다시 호출)
                
        except Exception as e:
            self.get_logger().error(f"문 상태 확인 중 오류: {str(e)}")
    
    def send_exit_event(self):
        """GUI에 엘리베이터 하차 시작 이벤트 발송"""
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = GUI_EVENT_ELEVATOR_EXIT_START  # 엘리베이터 하차 시작
        msg.task_id = self.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = f"엘리베이터 하차 시작 - {self.target_floor}층"
        
        self.gui_event_pub.publish(msg)
        self.get_logger().info("📱 하차 이벤트 발송 완료")
        self.current_state = "EXIT_EVENT_SENT"
    
    def exit_elevator(self):
        """엘리베이터 외부로 이동 (Nav2 사용)"""
        # 디버그 모드일 때: 바로 완료
        if self.debug_mode:
            self.get_logger().info("🔧 디버그 모드: Nav2 시뮬레이션 완료")
            self.current_state = "ELEVATOR_EXIT_COMPLETED"
            return
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 엘리베이터 외부 위치 설정
        goal.pose.pose.position.x = ELEVATOR_EXIT_X
        goal.pose.pose.position.y = ELEVATOR_EXIT_Y
        goal.pose.pose.position.z = 0.0
        
        # 방향 설정 (쿼터니언으로 변환)
        q = tf_transformations.quaternion_from_euler(0, 0, ELEVATOR_EXIT_YAW)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        
        # Nav2 Action 전송
        self.get_logger().info(f"🎯 엘리베이터 외부 Nav2 목표 전송: ({ELEVATOR_EXIT_X}, {ELEVATOR_EXIT_Y})")
        
        self.nav2_client.send_goal_async(goal).add_done_callback(self.nav2_exit_goal_callback)
    
    def nav2_exit_goal_callback(self, future):
        """Nav2 목표 전송 완료 콜백 (하차용)"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("✅ Nav2 목표 수락됨 (하차)")
                goal_handle.get_result_async().add_done_callback(self.nav2_exit_result_callback)
            else:
                self.get_logger().error("❌ Nav2 목표 거부됨 (하차)")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"Nav2 목표 전송 오류 (하차): {e}")
            self.current_state = "ERROR"
    
    def nav2_exit_result_callback(self, future):
        """Nav2 결과 처리 콜백 (하차용)"""
        try:
            result = future.result().result
            if result.error_code == 0:  # SUCCESS
                self.get_logger().info("✅ 엘리베이터 하차 완료")
                self.current_state = "ELEVATOR_EXIT_COMPLETED"
            else:
                self.get_logger().error(f"❌ 엘리베이터 하차 실패: error_code={result.error_code}")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"Nav2 결과 처리 오류 (하차): {e}")
            self.current_state = "ERROR"
    
    def restore_normal_mode(self):
        """VS를 일반 주행 모드로 복원"""
        request = SetVSMode.Request()
        request.robot_id = self.robot_id
        request.mode_id = NORMAL_DRIVING_MODE  # 일반 주행 모드
        
        self.get_logger().info("🔄 VS 모드 복원 요청")
        
        future = self.set_vs_mode_client.call_async(request)
        future.add_done_callback(self.restore_normal_mode_callback)
    
    def restore_normal_mode_callback(self, future):
        """VS 모드 복원 응답 처리"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info("✅ VS 모드 복원 완료")
                self.current_state = "NORMAL_MODE_RESTORED"
            else:
                self.get_logger().error("❌ VS 모드 복원 실패")
                self.current_state = "ERROR"
                
        except Exception as e:
            self.get_logger().error(f"VS 모드 복원 중 오류: {str(e)}")
            self.current_state = "ERROR"
    
    def get_current_state(self):
        """현재 상태 반환"""
        return self.current_state

def main(args=None):
    rclpy.init(args=args)
    
    # 멀티스레드 실행자 생성
    executor = MultiThreadedExecutor()
    
    # 노드 생성 및 실행자에 추가
    node = RoomieECNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"❌ 시나리오 실행 중 오류 발생 - 종료: {e}")
    finally:
        # 타이머들 정리
        try:
            if hasattr(node, 'scenario_timer') and node.scenario_timer:
                node.scenario_timer.cancel()
            if hasattr(node, 'location_timer') and node.location_timer:
                node.location_timer.cancel()
            if hasattr(node, 'button_tracking_timer') and node.button_tracking_timer:
                node.button_tracking_timer.cancel()
            if hasattr(node, 'backup_timer') and node.backup_timer:
                node.backup_timer.cancel()
            if hasattr(node, 'simple_nav_monitor_timer') and node.simple_nav_monitor_timer:
                node.simple_nav_monitor_timer.cancel()
            if hasattr(node, 'elevator_status_timer') and node.elevator_status_timer:
                node.elevator_status_timer.cancel()
            if hasattr(node, 'door_status_timer') and node.door_status_timer:
                node.door_status_timer.cancel()
            if hasattr(node, 'elevator_arrival_timer') and node.elevator_arrival_timer:
                node.elevator_arrival_timer.cancel()
            if hasattr(node, 'door_status_exit_timer') and node.door_status_exit_timer:
                node.door_status_exit_timer.cancel()
        except Exception as e:
            node.get_logger().error(f"타이머 정리 중 오류: {e}")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 