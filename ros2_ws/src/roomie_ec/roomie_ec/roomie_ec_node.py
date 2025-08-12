#!/usr/bin/env python3
"""
Roomie Elevator Controller Node

이 노드는 로봇이 엘리베이터를 이용한 층간 이동을 수행하는 메인 컨트롤러입니다.
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
    """
    Roomie 엘리베이터 제어 노드
    
    이 클래스는 로봇이 엘리베이터를 이용하여 층간 이동을 수행하는 4개의 시나리오를 관리합니다:
    
    🏗️ 시나리오 1: 엘리베이터 외부 조작
       - 팔 설정 → 위치 확인 → 버튼 추적 → 클릭 → 후진 → 중앙 이동
    
    🛗 시나리오 2: 엘리베이터 탑승  
       - 엘리베이터 도착 확인 → 문 열림 대기 → 내부로 이동
    
    🎯 시나리오 3: 엘리베이터 내부 조작
       - 내부 모드 설정 → 층 버튼 클릭 → 중앙 정렬
    
    🚪 시나리오 4: 엘리베이터 하차
       - 목적지 도착 확인 → 문 열림 대기 → 외부로 이동 → 모드 복원
    
    각 시나리오는 current_state를 기반으로 순차적으로 실행되며,
    디버그 모드에서는 강제 완료 지점이 설정되어 있습니다.
    """
    
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
        self.is_backup_started = False  # 후진 이동 시작 플래그
        
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
        
        # 엘리베이터 중앙 이동용 변수들
        self.elevator_center_timer = None
        self.elevator_center_step = 0
        self.elevator_center_start_time = 0.0
        
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
        """메인 시나리오 단계별 실행 - 각 시나리오별 메서드로 분리"""
        # 시나리오 1: 엘리베이터 외부 조작 (INIT ~ SCENARIO_2_READY)
        if self.current_state in ["INIT", "SETTING_ARM_FORWARD", "LOCATION_CONFIRMED", 
                                  "ELEVATOR_MODE_SET", "SENDING_GUI_WARNING", "TRACKING_BUTTON",
                                  "BUTTON_REACHED", "CLICKING_BUTTON", "BUTTON_CLICKED", 
                                  "RETURNING_ARM", "ARM_RETURNED", "SENDING_MOVEMENT_EVENT",
                                  "BACKING_UP", "BACKUP_COMPLETED", "MOVING_TO_ELEVATOR_CENTER",
                                  "ELEVATOR_CENTER_REACHED"]:
            self.handle_scenario_1()
            
        # 시나리오 2: 엘리베이터 탑승 (SCENARIO_2_READY ~ SCENARIO_3_READY)
        elif self.current_state in ["SCENARIO_2_READY", "CHECKING_ELEVATOR_STATUS", 
                                    "ELEVATOR_ARRIVED", "WAITING_FOR_DOOR_OPEN", "DOOR_OPENED",
                                    "BOARDING_EVENT_SENT", "MOVING_TO_ELEVATOR_INTERIOR",
                                    "ELEVATOR_INTERIOR_REACHED"]:
            self.handle_scenario_2()
            
        # 시나리오 3: 엘리베이터 내부 조작 (SCENARIO_3_READY ~ SCENARIO_4_READY)
        elif self.current_state in ["SCENARIO_3_READY", "SETTING_ELEVATOR_INTERIOR_MODE",
                                    "ELEVATOR_INTERIOR_MODE_SET", "SENDING_INTERIOR_GUI_WARNING",
                                    "INTERIOR_GUI_WARNING_SENT", "SETTING_ARM_FRONT_POSITION",
                                    "ARM_FRONT_POSITION_SET", "CLICKING_INTERIOR_BUTTON",
                                    "INTERIOR_BUTTON_CLICKED", "SETTING_ARM_UPWARD", "ARM_UPWARD_SET",
                                    "SENDING_INTERIOR_MOVEMENT_EVENT", "INTERIOR_MOVEMENT_EVENT_SENT",
                                    "MOVING_ELEVATOR_CENTER", "ELEVATOR_CENTER_ALIGNED"]:
            self.handle_scenario_3()
            
        # 시나리오 4: 엘리베이터 하차 (SCENARIO_4_READY ~ COMPLETED)
        elif self.current_state in ["SCENARIO_4_READY", "CHECKING_ELEVATOR_ARRIVAL",
                                    "ELEVATOR_ARRIVED_EXIT", "WAITING_FOR_DOOR_OPEN_EXIT",
                                    "DOOR_OPENED_EXIT", "SENDING_EXIT_EVENT", "EXIT_EVENT_SENT",
                                    "EXITING_ELEVATOR", "ELEVATOR_EXIT_COMPLETED", 
                                    "RESTORING_NORMAL_MODE", "NORMAL_MODE_RESTORED"]:
            self.handle_scenario_4()
            
        # 완료 및 에러 처리
        elif self.current_state == "COMPLETED":
            self.handle_completion()
            
        elif self.current_state == "ERROR":
            self.handle_error()

    # ===== 시나리오 1: 엘리베이터 외부 조작 =====
    def handle_scenario_1(self):
        """시나리오 1: 엘리베이터 외부 버튼 조작 및 중앙 이동"""
        if self.current_state == "INIT":
            self.current_state = "SETTING_ARM_FORWARD"
            self.get_logger().info("🚀 시나리오 1 시작 - 팔 정면 설정")
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
            self.track_button()
            
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
            self.get_logger().info("🤖 팔 상향 위치 완료")
            self.send_movement_event()
            
        elif self.current_state == "SENDING_MOVEMENT_EVENT":
            self.current_state = "BACKING_UP"
            self.get_logger().info("📱 이동 이벤트 발송 완료")
            self.get_logger().info("🚗 후진 이동 시작 (전광판 인식 준비)")
            self.start_backup_movement()
            self.is_backup_started = True
            
        elif self.current_state == "BACKING_UP":
            # 후진 이동 중 - 타이머가 알아서 처리
            pass
            
        elif self.current_state == "BACKUP_COMPLETED":
            self.current_state = "MOVING_TO_ELEVATOR_CENTER"
            self.get_logger().info("✅ 후진 이동 완료")
            self.get_logger().info("🎯 엘리베이터 중앙으로 이동 시작")
            self.move_to_elevator_center()
            
        elif self.current_state == "ELEVATOR_CENTER_REACHED":
            self.current_state = "SCENARIO_2_READY"
            # self.current_state = "COMPLETED"  # 강제 완료 (사용자 의도)
            self.get_logger().info("✅ 엘리베이터 중앙 도달")
            self.get_logger().info("🎉 시나리오 1 완료 - 시나리오 2 준비")

    # ===== 시나리오 2: 엘리베이터 탑승 =====
    def handle_scenario_2(self):
        """시나리오 2: 엘리베이터 도착 확인 및 탑승"""
        if self.current_state == "SCENARIO_2_READY":
            self.current_state = "CHECKING_ELEVATOR_STATUS"
            self.get_logger().info("🚀 시나리오 2 시작")
            self.get_logger().info("🛗 엘리베이터 도착 판단 시작")
            self.elevator_status_start_time = time.time()
            
            # 엘리베이터 상태 확인 타이머 시작
            self.start_timer('elevator_status_timer', 1.0, self.check_elevator_status, "엘리베이터 상태 확인")
            
        elif self.current_state == "ELEVATOR_ARRIVED":
            self.current_state = "WAITING_FOR_DOOR_OPEN"
            self.get_logger().info("✅ 엘리베이터 도착 확인")
            self.get_logger().info("🚪 문 열림 대기 시작")
            self.door_status_start_time = time.time()
            
            # 엘리베이터 상태 확인 타이머 정지
            self.stop_timer('elevator_status_timer', "엘리베이터 상태 확인")
            
            # 문 상태 확인 타이머 시작
            self.start_timer('door_status_timer', 1.0, self.wait_for_door_open, "문 상태 확인")
            
        elif self.current_state == "DOOR_OPENED":
            self.get_logger().info("✅ 문 열림 확인")
            self.send_boarding_event()  # 이 함수에서 상태를 BOARDING_EVENT_SENT로 변경
            
        elif self.current_state == "BOARDING_EVENT_SENT":
            self.current_state = "MOVING_TO_ELEVATOR_INTERIOR"
            self.get_logger().info("📱 탑승 이벤트 발송 완료")
            self.get_logger().info("🚶 엘리베이터 내부로 이동 시작")
            self.move_to_elevator_interior()
            
        elif self.current_state == "MOVING_TO_ELEVATOR_INTERIOR":   
            # Nav2 이동 중 - 콜백에서 상태 변경됨
            pass
            
        elif self.current_state == "ELEVATOR_INTERIOR_REACHED":
            self.current_state = "SCENARIO_3_READY"
            self.current_state = "COMPLETED"  # 강제 완료 (사용자 의도)
            self.get_logger().info("✅ 엘리베이터 내부 도달")
            self.get_logger().info("🎉 시나리오 2 완료 - 시나리오 3 준비")

    # ===== 시나리오 3: 엘리베이터 내부 조작 =====
    def handle_scenario_3(self):
        """시나리오 3: 엘리베이터 내부 버튼 조작 및 중앙 정렬"""
        if self.current_state == "SCENARIO_3_READY":
            self.current_state = "SETTING_ELEVATOR_INTERIOR_MODE"
            self.get_logger().info("🚀 시나리오 3 시작 - 엘리베이터 내부 모드 설정")
            self.set_elevator_interior_mode()
            
        elif self.current_state == "ELEVATOR_INTERIOR_MODE_SET":
            self.current_state = "SENDING_INTERIOR_GUI_WARNING"
            self.get_logger().info("✅ 엘리베이터 내부 모드 설정 완료")
            self.send_interior_gui_warning()
            
        elif self.current_state == "INTERIOR_GUI_WARNING_SENT":
            self.current_state = "SETTING_ARM_FRONT_POSITION"
            self.get_logger().info("🤖 팔 정면 위치 설정 시작")
            self.set_arm_front_position()
            
        elif self.current_state == "ARM_FRONT_POSITION_SET":
            self.current_state = "CLICKING_INTERIOR_BUTTON"
            self.get_logger().info("🎯 엘리베이터 내부 버튼 클릭 시작")
            self.click_interior_button()
            
        elif self.current_state == "INTERIOR_BUTTON_CLICKED":
            self.current_state = "SETTING_ARM_UPWARD"
            self.get_logger().info("✅ 내부 버튼 클릭 완료")
            self.set_arm_upward()
            
        elif self.current_state == "ARM_UPWARD_SET":
            self.current_state = "SENDING_INTERIOR_MOVEMENT_EVENT"
            self.get_logger().info("📱 내부 이동 이벤트 발송 시작")
            self.send_interior_movement_event()
            
        elif self.current_state == "INTERIOR_MOVEMENT_EVENT_SENT":  ## 내부 이동
            self.current_state = "MOVING_ELEVATOR_CENTER"
            self.get_logger().info("🔄 엘리베이터 중앙 이동 시작")
            self.move_to_elevator_center_manual()
            
        elif self.current_state == "MOVING_ELEVATOR_CENTER":
            # 엘리베이터 중앙 이동 중 - 타이머가 알아서 처리
            pass
            
        elif self.current_state == "ELEVATOR_CENTER_ALIGNED":
            self.current_state = "SCENARIO_4_READY"
            self.current_state = "COMPLETED"  # 강제 완료 (사용자 의도)
            self.get_logger().info("🚗 엘리베이터 내부 중앙으로 이동 완료")
            self.get_logger().info("🎉 시나리오 3 완료 - 시나리오 4 준비")

    # ===== 시나리오 4: 엘리베이터 하차 =====
    def handle_scenario_4(self):
        """시나리오 4: 엘리베이터 도착 확인 및 하차"""
        if self.current_state == "SCENARIO_4_READY":
            self.current_state = "CHECKING_ELEVATOR_ARRIVAL"
            self.get_logger().info("🚀 시나리오 4 시작 - 엘리베이터 도착 확인")
            self.elevator_arrival_start_time = time.time()
            
            # 엘리베이터 도착 확인 타이머 시작
            self.start_timer('elevator_arrival_timer', 1.0, self.check_elevator_arrival, "엘리베이터 도착 확인")
            
        elif self.current_state == "ELEVATOR_ARRIVED_EXIT":
            self.current_state = "WAITING_FOR_DOOR_OPEN_EXIT"
            self.get_logger().info("✅ 엘리베이터 도착 확인 (6층)")
            self.get_logger().info("🚪 문 열림 대기 시작")
            self.door_status_exit_start_time = time.time()
            
            # 엘리베이터 도착 확인 타이머 정지
            self.stop_timer('elevator_arrival_timer', "엘리베이터 도착 확인")
            
            # 문 상태 확인 타이머 시작
            self.start_timer('door_status_exit_timer', 1.0, self.wait_for_door_open_exit, "하차용 문 상태 확인")
            
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
            self.stop_all_timers()

    # ===== 완료 및 에러 처리 =====
    def handle_completion(self):
        """시나리오 완료 처리"""
        self.stop_all_timers()
        self.get_logger().info("🎉 시나리오 완료")
            
    def handle_error(self):
        """에러 상태 처리"""
        self.stop_all_timers()
        self.get_logger().error("❌ 시나리오 실행 중 오류 발생 - 종료")

    def stop_all_timers(self):
        """모든 타이머 정지 - 간단하게 정리된 도우미 메서드"""
        timer_list = [
            'scenario_timer', 'location_timer', 'button_tracking_timer', 
            'backup_timer', 'elevator_status_timer', 'door_status_timer',
            'elevator_arrival_timer', 'door_status_exit_timer', 'simple_nav_monitor_timer'
        ]
        
        for timer_name in timer_list:
            self.stop_timer(timer_name)

    # ===== 공통 타이머 관리 도우미 메서드들 =====
    def start_timer(self, timer_name, interval, callback_method, description=""):
        """타이머 시작 도우미 메서드 - 중복 코드 제거"""
        # 기존 타이머가 있으면 먼저 정지
        self.stop_timer(timer_name)
        
        # 새 타이머 생성
        timer = self.create_timer(
            interval,
            callback_method,
            callback_group=self.callback_group
        )
        
        # 타이머 저장
        setattr(self, timer_name, timer)
        
        if description:
            self.get_logger().info(f"⏰ {description} 타이머 시작 ({interval}초 간격)")
    
    def stop_timer(self, timer_name, description=""):
        """타이머 정지 도우미 메서드 - 안전한 타이머 정지"""
        timer = getattr(self, timer_name, None)
        if timer:
            timer.cancel()
            setattr(self, timer_name, None)
            if description:
                self.get_logger().info(f"⏹️ {description} 타이머 정지")

    # ===== 공통 Nav2 이동 도우미 메서드들 =====
    def move_to_position_nav2(self, x, y, yaw, success_state, description, debug_message="Nav2 시뮬레이션 완료"):
        """Nav2를 사용한 위치 이동 공통 메서드 - 중복 코드 제거"""
        # 디버그 모드일 때: 바로 완료
        if self.debug_mode:
            self.get_logger().info(f"🔧 디버그 모드: {debug_message}")
            self.current_state = success_state
            return
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 위치 설정
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        # 방향 설정 (쿼터니언으로 변환)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        
        # Nav2 Action 전송
        self.get_logger().info(f"🎯 {description} Nav2 목표 전송: ({x}, {y})")
        
        # 성공 상태를 저장해두고 콜백에서 사용
        self._nav2_success_state = success_state
        self._nav2_description = description
        
        self.nav2_client.send_goal_async(goal).add_done_callback(self.nav2_common_goal_callback)
    
    def nav2_common_goal_callback(self, future):
        """Nav2 목표 전송 완료 공통 콜백"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("✅ Nav2 목표 수락됨")
                goal_handle.get_result_async().add_done_callback(self.nav2_common_result_callback)
            else:
                self.get_logger().error("❌ Nav2 목표 거부됨")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"Nav2 목표 전송 오류: {e}")
            self.current_state = "ERROR"
    
    def nav2_common_result_callback(self, future):
        """Nav2 결과 처리 공통 콜백"""
        try:
            result = future.result().result
            if result.error_code == 0:  # SUCCESS
                self.get_logger().info(f"✅ {self._nav2_description} 완료")
                self.current_state = self._nav2_success_state
            else:
                self.get_logger().error(f"❌ {self._nav2_description} 실패: error_code={result.error_code}")
                self.current_state = "ERROR"
        except Exception as e:
            self.get_logger().error(f"{self._nav2_description} 결과 처리 오류: {e}")
            self.current_state = "ERROR"

    # ===== 공통 GUI 이벤트 도우미 메서드들 =====
    def send_gui_event(self, event_id, detail, log_message, new_state=None):
        """GUI 이벤트 발송 공통 메서드 - 중복 코드 제거"""
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = event_id
        msg.task_id = self.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = detail
        
        self.gui_event_pub.publish(msg)
        self.get_logger().info(f"📱 {log_message}")
        
        # 상태 변경이 필요한 경우
        if new_state:
            self.current_state = new_state

    # ===== 공통 서비스 호출 도우미 메서드들 =====
    def call_door_status_service(self, expected_state, timer_attr, start_time_attr, count_attr, 
                                success_state, timeout_seconds, description):
        """DoorStatus 서비스 호출 공통 메서드 - 중복 코드 제거"""
        # 타이머가 취소되었거나 상태가 변경된 경우 처리하지 않음
        if self.current_state != expected_state or getattr(self, timer_attr) is None:
            return
            
        # 타임아웃 체크
        start_time = getattr(self, start_time_attr)
        if time.time() - start_time > timeout_seconds:
            self.get_logger().error(f"문 열림 대기 타임아웃 ({timeout_seconds}초)")
            self.current_state = "ERROR"
            return
            
        # DoorStatus 서비스 요청
        request = DoorStatus.Request()
        request.robot_id = self.robot_id
        
        count = getattr(self, count_attr, 0)
        self.get_logger().info(f"🚪 {description} 요청 #{count + 1}")
        
        # 성공 상태를 저장해두고 콜백에서 사용
        self._door_success_state = success_state
        self._door_description = description
        
        future = self.door_status_client.call_async(request)
        future.add_done_callback(self.door_status_common_callback)
        
        # 카운트 증가
        setattr(self, count_attr, count + 1)
    
    def door_status_common_callback(self, future):
        """문 상태 응답 처리 공통 콜백"""
        try:
            response = future.result()
            
            if response.success:
                if response.door_opened:
                    self.get_logger().info(f"✅ 문 열림 확인 - {self._door_description} (success: {response.success}, door_opened: {response.door_opened})")
                    self.current_state = self._door_success_state
                else:
                    self.get_logger().info(f"🚪 문이 아직 닫힌 상태 - {self._door_description} (success: {response.success}, door_opened: {response.door_opened})")
                    # 계속 확인 (타이머가 다시 호출)
            else:
                self.get_logger().info(f"🚪 문 상태 확인 실패 - {self._door_description} (success: {response.success}, door_opened: {response.door_opened})")
                # 계속 확인 (타이머가 다시 호출)
                
        except Exception as e:
            self.get_logger().error(f"문 상태 확인 중 오류: {str(e)}")

    # ===== 공통 액션 호출 도우미 메서드들 =====
    def call_set_pose_action(self, pose_id, success_state, description):
        """SetPose 액션 호출 도우미 메서드 - 중복 코드 제거"""
        goal = SetPose.Goal()
        goal.robot_id = self.robot_id
        goal.pose_id = pose_id
        
        def callback(future):
            try:
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info(f"✅ SetPose 액션 수락됨 (pose_id: {pose_id})")
                    goal_handle.get_result_async().add_done_callback(result_callback)
                else:
                    self.get_logger().error(f"❌ SetPose 액션 거부됨 (pose_id: {pose_id})")
                    self.current_state = "ERROR"
            except Exception as e:
                self.get_logger().error(f"❌ SetPose 액션 오류: {e}")
                self.current_state = "ERROR"
        
        def result_callback(future):
            try:
                result = future.result().result
                if result.success:
                    self.get_logger().info(f"✅ {description} 완료")
                    self.current_state = success_state
                else:
                    self.get_logger().error(f"❌ {description} 실패")
                    # 디버그 모드에서는 실패해도 계속 진행
                    if self.debug_mode:
                        self.get_logger().info("🔧 디버그 모드: 실패해도 계속 진행")
                        self.current_state = success_state
                    else:
                        self.current_state = "ERROR"
            except Exception as e:
                self.get_logger().error(f"❌ {description} 결과 처리 오류: {e}")
                self.current_state = "ERROR"
        
        self.get_logger().info(f"🤖 {description} 액션 시작 (pose_id: {pose_id})")
        try:
            self.set_pose_client.send_goal_async(goal).add_done_callback(callback)
        except Exception as e:
            self.get_logger().error(f"❌ {description} 액션 전송 실패: {e}")
            self.current_state = "ERROR"

    def call_click_button_action(self, button_id, success_state, description):
        """ClickButton 액션 호출 도우미 메서드 - 중복 코드 제거"""
        goal = ClickButton.Goal()
        goal.robot_id = self.robot_id
        goal.button_id = button_id
        
        def callback(future):
            try:
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info(f"✅ ClickButton 액션 수락됨 (button_id: {button_id})")
                    goal_handle.get_result_async().add_done_callback(result_callback)
                else:
                    self.get_logger().error(f"❌ ClickButton 액션 거부됨 (button_id: {button_id})")
                    self.current_state = "ERROR"
            except Exception as e:
                self.get_logger().error(f"❌ ClickButton 액션 오류: {e}")
                self.current_state = "ERROR"
        
        def result_callback(future):
            try:
                result = future.result().result
                if result.success:
                    self.get_logger().info(f"✅ {description} 완료: {result.message}")
                    self.current_state = success_state
                else:
                    self.get_logger().error(f"❌ {description} 실패: {result.message}")
                    # 디버그 모드에서는 실패해도 계속 진행
                    if self.debug_mode:
                        self.get_logger().info("🔧 디버그 모드: 실패해도 계속 진행")
                        self.current_state = success_state
                    else:
                        self.current_state = "ERROR"
            except Exception as e:
                self.get_logger().error(f"❌ {description} 결과 처리 오류: {e}")
                self.current_state = "ERROR"
        
        self.get_logger().info(f"🎯 {description} 액션 시작 (button_id: {button_id})")
        try:
            self.click_button_client.send_goal_async(goal).add_done_callback(callback)
        except Exception as e:
            self.get_logger().error(f"❌ {description} 액션 전송 실패: {e}")
            self.current_state = "ERROR"
    
    def set_arm_forward(self):
        """팔 정면 설정 액션 실행 - 도우미 메서드 사용"""
        # 특별 처리: CHECKING_LOCATION 상태로 변경하면서 타이머 시작
        def custom_success_callback():
            self.current_state = "CHECKING_LOCATION"
            self.location_check_start_time = time.time()
            
        # 임시로 성공 상태를 설정하고 후에 변경
        self.call_set_pose_action_with_custom_callback(4, "CHECKING_LOCATION", "팔 정면 설정", custom_success_callback)
    
    def call_set_pose_action_with_custom_callback(self, pose_id, success_state, description, success_callback=None):
        """SetPose 액션 호출 도우미 메서드 (커스텀 콜백 지원)"""
        goal = SetPose.Goal()
        goal.robot_id = self.robot_id
        goal.pose_id = pose_id
        
        def callback(future):
            try:
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info(f"✅ SetPose 액션 수락됨 (pose_id: {pose_id})")
                    goal_handle.get_result_async().add_done_callback(result_callback)
                else:
                    self.get_logger().error(f"❌ SetPose 액션 거부됨 (pose_id: {pose_id})")
                    self.current_state = "ERROR"
            except Exception as e:
                self.get_logger().error(f"❌ SetPose 액션 오류: {e}")
                self.current_state = "ERROR"
        
        def result_callback(future):
            try:
                result = future.result().result
                if result.success:
                    self.get_logger().info(f"✅ {description} 완료")
                    if success_callback:
                        success_callback()
                    else:
                        self.current_state = success_state
                else:
                    self.get_logger().error(f"❌ {description} 실패")
                    # 디버그 모드에서는 실패해도 계속 진행
                    if self.debug_mode:
                        self.get_logger().info("🔧 디버그 모드: 실패해도 계속 진행")
                        if success_callback:
                            success_callback()
                        else:
                            self.current_state = success_state
                    else:
                        self.current_state = "ERROR"
            except Exception as e:
                self.get_logger().error(f"❌ {description} 결과 처리 오류: {e}")
                self.current_state = "ERROR"
        
        self.get_logger().info(f"🤖 {description} 액션 시작 (pose_id: {pose_id})")
        try:
            self.set_pose_client.send_goal_async(goal).add_done_callback(callback)
        except Exception as e:
            self.get_logger().error(f"❌ {description} 액션 전송 실패: {e}")
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
        """GUI에 엘리베이터 버튼 조작 시작 이벤트 발송 - 공통 함수 사용"""
        self.send_gui_event(
            event_id=GUI_EVENT_BUTTON_OPERATION_START,
            detail=f"엘리베이터 버튼 조작 시작 - {self.current_floor}층 → {self.target_floor}층",
            log_message="GUI 이벤트 발송 완료"
        )
    
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
        """버튼 클릭 액션 실행 - 도우미 메서드 사용"""
        self.call_click_button_action(TARGET_BUTTON_ID, "BUTTON_CLICKED", "버튼 클릭")
    
    def return_arm(self):
        """팔 원위치 액션 실행 - 도우미 메서드 사용"""
        self.call_set_pose_action(5, "ARM_RETURNED", "팔 원위치")
    
    def send_movement_event(self):
        """GUI에 이동 시작 이벤트 발송 - 공통 함수 사용"""
        self.send_gui_event(
            event_id=GUI_EVENT_MOVEMENT_START,
            detail=f"이동 시작 - {self.current_floor}층 → {self.target_floor}층",
            log_message="이동 이벤트 발송 완료"
        )
    
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
                # 실패 응답의 모든 필드 출력
                self.get_logger().info(f"🛗 엘리베이터 상태 확인 실패 - success: {response.success}, position: {response.position}, direction: {response.direction}")
                # 계속 확인 (타이머가 다시 호출)
                
        except Exception as e:
            self.get_logger().error(f"엘리베이터 상태 확인 중 오류: {str(e)}")
    
    def wait_for_door_open(self):
        """문 열림 대기 - 공통 함수 사용"""
        self.call_door_status_service(
            expected_state="WAITING_FOR_DOOR_OPEN",
            timer_attr="door_status_timer",
            start_time_attr="door_status_start_time",
            count_attr="door_status_count",
            success_state="DOOR_OPENED",
            timeout_seconds=DOOR_STATUS_TIMEOUT,
            description="문 상태 확인"
        )
    

    
    def send_boarding_event(self):
        """GUI에 엘리베이터 탑승 시작 이벤트 발송 - 공통 함수 사용"""
        # 문 상태 확인 타이머 정지 (중요!)
        self.stop_timer('door_status_timer', "문 상태 확인")
        
        self.send_gui_event(
            event_id=GUI_EVENT_ELEVATOR_BOARDING_START,
            detail=f"엘리베이터 탑승 시작 - {self.current_floor}층 → {self.target_floor}층",
            log_message="탑승 이벤트 발송 완료",
            new_state="BOARDING_EVENT_SENT"
        )
    
    def move_to_elevator_interior(self):
        """엘리베이터 내부로 이동 (Nav2 사용) - 공통 함수 사용"""
        self.move_to_position_nav2(
            x=ELEVATOR_INTERIOR_X,
            y=ELEVATOR_INTERIOR_Y,
            yaw=ELEVATOR_INTERIOR_YAW,
            success_state="ELEVATOR_INTERIOR_REACHED",
            description="엘리베이터 내부 이동"
        )
    

    
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
        """GUI에 엘리베이터 내부 버튼 조작 시작 이벤트 발송 - 공통 함수 사용"""
        self.send_gui_event(
            event_id=GUI_EVENT_BUTTON_OPERATION_START,
            detail=f"엘리베이터 내부 버튼 조작 시작 - {self.current_floor}층 → {self.target_floor}층",
            log_message="내부 팔 움직임 GUI 경고 발송 완료",
            new_state="INTERIOR_GUI_WARNING_SENT"
        )
    
    def click_interior_button(self):
        """엘리베이터 내부 버튼 클릭 (추적 없이) - 도우미 메서드 사용"""
        self.call_click_button_action(6, "INTERIOR_BUTTON_CLICKED", "엘리베이터 내부 버튼 클릭")
    
    def set_arm_front_position(self):
        """팔 정면 위치 설정 액션 실행 - 도우미 메서드 사용"""
        self.call_set_pose_action(4, "ARM_FRONT_POSITION_SET", "팔 정면 위치 설정")

    def set_arm_upward(self):
        """팔 상향 설정 액션 실행 - 도우미 메서드 사용"""
        self.call_set_pose_action(6, "ARM_UPWARD_SET", "팔 상향 설정")
    
    def send_interior_movement_event(self):
        """GUI에 엘리베이터 내부 이동 시작 이벤트 발송 - 공통 함수 사용"""
        self.send_gui_event(
            event_id=GUI_EVENT_MOVEMENT_START,
            detail=f"엘리베이터 내부 이동 시작 - {self.current_floor}층 → {self.target_floor}층",
            log_message="내부 이동 이벤트 발송 완료",
            new_state="INTERIOR_MOVEMENT_EVENT_SENT"
        )



    def move_to_elevator_center_manual(self):
        """엘리베이터 내부 중앙으로 수동 이동 (4단계 동작)
        
        1단계: 각도 조정 (4초) - 왼쪽 바퀴 후진으로 로봇 방향 조정
        2단계: 후진 이동 (5초) - 엘리베이터 중앙으로 후진
        3단계: 정지 (1초) - 움직임 안정화
        4단계: 최종 회전 (3.2초) - 엘리베이터 문을 향하도록 방향 정렬
        """
        # 이미 진행 중이면 중복 실행 방지
        if self.elevator_center_timer is not None:
            return
            
        self.elevator_center_step = 1
        self.elevator_center_start_time = time.time()
        self.current_state = "MOVING_ELEVATOR_CENTER"  # 상태 변경
        
        # 1단계: 왼쪽 바퀴만 뒤로 (각도 조정)
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.03  # 좌측 바퀴만 후진 효과
        cmd_vel.angular.z = 0.15  # 우회전
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info("🔄 1단계: 각도 조정 시작 (왼쪽 바퀴 후진)")
        
        # 타이머로 단계별 진행 관리 (0.1초마다 상태 확인)
        self.elevator_center_timer = self.create_timer(0.1, self.elevator_center_control)
    
    def elevator_center_control(self):
        """엘리베이터 중앙 이동 제어 - 시간 기반 단계별 실행"""
        elapsed_time = time.time() - self.elevator_center_start_time
        
        if self.elevator_center_step == 1:  # 각도 조정 단계 (4초간)
            if elapsed_time >= 5.0:
                self.elevator_center_step = 2
                self.elevator_center_start_time = time.time()
                
                # 2단계: 후진으로 중앙까지 이동
                cmd_vel = Twist()
                cmd_vel.linear.x = -0.15  # 후진
                cmd_vel.angular.z = 0.0   # 직진
                self.cmd_vel_pub.publish(cmd_vel)
                
                self.get_logger().info("⬅️ 2단계: 후진으로 중앙 이동")
                
        elif self.elevator_center_step == 2:  # 후진 이동 단계 (5초간)
            if elapsed_time >= 4.5:
                self.elevator_center_step = 3
                self.elevator_center_start_time = time.time()
                
                # 3단계: 정지 (움직임 안정화)
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0  # 정지
                cmd_vel.angular.z = 0.0  # 정지
                self.cmd_vel_pub.publish(cmd_vel)
                
                self.get_logger().info("⏹️ 3단계: 정지 (움직임 안정화)")
                
        elif self.elevator_center_step == 3:  # 정지 단계 (1초간)
            if elapsed_time >= 1.0:
                self.elevator_center_step = 4
                self.elevator_center_start_time = time.time()

                # 4단계: 제자리 회전 (문을 향하도록 방향 조정)
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0    # 이동 없음
                cmd_vel.angular.z = -0.15  # 좌회전
                self.cmd_vel_pub.publish(cmd_vel)
                
                self.get_logger().info("🔄 4단계: 제자리 회전으로 최종 정렬")
                
        elif self.elevator_center_step == 4:  # 최종 회전 단계 (3.2초간)
            if elapsed_time >= 4.1:
                # 완전 정지
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                
                # 타이머 정리 및 상태 변경
                if self.elevator_center_timer:
                    self.elevator_center_timer.cancel()
                    self.elevator_center_timer = None
                
                self.get_logger().info("✅ 엘리베이터 중앙 정렬 완료")
                self.current_state = "ELEVATOR_CENTER_ALIGNED"

    
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
        """문 열림 대기 (6층에서) - 공통 함수 사용"""
        self.call_door_status_service(
            expected_state="WAITING_FOR_DOOR_OPEN_EXIT",
            timer_attr="door_status_exit_timer",
            start_time_attr="door_status_exit_start_time",
            count_attr="door_status_exit_count",
            success_state="DOOR_OPENED_EXIT",
            timeout_seconds=DOOR_STATUS_TIMEOUT,
            description="하차용 문 상태 확인"
        )
    

    
    def send_exit_event(self):
        """GUI에 엘리베이터 하차 시작 이벤트 발송 - 공통 함수 사용"""
        # 하차용 문 상태 확인 타이머 정지 (중요!)
        self.stop_timer('door_status_exit_timer', "하차용 문 상태 확인")
        
        self.send_gui_event(
            event_id=GUI_EVENT_ELEVATOR_EXIT_START,
            detail=f"엘리베이터 하차 시작 - {self.target_floor}층",
            log_message="하차 이벤트 발송 완료",
            new_state="EXIT_EVENT_SENT"
        )
    
    def exit_elevator(self):
        """엘리베이터 외부로 이동 (Nav2 사용) - 공통 함수 사용"""
        self.move_to_position_nav2(
            x=ELEVATOR_EXIT_X,
            y=ELEVATOR_EXIT_Y,
            yaw=ELEVATOR_EXIT_YAW,
            success_state="ELEVATOR_EXIT_COMPLETED",
            description="엘리베이터 하차"
        )
    

    
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
            if hasattr(node, 'elevator_center_timer') and node.elevator_center_timer:
                node.elevator_center_timer.cancel()
        except Exception as e:
            node.get_logger().error(f"타이머 정리 중 오류: {e}")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 