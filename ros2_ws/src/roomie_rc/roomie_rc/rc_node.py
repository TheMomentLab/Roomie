#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile

# ROS2 메시지 import
from roomie_msgs.action import PerformTask, StartCountdown
from roomie_msgs.msg import RobotState, RobotGuiEvent
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

# tf2 관련 import
import tf2_ros
from tf2_ros import TransformException
import tf_transformations

# 클라이언트 모듈 import
from .rms_client import RMSClient
from .gui_client import GUIClient
from .nav_client import NavClient
from .vs_client import VSClient
from .ioc_client import IOCClient
from .arm_client import ArmClient
from .location_manager import LocationManager

# 설정 import
from .config import *


class RCState:
    """RC 노드의 상태 정의"""
    # 1. 시작
    IDLE = "idle"                    # 대기 상태
    TASK_ASSIGNED = "task_assigned"  # 작업 할당됨
    
    # 2. 픽업 단계
    PICKUP_MOVING = "pickup_moving"  # 픽업 이동 중
    COUNTDOWN_START = "countdown_start"  # 출발 카운트다운 시작
    COUNTDOWN_COMPLETE = "countdown_complete"  # 카운트다운 완료
    NAVIGATION = "navigation"        # 내비게이션 중
    PICKUP_ARM_ROTATING = "pickup_arm_rotating"      # 픽업 시 팔 회전
    PICKUP_LOCATION_CHECK = "pickup_location_check"  # 픽업 시 위치 확인
    PICKUP_ARM_RETURN = "pickup_arm_return"         # 픽업 시 팔 복귀
    ARRIVED = "arrived"             # 목적지 도착
    
    # 3. 물품 적재 단계
    DRAWER_OPENING = "drawer_opening"  # 서랍 열기 중
    DRAWER_OPENED = "drawer_opened"    # 서랍 열림 완료
    LOADING_CHECK = "loading_check"    # 물품 적재 확인 중
    DOOR_CHECK = "door_check"          # 서랍 열림 상태 확인 중
    ITEM_CHECK = "item_check"          # 물품 적재 여부 확인 중
    DRAWER_CLOSING = "drawer_closing"  # 서랍 닫기 중
    
    # 4. 배송 단계
    DELIVERY_COUNTDOWN = "delivery_countdown"  # 배송 출발 카운트다운
    DELIVERY_MOVING = "delivery_moving"  # 배송지로 이동 중
    DELIVERY_ARM_ROTATING = "delivery_arm_rotating"      # 배송 시 팔 회전
    DELIVERY_LOCATION_CHECK = "delivery_location_check"  # 배송 시 위치 확인
    DELIVERY_ARM_RETURN = "delivery_arm_return"         # 배송 시 팔 복귀
    DELIVERY_ARM_RETURN_COMPLETE = "delivery_arm_return_complete"         # 배송 시 팔 복귀
    DELIVERY_COMPLETE = "delivery_complete"      # 배송 도착 완료
    
    # 5. 수령 확인 단계
    UNLOAD_DRAWER_OPENING = "unload_drawer_opening"  # 수령 확인용 서랍 열기 중
    UNLOAD_DRAWER_OPENED = "unload_drawer_opened"    # 수령 확인용 서랍 열림 완료
    UNLOAD_DOOR_CHECK = "unload_door_check"         # 수령 확인용 서랍 열림 상태 확인 중
    UNLOAD_ITEM_CHECK = "unload_item_check"         # 수령 확인용 물품 적재 여부 확인 중
    UNLOAD_DRAWER_CLOSING = "unload_drawer_closing"  # 수령 확인용 서랍 닫기 중
    
    # 6. 복귀 단계
    RETURN_COUNTDOWN = "return_countdown"        # 복귀 카운트다운
    RETURN_MOVING = "return_moving"              # 복귀 이동 중
    RETURN_COMPLETE = "return_complete"          # 복귀 완료
    
    # 기타
    CALL_MOVING = "call_moving"      # 호출 이동 중
    ERROR = "error"                  # 에러 상태


class RCNode(Node):
    """
    Roomie Robot Controller (RC) Node
    로봇 제어의 중심 노드
    """
    
    def __init__(self):
        super().__init__('rc_node')
        
        # 파라미터 선언
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('start_state', 'idle')
        self.declare_parameter('start_robot_state', 2)
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('current_floor_id', 0)
        
        # 디버그 모드 설정
        self.debug_mode = self.get_parameter('debug_mode').value
        self.get_logger().info(f'디버그 모드: {self.debug_mode}')
        
        # 로봇 정보
        self.robot_id = self.get_parameter('robot_id').value
        self.current_floor_id = self.get_parameter('current_floor_id').value
        
        # 작업 관련 정보
        self.current_task = None
        self.current_goal_handle = None
        
        # 시작 상태 설정
        self.robot_state = self.get_parameter('start_robot_state').value
        self.current_state = self.get_parameter('start_state').value
        self.get_logger().info(f'시작 상태: {self.current_state}')
        self.get_logger().info(f'시작 로봇 상태: {self.robot_state}')
        
        if self.debug_mode:
            self.get_logger().info(f'초기 상태: {self.current_state}')
            self.get_logger().info(f'초기 로봇 상태: {self.robot_state}')
        
        # 타이머들
        self.feedback_timer = None
        self.status_timer = None
        
        # 타이머 카운트
        self.door_check_count = 0
        
        # 위치 관리자 초기화
        self.location_manager = LocationManager()
        
        # tf2 관련 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 현재 위치 정보
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0
        self.current_pose.orientation.w = 1.0
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = 0.0
        
        # QoS 프로파일 설정
        self.qos_profile = QoSProfile(depth=10)
        
        # Callback Group 설정 (재진입 가능)
        self.callback_group = ReentrantCallbackGroup()
        
        # 클라이언트 초기화 및 연결 대기
        if not self._init_and_wait_for_clients():
            self.get_logger().error('필수 클라이언트 연결 실패')
            raise RuntimeError('필수 클라이언트 연결 실패')
        
        # 주기적 상태 발행 타이머 (1초마다)
        self.status_timer = self.create_timer(1.0, self.publish_periodic_status)
        
        self.get_logger().info('RC Node가 시작되었습니다.')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(f'현재 층: {self.current_floor_id} (1층)')
        
        # 초기 상태 발행
        self.publish_initial_status()
        
        # 디버그 모드에서 테스트용 task 설정
        if self.debug_mode:
            # 테스트용 task 생성
            self.current_task = PerformTask.Goal()
            self.current_task.task_id = TASK_ID
            self.current_task.task_type_id = 0
            self.current_task.target_location_id = 101
            self.current_task.pickup_location_id = 2
            self.current_task.order_info = '{ "room_number": "101", "items": [ { "name": "스파게티", "quantity": 2 }, { "name": "피자", "quantity": 1 } ]}'
            
            # 현재 상태에 따른 시작 메서드 호출
            self._start_from_current_state()
    
    def wait_for_action_server(self, client, server_name, timeout_sec=60.0):
        """Action Server 연결 대기"""
        if not client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f'⚠️ {server_name} Action Server 연결 실패 (타임아웃: {timeout_sec}초)')
            return False
        return True

    def wait_for_service(self, client, service_name, timeout_sec=60.0):
        """Service Server 연결 대기"""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'⚠️ {service_name} Service Server 연결 실패 (타임아웃: {timeout_sec}초)')
            return False
        return True

    def wait_for_topic_connection(self, publisher_or_subscriber, topic_name, is_publisher, timeout_sec=20.0):
        """토픽 연결 대기 (Publisher/Subscriber)"""
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while self.get_clock().now() < end_time:
            if is_publisher:
                if publisher_or_subscriber.get_subscription_count() > 0:
                    return True
            else:
                if publisher_or_subscriber.get_publisher_count() > 0:
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        
        connection_type = "구독자" if is_publisher else "발행자"
        self.get_logger().error(f'⚠️ {topic_name} 토픽의 {connection_type} 연결 실패 (타임아웃: {timeout_sec}초)')
        return False

    def _init_and_wait_for_clients(self):
        """클라이언트 초기화 및 연결 대기"""
        try:
            # 클라이언트 초기화
            self.get_logger().info('=== 클라이언트 초기화 시작 ===')
            
            # RMS 클라이언트
            self.get_logger().info('RMS 클라이언트 초기화...')
            self.rms_client = RMSClient(self)
            self.get_logger().info('✅ RMS 클라이언트 준비 완료')
            
            # GUI 클라이언트
            self.get_logger().info('GUI 클라이언트 초기화...')
            self.gui_client = GUIClient(self)
            if not self.wait_for_action_server(self.gui_client.start_countdown_client, 'GUI StartCountdown'):
                return False
            if not self.wait_for_action_server(self.gui_client.return_countdown_client, 'GUI ReturnCountdown'):
                return False
            if not self.wait_for_topic_connection(self.gui_client.gui_event_pub, '/robot_gui/event', True):
                return False
            if not self.wait_for_topic_connection(self.gui_client.gui_event_sub, '/robot_gui/event', False):
                return False
            self.get_logger().info('✅ GUI 클라이언트 준비 완료')
            
            # Nav2 클라이언트
            self.get_logger().info('Nav2 클라이언트 초기화...')
            self.nav_client = NavClient(self)
            if not self.wait_for_action_server(self.nav_client.nav_to_pose_client, 'NavigateToPose'):
                return False
            self.get_logger().info('✅ Nav2 클라이언트 준비 완료')
            
            # VS 클라이언트
            self.get_logger().info('VS 클라이언트 초기화...')
            self.vs_client = VSClient(self)
            if not self.wait_for_service(self.vs_client.location_client, '/vs/command/location'):
                return False
            self.get_logger().info('✅ VS 클라이언트 준비 완료')
            
            # # IOC 클라이언트
            # self.get_logger().info('IOC 클라이언트 초기화...')
            # self.ioc_client = IOCClient(self)
            # # Service Server 연결 대기
            # if not self.wait_for_service(self.ioc_client.control_lock_client, '/ioc/control_lock'):
            #     return False
            # if not self.wait_for_service(self.ioc_client.check_door_client, '/ioc/check_door_state'):
            #     return False
            # if not self.wait_for_service(self.ioc_client.check_item_client, '/ioc/check_item_loaded'):
            #     return False
            # self.get_logger().info('✅ IOC 클라이언트 준비 완료')
            
            # # Arm 클라이언트
            # self.get_logger().info('Arm 클라이언트 초기화...')
            # self.arm_client = ArmClient(self)
            # if not self.wait_for_action_server(self.arm_client.set_pose_client, '/arm/action/set_pose'):
            #     return False
            # self.get_logger().info('✅ Arm 클라이언트 준비 완료')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'클라이언트 초기화 중 오류 발생: {str(e)}')
            return False
    
    def publish_initial_status(self):
        """초기 상태 발행"""
        self.get_logger().info('초기 상태 발행 중...')
        self.rms_client.publish_robot_state(self.robot_state)
        self.rms_client.publish_battery_status(85.0, False)
        self.rms_client.publish_roomie_pose(self.current_floor_id, 0.0, 0.0, 0.0)
    
    def publish_periodic_status(self):
        """주기적 상태 발행"""
        self.rms_client.publish_robot_state(self.robot_state)
        self.rms_client.publish_battery_status(85.0, False)  # TODO: 실제 배터리 상태 읽기
        
        # 현재 로봇 위치 업데이트 및 발행
        self.update_robot_pose()
    
    def change_robot_state(self, new_state):
        """로봇 상태 변경"""
        self.robot_state = new_state
        self.get_logger().info(f'로봇 상태 변경: {new_state}')
        self.rms_client.publish_robot_state(new_state)
    
    def set_error_state(self, error_message=""):
        """에러 상태 설정 및 작업 초기화"""
        self.get_logger().error(f'에러 상태 설정: {error_message}')
        self.current_state = RCState.ERROR
        
        # 작업 초기화 (새로운 작업을 받을 수 있도록)
        if self.current_task:
            self.get_logger().info(f'작업 초기화: task_id={self.current_task.task_id}')
            self.current_task = None
            self.current_goal_handle = None
            
        # 피드백 타이머 정리
        if self.feedback_timer:
            self.feedback_timer.cancel()
            self.feedback_timer = None
    
    def handle_task_assignment(self, request, goal_handle):
        """작업 할당 처리"""
        self.get_logger().info(f'🔵 작업 할당 받음: task_id={request.task_id}, type={request.task_type_id}')
        self.get_logger().info(f'🔵 목적지: {request.target_location_id}, 픽업장소: {request.pickup_location_id}')
        
        # 작업 중복 방지: 현재 작업이 진행 중이면 거부
        if self.current_task and self.current_state != RCState.IDLE:
            self.get_logger().warn(f'⚠️ 작업 중복 방지: 현재 작업(task_id={self.current_task.task_id}) 진행 중')
            self.get_logger().warn(f'⚠️ 현재 상태: {self.current_state}')
            
            # 작업 거부 결과 반환
            result = PerformTask.Result()
            result.robot_id = request.robot_id
            result.task_id = request.task_id
            result.success = False
            result.message = f"작업 중복: 현재 작업(task_id={self.current_task.task_id}) 진행 중"
            goal_handle.abort()
            return result
        
        self.get_logger().info('=== 작업 할당 처리 시작 ===')
        
        # 1. 작업 정보 저장
        self.current_task = request
        self.current_goal_handle = goal_handle
        self.current_state = RCState.TASK_ASSIGNED
        
        # 2. 작업 타입에 따른 상태 변경
        if request.task_type_id in [0, 1]:  # 배송 작업
            self.change_robot_state(10)  # 픽업 위치 이동
            self.current_state = RCState.PICKUP_MOVING
            self.get_logger().info(f'🟢 배송 작업 시작: {self.current_state}')
        else:  # 호출/길안내
            self.change_robot_state(20)  # 호출 위치 이동
            self.current_state = RCState.CALL_MOVING
            self.get_logger().info(f'🟢 호출 작업 시작: {self.current_state}')
        
        # 3. 피드백 타이머 시작 (10초 주기)
        if self.feedback_timer:
            self.feedback_timer.cancel()
        self.feedback_timer = self.create_timer(
            10.0,
            lambda: self._publish_task_feedback(goal_handle)
        )
        
        # 4. 카운트다운 시작
        self.current_state = RCState.COUNTDOWN_START
        self.get_logger().info(f'🟡 카운트다운 시작: {self.current_state}')
        
        self.gui_client.start_countdown(
            self.current_task.robot_id,
            self.current_task.task_id,
            self.current_task.task_type_id,
            callback=self._on_countdown_complete
        )
    
    def _publish_task_feedback(self, goal_handle):
        """주기적으로 피드백 발행"""
        if not self.current_task:
            return
            
        # goal_handle 상태 확인
        if goal_handle:
            try:
                # goal_handle이 살아있는지 확인
                if goal_handle.is_active:
                    self.get_logger().info(f'✅ Goal Handle 상태: ACTIVE - robot_id={self.current_task.robot_id}, task_id={self.current_task.task_id}')
                else:
                    self.get_logger().warn(f'⚠️ Goal Handle 상태: INACTIVE')
                    return
            except Exception as e:
                self.get_logger().warn(f'⚠️ Goal Handle 상태 확인 오류: {e}')
                return
            
        feedback_msg = PerformTask.Feedback()
        feedback_msg.robot_id = self.current_task.robot_id
        feedback_msg.task_id = self.current_task.task_id
        
        goal_handle.publish_feedback(feedback_msg)
    
    def _on_countdown_complete(self, result):
        """카운트다운 완료 콜백"""
        if not result.result.success:
            self.set_error_state('카운트다운 실패')
            return
            
        self.current_state = RCState.NAVIGATION
        self.get_logger().info(f'🟡 내비게이션 시작: {self.current_state}')
        
        # 레스토랑으로 이동 시작 (0도 방향으로 설정)
        pose = self.location_manager.get_pose(self.current_task.pickup_location_id, yaw=0.0)
        self.get_logger().info(f'📍 목표 위치: pickup_location_id={self.current_task.pickup_location_id}')
        self.nav_client.go_to_pose(
            pose,
            self._on_navigation_complete
        )

    def _on_navigation_complete(self, success, message):
        """내비게이션 완료 콜백"""
        if not success:
            self.set_error_state(f'내비게이션 실패: {message}')
            return
            
        self.get_logger().info('✅ 내비게이션 완료, 팔 회전 시작')
        self.current_state = RCState.PICKUP_ARM_ROTATING
        
        # 팔을 왼쪽으로 회전 (pose_id=2)
        self.get_logger().info(f'🤖 팔 회전 명령: pose_id=2')
        self.arm_client.set_arm_pose(
            2,
            lambda result: self._on_pickup_arm_rotation_complete(result.result.success if result else False)
        )
    
    def _on_pickup_arm_rotation_complete(self, success):
        """픽업 시 팔 회전 완료 콜백"""
        if not success:
            self.set_error_state('픽업 시 팔 회전 실패')
            return
            
        self.get_logger().info('✅ 팔 회전 완료, 위치 확인 시작')
        self.current_state = RCState.PICKUP_LOCATION_CHECK
        
        # 위치 확인 타이머 시작 (1초 간격)
        self.location_check_count = 0
        self.get_logger().info(f'🔍 위치 확인 시작: location_id=2 대기')
        self.location_check_timer = self.create_timer(
            1.0,
            self._on_pickup_check_location
        )
    
    def _on_pickup_check_location(self):
        """픽업 시 위치 확인 요청 (1초 간격)"""
        self.location_check_count += 1
        
        # 30초 타임아웃 체크
        if self.location_check_count > 30:
            self.location_check_timer.cancel()
            self.set_error_state('위치 확인 타임아웃')
            return
            
        # 위치 확인 요청
        self.get_logger().info(f'🔍 위치 확인 #{self.location_check_count}: robot_id={self.robot_id}')
        self.vs_client.check_location(
            self.robot_id,
            lambda success, location_id: self._on_pickup_location_response(success, location_id)
        )
    
    def _on_pickup_location_response(self, success, location_id):
        """픽업 시 위치 확인 응답 처리"""
        if not success:
            return  # 다음 타이머에서 다시 시도
            
        # location_id=2 확인
        if location_id == 2:
            self.get_logger().info('✅ 위치 확인 완료 (location_id=2), 팔 초기화 시작')
            self.location_check_timer.cancel()
            self.current_state = RCState.PICKUP_ARM_RETURN
            
            # 팔 초기 자세로 (pose_id=0)
            self.get_logger().info(f'🤖 팔 초기화 명령: pose_id=0')
            self.arm_client.set_arm_pose(
                0,
                lambda result: self._on_pickup_arm_return_complete(result.result.success if result else False)
            )
    
    def _on_pickup_arm_return_complete(self, success):
        """픽업 시 팔 초기화 완료 콜백"""
        if not success:
            self.set_error_state('픽업 시 팔 초기화 실패')
            return
            
        self.get_logger().info('✅ 팔 초기화 완료, 도착 처리 시작')
        self.current_state = RCState.ARRIVED
        
        # 로봇 상태 변경 (10 → 11)
        self.get_logger().info(f'🟢 픽업 완료: 상태 변경 10→11')
        self.change_robot_state(11)
        
        # GUI에 주문 정보 전송
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = 13
        msg.task_id = self.current_task.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = self.current_task.order_info
        
        self.gui_client.gui_event_pub.publish(msg)
        self.get_logger().info('GUI 이벤트 전송 완료')
        
        # GUI 이벤트 구독 시작
        self.gui_client.subscribe_events(self._handle_gui_event)


    def _handle_gui_event(self, msg):
        """GUI 이벤트 처리"""
        if msg.rgui_event_id == 104:  # 서랍 열기 요청
            self.get_logger().info(f'서랍 열기 요청 수신 - 현재 상태: {self.current_state}')
            self.get_logger().info(f'상태 비교: ARRIVED={RCState.ARRIVED}, DELIVERY_COMPLETE={RCState.DELIVERY_COMPLETE}')
            
            if self.current_state == RCState.ARRIVED:  # 물품 적재 단계
                self.get_logger().info('물품 적재를 위한 서랍 열기 시작')
                self.current_state = RCState.DRAWER_OPENING
                # 서랍 잠금 해제 요청
                self.ioc_client.control_lock(
                    self.robot_id,
                    False,  # locked=False
                    self._on_drawer_unlocked
                )
            elif self.current_state == RCState.DELIVERY_COMPLETE:  # 수령 확인 단계
                self.get_logger().info('수령 확인을 위한 서랍 열기 시작')
                self.current_state = RCState.UNLOAD_DRAWER_OPENING
                # 서랍 잠금 해제 요청
                self.ioc_client.control_lock(
                    self.robot_id,
                    False,  # locked=False
                    self._on_unload_drawer_unlocked
                )
            else:
                self.get_logger().warn(f'서랍 열기 요청 무시 - 현재 상태: {self.current_state}')
                self.get_logger().warn(f'예상 상태: ARRIVED 또는 DELIVERY_COMPLETE')
        elif msg.rgui_event_id == 105:  # 적재 완료 버튼 클릭
            self.get_logger().info(f'적재 완료 버튼 클릭 수신 - 현재 상태: {self.current_state}')
            if self.current_state == RCState.DRAWER_OPENED:  # 적재 단계
                self.get_logger().info('적재 완료 버튼 클릭 처리 시작')
                self.current_state = RCState.DOOR_CHECK
                
                # 서랍 열림 상태 확인 타이머 시작
                self.door_check_timer = self.create_timer(1.0, self._check_door_state)
            else:
                self.get_logger().warn(f'적재 완료 버튼 클릭 무시 - 현재 상태가 DRAWER_OPENED가 아님: {self.current_state}')
        elif msg.rgui_event_id == 100:  # 수령 확인 버튼 클릭
            self.get_logger().info(f'수령 확인 버튼 클릭 수신 - 현재 상태: {self.current_state}')
            if self.current_state == RCState.UNLOAD_DRAWER_OPENED:  # 수령 단계
                self.get_logger().info('수령 확인 버튼 클릭 처리 시작')
                self.current_state = RCState.UNLOAD_DOOR_CHECK
                self.door_check_count = 0
                
                # 서랍 열림 상태 확인 타이머 시작
                self.unload_door_check_timer = self.create_timer(1.0, self._check_unload_door_state)
            else:
                self.get_logger().warn(f'수령 확인 버튼 클릭 무시 - 현재 상태가 UNLOAD_DRAWER_OPENED가 아님: {self.current_state}')
    
    # 물품 적재 관련 메서드들
    def _on_drawer_unlocked(self, success, message):
        """서랍 잠금 해제 완료 콜백"""
        if not success:
            self.get_logger().error(f'서랍 잠금 해제 실패: {message}')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('서랍 잠금 해제 완료')
        self.current_state = RCState.DRAWER_OPENED
        
        # GUI에 서랍 열림 이벤트 전송
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = 16  # 서랍 열림 이벤트
        msg.task_id = self.current_task.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = ""
        
        self.gui_client.gui_event_pub.publish(msg)
        self.get_logger().info('서랍 잠금 해제 이벤트 전송 완료')
    
    def _check_door_state(self):
        """서랍 열림 상태 확인 (1초 간격)"""
        self.door_check_count += 1
        
        # 30초 타임아웃 체크
        if self.door_check_count > 30:
            self.get_logger().error('서랍 열림 상태 확인 타임아웃')
            self.door_check_timer.cancel()
            self.current_state = RCState.ERROR
            return
            
        # 서랍 상태 확인 요청
        self.ioc_client.check_door_state(
            self.robot_id,
            self._on_door_state_response
        )
    
    def _on_door_state_response(self, is_opened):
        """서랍 열림 상태 확인 응답 처리"""
            
        if not is_opened:  # 서랍이 닫힘
            self.get_logger().info('서랍이 닫힘, 물품 적재 확인 시작')
            self.door_check_timer.cancel()
            self.current_state = RCState.ITEM_CHECK
            
            # 물품 적재 확인
            self.ioc_client.check_item_loaded(
                self.robot_id,
                self._on_item_loaded_response
            )
        else:
            self.get_logger().warn('서랍 열림')
    
    def _on_item_loaded_response(self, item_loaded):
        """물품 적재 확인 응답 처리"""
            
        if not item_loaded:  # 물품이 없음
            self.get_logger().info('물품이 적재되지 않음, GUI 이벤트 전송')
            
            # GUI에 물품 미적재 이벤트 전송
            msg = RobotGuiEvent()
            msg.robot_id = self.robot_id
            msg.rgui_event_id = 27  # 물품 미적재 이벤트
            msg.task_id = self.current_task.task_id
            msg.timestamp = self.get_clock().now().to_msg()
            msg.detail = ""
            
            self.gui_client.gui_event_pub.publish(msg)
            self.get_logger().info('물품 미적재 이벤트 전송 완료')
            
            # 다시 적재 완료 버튼 대기 상태로
            self.current_state = RCState.DRAWER_OPENED
            return
            
        # 물품이 적재됨 - 서랍 잠금 요청
        self.get_logger().info('물품이 적재됨, 서랍 잠금 요청')
        self.current_state = RCState.DRAWER_CLOSING
        
        self.ioc_client.control_lock(
            self.robot_id,
            True,  # locked=True
            self._on_drawer_locked
        )
    
    def _on_drawer_locked(self, success, message):
        """서랍 잠금 완료 콜백"""
        if not success:
            self.get_logger().error(f'서랍 잠금 실패: {message}')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('서랍 잠금 완료')
        
        # 로봇 상태 변경 (11 → 12)
        self.change_robot_state(12)
        
        # 배송 출발 카운트다운 시작
        self.current_state = RCState.DELIVERY_COUNTDOWN
        
        # 출발 카운트다운 액션 요청
        self.get_logger().info('출발 카운트다운 시작')
        self.gui_client.start_countdown(
            self.robot_id,
            self.current_task.task_id,
            self.current_task.task_type_id,
            self._on_delivery_countdown_complete
        )


    
    def _on_delivery_countdown_complete(self, result):
        """배송 출발 카운트다운 완료 콜백"""
        if not result.result.success:
            self.get_logger().error('출발 카운트다운 실패')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('출발 카운트다운 완료, 목적지로 이동 시작')
        self.current_state = RCState.DELIVERY_MOVING
        
        # 목적지로 이동 시작 (0도 방향으로 설정)
        self.nav_client.go_to_pose(
            self.location_manager.get_pose(self.current_task.target_location_id, yaw=0.0),
            self._on_delivery_navigation_complete
        )
    
    def _on_delivery_navigation_complete(self, success, message):
        """목적지 이동 완료 콜백"""
        if not success:
            self.get_logger().error(f'목적지 이동 실패: {message}')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('목적지 도착')
        
        # 팔 회전 시작 (pose_id=2)
        self.current_state = RCState.DELIVERY_ARM_ROTATING
        self.arm_client.set_arm_pose(2, self._on_delivery_arm_rotation_complete)
    
    def _on_delivery_arm_rotation_complete(self, result):
        """배송 시 팔 회전 완료 콜백"""
        if not result or not result.result.success:
            self.get_logger().error('배송 시 팔 회전 실패')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('배송 시 팔 회전 완료')
        
        # 위치 확인 시작
        self.current_state = RCState.DELIVERY_LOCATION_CHECK
        self.location_check_count = 0
        self.get_logger().info(f'🔍 배송 시 위치 확인 시작: location_id={self.current_task.target_location_id} 대기')
        self.location_check_timer = self.create_timer(
            1.0,
            self._on_delivery_check_location
        )
    
    def _on_delivery_check_location(self):
        """배송 시 위치 확인 요청 (1초 간격)"""
        self.location_check_count += 1
        
        # 30초 타임아웃 체크
        if self.location_check_count > 30:
            self.location_check_timer.cancel()
            self.set_error_state('배송 시 위치 확인 타임아웃')
            return
            
        # 위치 확인 요청
        self.get_logger().info(f'🔍 배송 시 위치 확인 #{self.location_check_count}: robot_id={self.robot_id}')
        self.vs_client.check_location(
            self.robot_id,
            lambda success, location_id: self._on_delivery_location_response(success, location_id)
        )
    
    def _on_delivery_location_response(self, success, location_id):
        """배송 시 위치 확인 응답 처리"""
        if not success:
            return  # 다음 타이머에서 다시 시도
            
        # target_location_id 확인
        if location_id == self.current_task.target_location_id:
            self.get_logger().info(f'✅ 배송 시 위치 확인 완료 (location_id={location_id}), 팔 초기화 시작')
            self.location_check_timer.cancel()
            self.current_state = RCState.DELIVERY_ARM_RETURN
            
            # 팔 초기 자세로 (pose_id=0)
            self.get_logger().info(f'🤖 팔 초기화 명령: pose_id=0')
            self.arm_client.set_arm_pose(
                0,
                lambda result: self._on_delivery_arm_return_complete(result.result.success if result else False)
            )
        else:
            self.get_logger().warn(f'⚠️ 배송 시 위치 불일치: 현재={location_id}, 목표={self.current_task.target_location_id}')
    
    def _on_delivery_arm_return_complete(self, result):
        """배송 시 팔 초기 자세 복귀 완료 콜백"""
        if not result or not result.result.success:
            self.get_logger().error('배송 시 팔 초기 자세 복귀 실패')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('배송 시 팔 초기 자세 복귀 완료')
        

        self.current_state = RCState.DELIVERY_ARM_RETURN_COMPLETE
        # 제자리 회전 시작
        self._rotate_in_place()
    
    def _rotate_in_place(self):
        """제자리 회전 수행"""
        self.get_logger().info('제자리 회전 시작')
        
        # 현재 위치에서 오른쪽으로 90도 회전하는 목표 생성
        current_pose = self.location_manager.get_pose(self.current_task.target_location_id)
        target_pose = current_pose
        # 90도 시계방향 회전을 위한 쿼터니언 (w=0.7071, z=-0.7071)
        target_pose.pose.orientation.w = 0.7071  # cos(pi/4)
        target_pose.pose.orientation.z = -0.7071  # -sin(pi/4)
        
        self.nav_client.go_to_pose(target_pose, self._on_rotation_complete)
    
    def _on_rotation_complete(self, success, message):
        """제자리 회전 완료 콜백"""
        if not success:
            self.get_logger().error(f'제자리 회전 실패: {message}')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('제자리 회전 완료')
        
        # 로봇 상태 변경 (12 → 13)
        self.change_robot_state(13)
        self.current_state = RCState.DELIVERY_COMPLETE
        
        # GUI에 배송 완료 이벤트 전송 (rgui_event_id=15)
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = 15  # 배송 완료 이벤트
        msg.task_id = self.current_task.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = ""
        
        self.gui_client.gui_event_pub.publish(msg)
        self.get_logger().info('배송 완료 이벤트 전송 완료')
    
    # 수령 확인 관련 메서드들
    def _on_unload_drawer_unlocked(self, success, message):
        """수령 확인용 서랍 잠금 해제 완료 콜백"""
        if not success:
            self.get_logger().error(f'수령 확인용 서랍 잠금 해제 실패: {message}')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('수령 확인용 서랍 잠금 해제 완료')
        self.current_state = RCState.UNLOAD_DRAWER_OPENED
        
        # GUI에 서랍 열림 이벤트 전송
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = 16  # 서랍 열림 이벤트
        msg.task_id = self.current_task.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = ""
        
        self.gui_client.gui_event_pub.publish(msg)
        self.get_logger().info('수령 확인용 서랍 열림 이벤트 전송 완료')
    
    def _check_unload_door_state(self):
        """수령 확인용 서랍 열림 상태 확인 (1초 간격)"""
        self.door_check_count += 1
        
        # 30초 타임아웃 체크
        if self.door_check_count > 30:
            self.get_logger().error('서랍 열림 상태 확인 타임아웃')
            self.unload_door_check_timer.cancel()
            self.current_state = RCState.ERROR
            return
            
        # 서랍 상태 확인 요청
        self.ioc_client.check_door_state(
            self.robot_id,
            self._on_unload_door_state_response
        )
    
    def _on_unload_door_state_response(self, is_opened):
        """수령 확인용 서랍 열림 상태 확인 응답 처리"""
        if not is_opened:  # 서랍이 닫힘
            self.get_logger().info('서랍이 닫힘, 물품 적재 확인 시작')
            self.unload_door_check_timer.cancel()
            self.current_state = RCState.UNLOAD_ITEM_CHECK
            
            # 물품 적재 확인
            self.ioc_client.check_item_loaded(
                self.robot_id,
                self._on_unload_item_loaded_response
            )
    
    def _on_unload_item_loaded_response(self, item_loaded):
        """수령 확인용 물품 적재 확인 응답 처리"""
        if item_loaded:  # 물품이 있음
            self.get_logger().info('물품이 있음, GUI 이벤트 전송')
            
            # GUI에 물품 있음 이벤트 전송
            msg = RobotGuiEvent()
            msg.robot_id = self.robot_id
            msg.rgui_event_id = 27  # 물품 있음 이벤트
            msg.task_id = self.current_task.task_id
            msg.timestamp = self.get_clock().now().to_msg()
            msg.detail = ""
            
            self.gui_client.gui_event_pub.publish(msg)
            self.get_logger().info('물품 있음 이벤트 전송 완료')
            
            # 다시 적재 완료 버튼 대기 상태로
            self.current_state = RCState.UNLOAD_DRAWER_OPENED
            return
            
        # 물품이 없음 - 서랍 잠금 요청
        self.get_logger().info('물품이 없음, 서랍 잠금 요청')
        self.current_state = RCState.UNLOAD_DRAWER_CLOSING
        
        self.ioc_client.control_lock(
            self.robot_id,
            True,  # locked=True
            self._on_unload_drawer_locked
        )
    
    def _on_unload_drawer_locked(self, success, message):
        """수령 확인용 서랍 잠금 완료 콜백"""
        if not success:
            self.get_logger().error(f'수령 확인용 서랍 잠금 실패: {message}')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('수령 확인용 서랍 잠금 완료')
        
        # GUI에 수령 확인 완료 이벤트 전송
        msg = RobotGuiEvent()
        msg.robot_id = self.robot_id
        msg.rgui_event_id = 18  # 수령 확인 완료 이벤트
        msg.task_id = self.current_task.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = ""
        
        self.gui_client.gui_event_pub.publish(msg)
        self.get_logger().info('✅ 수령 확인 완료 이벤트 전송 완료')
        
        # 작업 완료 처리 (Goal Handle이 활성 상태일 때만)
        if self.current_goal_handle and self.current_goal_handle.is_active:
            try:
                self.get_logger().info(f'🎉 작업 완료 처리 - robot_id={self.robot_id}, task_id={self.current_task.task_id}')
                result = PerformTask.Result()
                result.robot_id = self.robot_id
                result.task_id = self.current_task.task_id
                result.success = True
                self.current_goal_handle.succeed()
                self.get_logger().info(f'🎉 Goal Handle SUCCEED 호출 완료')
            except Exception as e:
                self.get_logger().error(f'❌ Goal Handle 처리 중 오류: {e}')
        else:
            self.get_logger().info(f'ℹ️ Goal Handle이 이미 비활성 상태이므로 작업 완료 처리 건너뜀')
        
        # 로봇 상태 변경 (13 → 30)
        self.change_robot_state(30)
        
        # 복귀 카운트다운 시작
        self.current_state = RCState.RETURN_COUNTDOWN
        self.gui_client.start_return_countdown(
            self._on_return_countdown_complete
        )
    
    def _on_return_countdown_complete(self, result):
        """복귀 카운트다운 완료 콜백"""
        if not result.result.success:
            self.get_logger().error('복귀 카운트다운 실패')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('복귀 카운트다운 완료, LOB_WAITING으로 이동 시작')
        self.current_state = RCState.RETURN_MOVING
        
        # LOB_WAITING 위치로 이동 (90도 방향으로 설정)
        self.nav_client.go_to_pose(
            self.location_manager.get_pose(0, yaw=1.57),  # LOB_WAITING location_id=0
            self._on_return_navigation_complete
        )
    
    def _on_return_navigation_complete(self, success, message):
        """복귀 이동 완료 콜백"""
        if not success:
            self.get_logger().error(f'복귀 이동 실패: {message}')
            self.current_state = RCState.ERROR
            return
            
        self.get_logger().info('🏠 복귀 완료')
        self.current_state = RCState.RETURN_COMPLETE
        
        # 로봇 상태 변경 (30 → 2)
        self.get_logger().info(f'🟢 초기 상태 복귀: 상태 변경 30→2')
        self.change_robot_state(2)
        
        # 초기 상태로 복귀
        self.current_state = RCState.IDLE
        self.current_task = None
        self.current_goal_handle = None
        self.get_logger().info(f'🔄 시스템 초기화 완료: IDLE 상태')
    




    
    def _start_from_current_state(self):
        """현재 상태에 따른 시작 메서드 호출"""
        self.get_logger().info(f'상태 {self.current_state}에서 시작합니다.')
        
        # 1. 픽업 단계
        if self.current_state == RCState.COUNTDOWN_START:
            # 출발 카운트다운 시작
            goal_msg = StartCountdown.Goal()
            goal_msg.robot_id = self.robot_id
            goal_msg.task_id = self.current_task.task_id
            goal_msg.task_type_id = self.current_task.task_type_id
            self.gui_client.start_countdown(
                self.robot_id,
                self.current_task.task_id,
                self.current_task.task_type_id,
                self._on_countdown_complete
            )
            
        elif self.current_state == RCState.NAVIGATION:
            # 레스토랑으로 이동 시작
            self.nav_client.go_to_pose(
                self.location_manager.get_pose(self.current_task.pickup_location_id),
                self._on_navigation_complete
            )
            
        elif self.current_state == RCState.PICKUP_ARM_ROTATING:
            # 팔 회전 시작 (pose_id=2)
            self.arm_client.set_arm_pose(2, self._on_pickup_arm_rotation_complete)
            
        elif self.current_state == RCState.PICKUP_LOCATION_CHECK:
            # 위치 확인 시작
            self._on_pickup_check_location()
            
        elif self.current_state == RCState.PICKUP_ARM_RETURN:
            # 팔 초기 자세로 복귀 시작 (pose_id=0)
            self.arm_client.set_arm_pose(0, self._on_pickup_arm_return_complete)
            
        # 2. 물품 적재 단계 - GUI 이벤트 기반으로 동작하므로 자동 실행하지 않음
        elif self.current_state == RCState.ARRIVED:
            # 픽업 완료 상태에서 GUI 이벤트 대기
            self.get_logger().info('물품 적재를 위한 GUI 이벤트(서랍 열기) 대기 중...')
            
        elif self.current_state == RCState.DRAWER_OPENING:
            # 서랍 열기 시작 (GUI 이벤트에 의해 트리거됨)
            self.ioc_client.control_lock(self.robot_id, False, self._on_drawer_unlocked)
            
        elif self.current_state == RCState.DRAWER_OPENED:
            # 서랍 열림 상태에서 적재 완료 버튼 대기
            self.get_logger().info('물품 적재 완료 버튼 대기 중...')
            
        elif self.current_state == RCState.DOOR_CHECK:
            # 서랍 열림 상태 확인 시작 (적재 완료 버튼에 의해 트리거됨)
            self._check_door_state()
            
        elif self.current_state == RCState.ITEM_CHECK:
            # 물품 적재 확인 시작
            self.ioc_client.check_item_loaded(self.robot_id, self._on_item_loaded_response)
            
        elif self.current_state == RCState.DRAWER_CLOSING:
            # 서랍 잠금 시작
            self.ioc_client.control_lock(self.robot_id, True, self._on_drawer_locked)
            
        # 3. 배송 단계
        elif self.current_state == RCState.DELIVERY_COUNTDOWN:
            # 배송 출발 카운트다운 시작
            self.gui_client.start_countdown(
                self.robot_id,
                self.current_task.task_id,
                self.current_task.task_type_id,
                self._on_delivery_countdown_complete
            )
            
        elif self.current_state == RCState.DELIVERY_MOVING:
            # 목적지로 이동 시작
            self.nav_client.go_to_pose(
                self.location_manager.get_pose(self.current_task.target_location_id),
                self._on_delivery_navigation_complete
            )
            
        elif self.current_state == RCState.DELIVERY_ARM_ROTATING:
            # 배송 시 팔 회전 시작 (pose_id=2)
            self.arm_client.set_arm_pose(2, self._on_delivery_arm_rotation_complete)
            
        elif self.current_state == RCState.DELIVERY_LOCATION_CHECK:
            # 배송 시 위치 확인 시작
            self.location_check_count = 0
            self.get_logger().info(f'🔍 배송 시 위치 확인 시작: location_id={self.current_task.target_location_id} 대기')
            self.location_check_timer = self.create_timer(
                1.0,
                self._on_delivery_check_location
            )
            
        elif self.current_state == RCState.DELIVERY_ARM_RETURN:
            # 배송 시 팔 초기 자세로 복귀 시작 (pose_id=0)
            self.arm_client.set_arm_pose(
                0,
                lambda result: self._on_delivery_arm_return_complete(result.result.success if result else False)
            )
            
            
        elif self.current_state == RCState.DELIVERY_ARM_RETURN_COMPLETE:
            # 배송 시 팔 초기 자세로 복귀 완료 
            self._rotate_in_place()

            
            
        elif self.current_state == RCState.DELIVERY_COMPLETE:
            # 배송 완료 이벤트 전송 (rgui_event_id=15)
            msg = RobotGuiEvent()
            msg.robot_id = self.robot_id
            msg.rgui_event_id = 15  # 배송 완료 이벤트
            msg.task_id = self.current_task.task_id
            msg.timestamp = self.get_clock().now().to_msg()
            msg.detail = ""
            
            self.gui_client.gui_event_pub.publish(msg)
            self.get_logger().info('배송 완료 이벤트 전송 완료')
            self.get_logger().info('수령 확인을 위한 GUI 이벤트 대기 중...')
            
            # GUI 이벤트 구독 설정 (수령 확인 단계를 위해)
            self.gui_client.subscribe_events(self._handle_gui_event)
            
        # 4. 수령 확인 단계 - GUI 이벤트 기반으로 동작하므로 자동 실행하지 않음
        
        elif self.current_state == RCState.UNLOAD_DRAWER_OPENING:
            # 수령 확인용 서랍 열기 시작 (GUI 이벤트에 의해 트리거됨)
            self.ioc_client.control_lock(self.robot_id, False, self._on_unload_drawer_unlocked)
            
        elif self.current_state == RCState.UNLOAD_DRAWER_OPENED:
            # 서랍 열림 상태에서 수령 확인 버튼 대기
            self.get_logger().info('수령 확인 버튼 대기 중...')
            
        elif self.current_state == RCState.UNLOAD_DOOR_CHECK:
            # 수령 확인용 서랍 열림 상태 확인 시작 (수령 확인 버튼에 의해 트리거됨)
            self._check_unload_door_state()
            
        elif self.current_state == RCState.UNLOAD_ITEM_CHECK:
            # 수령 확인용 물품 적재 확인 시작
            self.ioc_client.check_item_loaded(self.robot_id, self._on_unload_item_loaded_response)
            
        elif self.current_state == RCState.UNLOAD_DRAWER_CLOSING:
            # 수령 확인용 서랍 잠금 시작
            self.ioc_client.control_lock(self.robot_id, True, self._on_unload_drawer_locked)
            
        # 5. 복귀 단계
        elif self.current_state == RCState.RETURN_COUNTDOWN:
            # 복귀 카운트다운 시작
            self.gui_client.start_return_countdown(self._on_return_countdown_complete)
            
        elif self.current_state == RCState.RETURN_MOVING:
            # 복귀 이동 시작
            self.nav_client.go_to_pose(
                self.location_manager.get_pose(0, yaw=1.57),  # LOB_WAITING location_id=0
                self._on_return_navigation_complete
            )
            
        elif self.current_state == RCState.RETURN_COMPLETE:
            # 복귀 완료 처리
            self.get_logger().info('🏠 복귀 완료')
            self.change_robot_state(2)
            self.current_state = RCState.IDLE
            self.current_task = None
            self.current_goal_handle = None
            self.get_logger().info(f'🔄 시스템 초기화 완료: IDLE 상태')

    def update_robot_pose(self):
        """로봇의 현재 위치를 업데이트"""
        try:
            # map 프레임에서 base_link 프레임으로의 변환 가져오기
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            
            # 위치 정보 업데이트
            self.current_pose.position.x = transform.transform.translation.x
            self.current_pose.position.y = transform.transform.translation.y
            self.current_pose.position.z = transform.transform.translation.z
            self.current_pose.orientation = transform.transform.rotation
            
            # 쿼터니언을 오일러 각도로 변환 (yaw만)
            euler = tf_transformations.euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])
            yaw = euler[2]  # z축 회전 (yaw)
            
            # RMS에 현재 위치 발행
            self.rms_client.publish_roomie_pose(
                self.current_floor_id,
                self.current_pose.position.x,
                self.current_pose.position.y,
                yaw
            )
            
            if self.debug_mode:
                self.get_logger().info(f'📍 위치 업데이트: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}, yaw={yaw:.2f}')
                
        except TransformException as e:
            # tf 변환을 찾을 수 없는 경우 (로봇이 아직 초기화되지 않았거나 amcl이 실행되지 않은 경우)
            self.get_logger().warn(f'⚠️ TF 변환 실패: {e}')
            self.get_logger().warn(f'⚠️ map → base_link 변환을 찾을 수 없습니다. AMCL이 실행 중인지 확인하세요.')
            # 기본값으로 위치 발행
            self.rms_client.publish_roomie_pose(self.current_floor_id, 0.0, 0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f'❌ 위치 업데이트 중 오류: {e}')
            # 기본값으로 위치 발행
            self.rms_client.publish_roomie_pose(self.current_floor_id, 0.0, 0.0, 0.0)
    
    def get_current_robot_pose(self):
        """현재 로봇 위치 반환"""
        return self.current_pose
    
    def get_current_robot_position(self):
        """현재 로봇 위치 (x, y, yaw) 반환"""
        euler = tf_transformations.euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])
        yaw = euler[2]
        return (
            self.current_pose.position.x,
            self.current_pose.position.y,
            yaw
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        rc_node = RCNode()
        rclpy.spin(rc_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'rc_node' in locals():
            rc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()