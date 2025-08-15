#!/usr/bin/env python3

"""GC 상태 기계 노드"""

import time
from dataclasses import dataclass
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit

from std_srvs.srv import Trigger

from roomie_msgs.msg import RobotGuiEvent, RobotState, Tracking
from roomie_msgs.srv import SetVSMode, Location
from roomie_msgs.action import Enroll
# from roomie_msgs.msg import ReadCardRequest, ReadCardResponse  # IOC 통신 사용 시 활성화
from roomie_rc.location_manager import LocationManager


class GCState:
    # 시나리오 1: 목적지 입력 대기
    WAITING_DEST_INPUT = 'waiting_dest_input'
    # 시나리오 2: 등록 요청 보냄
    ENROLL_REQUESTED = 'enroll_requested'
    # 시나리오 2: 등록 중
    ENROLLING = 'enrolling'
    # 시나리오 2: 등록 완료 후 추적모드 전환 요청 대기
    WAITING_TRACKING_REQUEST = 'waiting_tracking_request'
    # 시나리오 3: 길안내 이동 시작
    START_MOVING = 'start_moving'
    # 시나리오 3: 길안내 주행 중
    NAVIGATING = 'navigating'
    # 시나리오 4: 목적지 도착
    ARRIVED = 'arrived'
    # 시나리오 4: 추적 중지 요청
    STOP_TRACKING = 'stop_tracking'
    # 시나리오 4: 복귀 전 대기
    WAIT_BEFORE_RETURN = 'wait_before_return'
    # 시나리오 4: 대기 위치로 복귀
    RETURNING = 'returning'


@dataclass
class GCConfig:
    debug_mode: bool = True
    start_state: str = GCState.WAITING_DEST_INPUT
    robot_id: int = 0
    task_id: int = 0
    default_destination_id: int = 101  # debug에서 기본 목적지

    # Topics
    gui_event_topic: str = '/robot_gui/event'
    robot_state_topic: str = '/roomie/status/robot_state'
    vs_set_mode_service: str = '/vs/command/set_vs_mode'
    vs_stop_tracking_service: str = '/vs/command/stop_tracking'
    vs_tracking_topic: str = '/vs/tracking'
    ioc_read_card_request: str = '/ioc/read_card_request'
    ioc_read_card_response: str = '/ioc/read_card_response'


class RoomieGCNode(Node):
    def __init__(self) -> None:
        super().__init__('roomie_gc_node')

        # Parameters
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('start_state', GCState.WAITING_DEST_INPUT)
        self.config = GCConfig(
            debug_mode=self.get_parameter('debug_mode').value,
            start_state=self.get_parameter('start_state').value,
        )

        self.get_logger().info(f'debug_mode={self.config.debug_mode}')
        self.get_logger().info(f'start_state={self.config.start_state}')

        # State
        self.current_state = self.config.start_state
        self.robot_state_id = 2  # 기본: 작업대기
        self.destination_id: Optional[int] = None
        # One-shot guards
        self.enroll_goal_sent = False
        self.nav_goal_sent = False
        self.stop_tracking_requested = False
        self.pending_tracking_request = False  # 등록 중(ENROLLING) 동안 106 수신 시 보류 플래그
        self.return_nav_goal_sent = False  # 복귀 주행 goal 송신 여부
        self._card_mock_timer = None  # RGUI 103에 대한 모의(IOC 우회) 타이머

        # Callback group
        self.cb_group = ReentrantCallbackGroup()

        # Publishers
        self.gui_event_pub = self.create_publisher(RobotGuiEvent, self.config.gui_event_topic, 10)
        self.robot_state_pub = self.create_publisher(RobotState, self.config.robot_state_topic, 10)
        # IOC card read topics (비활성화: 타이머 우회 사용)
        # self.card_request_pub = self.create_publisher(ReadCardRequest, self.config.ioc_read_card_request, 10)
        # SpeedLimit publisher (Nav2 DWB)
        self.declare_parameter('speed_limit_topic', 'speed_limit')
        # SpeedLimit 기본값: 절대값 모드 사용(percentage=False).
        # - LOST(2) 시 거의 정지(lost_stop_speed)
        # - RESUME/NORMAL 시 0.0으로 리미터 해제
        self.declare_parameter('sl_percentage', False)
        self.declare_parameter('sl_slow_scale', 0.05)
        self.declare_parameter('sl_resume_scale', 0.0)
        self.declare_parameter('sl_lost_stop_speed', 0.001)
        self.declare_parameter('sl_repeat_count', 5)
        self.declare_parameter('sl_repeat_period', 0.1)
        self.speed_limit_topic = self.get_parameter('speed_limit_topic').value
        self.sl_percentage = bool(self.get_parameter('sl_percentage').value)
        self.sl_slow_scale = float(self.get_parameter('sl_slow_scale').value)
        self.sl_resume_scale = float(self.get_parameter('sl_resume_scale').value)
        self.sl_lost_stop_speed = float(self.get_parameter('sl_lost_stop_speed').value)
        self.sl_repeat_count = int(self.get_parameter('sl_repeat_count').value)
        self.sl_repeat_period = float(self.get_parameter('sl_repeat_period').value)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, self.speed_limit_topic, QoSProfile(depth=10))
        # Repeat helper like speed_limit_controller
        self._sl_repeat_remaining = 0
        self._sl_last_msg: Optional[SpeedLimit] = None
        self.create_timer(self.sl_repeat_period, self._sl_repeat_publish)

        # Subscribers
        self.tracking_sub = self.create_subscription(Tracking, self.config.vs_tracking_topic, self._on_tracking, QoSProfile(depth=10))
        self.gui_event_sub = self.create_subscription(RobotGuiEvent, self.config.gui_event_topic, self._on_gui_event, 10)
        # IOC 응답 구독 비활성화: 타이머 우회 사용
        # self.card_response_sub = self.create_subscription(ReadCardResponse, self.config.ioc_read_card_response, self._on_card_response, 10)

        # Service clients
        self.set_vs_mode_client = self.create_client(SetVSMode, self.config.vs_set_mode_service)
        self.stop_tracking_client = self.create_client(Trigger, self.config.vs_stop_tracking_service)
        self.location_client = self.create_client(Location, '/vs/command/location')  # 필요 시 사용

        # Action clients
        self.enroll_client = ActionClient(self, Enroll, '/vs/action/enroll')
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Periodic timers
        self.create_timer(5.0, self._publish_robot_state)

        # Utilities
        self.location_manager = LocationManager()
        self._wait_before_return_timer = None  # 복귀 전 대기 타이머

        # Init logs and connection checks
        self._log_interfaces()
        if not self._wait_for_interfaces():
            self.get_logger().error('필수 인터페이스 연결 실패. 노드를 종료합니다.')
            return

        # ===== Mid-run start handling =====
        # start_state로 중간 시작 시 목적지가 비어 있으면 디버그 기본값으로 채움
        if self.current_state != GCState.WAITING_DEST_INPUT and self.destination_id is None:
            if self.config.debug_mode:
                self.destination_id = self.config.default_destination_id
                self.get_logger().info(f'[DEBUG] start_state={self.current_state} → 목적지 기본 설정: {self.destination_id}')

        # Kick off state machine
        self.create_timer(0.1, self._tick)

    # ===== Interface Helpers =====
    def _log_interfaces(self) -> None:
        # 기본 INFO 레벨에서는 노이즈를 줄이기 위해 DEBUG로만 노출
        self.get_logger().debug('Checking interfaces:')
        self.get_logger().debug(f' - Service: {self.config.vs_set_mode_service}')
        self.get_logger().debug(f' - Service: {self.config.vs_stop_tracking_service}')
        self.get_logger().debug(' - Action: /vs/action/enroll')
        self.get_logger().debug(f' - Topic pub: {self.config.ioc_read_card_request}')
        self.get_logger().debug(f' - Topic sub: {self.config.ioc_read_card_response}')
        self.get_logger().debug(' - Action: navigate_to_pose')
        self.get_logger().debug(f' - Topic pub: {self.config.gui_event_topic}')
        self.get_logger().debug(f' - Topic sub: {self.config.vs_tracking_topic}')
        self.get_logger().debug(f' - Topic pub: {self.config.robot_state_topic}')

    def _wait_for_interfaces(self) -> bool:
        import time

        def wait_with_logs(check_fn, label: str, timeout: float = 60.0, interval: float = 5.0) -> bool:
            start = time.time()
            next_log = 0.0
            while True:
                if check_fn():
                    self.get_logger().info(f"연결 완료: {label}")
                    return True
                elapsed = time.time() - start
                if elapsed >= timeout:
                    self.get_logger().error(f"연결 실패(타임아웃 {timeout:.0f}s): {label}")
                    return False
                if elapsed >= next_log:
                    remaining = max(0.0, timeout - elapsed)
                    self.get_logger().info(f"연결 중: {label} ... (경과 {elapsed:.1f}s, 남은 {remaining:.1f}s)")
                    next_log += interval
                time.sleep(0.2)

        ok = True
        ok &= wait_with_logs(lambda: self.set_vs_mode_client.wait_for_service(timeout_sec=0.1), '/vs/command/set_vs_mode')
        ok &= wait_with_logs(lambda: self.stop_tracking_client.wait_for_service(timeout_sec=0.1), '/vs/command/stop_tracking')
        ok &= wait_with_logs(lambda: self.enroll_client.wait_for_server(timeout_sec=0.1), '/vs/action/enroll')
        ok &= wait_with_logs(lambda: self.nav2_client.wait_for_server(timeout_sec=0.1), 'navigate_to_pose (Nav2 action)')

        if ok:
            self.get_logger().info('모든 필수 인터페이스 연결 완료. 노드 준비됨')
        return ok

    # ===== State Machine =====
    def _tick(self) -> None:
        if self.current_state == GCState.WAITING_DEST_INPUT:
            self._state_waiting_dest_input()
        elif self.current_state == GCState.ENROLL_REQUESTED:
            self._state_enroll_requested()
        elif self.current_state == GCState.ENROLLING:
            self._state_enrolling()
        elif self.current_state == GCState.WAITING_TRACKING_REQUEST:
            # 등록 완료 후 RGUI 106을 대기하는 상태이므로 폴링 로직은 없음
            pass
        elif self.current_state == GCState.START_MOVING:
            self._state_start_moving()
        elif self.current_state == GCState.NAVIGATING:
            self._state_navigating()
        elif self.current_state == GCState.ARRIVED:
            self._state_arrived()
        elif self.current_state == GCState.STOP_TRACKING:
            self._state_stop_tracking()
        elif self.current_state == GCState.WAIT_BEFORE_RETURN:
            self._state_wait_before_return()
        elif self.current_state == GCState.RETURNING:
            self._state_returning()


    # ===== 시나리오 1: 시작 ~ GUI 106 수신 =====
    def _on_gui_event(self, msg: RobotGuiEvent) -> None:
        # 102: 사용자 점유, 103: 카드키 입력 선택, 106: 인식모드 전환 요청
        if msg.rgui_event_id == 102:
            # 작업대기 -> 목적지 입력대기 (중복 수신 시 NOP 처리 로그 남김)
            self.get_logger().info('RGUI 102 수신: 사용자 점유 상태')
            if self.robot_state_id == 21:
                self.get_logger().info('RGUI 102 수신: 이미 robot_state_id=21 상태 - NOP 처리')
            else:
                prev = self.robot_state_id
                self.robot_state_id = 21
                self.get_logger().info(f'RobotState: {prev} -> 21 (작업대기 → 길안내 목적지 입력대기)')
                self._publish_robot_state()
            if self.current_state != GCState.WAITING_DEST_INPUT:
                self.get_logger().info(f'RGUI 102 처리: current_state={self.current_state} → WAITING_DEST_INPUT로 정렬')
            self.current_state = GCState.WAITING_DEST_INPUT
        elif msg.rgui_event_id == 103:
            self.get_logger().info('RGUI 103 수신: [카드키로 입력] 선택')
            # IOC 통신 우회: 5초 후 목적지=101로 저장하고 RGUI 9 발행
            def _mock_complete():
                try:
                    self.destination_id = 101
                    self.get_logger().info('RGUI 103 처리(우회): 5초 후 목적지=101 설정 완료')
                    self._send_gui_event(9, '101')
                finally:
                    # one-shot 타이머 해제
                    if self._card_mock_timer is not None:
                        self._card_mock_timer.cancel()
                        self._card_mock_timer = None
            # 기존 타이머가 있으면 취소 후 재설정
            if self._card_mock_timer is not None:
                self._card_mock_timer.cancel()
                self._card_mock_timer = None
            self._card_mock_timer = self.create_timer(5.0, _mock_complete)
            self.get_logger().info('RGUI 103 처리(우회): 5초 후 목적지=101 및 RGUI 9 발행 예정')
        elif msg.rgui_event_id == 106:
            self.get_logger().info(f'RGUI 106 수신: 인식모드 전환 요청 (current_state={self.current_state}, destination_id={self.destination_id})')
            # 등록 이전이면 등록모드로, 등록 완료 후면 추적모드로
            if self.current_state in (GCState.WAITING_DEST_INPUT, GCState.ENROLL_REQUESTED) and self.destination_id is not None:
                # 등록 모드 전환으로 진행
                self.current_state = GCState.ENROLL_REQUESTED
            elif self.current_state == GCState.WAITING_TRACKING_REQUEST:
                # 추적 모드 전환 및 주행 시작으로 진행
                self.current_state = GCState.START_MOVING
            elif self.current_state == GCState.ENROLLING:
                # 등록 중에는 추적 모드 전환을 보류하고 등록 완료 시 처리
                self.pending_tracking_request = True
                self.get_logger().info('등록 중 106 수신: 추적 모드 전환 요청 보류 설정(pending_tracking_request=True)')

    # ===== 콜백: IOC 카드 응답 (타입드 메시지 기준, 현재 구독 비활성) =====
    def _on_card_response(self, msg) -> None:
        # 기대 타입: roomie_msgs/ReadCardResponse (success: bool, location_id: int32)
        try:
            success = bool(getattr(msg, 'success', False))
            location_id = int(getattr(msg, 'location_id', -1))
        except Exception as e:
            self.get_logger().error(f'IOC read_card_response 파싱 실패: {e} | msg={msg}')
            return
        self.get_logger().info(f'IOC read_card_response 수신: success={success}, location_id={location_id}')
        if success and location_id > 0:
            self.destination_id = location_id
            self.get_logger().info(f'카드 인식 성공: location_id={self.destination_id}')
            self._send_gui_event(9, str(self.destination_id))
        else:
            self.get_logger().warn('카드 인식 실패')

    # ===== [시나리오 1] 목적지 입력 대기 =====
    def _state_waiting_dest_input(self) -> None:
        # 이벤트 기반 처리. 여기서는 아무 것도 하지 않음.
        return

    # ===== [시나리오 2] 등록 모드 전환 요청 =====
    def _state_enroll_requested(self) -> None:
        # VS 모드 전환: 등록(1)
        req = SetVSMode.Request()
        req.robot_id = self.config.robot_id
        req.mode_id = 1
        self.get_logger().info('VS 등록 모드 전환 요청 → 서비스 호출 시작')
        future = self.set_vs_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(f'VS 등록 모드 전환 결과 수신: success={getattr(f.result(), "success", False)}'))
        self.current_state = GCState.ENROLLING

    # ===== [시나리오 2] 투숙객 등록 =====
    def _state_enrolling(self) -> None:
        # 등록 액션 호출 (1회만)
        if self.enroll_goal_sent:
            return
        self.enroll_goal_sent = True
        goal = Enroll.Goal()
        goal.duration_sec = 5.0
        self.enroll_client.send_goal_async(goal).add_done_callback(self._on_enroll_goal_response)
        self.get_logger().info('투숙객 등록 요청')

    # ===== 등록 액션 콜백들 =====
    def _on_enroll_goal_response(self, future) -> None:
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_enroll_result)

    def _on_enroll_result(self, future) -> None:
        res = future.result().result
        self.get_logger().info(f'등록 완료: success={res.success}')
        self._send_gui_event(10, '길안내 이동 시작')
        # 등록 중 106이 먼저 들어온 경우 바로 추적 모드 전환으로 진행
        if self.pending_tracking_request:
            self.get_logger().info('보류된 106 처리: START_MOVING으로 전이')
            self.pending_tracking_request = False
            self.current_state = GCState.START_MOVING
        else:
            self.current_state = GCState.WAITING_TRACKING_REQUEST
        # 다음 단계 대비 초기화
        self.nav_goal_sent = False

    # ===== [시나리오 3] 추적 모드 전환 및 주행 시작 =====
    def _state_start_moving(self) -> None:
        # VS 모드: 추적(2) 전환, 로봇 상태 22, Nav2 goal 전송
        req = SetVSMode.Request()
        req.robot_id = self.config.robot_id
        req.mode_id = 2
        self.get_logger().info('VS 추적 모드 전환 요청 → 서비스 호출 시작')
        self.set_vs_mode_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f'VS 추적 모드 전환 결과 수신: success={getattr(f.result(), "success", False)}')
        )
        # 로봇 상태 변경: 21 -> 22
        self.robot_state_id = 22
        self.get_logger().info('RobotState: 21 -> 22 (목적지 입력대기 → 길안내 이동)')
        self._publish_robot_state()
        self.get_logger().info('길안내 주행 시작 (Nav2)')
        # Nav2 목표 전송
        if not self.nav_goal_sent:
            self.nav_goal_sent = True
            goal = NavigateToPose.Goal()
            goal.pose = self.location_manager.get_pose(self.destination_id, yaw=0.0)
            if goal.pose is not None:
                self.get_logger().info(
                    f'Nav2 Goal 전송: frame_id={goal.pose.header.frame_id}, '
                    f'x={goal.pose.pose.position.x:.3f}, y={goal.pose.pose.position.y:.3f}'
                )
            self.nav2_client.send_goal_async(goal).add_done_callback(self._on_nav_goal_response)
        self.current_state = GCState.NAVIGATING

    # ===== [시나리오 3 보조] VS 추적 이벤트 처리 =====
    def _on_tracking(self, msg: Tracking) -> None:
        # 감속/이탈/재개: 주행 단계에서만 처리
        self.get_logger().info(f'tracking: id={msg.id}, event={msg.event}')
        if self.current_state not in (GCState.START_MOVING, GCState.NAVIGATING):
            self.get_logger().debug(f'Ignore tracking(event={msg.event}) in state={self.current_state}')
            return

        # SpeedLimit 적용
        sl = SpeedLimit()
        sl.percentage = self.sl_percentage
        if msg.event == 1:  # slow_down
            sl.speed_limit = self.sl_slow_scale
            self._publish_speed_limit_with_repeat(sl, 'SLOW_DOWN')
        elif msg.event == 2:  # lost
            sl.speed_limit = self.sl_lost_stop_speed if not sl.percentage else 0.0
            self._publish_speed_limit_with_repeat(sl, 'LOST')
        elif msg.event == 3:  # resume
            sl.speed_limit = self.sl_resume_scale if not sl.percentage else 1.0
            self._publish_speed_limit_with_repeat(sl, 'RESUME/NORMAL')
        elif msg.event == 0:  # none → normal
            sl.speed_limit = self.sl_resume_scale if not sl.percentage else 1.0
            self._publish_speed_limit_with_repeat(sl, 'NONE→NORMAL')

        # GUI 통지 (주행 단계에서만)
        if msg.event == 2:  # lost
            self._send_gui_event(21, '투숙객 이탈')
        elif msg.event == 3:  # resume
            self._send_gui_event(22, '투숙객 이탈 후 재등록')

    # ===== [시나리오 3] 길안내 주행 중 =====
    def _state_navigating(self) -> None:
        # Nav2 액션 결과 콜백에서 상태 전이 처리. 여기서는 폴링 없음.
        pass

    # ===== Nav2 액션 콜백들 =====
    def _on_nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        accepted = getattr(goal_handle, 'accepted', True)
        self.get_logger().info(f'Nav2 Goal 수락={accepted}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        try:
            goal_result = future.result()
            status = getattr(goal_result, 'status', None)
            self.get_logger().info(f'Nav2 결과 수신: status={status} → 도착 처리 전이')
        except Exception as e:
            self.get_logger().error(f'Nav2 결과 수신 에러: {e}')
        self.current_state = GCState.ARRIVED
        # 다음 단계 대비 초기화
        self.stop_tracking_requested = False

        self._state_arrived()

    # ===== [시나리오 4] 목적지 도착 처리 =====
    def _state_arrived(self) -> None:
        # 도착 시 추적 중지 단계로 전이 (GUI 이벤트는 stop_tracking 성공 후 전송)
        self.current_state = GCState.STOP_TRACKING

    # ===== [시나리오 4] 추적 중지 및 대기모드 전환 =====
    def _state_stop_tracking(self) -> None:
        if self.stop_tracking_requested:
            return
        self.stop_tracking_requested = True
        req = Trigger.Request()
        future = self.stop_tracking_client.call_async(req)
        self.get_logger().info('VS 추적 중지 요청')
        future.add_done_callback(self._on_stop_tracking_done)


    def _on_stop_tracking_done(self, future) -> None:
        try:
            res = future.result()
            self.get_logger().info(f'추적 중지 결과 수신: success={getattr(res, "success", False)}, message={getattr(res, "message", "")}')
        except Exception as e:
            self.get_logger().error(f'추적 중지 결과 수신 에러: {e}')
            return
        self._send_gui_event(11, '길안내 이동 종료')
        set_req = SetVSMode.Request()
        set_req.robot_id = self.config.robot_id
        set_req.mode_id = 0
        # VS 대기모드(0) 전환 결과 로그 추가
        self.set_vs_mode_client.call_async(set_req).add_done_callback(
            lambda f: self.get_logger().info(
                f'VS 대기 모드 전환 결과 수신: success={getattr(f.result(), "success", False)}'
            )
        )
        # 로봇 상태 변경: 22 -> 30
        self.robot_state_id = 30
        self.get_logger().info('RobotState: 22 -> 30 (길안내 이동 → 대기위치로 이동)')
        self._publish_robot_state()
        # 복귀 전에 10초 대기 상태로 전이
        self.current_state = GCState.WAIT_BEFORE_RETURN

    # ===== [시나리오 4] 복귀 전 대기 =====
    def _state_wait_before_return(self) -> None:
        if self._wait_before_return_timer is not None:
            return
        self.get_logger().info('복귀 전 대기: 10초 대기 시작')

        def _after_wait():
            try:
                self.get_logger().info('복귀 전 대기: 10초 대기 완료 → 복귀 주행으로 전이')
                self.current_state = GCState.RETURNING
            finally:
                if self._wait_before_return_timer is not None:
                    self._wait_before_return_timer.cancel()
                    self._wait_before_return_timer = None

        self._wait_before_return_timer = self.create_timer(10.0, _after_wait)

    # ===== [시나리오 4] 대기위치 복귀 =====
    def _state_returning(self) -> None:
        # 대기 위치(LOB_WAITING=0)로 Nav2 복귀 주행
        if self.return_nav_goal_sent:
            return
        goal_pose = self.location_manager.get_pose(0, yaw=1.57)
        if goal_pose is None:
            self.get_logger().warn('복귀 목표 생성 실패 (location_id=0)')
            return
        self.return_nav_goal_sent = True
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.get_logger().info(
            f'대기위치 복귀 주행 시작 (Nav2): frame_id={goal.pose.header.frame_id}, '
            f'x={goal.pose.pose.position.x:.3f}, y={goal.pose.pose.position.y:.3f}'
        )
        self.nav2_client.send_goal_async(goal).add_done_callback(self._on_return_nav_goal_response)

    def _on_return_nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        accepted = getattr(goal_handle, 'accepted', True)
        self.get_logger().info(f'복귀 Nav2 Goal 수락={accepted}')
        
        if not accepted:
            return
        goal_handle.get_result_async().add_done_callback(self._on_return_nav_result)

    def _on_return_nav_result(self, future) -> None:
        try:
            result = future.result()
            status = getattr(result, 'status', None)
            self.get_logger().info(f'복귀 Nav2 결과 수신: status={status} (대기위치 도착)')
        except Exception as e:
            self.get_logger().error(f'복귀 Nav2 결과 수신 에러: {e}')
        # 도착 후 로봇 상태를 작업대기(2)로 전환
        self.robot_state_id = 2
        self.get_logger().info('RobotState: 30 -> 2 (대기위치로 이동 → 작업대기)')
        self._publish_robot_state()
        # 다음 주기에서 시나리오 종료/대기 유지

    # ===== Utilities =====
    def _publish_robot_state(self) -> None:
        msg = RobotState()
        msg.robot_id = self.config.robot_id
        msg.robot_state_id = self.robot_state_id
        self.robot_state_pub.publish(msg)

    def _send_gui_event(self, event_id: int, detail: str = '') -> None:
        msg = RobotGuiEvent()
        msg.robot_id = self.config.robot_id
        msg.rgui_event_id = event_id
        msg.task_id = self.config.task_id
        msg.timestamp = self.get_clock().now().to_msg()
        msg.detail = detail
        self.gui_event_pub.publish(msg)
        self.get_logger().info(f'GUI 이벤트 전송: id={event_id}, detail={detail}')

    def _destination_to_pose(self, location_id: int) -> PoseStamped:
        return self.location_manager.get_pose(location_id, yaw=0.0)

    # ===== SpeedLimit helper =====
    def _publish_speed_limit_with_repeat(self, sl: SpeedLimit, tag: str) -> None:
        self.speed_limit_pub.publish(sl)
        self._sl_last_msg = sl
        self._sl_repeat_remaining = self.sl_repeat_count
        self.get_logger().info(
            f'SpeedLimit 적용({tag}): {sl.speed_limit} (percentage={sl.percentage})'
        )

    def _sl_repeat_publish(self) -> None:
        if self._sl_repeat_remaining > 0 and self._sl_last_msg is not None:
            self.speed_limit_pub.publish(self._sl_last_msg)
            self._sl_repeat_remaining -= 1


def main() -> None:
    rclpy.init()
    node = RoomieGCNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()



