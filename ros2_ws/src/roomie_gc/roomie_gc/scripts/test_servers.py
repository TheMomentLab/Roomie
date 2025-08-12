#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from roomie_msgs.srv import SetVSMode
from roomie_msgs.action import Enroll
from roomie_msgs.msg import RobotGuiEvent, Tracking
from std_msgs.msg import Empty, String


class TestServers(Node):
    def __init__(self):
        super().__init__('gc_test_servers')
        self.cb = ReentrantCallbackGroup()
        self.vs_mode = 0  # 0: 대기, 1: 등록, 2: 추적

        # VS services
        self.set_vs_mode_srv = self.create_service(SetVSMode, '/vs/command/set_vs_mode', self._set_vs_mode, callback_group=self.cb)
        self.stop_tracking_srv = self.create_service(Trigger, '/vs/command/stop_tracking', self._stop_tracking, callback_group=self.cb)

        # parameters (nav mock removed)

        # VS enroll action
        self.enroll_action = ActionServer(self, Enroll, '/vs/action/enroll', self._enroll_execute, callback_group=self.cb)

        # IOC read card topics (request/response)
        self.card_req_sub = self.create_subscription(Empty, '/ioc/read_card_request', self._on_card_request, 10)
        self.card_resp_pub = self.create_publisher(String, '/ioc/read_card_response', 10)

        self.get_logger().info('GC 테스트 서버가 시작되었습니다')
        # VS tracking publisher (mock)
        self.tracking_pub = self.create_publisher(Tracking, '/vs/tracking', 10)
        self.tracking_timer = None  # VS 모드=2(추적)일 때만 활성

        # RGUI event publisher/subscriber (drive scenario 1)
        self.gui_pub = self.create_publisher(RobotGuiEvent, '/robot_gui/event', 10)
        self.gui_sub = self.create_subscription(RobotGuiEvent, '/robot_gui/event', self._on_gui_event, 10)
        # 순차 이벤트: 102 -> 103 -> (RC가 9를 발행하면) -> 106
        self.step = 0
        self.received_9 = False
        self.sent_second_106 = False
        self.gui_seq_timer = self.create_timer(1.0, self._drive_gui_sequence)

    def _set_vs_mode(self, request, response):
        self.get_logger().info(f'[TEST] set_vs_mode: robot_id={request.robot_id}, mode_id={request.mode_id}')
        # 1초 지연 후 응답
        import time
        time.sleep(1.0)
        # VS 모드 갱신 및 tracking 타이머 제어
        self.vs_mode = request.mode_id
        if self.vs_mode == 2 and self.tracking_timer is None:
            self.tracking_timer = self.create_timer(1.0, self._pub_tracking)
            self.get_logger().info('[TEST] VS tracking 발행 시작 (mode=2)')
        elif self.vs_mode != 2 and self.tracking_timer is not None:
            self.tracking_timer.cancel()
            self.tracking_timer = None
            self.get_logger().info('[TEST] VS tracking 발행 중지 (mode!=2)')
        self.get_logger().info(f'[TEST] VS mode changed -> {self.vs_mode}')
        response.robot_id = request.robot_id
        response.success = True
        return response

    def _stop_tracking(self, request, response):
        self.get_logger().info('[TEST] stop_tracking called')
        # tracking 발행 중지
        if self.tracking_timer is not None:
            self.tracking_timer.cancel()
            self.tracking_timer = None
            self.get_logger().info('[TEST] VS tracking 발행 중지 (stop_tracking)')
        response.success = True
        response.message = 'stopped'
        return response

    def _enroll_execute(self, goal_handle):
        self.get_logger().info('[TEST] enroll started')
        # 3초 동안 1초 간격으로 피드백 발행 후 성공 처리
        import time
        for i in range(3):
            goal_handle.publish_feedback(Enroll.Feedback(progress=float(i + 1) / 3.0))
            time.sleep(1.0)
        goal_handle.succeed()
        result = Enroll.Result()
        result.success = True
        self.get_logger().info('[TEST] enroll finished')
        return result

    def _on_card_request(self, _msg: Empty):
        self.get_logger().info('[TEST] read_card_request 수신')
        # 3초 지연 후 응답 발행
        import threading, time, json
        def worker():
            time.sleep(3.0)
            payload = {'success': True, 'location_id': 101}
            out = String()
            out.data = json.dumps(payload)
            self.card_resp_pub.publish(out)
            self.get_logger().info('[TEST] read_card_response 발행: success=True, location_id=101')
        threading.Thread(target=worker, daemon=True).start()

    def _pub_tracking(self):
        msg = Tracking()
        msg.id = 0
        msg.event = 0
        self.tracking_pub.publish(msg)

    def _drive_gui_sequence(self):
        now = self.get_clock().now().to_msg()
        if self.step == 0:
            ev = RobotGuiEvent()
            ev.robot_id = 0
            ev.rgui_event_id = 102
            ev.task_id = 0
            ev.timestamp = now
            ev.detail = '사용자 점유 상태'
            self.gui_pub.publish(ev)
            self.get_logger().info('[TEST] RGUI 102 발행')
            self.step = 1
        elif self.step == 1:
            ev = RobotGuiEvent()
            ev.robot_id = 0
            ev.rgui_event_id = 103
            ev.task_id = 0
            ev.timestamp = now
            ev.detail = '[카드키로 입력] 선택'
            self.gui_pub.publish(ev)
            self.get_logger().info('[TEST] RGUI 103 발행')
            self.step = 2
        elif self.step == 2:
            if self.received_9:
                ev = RobotGuiEvent()
                ev.robot_id = 0
                ev.rgui_event_id = 106
                ev.task_id = 0
                ev.timestamp = now
                ev.detail = '인식모드 전환 요청'
                self.gui_pub.publish(ev)
                self.get_logger().info('[TEST] RGUI 106 발행 (9 수신 확인 후)')
                self.step = 3
        elif self.step == 3 and self.vs_mode == 0 and self.tracking_timer is None:
            # 시나리오 완료 후 자동 반복 방지: 타이머 중지
            self.get_logger().info('[TEST] 시나리오 완료: VS mode=0, tracking 중지 확인 → GUI 시퀀스 타이머 종료')
            if self.gui_seq_timer is not None:
                self.gui_seq_timer.cancel()
                self.gui_seq_timer = None

    def _on_gui_event(self, msg: RobotGuiEvent):
        # RC가 9(호실 번호 인식 완료)를 발행하면 이후 106 발행 허용
        if msg.rgui_event_id == 9:
            self.received_9 = True
            self.get_logger().info('[TEST] RGUI 9 수신 확인')
        # RC가 길안내 이동 시작(10) 이벤트를 발행하면, RGUI가 두 번째 106을 발행
        if msg.rgui_event_id == 10 and not self.sent_second_106:
            ev = RobotGuiEvent()
            ev.robot_id = 0
            ev.rgui_event_id = 106
            ev.task_id = 0
            ev.timestamp = self.get_clock().now().to_msg()
            ev.detail = '인식모드 전환 요청(추적)'
            self.gui_pub.publish(ev)
            self.get_logger().info('[TEST] RGUI 106 발행 (등록 완료 후 추적모드 전환 요청)')
            self.sent_second_106 = True

        # tracking 타이머는 VS set_vs_mode(mode_id=2) 처리에서 제어함


def main():
    rclpy.init()
    node = TestServers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


