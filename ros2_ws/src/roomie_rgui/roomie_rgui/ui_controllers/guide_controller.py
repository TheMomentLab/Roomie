"""
GuideController - 길안내 관련 화면들을 처리하는 컨트롤러
GUIDANCE_SCREEN, INPUT_METHOD_SELECTION, CARD_KEY_WAITING,
REGISTERING, RECHECKING, GUIDE_REQUEST, DESTINATION_ARRIVED
"""

from PyQt6.QtWidgets import QPushButton
from PyQt6.QtCore import QTimer
from .base_controller import BaseController
import socket
import threading
import numpy as np
import cv2
from PyQt6.QtGui import QImage, QPixmap


class GuideController(BaseController):
    def __init__(self, widget, screen_manager, node, ui_filename):
        super().__init__(widget, screen_manager, node, ui_filename)
        # 가이드 화면은 활성화 시점에 이벤트를 연결
        self.card_key_countdown_timer: QTimer | None = None
        self.card_key_countdown_remaining: int = 0
        # REGISTERING용 UDP 영상 수신기
        self.udp_thread: threading.Thread | None = None
        self.udp_running: bool = False
        self.udp_sock: socket.socket | None = None
        self.latest_frame_rgb: np.ndarray | None = None
        self.latest_frame_lock = threading.Lock()
        self.video_timer: QTimer | None = None
        self.video_label = None

    def on_screen_activated(self):
        """화면이 활성화될 때 호출됨"""
        self.setup_events()

    def setup_events(self):
        """현재 활성화된 가이드 화면의 이벤트 설정"""
        if "GUIDANCE_SCREEN.ui" in self.ui_filename:
            self.setup_guidance_screen_events()
        elif "INPUT_METHOD_SELECTION.ui" in self.ui_filename:
            self.setup_input_method_selection_events()
        elif "CARD_KEY_WAITING.ui" in self.ui_filename:
            self.setup_card_key_waiting_events()
        elif "REGISTERING.ui" in self.ui_filename:
            self.setup_registering_events()
        elif "RECHECKING.ui" in self.ui_filename:
            self.setup_rechecking_events()
        elif "GUIDE_REQUEST.ui" in self.ui_filename:
            self.setup_guide_request_events()
        elif "DESTINATION_ARRIVED.ui" in self.ui_filename:
            self.setup_destination_arrived_events()
        else:
            self.log_info("알 수 없는 가이드 화면 - 기본 설정만 수행")

    # GUIDANCE_SCREEN
    def setup_guidance_screen_events(self):
        self.log_info("GUIDANCE_SCREEN 준비")
        # 전체 화면 터치 영역이 있다면 연결
        self.setup_touch_event("fullScreenTouchArea", self.on_guidance_touch)

    def on_guidance_touch(self):
        self.log_info("GUIDANCE_SCREEN 터치")
        # 필요 시 다음 화면으로 전환 로직을 여기서 구현

    # INPUT_METHOD_SELECTION
    def setup_input_method_selection_events(self):
        self.log_info("INPUT_METHOD_SELECTION 준비")
        # 카드키 입력만 사용
        self.setup_button_event("cardKeyButton", self.on_select_card_key)

    def on_select_card_key(self):
        self.log_info("카드키 입력 방식 선택")
        # 다음 화면으로 전환 (예: 카드키 대기)
        self.screen_manager.show_screen("CARD_KEY_WAITING")

    # CARD_KEY_WAITING
    def setup_card_key_waiting_events(self):
        self.log_info("CARD_KEY_WAITING 준비")
        self.setup_button_event("cancelButton", self.on_cancel_waiting)
        # 카운트다운 초기화 및 시작
        self._start_card_key_countdown()

    def on_cancel_waiting(self):
        self.log_info("카드키 대기 취소")
        self._stop_card_key_countdown()
        self.screen_manager.show_screen("INPUT_METHOD_SELECTION")

    # 카드키 대기 카운트다운 로직 (라벨 숫자만 갱신, 타임아웃 시 추가 동작 없음)
    def _start_card_key_countdown(self):
        try:
            from PyQt6.QtWidgets import QLabel
            countdown_label = self.widget.findChild(QLabel, "countdownCircle")
            if countdown_label is None:
                self.log_warn("countdownCircle 라벨을 찾을 수 없어 카운트다운을 시작하지 않습니다")
                return
            # 이미 타이머가 있으면 정지
            if self.card_key_countdown_timer is not None:
                self.card_key_countdown_timer.stop()
                self.card_key_countdown_timer.deleteLater()
                self.card_key_countdown_timer = None
            # 초기 남은 시간: 라벨의 현재 숫자에서 파싱, 실패 시 30
            try:
                initial_text = countdown_label.text().strip()
                self.card_key_countdown_remaining = int(initial_text) if initial_text.isdigit() else 30
            except Exception:
                self.card_key_countdown_remaining = 30
            # 타이머 생성 및 시작
            self.card_key_countdown_timer = QTimer(self.widget)
            self.card_key_countdown_timer.setInterval(1000)
            self.card_key_countdown_timer.timeout.connect(self._on_card_key_countdown_tick)
            self.card_key_countdown_timer.start()
            # 첫 표시 동기화
            countdown_label.setText(str(self.card_key_countdown_remaining))
            self.log_info(f"카운트다운 시작: {self.card_key_countdown_remaining}초")
        except Exception as e:
            self.log_error(f"카운트다운 시작 실패: {e}")

    def _on_card_key_countdown_tick(self):
        try:
            from PyQt6.QtWidgets import QLabel
            countdown_label = self.widget.findChild(QLabel, "countdownCircle")
            if countdown_label is None:
                self._stop_card_key_countdown()
                return
            if self.card_key_countdown_remaining > 0:
                self.card_key_countdown_remaining -= 1
                countdown_label.setText(str(self.card_key_countdown_remaining))
            if self.card_key_countdown_remaining <= 0:
                # 타임아웃 동작은 요구하지 않으므로, 숫자 0에서 정지만 수행
                self._stop_card_key_countdown()
        except Exception as e:
            self.log_error(f"카운트다운 틱 처리 실패: {e}")

    def _stop_card_key_countdown(self):
        if self.card_key_countdown_timer is not None:
            self.card_key_countdown_timer.stop()
            self.card_key_countdown_timer.deleteLater()
            self.card_key_countdown_timer = None
            self.log_info("카운트다운 정지")

    # REGISTERING
    def setup_registering_events(self):
        self.log_info("REGISTERING 준비")
        # UI에 backButton만 정의되어 있음
        self.setup_button_event("backButton", self.on_cancel_registering)
        # 영상 수신 시작
        self._start_registering_video()

    def on_cancel_registering(self):
        self.log_info("등록 취소")
        self._stop_registering_video()
        self.screen_manager.show_screen("INPUT_METHOD_SELECTION")

    def on_registering_done(self):
        self.log_info("등록 완료")
        self._stop_registering_video()
        self.screen_manager.show_screen("RECHECKING")

    # === REGISTERING 영상 수신/표시 ===
    def _start_registering_video(self):
        try:
            # 표시 라벨 준비 (detectionAreaFrame 내부에 꽉 차게)
            from PyQt6.QtWidgets import QLabel
            detection_frame = self.widget.findChild(QLabel, "detectionAreaFrame")
            if detection_frame is None:
                # 실제 타입은 QFrame이나, findChild 시 타입은 상관없음
                detection_frame = self.widget.findChild(object, "detectionAreaFrame")
            if detection_frame is None:
                self.log_warn("detectionAreaFrame을 찾을 수 없어 영상 표시를 생략합니다")
                return
            if self.video_label is None:
                self.video_label = QLabel(detection_frame)
                self.video_label.setGeometry(0, 0, detection_frame.width(), detection_frame.height())
                self.video_label.setScaledContents(True)
                self.video_label.raise_()
            # 수신 스레드 시작
            if self.udp_running:
                self._stop_registering_video()
            self.udp_running = True
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            host = '0.0.0.0'
            port = int(self._get_env("RGUI_UDP_PORT", 5005))
            self.udp_sock.bind((host, port))
            self.udp_sock.settimeout(0.5)
            self.udp_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
            self.udp_thread.start()
            # GUI 타이머로 주기적 갱신
            if self.video_timer is None:
                self.video_timer = QTimer(self.widget)
                self.video_timer.setInterval(66)  # ~15fps
                self.video_timer.timeout.connect(self._update_video_label)
            self.video_timer.start()
            self.log_info("REGISTERING 영상 수신 시작")
        except Exception as e:
            self.log_error(f"REGISTERING 영상 수신 시작 실패: {e}")

    def _stop_registering_video(self):
        try:
            if self.video_timer is not None:
                self.video_timer.stop()
            self.udp_running = False
            if self.udp_sock is not None:
                try:
                    self.udp_sock.close()
                except Exception:
                    pass
                self.udp_sock = None
            if self.udp_thread is not None and self.udp_thread.is_alive():
                self.udp_thread.join(timeout=0.5)
            self.udp_thread = None
            self.log_info("REGISTERING 영상 수신 정지")
        except Exception as e:
            self.log_warn(f"REGISTERING 영상 수신 정지 중 경고: {e}")

    def _udp_receive_loop(self):
        while self.udp_running and self.udp_sock is not None:
            try:
                data, _ = self.udp_sock.recvfrom(65536)
                if not data:
                    continue
                arr = np.frombuffer(data, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                with self.latest_frame_lock:
                    self.latest_frame_rgb = rgb
            except socket.timeout:
                continue
            except Exception:
                continue

    def _update_video_label(self):
        if self.video_label is None:
            return
        frame = None
        with self.latest_frame_lock:
            if self.latest_frame_rgb is not None:
                frame = self.latest_frame_rgb.copy()
        if frame is None:
            return
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qimg))

    def _get_env(self, key: str, default):
        import os
        return os.environ.get(key, default)

    # RECHECKING
    def setup_rechecking_events(self):
        self.log_info("RECHECKING 준비")
        self.setup_button_event("reenterButton", self.on_reenter)
        self.setup_button_event("confirmButton", self.on_recheck_confirm)

    def on_reenter(self):
        self.log_info("재입력 선택")
        self.screen_manager.show_screen("REGISTERING")

    def on_recheck_confirm(self):
        self.log_info("확인 완료")
        self.screen_manager.show_screen("GUIDE_REQUEST")

    # GUIDE_REQUEST
    def setup_guide_request_events(self):
        self.log_info("GUIDE_REQUEST 준비")
        # 뒤로가기 버튼
        self.setup_button_event("backButton", self.on_guide_request_back)
        # 카드 전체 터치 영역
        self.setup_button_event("touchButton", self.on_request_guidance)

    def on_request_guidance(self):
        self.log_info("길안내 요청")
        # 길안내 시작 이벤트 발행 (참고: rgui_node에서 10=길안내 시작 정의)
        self.publish_event(event_id=10, detail="")
        # 입력 방식 선택 화면으로 전환
        self.screen_manager.show_screen("INPUT_METHOD_SELECTION")

    def on_guide_request_back(self):
        self.log_info("GUIDE_REQUEST 뒤로가기")
        self.screen_manager.show_screen("TOUCH_SCREEN")

    # DESTINATION_ARRIVED
    def setup_destination_arrived_events(self):
        self.log_info("DESTINATION_ARRIVED 준비")
        self.setup_button_event("okButton", self.on_destination_confirm)

    def on_destination_confirm(self):
        self.log_info("도착 확인")
        # 길안내 종료 이벤트 발행 (참고: rgui_node에서 11=길안내 종료 정의)
        self.publish_event(event_id=11, detail="") 