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
        elif "GUI_5_1_RECHECKING.ui" in self.ui_filename:
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
        # GUIDANCE_SCREEN 진입 시 인식모드(추적모드) 전환 요청 발행
        self.publish_event(event_id=106, detail="2")
        # 목적지 표시 업데이트
        try:
            from PyQt6.QtWidgets import QLabel
            dest_label = self.widget.findChild(QLabel, "destinationText")
            if dest_label is not None:
                dest = getattr(self.node, "current_destination", None)
                if dest:
                    dest_label.setText(f"목적지 : 객실{dest}호")
        except Exception as e:
            self.log_warn(f"목적지 라벨 갱신 중 경고: {e}")
        # 영상 표시 시작 (guidanceFrame 내부)
        self._start_udp_video(target_frame_name="guidanceFrame")
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
        # 이미지 보장: qrc→절대경로 폴백
        base_dir = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets"
        self._set_label_pixmap_with_fallback("cardKeyImage", ":/roomie_rgui/assets/rgui_card.png", f"{base_dir}/rgui_card.png")
        self._set_label_pixmap_with_fallback("directInputImage", ":/roomie_rgui/assets/rgui_touch.png", f"{base_dir}/rgui_touch.png")

    def on_select_card_key(self):
        self.log_info("카드키 입력 방식 선택")
        # 다음 화면으로 전환 (예: 카드키 대기)
        self.screen_manager.show_screen("CARD_KEY_WAITING")

    # CARD_KEY_WAITING
    def setup_card_key_waiting_events(self):
        self.log_info("CARD_KEY_WAITING 준비")
        # 취소 버튼은 선택 사항: 있을 때만 연결
        try:
            btn = self.find_widget("cancelButton")
            if btn:
                self.setup_button_event("cancelButton", self.on_cancel_waiting)
        except Exception:
            pass
        # 카운트다운 초기화 및 시작
        self._start_card_key_countdown()
        # 이미지 보장: qrc→절대경로 폴백
        base = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets/rgui_scan.png"
        self._set_label_pixmap_with_fallback("cardKeyImage", ":/roomie_rgui/assets/rgui_scan.png", base)

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
        # REGISTERING 진입 시 인식모드(등록모드) 전환 요청 발행
        self.publish_event(event_id=106, detail="1")
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
            # 공용 UDP 표시 진입점 사용
            self._start_udp_video(target_frame_name="detectionAreaFrame")
            self.log_info("REGISTERING 영상 수신 시작")
        except Exception as e:
            self.log_error(f"REGISTERING 영상 수신 시작 실패: {e}")

    # === 공용 UDP 영상 수신/표시 ===
    def _start_udp_video(self, target_frame_name: str):
        """주어진 프레임 이름 내부에 UDP로 수신한 영상을 표시한다."""
        try:
            # 표시 라벨 준비 (프레임 내부에 꽉 차게)
            from PyQt6.QtWidgets import QLabel, QFrame, QWidget
            target_frame = self.widget.findChild(QLabel, target_frame_name)
            if target_frame is None:
                target_frame = self.widget.findChild(QFrame, target_frame_name)
            if target_frame is None:
                target_frame = self.widget.findChild(QWidget, target_frame_name)
            if target_frame is None:
                self.log_warn(f"{target_frame_name}을(를) 찾을 수 없어 영상 표시를 생략합니다")
                return
            if self.video_label is None:
                self.video_label = QLabel(target_frame)
            self.video_label.setParent(target_frame)
            self.video_label.setGeometry(0, 0, target_frame.width(), target_frame.height())
            self.video_label.setScaledContents(True)
            self.video_label.raise_()
            try:
                # 부모 프레임 리사이즈 시 라벨을 꽉 채우도록 동기화
                original_resize = getattr(target_frame, "resizeEvent", None)
                def _on_target_resize(event):
                    self.video_label.setGeometry(0, 0, target_frame.width(), target_frame.height())
                    if original_resize:
                        type(target_frame).resizeEvent(target_frame, event)
                target_frame.resizeEvent = _on_target_resize
            except Exception:
                pass
            self.video_label.show()

            # 수신 스레드 시작 (중복 방지 위해 기존 정리)
            if self.udp_running:
                self._stop_registering_video()
            self.udp_running = True
            import socket
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            host = '0.0.0.0'
            port = int(self._get_env("RGUI_UDP_PORT", 5005))
            self.udp_sock.bind((host, port))
            self.log_info(f"UDP 소켓 바인드 성공: {host}:{port}")
            self.udp_sock.settimeout(0.5)
            import threading
            self._first_frame_logged = False
            self._first_display_logged = False
            self.udp_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
            self.udp_thread.start()

            # GUI 타이머로 주기적 갱신
            from PyQt6.QtCore import QTimer
            if self.video_timer is None:
                self.video_timer = QTimer(self.widget)
                self.video_timer.setInterval(66)  # ~15fps
                self.video_timer.timeout.connect(self._update_video_label)
            self.video_timer.start()
            self.log_info(f"UDP 영상 수신 시작: frame={target_frame_name}")
        except Exception as e:
            self.log_error(f"UDP 영상 시작 실패({target_frame_name}): {e}")

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

    # 별칭: 공용 정지
    def _stop_udp_video(self):
        self._stop_registering_video()

    def _udp_receive_loop(self):
        while self.udp_running and self.udp_sock is not None:
            try:
                data, _ = self.udp_sock.recvfrom(65536)
                if not data:
                    continue
                arr = np.frombuffer(data, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    if not getattr(self, "_first_frame_logged", False):
                        self.log_warn("UDP 프레임 디코드 실패 - JPEG 포맷 확인 필요")
                        self._first_frame_logged = True
                    continue
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                with self.latest_frame_lock:
                    self.latest_frame_rgb = rgb
                if not getattr(self, "_first_frame_logged", False):
                    h, w = rgb.shape[:2]
                    self.log_info(f"첫 프레임 수신: {w}x{h}")
                    self._first_frame_logged = True
            except socket.timeout:
                continue
            except Exception as e:
                # 과도한 로그 방지: 최초 1회만 상세 출력
                if not getattr(self, "_first_frame_logged", False):
                    self.log_warn(f"UDP 수신 예외: {e}")
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
        if not getattr(self, "_first_display_logged", False):
            self.log_info("첫 프레임 화면 표시 완료")
            self._first_display_logged = True

    def _get_env(self, key: str, default):
        import os
        return os.environ.get(key, default)

    # RECHECKING
    def setup_rechecking_events(self):
        self.log_info("RECHECKING 준비")
        self.setup_button_event("reenterButton", self.on_reenter)
        self.setup_button_event("confirmButton", self.on_recheck_confirm)
        # 안내 이미지 배치
        self._load_rechecking_image()
        # 영상 표시 시작 (detectionFrame 내부)
        self._start_udp_video(target_frame_name="detectionFrame")

    def on_reenter(self):
        self.log_info("재입력 선택")
        self.screen_manager.show_screen("REGISTERING")

    def on_recheck_confirm(self):
        self.log_info("확인 완료")
        self.screen_manager.show_screen("GUIDE_REQUEST")

    def _load_rechecking_image(self):
        """RECHECKING 화면에 rgui_guide_out.png 이미지를 표시 (qrc 사용)"""
        try:
            from PyQt6.QtWidgets import QLabel
            from PyQt6.QtGui import QPixmap
            # 부모 프레임: 우선 detectionFrame 안에 배치, 없으면 루트 위젯 사용
            parent = self.find_widget("detectionFrame")
            if parent is None:
                parent = self.widget
            # 기존 라벨이 있으면 재사용, 없으면 생성
            image_label = parent.findChild(QLabel, "recheckImage") if hasattr(parent, 'findChild') else None
            if image_label is None:
                image_label = QLabel(parent)
                image_label.setObjectName("recheckImage")
            # qrc 이미지 로드
            qrc_path = ":/roomie_rgui/assets/rgui_guide_out.png"
            pix = QPixmap(qrc_path)
            if pix.isNull():
                self.log_warn(f"RECHECKING qrc 이미지 로드 실패: {qrc_path}")
                return
            image_label.setPixmap(pix)
            image_label.setScaledContents(True)
            # 부모 크기에 맞춰 중앙 배치
            try:
                w = parent.width() if hasattr(parent, 'width') else 600
                h = parent.height() if hasattr(parent, 'height') else 400
                target_w = int(min(w * 0.6, h * 0.6))
                target_h = target_w
                x = int((w - target_w) / 2)
                y = int((h - target_h) / 2)
                image_label.setGeometry(x, y, target_w, target_h)
            except Exception:
                image_label.setGeometry(50, 50, 400, 400)
            image_label.show()
            self.log_info("RECHECKING qrc 이미지 로드 성공")
        except Exception as e:
            self.log_warn(f"RECHECKING 이미지 설정 중 경고: {e}")

    # GUIDE_REQUEST
    def setup_guide_request_events(self):
        self.log_info("GUIDE_REQUEST 준비")
        # 뒤로가기 버튼
        self.setup_button_event("backButton", self.on_guide_request_back)
        # 카드 전체 터치 영역(터치 전용 헬퍼로 z-order 보정 포함)
        self.setup_touch_event("touchButton", self.on_request_guidance)
        # 루트 전체 화면 터치 대체 (버튼 z-order/미탐색 대비)
        try:
            from PyQt6.QtWidgets import QWidget
            from PyQt6.QtCore import Qt
            original_release = getattr(self.widget, "mouseReleaseEvent", None)
            def on_root_mouse_release(event):
                if event.button() == Qt.MouseButton.LeftButton:
                    self.log_info("GUIDE_REQUEST 전체 화면 터치 감지 - on_request_guidance")
                    self.on_request_guidance()
                if original_release:
                    QWidget.mouseReleaseEvent(self.widget, event)
            self.widget.mouseReleaseEvent = on_root_mouse_release
        except Exception as e:
            self.log_warn(f"GUIDE_REQUEST 전체 화면 터치 연결 중 경고: {e}")
        # 이미지 보장: qrc→절대경로 폴백
        self._load_guide_request_image()

    def _set_label_pixmap_with_fallback(self, label_name: str, qrc_path: str, abs_path: str):
        try:
            from PyQt6.QtWidgets import QLabel
            from PyQt6.QtGui import QPixmap
            label = self.widget.findChild(QLabel, label_name)
            if label is None:
                self.log_warn(f"라벨을 찾을 수 없음: {label_name}")
                return
            pix = QPixmap(qrc_path)
            if pix.isNull() and abs_path:
                pix = QPixmap(abs_path)
            if pix.isNull():
                self.log_warn(f"이미지 로드 실패(qrc/abs): {qrc_path} | {abs_path}")
                return
            label.setPixmap(pix)
            label.setScaledContents(True)
            self.log_info(f"이미지 로드 성공: {qrc_path if abs_path is None else (qrc_path + ' | ' + abs_path)}")
        except Exception as e:
            self.log_warn(f"이미지 설정 중 경고({label_name}): {e}")

    def _load_guide_request_image(self):
        base = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets/rgui_guide_1.png"
        self._set_label_pixmap_with_fallback("roadImage", ":/roomie_rgui/assets/rgui_guide_1.png", base)

    def on_request_guidance(self):
        self.log_info("길안내 요청")
        # 입력 방식 선택 화면으로 전환 (id=10은 여기서 발행하지 않음)
        self.screen_manager.show_screen("INPUT_METHOD_SELECTION")

    def on_guide_request_back(self):
        self.log_info("GUIDE_REQUEST 뒤로가기")
        self.screen_manager.show_screen("TOUCH_SCREEN")

    # DESTINATION_ARRIVED
    def setup_destination_arrived_events(self):
        self.log_info("DESTINATION_ARRIVED 준비")
        self.setup_button_event("okButton", self.on_destination_confirm)
        # .ui에서 qrc 픽스맵을 이미 설정하므로 별도 이미지 로드는 생략

    def on_destination_confirm(self):
        self.log_info("도착 확인")
        # 길안내 종료 이벤트 발행 (참고: rgui_node에서 11=길안내 종료 정의)
        self.publish_event(event_id=11, detail="") 