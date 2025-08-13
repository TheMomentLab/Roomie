# screen_manager.py

from PyQt6.QtWidgets import QStackedWidget
from PyQt6.QtCore import Qt, QUrl, QTimer
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput
import os
from .ui_loader import load_ui

# 컨트롤러 import
from .ui_controllers import BaseController, CommonController, DeliveryController, ElevatorController
from .ui_controllers import GuideController
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot


class ScreenManager(QStackedWidget):
    # 메인 스레드에서 실행되도록 위임하기 위한 신호들
    requestShowScreen = pyqtSignal(str)
    requestPlayAudio = pyqtSignal(str, str)
    requestNotifyDrawerOpened = pyqtSignal(str)
    requestUpdateCountdown = pyqtSignal(int, str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Roomie RGUI")
        
        # 창 크기 설정 (UI 파일들이 1920x1080으로 설계됨)
        self.setFixedSize(1920, 1080)
        
        # 전체화면 설정
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        
        # 스타일 설정
        self.setStyleSheet("""
            QStackedWidget {
                background-color: #171E26;
            }
            QWidget {
                background-color: #171E26;
            }
        """)

        # 음성 재생 관련 설정
        self.audio_output = QAudioOutput()
        self.media_player = QMediaPlayer()
        self.media_player.setAudioOutput(self.audio_output)
        self.audio_enabled = True
        self.audio_volume = 0.7
        self.audio_output.setVolume(self.audio_volume)
        
        # 음성 설정 로그
        self.node.get_logger().info(f"🔊 음성 시스템 초기화: enabled={self.audio_enabled}, volume={self.audio_volume}")

        # 신호 연결 (메인 스레드에서 실행)
        self.requestShowScreen.connect(self._show_screen_impl)
        self.requestPlayAudio.connect(self._play_audio_file_internal)
        self.requestNotifyDrawerOpened.connect(self._notify_drawer_opened_impl)
        self.requestUpdateCountdown.connect(self._update_countdown_labels)

        # 화면별 음성 파일 매핑
        self.screen_audio_map = {
            # 공통 화면
            "TOUCH_SCREEN": None,  # 터치 화면은 음성 없음
            "CHARGING": None,      # 충전 화면은 음성 없음
            
            # 배송 관련
            "PICKUP_MOVING": "ui/voice/audio_8_픽업_장소로_이동을_시작합니다_.mp3",
            "PICKUP_ARRIVED": "ui/voice/audio_0_픽업_로봇이_도착하였습니다__주문_정보를_확인해주세요_.mp3",
            "CHECKING_ORDER": None,  # 주문 확인 화면은 음성 없음
            "PICKUP_DRAWER_CONTROL": "ui/voice/audio_1_서랍_열기_버튼을_클릭하여_음식을_넣어주세요.mp3",
            "COUNTDOWN": "ui/voice/audio_3_잠시_후_로봇이_출발합니다__주의해주세요_.mp3",
            "DELIVERY_MOVING": "ui/voice/audio_4_배송_장소로_이동을_시작합니다_.mp3",
            "DELIVERY_ARRIVED": "ui/voice/audio_5_배송_로봇이_도착하였습니다__주문_정보를_확인해주세요_.mp3",
            "DELIVERY_DRAWER_CONTROL": "ui/voice/audio_12_서랍_열기_버튼을_클릭하여_음식을_수령해주세요_.mp3",
            "THANK_YOU": "ui/voice/audio_6_주문해주셔서_감사합니다__맛있게_드세요_.mp3",
            
            # 엘리베이터 관련
            "ELEVATOR_MANIPULATING": "ui/voice/audio_9_엘리베이터_버튼_조작을_시작합니다__주의해주세요_.mp3",
            "ELEVATOR_CALLING": None,  # 엘리베이터 호출 화면은 음성 없음
            "ELEVATOR_BOARDING": "ui/voice/audio_10_엘리베이터_탑승을_시작합니다__주의해주세요_.mp3",
            "ELEVATOR_MOVING_TO_TARGET": None,  # 엘리베이터 이동 화면은 음성 없음
            "ELEVATOR_EXITING": "ui/voice/audio_11_엘리베이터_하차를_시작합니다__주의해주세요_.mp3",
            
            # 기타
            "RETURN_TO_BASE": "ui/voice/audio_7_복귀_장소로_이동을_시작합니다_.mp3",
            # 가이드 관련 (voice_guide 폴더 연결)
            "GUIDANCE_SCREEN": "ui/voice/voice_guide/audio_3_목적지로_안내를_시작합니다_.mp3",
            "INPUT_METHOD_SELECTION": None,
            "CARD_KEY_WAITING": "ui/voice/voice_guide/audio_0_객실_카드키를_RFID_리더기에_태깅해주세요_.mp3",
            "REGISTERING": "ui/voice/voice_guide/audio_1_사용자를_인식중입니다__화면에_보이도록_위치해주세요_.mp3",
            "RECHECKING": "ui/voice/voice_guide/audio_4_인식에_실패하였습니다__화면에_다시_위치해주세요_.mp3",
            "GUIDE_REQUEST": None,
            "DESTINATION_ARRIVED": "ui/voice/voice_guide/audio_2_목적지에_도착하였습니다__완료_버튼을_눌러주세요_.mp3",
        }
        
        # 현재 화면 정보
        self.current_screen_name = None
        
        # 화면별 위젯과 컨트롤러 저장
        self.screen_widgets = {}
        self.screen_controllers = {}
        self.screen_indices = {}

        # UI 경로 매핑
        self.ui_paths = {
            # 공통 화면
            "TOUCH_SCREEN": "ui/common/TOUCH_SCREEN.ui",
            "COUNTDOWN": "ui/countdown/COUNTDOWN.ui",
            "RETURN_TO_BASE": "ui/common/RETURN_TO_BASE.ui",
            "CHARGING": "ui/common/CHARGING.ui",
            
            # 배송 화면들
            "PICKUP_MOVING": "ui/delivery/DELI_1_PICKUP_MOVING.ui",
            "PICKUP_ARRIVED": "ui/delivery/DELI_2_PICKUP_ARRIVAL.ui", 
            "CHECKING_ORDER": "ui/delivery/DELI_3_CHECKING_ORDER.ui",
            "PICKUP_DRAWER_CONTROL": "ui/delivery/DELI_4_PICKUP_DRAWER_CONTROL.ui",
            "DELIVERY_MOVING": "ui/delivery/DELI_5_DELIVERY_MOVING.ui",
            "DELIVERY_ARRIVED": "ui/delivery/DELI_6_DELIVERY_ARRIVAL.ui",
            "DELIVERY_DRAWER_CONTROL": "ui/delivery/DELI_7_DELIVERY_DRAWER_CONTROL.ui",
            "THANK_YOU": "ui/delivery/DELI_8_THANK_YOU.ui",
            
            # 엘리베이터 화면들
            "ELEVATOR_MANIPULATING": "ui/elevator/ELE_1_MANIPULATING.ui",
            "ELEVATOR_CALLING": "ui/elevator/ELE_2_CALLING.ui",
            "ELEVATOR_BOARDING": "ui/elevator/ELE_3_BOARDING.ui",
            "ELEVATOR_MOVING_TO_TARGET": "ui/elevator/ELE_4_MOVING_TO_TARGET.ui",
            "ELEVATOR_EXITING": "ui/elevator/ELE_5_EXITING.ui",
            # 가이드 화면들 (실제 파일명 반영)
            "GUIDANCE_SCREEN": "ui/guide/GUI_5_GUIDANCE_SCREEN.ui",
            "INPUT_METHOD_SELECTION": "ui/guide/GUI_2_INPUT_METHOD_SELECTION.ui",
            "CARD_KEY_WAITING": "ui/guide/GUI_3_CARD_KEY_WAITING.ui",
            "REGISTERING": "ui/guide/GUI_4_REGISTERING.ui",
            "RECHECKING": "ui/guide/GUI_5_1_RECHECKING.ui",
            "GUIDE_REQUEST": "ui/guide/GUI_1_GUIDE_REQUEST.ui",
            "DESTINATION_ARRIVED": "ui/guide/GUI_6_DESTINATION_ARRIVED.ui",
        }

        # 컨트롤러 팩토리 매핑
        self.controller_map = {
            "TOUCH_SCREEN": CommonController,
            "COUNTDOWN": CommonController,
            "RETURN_TO_BASE": CommonController,
            "CHARGING": CommonController,
            "PICKUP_MOVING": DeliveryController,
            "PICKUP_ARRIVED": DeliveryController,
            "CHECKING_ORDER": DeliveryController,
            "PICKUP_DRAWER_CONTROL": DeliveryController,
            "DELIVERY_MOVING": DeliveryController,
            "DELIVERY_ARRIVED": DeliveryController,
            "DELIVERY_DRAWER_CONTROL": DeliveryController,
            "THANK_YOU": DeliveryController,
            "ELEVATOR_MANIPULATING": ElevatorController,
            "ELEVATOR_CALLING": ElevatorController,
            "ELEVATOR_BOARDING": ElevatorController,
            "ELEVATOR_MOVING_TO_TARGET": ElevatorController,
            "ELEVATOR_EXITING": ElevatorController,
            # 가이드 화면들
            "GUIDANCE_SCREEN": GuideController,
            "INPUT_METHOD_SELECTION": GuideController,
            "CARD_KEY_WAITING": GuideController,
            "REGISTERING": GuideController,
            "RECHECKING": GuideController,
            "GUIDE_REQUEST": GuideController,
            "DESTINATION_ARRIVED": GuideController,
        }



        # 모든 화면 미리 로드
        self.preload_all_screens()
        
        # 초기 화면 표시
        self.show_screen("TOUCH_SCREEN")
        self.show()
        
        # 전체화면으로 실행
        self.showFullScreen()
        
        # 키 이벤트 포커스 설정
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setFocus()

    def keyPressEvent(self, event: QKeyEvent):
        """키보드 이벤트 처리"""
        if event.key() == Qt.Key.Key_Escape:
            self.close()
        elif event.key() == Qt.Key.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        super().keyPressEvent(event)

    def preload_all_screens(self):
        """모든 화면을 미리 로드"""
        self.node.get_logger().info("🔄 모든 화면 로딩 시작...")
        
        for screen_name, ui_path in self.ui_paths.items():
            try:
                # 빈 위젯 생성
                from PyQt6.QtWidgets import QWidget
                widget = QWidget()
                
                # UI 파일 로드
                load_ui(widget, ui_path)
                
                # 화면을 스택에 추가
                index = self.addWidget(widget)
                self.screen_widgets[screen_name] = widget
                self.screen_indices[screen_name] = index
                
                # 컨트롤러 생성
                controller_class = self.controller_map.get(screen_name, BaseController)
                controller = controller_class(widget, self, self.node, ui_path)
                self.screen_controllers[screen_name] = controller
                
                self.node.get_logger().info(f"컨트롤러 생성: {controller_class.__name__} for {screen_name}")
                self.node.get_logger().info(f"{screen_name} 로드 완료 (index: {index})")
                
            except Exception as e:
                self.node.get_logger().error(f"❌ {screen_name} 로드 실패: {e}")
        
        self.node.get_logger().info(f"총 {len(self.screen_widgets)}개 화면 로드 완료!")

    def show_screen(self, screen_name):
        """지정된 화면으로 전환 (스레드 안전)"""
        if QThread.currentThread() != self.thread():
            # 메인 스레드로 위임
            self.requestShowScreen.emit(screen_name)
            return True
        return self._show_screen_impl(screen_name)

    def _show_screen_impl(self, screen_name):
        """실제 화면 전환 구현 (메인 스레드에서만 호출)"""
        if screen_name not in self.screen_indices:
            self.node.get_logger().warn(f"존재하지 않는 화면: {screen_name}")
            return False
        
        index = self.screen_indices[screen_name]
        self.setCurrentIndex(index)
        self.current_screen_name = screen_name
        self.node.get_logger().info(f"📺 화면 전환: {screen_name} (index: {index})")
        
        # 화면 전환 시 음성 재생
        self.node.get_logger().info(f"🎵 {screen_name} 화면 음성 재생 호출")
        self.play_screen_audio(screen_name)
        
        # 화면 전환 시 해당 컨트롤러의 이벤트 활성화
        controller = self.screen_controllers.get(screen_name)
        if controller and hasattr(controller, 'on_screen_activated'):
            controller.on_screen_activated()
            self.node.get_logger().info(f"🎯 {screen_name} 컨트롤러 이벤트 활성화")
        
        return True
    
    def play_screen_audio(self, screen_name):
        """화면 전환 시 음성 재생"""
        self.node.get_logger().info(f"🔊 음성 재생 시도: {screen_name}")
        
        if not self.audio_enabled:
            self.node.get_logger().warn(f"🔇 음성 재생이 비활성화됨: {screen_name}")
            return
            
        if screen_name in self.screen_audio_map:
            audio_file = self.screen_audio_map[screen_name]
            self.node.get_logger().info(f"🎵 {screen_name} 음성 파일: {audio_file}")
            
            if audio_file is not None:  # None이 아닌 경우에만 재생
                # 즉시 실행 (QTimer.singleShot 대신)
                self.node.get_logger().info(f"🎵 즉시 음성 재생 호출: {audio_file}")
                self._play_audio_file_internal(audio_file, f"화면 음성")
                self.node.get_logger().info(f"▶️ {screen_name} 음성 재생 시작")
            else:
                self.node.get_logger().info(f"🔇 {screen_name} 화면은 음성이 설정되지 않음")
        else:
            self.node.get_logger().warn(f"⚠️ {screen_name} 화면의 음성 매핑이 없음")
    
    def play_audio_file(self, audio_filename):
        """특정 음성 파일을 직접 재생 (스레드 안전)"""
        if not self.audio_enabled:
            return
        
        audio_file = f"ui/voice/{audio_filename}"
        if QThread.currentThread() != self.thread():
            # 메인 스레드로 위임
            self.requestPlayAudio.emit(audio_file, "개별 음성")
            return
        # 메인 스레드인 경우 즉시 실행
        self._play_audio_file_internal(audio_file, f"개별 음성")
    
    def _resolve_audio_relpath(self, audio_file: str) -> str:
        """음성 파일 상대 경로를 ui/voice 또는 ui/voice/voice_delivery 내에서 해석한다."""
        try:
            base_dir = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui"
            candidates = []
            # 주어진 경로 우선
            if audio_file:
                candidates.append(audio_file)
                basename = os.path.basename(audio_file)
                # 기존 경로(prefix: ui/voice/)를 voice_delivery/voice_guide로 대체한 후보
                if "ui/voice/" in audio_file and "ui/voice/voice_delivery/" not in audio_file:
                    candidates.append(audio_file.replace("ui/voice/", "ui/voice/voice_delivery/"))
                if "ui/voice/" in audio_file and "ui/voice/voice_guide/" not in audio_file:
                    candidates.append(audio_file.replace("ui/voice/", "ui/voice/voice_guide/"))
                # 베이스 네임만으로 세 위치 모두 확인
                candidates.append(f"ui/voice/{basename}")
                candidates.append(f"ui/voice/voice_delivery/{basename}")
                candidates.append(f"ui/voice/voice_guide/{basename}")
            
            for rel in candidates:
                abs_path = os.path.join(base_dir, rel)
                if os.path.exists(abs_path):
                    return rel
        except Exception as e:
            # 문제가 있어도 원래 값을 반환하여 기존 동작 유지
            self.node.get_logger().warn(f"오디오 경로 해석 중 경고: {e}")
        return audio_file
    
    @pyqtSlot(str, str)
    def _play_audio_file_internal(self, audio_file, log_type):
        """내부 음성 재생 메서드 (메인 스레드에서만 호출)"""
        self.node.get_logger().info(f"🎵 _play_audio_file_internal 진입: {audio_file}")
        
        # 새 폴더 구조(ui/voice/voice_delivery) 및 구 구조(ui/voice) 모두 지원
        base_dir = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui"
        resolved_rel = self._resolve_audio_relpath(audio_file)
        audio_path = os.path.join(base_dir, resolved_rel)
        
        self.node.get_logger().info(f"🔍 음성 파일 경로 확인: {audio_path}")
        
        if os.path.exists(audio_path):
            try:
                # 기존 재생 중인 음성 정지
                if self.media_player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
                    self.media_player.stop()
                    self.node.get_logger().info("⏹️ 기존 음성 정지")
                
                # 새 음성 파일 설정 및 재생
                self.media_player.setSource(QUrl.fromLocalFile(audio_path))
                self.media_player.play()
                
                filename = os.path.basename(audio_path)
                self.node.get_logger().info(f"🔊 {log_type} 재생 시작: {filename}")
                
                # 재생 상태 확인 (약간의 지연 후)
                QTimer.singleShot(100, lambda: self._check_playback_state(filename))
                    
            except Exception as e:
                self.node.get_logger().error(f"❌ 음성 재생 중 오류: {e}")
        else:
            self.node.get_logger().warn(f"음성 파일을 찾을 수 없음: {audio_path}")
    
    def _check_playback_state(self, filename):
        """재생 상태 확인"""
        if self.media_player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            self.node.get_logger().info(f"✅ 음성 재생 성공: {filename}")
        else:
            self.node.get_logger().warn(f"⚠️ 음성 재생 실패: {filename}")
            # 오류 정보 출력
            error = self.media_player.error()
            if error != QMediaPlayer.Error.NoError:
                self.node.get_logger().error(f"❌ 미디어 플레이어 오류: {error}")
    
    def set_audio_enabled(self, enabled):
        """음성 재생 켜기/끄기"""
        self.audio_enabled = enabled
        self.node.get_logger().info(f"🔊 음성 재생: {'켜짐' if enabled else '꺼짐'}")
    
    def set_audio_volume(self, volume):
        """음성 볼륨 설정 (0.0 ~ 1.0)"""
        self.audio_volume = max(0.0, min(1.0, volume))
        self.audio_output.setVolume(self.audio_volume)
        self.node.get_logger().info(f"🔊 음성 볼륨: {self.audio_volume}")

    def get_current_screen_name(self):
        """현재 표시 중인 화면 이름 반환"""
        return self.current_screen_name

    def get_screen_controller(self, screen_name):
        """특정 화면의 컨트롤러 반환"""
        return self.screen_controllers.get(screen_name)
    
    def get_current_controller(self):
        """현재 활성화된 화면의 컨트롤러 반환"""
        if self.current_screen_name and self.current_screen_name in self.screen_controllers:
            return self.screen_controllers[self.current_screen_name]
        return None
    
    def notify_drawer_opened(self, detail=""):
        """현재 컨트롤러에 서랍 열림 알림 (스레드 안전)"""
        if QThread.currentThread() != self.thread():
            self.requestNotifyDrawerOpened.emit(detail)
            return
        self._notify_drawer_opened_impl(detail)

    @pyqtSlot(str)
    def _notify_drawer_opened_impl(self, detail=""):
        controller = self.get_current_controller()
        if controller and hasattr(controller, 'on_drawer_opened'):
            controller.on_drawer_opened(detail)

    # 카운트다운 라벨 업데이트 (COUNTDOWN 화면)
    def update_countdown_display(self, remaining_time: int, action_text: str):
        """COUNTDOWN 라벨 갱신 (스레드 안전)"""
        if QThread.currentThread() != self.thread():
            self.requestUpdateCountdown.emit(remaining_time, action_text)
            return
        self._update_countdown_labels(remaining_time, action_text)

    @pyqtSlot(int, str)
    def _update_countdown_labels(self, remaining_time: int, action_text: str):
        try:
            countdown_widget = self.screen_widgets.get("COUNTDOWN")
            if not countdown_widget:
                self.node.get_logger().warn("COUNTDOWN 화면 위젯을 찾을 수 없음")
                return
            from PyQt6.QtWidgets import QLabel
            countdown_label = countdown_widget.findChild(QLabel, "countdownNumber")
            if countdown_label:
                countdown_label.setText(str(remaining_time))
            title_label = countdown_widget.findChild(QLabel, "countdownTitle")
            if title_label:
                title_label.setText(f"{remaining_time}초후에 {action_text}합니다.")
        except Exception as e:
            self.node.get_logger().error(f"카운트다운 화면 업데이트 실패: {e}")
