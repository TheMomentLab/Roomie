"""
CommonController - 공통 화면들 (TOUCH_SCREEN, COUNTDOWN 등)을 처리하는 컨트롤러
"""

from .base_controller import BaseController
from PyQt6.QtGui import QPixmap
import os


class CommonController(BaseController):
    def __init__(self, widget, screen_manager, node, ui_filename):
        super().__init__(widget, screen_manager, node, ui_filename)
        self.setup_events()
    
    def setup_events(self):
        """이벤트 연결 설정"""
        if "TOUCH_SCREEN.ui" in self.ui_filename:
            self.setup_touch_screen_events()
        elif "COUNTDOWN.ui" in self.ui_filename:
            self.setup_countdown_events()
        elif "CHARGING.ui" in self.ui_filename:
            self.setup_charging_events()
    
    def setup_touch_screen_events(self):
        """Touch Screen 이벤트 설정"""
        self.log_info("Touch Screen 이벤트 설정 중...")
        
        # 로봇 눈 이미지 로드
        self.load_robot_eyes()
        
        # 전체 화면 터치: 루트 위젯의 마우스 릴리즈를 후킹
        try:
            from PyQt6.QtWidgets import QWidget
            from PyQt6.QtCore import Qt
            original_release = getattr(self.widget, "mouseReleaseEvent", None)
            
            def on_root_mouse_release(event):
                if event.button() == Qt.MouseButton.LeftButton:
                    self.log_info("전체 화면 터치 감지 - on_user_occupied 호출")
                    self.on_user_occupied()
                if original_release:
                    QWidget.mouseReleaseEvent(self.widget, event)
            
            self.widget.mouseReleaseEvent = on_root_mouse_release
            self.log_info("전체 화면 터치(루트 위젯) 이벤트 연결 완료")
        except Exception as e:
            self.log_error(f"전체 화면 터치 이벤트 연결 실패: {e}")
    
    def setup_countdown_events(self):
        """카운트다운 화면 이벤트 설정"""
        self.log_info("카운트다운 화면 준비 완료")
        # 카운트다운은 외부 시스템에서 서비스 호출로 시작됨
        # UI에서 별도 버튼 이벤트는 없음
    
    def on_user_occupied(self):
        """사용자가 화면을 터치했을 때 - 점유 상태 알림"""
        self.log_info("👤 사용자가 화면을 터치했습니다")
        
        # 사용자 점유 상태 이벤트 발행 (rgui_event_id: 102)
        self.publish_event(event_id=102, detail="OCCUPIED")
        # 길안내 요청 화면으로 전환
        self.screen_manager.show_screen("GUIDE_REQUEST")
    
    def setup_charging_events(self):
        """충전 화면 이벤트 설정"""
        self.log_info("충전 화면 준비 완료")
        
        # 충전 이미지 로드
        self.load_charging_image()
    
    def load_robot_eyes(self):
        """로봇 눈 이미지 로드"""
        try:
            # 이미지 파일 경로 (소스 폴더 기준)
            image_path = os.path.join(
                "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets",
                "rgui_roomie_eyes.png"
            )
            
            # 이미지 라벨 찾기
            robot_eyes_label = self.find_widget("robotEyes")
            if robot_eyes_label:
                # 이미지 로드
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    robot_eyes_label.setPixmap(pixmap)
                    robot_eyes_label.setScaledContents(True)
                    self.log_info(f"✅ 로봇 눈 이미지 로드 성공: {image_path}")
                else:
                    self.log_error(f"❌ 이미지 로드 실패: {image_path}")
            else:
                self.log_error("❌ robotEyes 라벨을 찾을 수 없음")
                
        except Exception as e:
            self.log_error(f"❌ 이미지 로드 중 오류: {e}")
    
    def load_charging_image(self):
        """충전 이미지 로드"""
        try:
            # 이미지 파일 경로 (절대 경로 사용)
            image_path = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets/rgui_charging.png"
            
            # 이미지 라벨 찾기
            charging_image_label = self.find_widget("chargingImage")
            if charging_image_label:
                # 이미지 로드
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    charging_image_label.setPixmap(pixmap)
                    charging_image_label.setScaledContents(True)
                    self.log_info(f"✅ 충전 이미지 로드 성공: {image_path}")
                else:
                    self.log_error(f"❌ 이미지 로드 실패: {image_path}")
            else:
                self.log_error("❌ chargingImage 라벨을 찾을 수 없음")
                
        except Exception as e:
            self.log_error(f"❌ 이미지 로드 중 오류: {e}") 