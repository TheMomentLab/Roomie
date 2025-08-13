"""
ElevatorController - 엘리베이터 화면들을 처리하는 컨트롤러
"""

from .base_controller import BaseController
from PyQt6.QtGui import QPixmap
import os


class ElevatorController(BaseController):
    def __init__(self, widget, screen_manager, node, ui_filename):
        super().__init__(widget, screen_manager, node, ui_filename)
        self.setup_events()
    
    def setup_events(self):
        """이벤트 연결 설정"""
        if "ELE_1_MANIPULATING.ui" in self.ui_filename:
            self.setup_manipulating_events()
        elif "ELE_2_CALLING.ui" in self.ui_filename:
            self.setup_calling_events()
        elif "ELE_3_BOARDING.ui" in self.ui_filename:
            self.setup_boarding_events()
        elif "ELE_4_MOVING_TO_TARGET.ui" in self.ui_filename:
            self.setup_moving_events()
        elif "ELE_5_EXITING.ui" in self.ui_filename:
            self.setup_exiting_events()
    
    def setup_manipulating_events(self):
        """엘리베이터 버튼 조작 화면 이벤트 설정"""
        self.log_info("엘리베이터 버튼 조작 화면 준비 완료")
        
        # 주의 이미지 로드
        self.load_caution_image()
        
        # 이 화면은 RC에서 제어하므로 별도 UI 이벤트는 없음
        # 필요시 추가 버튼 이벤트를 여기에 구현
    
    def setup_calling_events(self):
        """엘리베이터 호출 화면 이벤트 설정"""
        self.log_info("엘리베이터 호출 화면 준비 완료")
        
        # 엘리베이터 이미지 로드
        self.load_elevator_image()
        
        # 호출 중 상태를 나타내는 시각적 효과 등을 여기에 구현할 수 있음
        # 예: 인디케이터 애니메이션
        self.start_calling_animation()
    
    def setup_boarding_events(self):
        """엘리베이터 탑승 화면 이벤트 설정"""
        self.log_info("엘리베이터 탑승 화면 준비 완료")
        
        # 주의 이미지 로드
        self.load_caution_image()
        
        # 탑승 시 주의사항 표시나 음성 안내 등을 여기에 구현
    
    def setup_moving_events(self):
        """엘리베이터 이동 화면 이벤트 설정"""
        self.log_info("엘리베이터 목표층 이동 화면 준비 완료")
        
        # 엘리베이터 이미지 로드
        self.load_elevator_image()
        
        # 이동 중 진행 상태를 나타내는 시각적 효과 등을 구현
        self.start_moving_animation()
    
    def setup_exiting_events(self):
        """엘리베이터 하차 화면 이벤트 설정"""
        self.log_info("엘리베이터 하차 화면 준비 완료")
        
        # 주의 이미지 로드
        self.load_caution_image()
        
        # 하차 시 주의사항 표시나 안전 관련 메시지 등을 구현
    
    def start_calling_animation(self):
        """엘리베이터 호출 중 애니메이션 시작"""
        try:
            # 호출 중 상태를 나타내는 시각적 효과 등을 여기에 구현할 수 있음
            # 예: 메인 메시지 깜빡임, 배경 색상 변화 등
            self.log_info("엘리베이터 호출 애니메이션 시작")
            # TODO: 호출 애니메이션 구현 (QTimer 사용)
        except Exception as e:
            self.log_error(f"호출 애니메이션 시작 실패: {e}")
    
    def start_moving_animation(self):
        """엘리베이터 이동 중 애니메이션 시작"""
        try:
            # 이동 중 상태를 나타내는 시각적 효과 등을 구현
            # 예: 엘리베이터 이미지 움직임, 진행률 표시 등
            self.log_info("엘리베이터 이동 애니메이션 시작")
                # TODO: 이동 애니메이션 구현 (QTimer 사용)
        except Exception as e:
            self.log_error(f"이동 애니메이션 시작 실패: {e}")
    
    def on_screen_activated(self):
        """화면이 활성화될 때 호출되는 메서드"""
        self.log_info(f"엘리베이터 화면 활성화: {self.ui_filename}")
        
        # 각 화면별로 활성화 시 특별한 처리가 필요한 경우 여기에 구현
        if "ELE_2_CALLING.ui" in self.ui_filename:
            self.start_calling_animation()
        elif "ELE_4_MOVING_TO_TARGET.ui" in self.ui_filename:
            self.start_moving_animation()
    
    def load_caution_image(self):
        """주의 이미지 로드"""
        try:
            # 이미지 파일 경로 (절대 경로 사용)
            image_path = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets/rgui_caution.png"
            
            # 이미지 라벨 찾기
            caution_image_label = self.find_widget("cautionImage")
            if caution_image_label:
                # 이미지 로드
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    caution_image_label.setPixmap(pixmap)
                    caution_image_label.setScaledContents(True)
                    self.log_info(f"✅ 주의 이미지 로드 성공: {image_path}")
                else:
                    self.log_error(f"❌ 이미지 로드 실패: {image_path}")
            else:
                self.log_error("❌ cautionImage 라벨을 찾을 수 없음")
                
        except Exception as e:
            self.log_error(f"❌ 이미지 로드 중 오류: {e}")
    
    def load_elevator_image(self):
        """엘리베이터 이미지 로드"""
        try:
            # 이미지 파일 경로 (절대 경로 사용)
            image_path = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets/rgui_elevator.png"
            
            # 이미지 라벨 찾기
            elevator_image_label = self.find_widget("elevatorImage")
            if elevator_image_label:
                # 이미지 로드
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    elevator_image_label.setPixmap(pixmap)
                    elevator_image_label.setScaledContents(True)
                    self.log_info(f"✅ 엘리베이터 이미지 로드 성공: {image_path}")
                else:
                    self.log_error(f"❌ 이미지 로드 실패: {image_path}")
            else:
                self.log_error("❌ elevatorImage 라벨을 찾을 수 없음")
                
        except Exception as e:
            self.log_error(f"❌ 이미지 로드 중 오류: {e}") 