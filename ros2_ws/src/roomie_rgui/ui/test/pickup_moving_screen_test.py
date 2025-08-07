#!/usr/bin/env python3
"""
Pickup Moving Screen UI 확인용 테스트 - 컨트롤러 기능 포함
"""

import sys
import os
from PyQt6 import uic
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QFont, QPixmap

# roomie_rgui 패키지 경로 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'roomie_rgui'))

from roomie_rgui.ui_controllers.delivery_controller import DeliveryController
from roomie_rgui.ui_loader import load_ui

class MockNode:
    """테스트용 Mock Node"""
    def __init__(self):
        self.logger = MockLogger()
    
    def get_logger(self):
        return self.logger
    
    def publish_event(self, event_id, robot_id, task_id=0, detail=""):
        print(f"📡 이벤트 발행: ID={event_id}, robot_id={robot_id}, detail='{detail}'")

class MockLogger:
    """테스트용 Mock Logger"""
    def info(self, message):
        print(f"[INFO] {message}")
    
    def warn(self, message):
        print(f"[WARN] {message}")
    
    def error(self, message):
        print(f"[ERROR] {message}")

class MockScreenManager:
    """테스트용 Mock Screen Manager"""
    def __init__(self):
        pass
    
    def show_screen(self, screen_name):
        print(f"📺 화면 전환: {screen_name}")

def main():
    app = QApplication(sys.argv)
    font = QFont("Malgun Gothic", 12)
    app.setFont(font)
    
    print("🚀 Pickup Moving Screen UI 확인 테스트")
    
    # UI 파일 경로
    ui_file = os.path.join(
        os.path.dirname(__file__), 
        '..', 
        'delivery', 
        'DELI_1_PICKUP_MOVING.ui'
    )
    
    try:
        # UI 파일 로드
        window = QWidget()
        load_ui(window, ui_file)
        print(f"✅ UI 파일 로드 성공: {ui_file}")
        
        # Mock 객체들 생성
        mock_node = MockNode()
        mock_screen_manager = MockScreenManager()
        
        # DeliveryController 생성 (이미지 로드 기능 포함)
        controller = DeliveryController(
            widget=window,
            screen_manager=mock_screen_manager,
            node=mock_node,
            ui_filename="DELI_1_PICKUP_MOVING.ui"
        )
        
        # 화면 활성화 (이미지 로드 실행)
        controller.on_screen_activated()
        
        # 전체화면으로 표시
        window.showFullScreen()
        print("✅ 화면이 전체화면으로 표시되었습니다.")
        print("💡 ESC 키로 종료할 수 있습니다.")
        
        sys.exit(app.exec())
        
    except Exception as e:
        print(f"❌ 오류: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()