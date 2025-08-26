"""
BaseController - 모든 UI 컨트롤러의 공통 부모 클래스
"""

from PyQt6.QtWidgets import QPushButton, QLabel
import os


class BaseController:
    def __init__(self, widget, screen_manager, node, ui_filename):
        self.widget = widget
        self.screen_manager = screen_manager
        self.node = node
        self.ui_filename = os.path.basename(ui_filename)
        
        self.log_info(f"컨트롤러 초기화: {self.ui_filename}")
    
    # 위젯 찾기 헬퍼
    def find_widget(self, widget_name, widget_type=None):
        """UI에서 특정 위젯 찾기"""
        if widget_type:
            return self.widget.findChild(widget_type, widget_name)
        else:
            # 여러 타입 시도
            return (self.widget.findChild(QPushButton, widget_name) or 
                   self.widget.findChild(QLabel, widget_name))
    
    # 이벤트 연결 헬퍼들
    def setup_button_event(self, button_name, callback):
        """버튼 클릭 이벤트 연결"""
        button = self.find_widget(button_name, QPushButton)
        if button:
            button.clicked.connect(callback)
            self.log_info(f"버튼 이벤트 연결: {button_name}")
            return True
        else:
            self.log_warn(f"버튼을 찾을 수 없음: {button_name}")
            return False
    
    def setup_touch_event(self, area_name, callback):
        """터치 영역 이벤트 연결"""
        touch_area = self.find_widget(area_name, QPushButton)
        if touch_area:
            # 디버깅: 위젯 속성 확인
            self.log_info(f"터치 위젯 발견: {area_name}")
            self.log_info(f"  - 크기: {touch_area.size()}")
            self.log_info(f"  - 위치: {touch_area.pos()}")
            self.log_info(f"  - 활성화: {touch_area.isEnabled()}")
            self.log_info(f"  - 보임: {touch_area.isVisible()}")
            self.log_info(f"  - 스타일: {touch_area.styleSheet()}")
            self.log_info(f"  - 포커스 정책: {touch_area.focusPolicy()}")
            self.log_info(f"  - 마스크: {touch_area.mask()}")
            self.log_info(f"  - 부모: {touch_area.parent()}")
            
            # 강제로 버튼을 최상위로 올리기
            touch_area.raise_()
            
            touch_area.clicked.connect(callback)
            
            # 디버깅: 테스트 클릭 이벤트도 연결
                        self.log_info(f"{area_name} 위젯이 눌렸습니다!")
            touch_area.released.connect(lambda:             self.log_info(f"{area_name} 위젯이 릴리즈되었습니다!"))
            
            # 시각적 피드백을 위해 hover 효과 추가
            original_style = touch_area.styleSheet()
            
            def on_enter(event):
                touch_area.setStyleSheet(original_style + "; background-color: rgba(255, 255, 255, 0.1);")
                self.log_info(f"{area_name} 마우스 진입!")
            
            def on_leave(event):
                touch_area.setStyleSheet(original_style)
                self.log_info(f"{area_name} 마우스 이탈!")
            
            def on_mouse_press(event):
                self.log_info(f"{area_name} 마우스 눌림! 버튼: {event.button()}")
                # 원래 이벤트도 처리하도록 전달
                QPushButton.mousePressEvent(touch_area, event)
            
            def on_mouse_release(event):
                self.log_info(f"{area_name} 마우스 릴리즈! 버튼: {event.button()}")
                # 왼쪽 버튼 클릭이면 직접 콜백 호출
                from PyQt6.QtCore import Qt
                if event.button() == Qt.MouseButton.LeftButton:
                    self.log_info(f"{area_name} 왼쪽 클릭 감지 - 콜백 직접 호출")
                    callback()
                # 원래 이벤트도 처리하도록 전달
                QPushButton.mouseReleaseEvent(touch_area, event)
            
            touch_area.enterEvent = on_enter
            touch_area.leaveEvent = on_leave
            touch_area.mousePressEvent = on_mouse_press
            touch_area.mouseReleaseEvent = on_mouse_release
            
            self.log_info(f"터치 이벤트 연결 완료: {area_name}")
            return True
        else:
            self.log_warn(f"터치 영역을 찾을 수 없음: {area_name}")
            
            # 디버깅: 사용가능한 모든 QPushButton 위젯 목록 출력
            all_buttons = self.widget.findChildren(QPushButton)
            self.log_info(f"사용 가능한 QPushButton 목록:")
            for btn in all_buttons:
                btn_name = btn.objectName() if btn.objectName() else "이름없음"
                self.log_info(f"  - {btn_name}: 크기={btn.size()}, 위치={btn.pos()}")
            
            return False
    
    # ROS2 이벤트 발행
    def publish_event(self, event_id, detail=""):
        """ROS2 GUI 이벤트 발행"""
        self.node.publish_event(event_id, robot_id=0, detail=detail)
        self.log_info(f"이벤트 발행: ID={event_id}, detail={detail}")
    

    
    # 로깅 헬퍼들
    def log_info(self, message):
        self.node.get_logger().info(f"[{self.__class__.__name__}] {message}")
    
    def log_warn(self, message):
        self.node.get_logger().warn(f"[{self.__class__.__name__}] {message}")
    
    def log_error(self, message):
        self.node.get_logger().error(f"[{self.__class__.__name__}] {message}")
    
    # 하위 클래스에서 구현해야 할 메서드
    def setup_events(self):
        """하위 클래스에서 구현: 이벤트 연결 로직"""
        pass 