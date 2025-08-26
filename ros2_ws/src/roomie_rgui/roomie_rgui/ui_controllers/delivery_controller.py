"""
DeliveryController - 배송 관련 화면들 (DELI_1~8)을 처리하는 컨트롤러
화면 전환은 외부 시스템에서 처리하고, 여기서는 사용자 입력에 대한 이벤트 발행만 수행
"""

from PyQt6.QtWidgets import QPushButton
from PyQt6.QtGui import QPixmap
from .base_controller import BaseController
import os


class DeliveryController(BaseController):
    def __init__(self, widget, screen_manager, node, ui_filename):
        super().__init__(widget, screen_manager, node, ui_filename)
        # 화면별 이벤트는 화면이 활성화될 때만 설정
    
    def on_screen_activated(self):
        """화면이 활성화될 때 호출됨 (ScreenManager에서)"""
        self.setup_events()
    
    def setup_events(self):
        """현재 활성화된 화면의 이벤트만 설정"""
        if "DELI_1" in self.ui_filename:
            self.setup_pickup_moving_events()
        elif "DELI_2" in self.ui_filename:
            self.setup_pickup_arrival_events()
        elif "DELI_3" in self.ui_filename:
            self.setup_order_confirm_events()
        elif "DELI_4" in self.ui_filename:
            self.setup_pickup_drawer_events()
        elif "DELI_5" in self.ui_filename:
            self.setup_delivery_moving_events()
        elif "DELI_6" in self.ui_filename:
            self.setup_delivery_arrival_events()
        elif "DELI_7" in self.ui_filename:
            self.setup_delivery_drawer_events()
        elif "DELI_8" in self.ui_filename:
            self.setup_thank_you_events()
    
    # DELI_1: 픽업 이동중
    def setup_pickup_moving_events(self):
        """픽업 장소로 이동중 화면"""
        self.log_info("픽업 이동중 화면 - 외부 시스템 대기")
        
        # 로봇 눈 이미지 로드
        self.load_robot_eyes()
        
        # 이 화면에서는 사용자 입력 없음, 외부 시스템에서 화면 전환
    
    # DELI_2: 픽업 도착
    def setup_pickup_arrival_events(self):
        """픽업 장소 도착 화면"""
        self.log_info("픽업 도착 화면 - 터치 대기")
        
        # 로봇 눈 이미지 로드
        self.load_robot_eyes()
        
        # 전체 화면 터치 이벤트 연결
        self.setup_touch_event("fullScreenTouchArea", self.on_pickup_arrival_touch)
    
    def on_pickup_arrival_touch(self):
        """픽업 도착 화면 터치 시"""
        self.log_info("픽업 도착 화면이 터치되었습니다!")
        
        # 주문 확인 화면으로 전환
        self.screen_manager.show_screen("CHECKING_ORDER")
    
    # DELI_3: 주문 확인
    def setup_order_confirm_events(self):
        """주문 확인 화면"""
        self.log_info("주문 확인 화면 준비")
        
        # 확인 버튼 이벤트 연결
        self.setup_button_event("confirmButton", self.on_order_confirmed)
    
    def on_order_confirmed(self):
        """확인 버튼 클릭 시"""
        self.log_info("주문 확인 완료!")
        
        # 픽업 서랍 조작 화면으로 전환
        self.screen_manager.show_screen("PICKUP_DRAWER_CONTROL")
    

    
    # DELI_4: 픽업 서랍 조작
    def setup_pickup_drawer_events(self):
        """픽업 서랍 조작 화면"""
        self.log_info("픽업 서랍 조작 화면 준비")
        
        # 픽업 이미지 로드
        self.load_pickup_image()
        
        # [서랍 열기] 버튼
        self.setup_button_event("openDrawerButton", self.on_request_drawer_open)
        # [적재 완료] 버튼  
        self.setup_button_event("loadingCompleteButton", self.on_loading_complete)
    
    def on_request_drawer_open(self):
        """[서랍 열기] 버튼 클릭 시"""
        self.log_info("[서랍 열기] 버튼이 클릭되었습니다")
        
        # 서랍 열기 클릭 이벤트 발행 (rgui_event_id: 104)
        self.publish_event(event_id=104, detail="")
        
        # 서랍 열기 후 추가 음성 재생
        self.screen_manager.play_audio_file("audio_2_음식을_넣은_후_문을_닫고_적재_완료_버튼을_클릭하여_주세요.mp3")
    
    def on_loading_complete(self):
        """[적재 완료] 버튼 클릭 시"""
        self.log_info("[적재 완료] 버튼이 클릭되었습니다")
        
        # 적재 완료 클릭 이벤트 발행 (rgui_event_id: 105)
        self.publish_event(event_id=105, detail="")
    
    # DELI_5: 배송 이동중
    def setup_delivery_moving_events(self):
        """배송지로 이동중 화면"""
        self.log_info("배송 이동중 화면 - 외부 시스템 대기")
        
        # 로봇 눈 이미지 로드
        self.load_robot_eyes()
        
        # 이 화면에서는 사용자 입력 없음, 외부 시스템에서 화면 전환
    
    # DELI_6: 배송지 도착
    def setup_delivery_arrival_events(self):
        """배송지 도착 화면"""
        self.log_info("배송지 도착 화면 - 터치 대기")
        
        # 로봇 눈 이미지 로드
        self.load_robot_eyes()
        
        # 전체 화면 터치 이벤트 연결
        self.setup_touch_event("fullScreenTouchArea", self.on_delivery_arrival_touch)
    
    def on_delivery_arrival_touch(self):
        """배송지 도착 화면 터치 시"""
        self.log_info("배송지 도착 화면이 터치되었습니다!")
        
        # 배송 서랍 조작 화면으로 전환
        self.screen_manager.show_screen("DELIVERY_DRAWER_CONTROL")
    
    # DELI_7: 배송 서랍 조작
    def setup_delivery_drawer_events(self):
        """배송 서랍 조작 화면"""
        self.log_info("배송 서랍 조작 화면 준비")
        
        # 수령 이미지 로드
        self.load_receive_image()
        
        # 서랍 열기 버튼 이벤트 연결
        self.setup_button_event("openDrawerButton", self.on_delivery_drawer_open)
        # 수령 완료 버튼 이벤트 연결 (초기에는 비활성화 상태)
        self.setup_button_event("pickupCompleteButton", self.on_pickup_complete)
    
    def on_delivery_drawer_open(self):
        """서랍 열기 버튼 클릭 시"""
        self.log_info("[배송 서랍 열기] 버튼이 클릭되었습니다")
        
        # 서랍 열기 클릭 이벤트 발행 (rgui_event_id: 104)
        self.publish_event(event_id=104, detail="")
        
        # 서랍 열기 후 추가 음성 재생
        self.screen_manager.play_audio_file("audio_13_음식을_받으신_후_수령_완료_버튼을_눌러주세요_.mp3")
    

    
    def on_pickup_complete(self):
        """[수령 완료] 버튼 클릭 시"""
        self.log_info("[수령 완료] 버튼이 클릭되었습니다")
        
        # 수령 완료 클릭 이벤트 발행 (rgui_event_id: 100)
        self.publish_event(event_id=100, detail="")
    
    def on_drawer_opened(self, detail=""):
        """서랍이 열렸을 때 호출되는 메서드"""
        self.log_info(f"서랍 열림 알림 수신: {detail}")
        
        # 현재 화면에 따라 버튼 활성화 처리
        current_screen = self.screen_manager.get_current_screen_name()
        
        if current_screen == "PICKUP_DRAWER_CONTROL":
            # 픽업 서랍 조작: 적재완료 버튼 활성화
            loading_button = self.widget.findChild(QPushButton, "loadingCompleteButton")
            if loading_button:
                loading_button.setEnabled(True)
                loading_button.setStyleSheet("background-color: #e74c3c; font-size: 18px; font-weight: bold;")
                self.log_info("적재완료 버튼이 활성화되었습니다")
                
        elif current_screen == "DELIVERY_DRAWER_CONTROL":
            # 배송 서랍 조작: 수령완료 버튼 활성화
            pickup_button = self.widget.findChild(QPushButton, "pickupCompleteButton")
            if pickup_button:
                pickup_button.setEnabled(True)
                pickup_button.setStyleSheet("background-color: #e74c3c; font-size: 18px; font-weight: bold;")
                self.log_info("수령완료 버튼이 활성화되었습니다")
    
    def show_pickup_order(self, items, room_number="202"):
        """주문 내역을 화면에 표시 (rgui_node.py에서 호출됨)"""
        from PyQt6.QtWidgets import QLabel
        
        self.log_info(f"주문 내역 표시 요청: {len(items)}개 항목, 호실: {room_number}호")
        
        # menuItems 위젯 찾기
        menu_items_label = self.widget.findChild(QLabel, "menuItems")
        if not menu_items_label:
            self.log_error("menuItems 라벨을 찾을 수 없습니다")
            return
        
        # 주문 내역 텍스트 생성
        menu_text = ""
        for item in items:
            name = item.get("name", "알 수 없는 메뉴")
            quantity = item.get("quantity", 1)
            menu_text += f"{name} {quantity}개\n"
        
        # 텍스트가 비어있으면 기본값 설정
        if not menu_text.strip():
            menu_text = "주문 내역이 없습니다"
        
        # 화면에 표시
        menu_items_label.setText(menu_text.strip())
        self.log_info(f"주문 내역 표시 완료:\n{menu_text}")
        
        # 호실 번호 업데이트
        room_number_label = self.widget.findChild(QLabel, "roomNumber")
        if room_number_label:
            room_number_label.setText(f"{room_number}호")
            self.log_info(f"호실 번호 표시: {room_number}호")
    
    def show_room_number(self, room_number):
        """호실 번호를 화면에 표시"""
        from PyQt6.QtWidgets import QLabel
        
        room_number_label = self.widget.findChild(QLabel, "roomNumber")
        if room_number_label:
            room_number_label.setText(f"{room_number}호")
            self.log_info(f"호실 번호 표시: {room_number}호")
        else:
            self.log_error("roomNumber 라벨을 찾을 수 없습니다")
    
    # DELI_8: 감사 인사
    def setup_thank_you_events(self):
        """감사 인사 화면"""
        self.log_info("감사 인사 화면 - 외부 복귀 카운트다운 서비스 요청 대기")
        
        # 로봇 눈 이미지 로드
        self.load_robot_eyes()
        
        # 이 화면에서는 사용자 입력 없음, 외부에서 복귀 카운트다운 서비스 요청시 카운트다운 시작
    
    def _set_label_pixmap_with_fallback(self, label_name: str, qrc_path: str, abs_path: str):
        """QRC 이미지 로드, 실패 시 절대경로 폴백"""
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

    def load_robot_eyes(self):
        """로봇 눈 이미지 로드 (UI 파일 설정 보강)"""
        # UI 파일에서 이미 robotEyes로 설정되어 있으므로 폴백만 제공
        base = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets/rgui_eye_2.png"
        self._set_label_pixmap_with_fallback("robotEyes", ":/roomie_rgui/assets/rgui_eye_2.png", base)
    
    def load_pickup_image(self):
        """픽업 이미지 로드"""
        try:
            # 이미지 파일 경로 (절대 경로 사용)
            image_path = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/test/rgui_pickup.png"
            
            # 이미지 라벨 찾기
            pickup_image_label = self.find_widget("pickupImage")
            if pickup_image_label:
                # 이미지 로드
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    pickup_image_label.setPixmap(pixmap)
                    pickup_image_label.setScaledContents(True)
                    self.log_info(f"픽업 이미지 로드 성공: {image_path}")
                else:
                    self.log_error(f"이미지 로드 실패: {image_path}")
            else:
                self.log_error("pickupImage 라벨을 찾을 수 없음")
                
        except Exception as e:
            self.log_error(f"이미지 로드 중 오류: {e}")
    
    def load_receive_image(self):
        """수령 이미지 로드"""
        try:
            # 이미지 파일 경로 (절대 경로 사용)
            image_path = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/test/rgui_receive.png"
            
            # 이미지 라벨 찾기
            receive_image_label = self.find_widget("receiveImage")
            if receive_image_label:
                # 이미지 로드
                pixmap = QPixmap(image_path)
                if not pixmap.isNull():
                    receive_image_label.setPixmap(pixmap)
                    receive_image_label.setScaledContents(True)
                    self.log_info(f"수령 이미지 로드 성공: {image_path}")
                else:
                    self.log_error(f"이미지 로드 실패: {image_path}")
            else:
                self.log_error("receiveImage 라벨을 찾을 수 없음")
                
        except Exception as e:
            self.log_error(f"이미지 로드 중 오류: {e}") 