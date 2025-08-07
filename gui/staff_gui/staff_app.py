import sys
import asyncio
import websockets
import requests
import json
import threading
import logging
import os
from datetime import datetime
from PyQt6 import QtWidgets, uic, QtCore, QtGui
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QPropertyAnimation
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QMessageBox, QFrame, QGraphicsOpacityEffect
)
from PyQt6.QtMultimedia import QSoundEffect
from config import RMS_WS_URL, RMS_HTTP_URL

# 로그 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('staff_gui.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# --- 통신 및 알림 클래스 ---
class CustomNotification(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint | Qt.WindowType.Tool | Qt.WindowType.WindowStaysOnTopHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setStyleSheet("background-color: rgba(30, 30, 30, 0.85); padding: 2px;")

        self.layout = QHBoxLayout(self)
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(350, 230)
        self.image_label.setScaledContents(True)
        
        self.layout.addWidget(self.image_label)
        
        # [핵심 수정] 위젯이 생성될 때부터 크기를 명확히 고정합니다.
        # 이미지(350) + 좌우패딩(2+2) = 354
        # 이미지(230) + 상하패딩(2+2) = 234
        self.setFixedSize(360, 240)
        
        self.hide_timer = QtCore.QTimer(self)
        self.hide_timer.setSingleShot(True)
        self.hide_timer.timeout.connect(self.hide)

    def show_notification(self, image_path):
        # (show_notification 메서드는 수정할 필요 없이 그대로 두시면 됩니다)
        parent_widget = self.parent()
        if not parent_widget:
            logger.error("알림 팝업의 부모 위젯이 설정되지 않았습니다.")
            return

        if image_path and os.path.exists(image_path):
            from PyQt6.QtGui import QPixmap
            pixmap = QPixmap(image_path)
            self.image_label.setPixmap(pixmap)
        else:
            logger.warning(f"알림 이미지를 찾을 수 없습니다: {image_path}")
            return
            
        parent_rect = parent_widget.geometry()
        x = parent_rect.right() - self.width() - 20
        y = parent_rect.bottom() - self.height() - 20
        
        self.move(x, y)
        
        self.show()
        self.hide_timer.start(3000)
        

        
        self.hide_timer = QtCore.QTimer(self)
        self.hide_timer.setSingleShot(True)
        self.hide_timer.timeout.connect(self.hide)



class Communicate(QObject):
    message_received = pyqtSignal(dict)
    connection_status = pyqtSignal(str)
    new_order_received = pyqtSignal(dict)
    pickup_arrival_received = pyqtSignal(dict)


# --- 메인 GUI 클래스 ---
class StaffGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi('staff_gui.ui', self)
        self.setWindowTitle("Staff GUI")

        self.orders_in_progress = {}
        self.orders_completed = {}
        self.selected_task_id = None
        
        # [수정] CustomNotification을 생성할 때 self를 부모로 전달합니다.
        self.notification_popup = CustomNotification(self) 
        
        self.order_sound_effect = QSoundEffect()
        self.pickup_sound_effect = QSoundEffect()

        self.set_order_sound("./sound/order_create.wav")
        self.set_pickup_sound("./sound/robot_arrival.wav")

        self.comm = Communicate()
        self.comm.message_received.connect(self.handle_websocket_message)
        self.comm.connection_status.connect(lambda msg: self.statusbar.showMessage(msg, 5000))
        self.comm.new_order_received.connect(self.show_new_order_notification)
        self.comm.pickup_arrival_received.connect(self.handle_pickup_arrival)

        self.listWidget_in_progress.itemClicked.connect(self.on_order_select_in_progress)
        self.listWidget_completed.itemClicked.connect(self.on_order_select_completed)
        self.readyButton.clicked.connect(self.mark_as_food_ready)

        self.start_websocket_connection()
        self.update_ui_labels()
        self.readyButton.setVisible(False)


    def reposition_notification(self):
        """알림 팝업이 보일 때 위치를 다시 계산하는 함수"""
        if self.notification_popup.isVisible():
            parent_rect = self.geometry()
            x = parent_rect.right() - self.notification_popup.width() - 3
            y = parent_rect.bottom() - self.notification_popup.height() - 3
            self.notification_popup.move(x, y)

    def moveEvent(self, event):
        """메인 창이 움직일 때마다 호출되는 이벤트 핸들러"""
        super().moveEvent(event)
        self.reposition_notification()

    def resizeEvent(self, event):
        """메인 창의 크기가 변경될 때마다 호출되는 이벤트 핸들러"""
        super().resizeEvent(event)
        self.reposition_notification()
    def set_order_sound(self, path):
        if os.path.exists(path):
            self.order_sound_effect.setSource(QtCore.QUrl.fromLocalFile(path))
            logger.info(f"주문 접수 알림음 설정: {path}")
        else:
            logger.warning(f"주문 접수 사운드 파일을 찾을 수 없습니다: {path}")

    def set_pickup_sound(self, path):
        if os.path.exists(path):
            self.pickup_sound_effect.setSource(QtCore.QUrl.fromLocalFile(path))
            logger.info(f"픽업 도착 알림음 설정: {path}")
        else:
            logger.warning(f"픽업 도착 사운드 파일을 찾을 수 없습니다: {path}")

    def show_new_order_notification(self, payload):
        self.add_new_order(payload)
        # 텍스트 메시지는 더 이상 필요 없음
        image_file = "./image/order_call.png"
        self.notification_popup.show_notification(image_path=image_file)
        self.order_sound_effect.play()

    def handle_pickup_arrival(self, payload):
        task_id = payload.get('task_id')
        self.update_order_status(task_id, "배송중")
        # 텍스트 메시지는 더 이상 필요 없음
        robot_image_file = "./image/robot_call.png"
        self.notification_popup.show_notification(image_path=robot_image_file)
        self.pickup_sound_effect.play()

    def find_task_id_by_item(self, item):
        for task_id, data in {**self.orders_in_progress, **self.orders_completed}.items():
            if data.get('list_item') == item:
                return task_id
        return None

    def on_order_select_in_progress(self, item):
        self.selected_task_id = self.find_task_id_by_item(item)
        self.update_order_details()
        self.readyButton.setVisible(True)

    def on_order_select_completed(self, item):
        self.selected_task_id = self.find_task_id_by_item(item)
        self.update_order_details()
        self.readyButton.setVisible(False)

    def update_ui_labels(self):
        self.label_in_progress.setText(f"진행 {len(self.orders_in_progress)}건")
        self.label_completed.setText(f"완료 {len(self.orders_completed)}건")

    def clear_order_details(self):
        self.orderTitle.setText("주문 #")
        layout = self.order_items_layout
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        self.label_delivery_location.setText("")
        self.label_delivery_time.setText("")
        self.selected_task_id = None
        self.readyButton.setVisible(False)

    # 2. add_new_order 함수를 아래 내용으로 교체
    def add_new_order(self, payload):
        task_id = payload.get('task_id')
        if task_id in self.orders_in_progress or task_id in self.orders_completed:
            return

        payload['timestamp'] = datetime.now()
        payload['status'] = "준비중"
        
        # 통합된 생성 함수 사용
        item_widget = self.create_order_item_widget(task_id, payload, "준비중")

        list_item = QListWidgetItem()
        list_item.setSizeHint(item_widget.sizeHint())
        
        self.listWidget_in_progress.insertItem(0, list_item)
        self.listWidget_in_progress.setItemWidget(list_item, item_widget)
        
        self.orders_in_progress[task_id] = {'payload': payload, 'list_item': list_item}
        self.update_ui_labels()

    # 3. mark_as_food_ready 함수를 아래 내용으로 교체
    def mark_as_food_ready(self):
        if self.selected_task_id is None:
            QMessageBox.warning(self, "오류", "준비완료 처리할 주문을 선택하세요.")
            return

        task_id = self.selected_task_id
        request_url = f"{RMS_HTTP_URL}/food_order_status_change"
        payload_to_send = {"type": "request", "action": "food_order_status_change", "payload": {"task_id": task_id}}

        try:
            response = requests.post(request_url, json=payload_to_send, timeout=5)
            response.raise_for_status()
            if response.json().get('payload', {}).get('status_changed') == 'food_ready':
                QMessageBox.information(self, "처리 완료", f"주문 #{task_id}이(가) '픽업 대기중' 상태로 변경되었습니다.")
                
                if task_id in self.orders_in_progress:
                    order_data = self.orders_in_progress[task_id]
                    order_data['payload']['status'] = "픽업 대기중"

                    # 통합된 생성 함수 사용
                    new_widget = self.create_order_item_widget(task_id, order_data['payload'], "픽업 대기중")
                    
                    list_item = order_data['list_item']
                    list_item.setSizeHint(new_widget.sizeHint())
                    self.listWidget_in_progress.setItemWidget(list_item, new_widget)
            else:
                raise Exception("서버 응답 오류")
        except requests.RequestException as e:
            QMessageBox.critical(self, "통신 오류", f"서버와 통신할 수 없습니다: {e}")
        except Exception as e:
            QMessageBox.critical(self, "처리 실패", f"서버에서 상태 변경을 실패했습니다: {e}")

    def update_order_details(self):
        if self.selected_task_id is None:
            self.clear_order_details()
            return

        order_data_dict = self.orders_in_progress.get(self.selected_task_id) or self.orders_completed.get(self.selected_task_id)
        if not order_data_dict:
            self.clear_order_details()
            return
        order_data = order_data_dict['payload']

        self.orderTitle.setText(f"주문 #{self.selected_task_id}")

        layout = self.order_items_layout
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        details = order_data.get('order_details') or order_data.get('request_details', {})
        items = details.get('items', [])
        total_quantity = 0
        total_price = 0

        for i, item_data in enumerate(items):
            name = item_data.get('name', 'N/A')
            quantity = item_data.get('quantity', 0)
            price = item_data.get('price', 0)
            
            total_quantity += quantity
            total_price += price * quantity
            
            layout.addWidget(QLabel(name), i, 0)
            layout.addWidget(QLabel(str(quantity)), i, 1, Qt.AlignmentFlag.AlignRight)
            layout.addWidget(QLabel(f"{price:,.0f}원"), i, 2, Qt.AlignmentFlag.AlignRight)
        
        if items:
            line = QFrame()
            line.setFrameShape(QFrame.Shape.HLine)
            line.setFrameShadow(QFrame.Shadow.Sunken)
            layout.addWidget(line, len(items), 0, 1, 3)

        total_label = QLabel("합계")
        total_label.setStyleSheet("font-weight: bold;")
        total_qty_label = QLabel(f"<b>{total_quantity}</b>")
        total_price_label = QLabel(f"<b>{total_price:,.0f}원</b>")

        layout.addWidget(total_label, len(items) + 1, 0)
        layout.addWidget(total_qty_label, len(items) + 1, 1, Qt.AlignmentFlag.AlignRight)
        layout.addWidget(total_price_label, len(items) + 1, 2, Qt.AlignmentFlag.AlignRight)

        location_text = order_data.get('request_location', 'N/A').replace("ROOM_", "") + "호"
        self.label_delivery_location.setText(location_text)
        
        timestamp = order_data.get('timestamp')
        if timestamp:
            self.label_delivery_time.setText(timestamp.strftime("%Y.%m.%d %H:%M"))

        self.readyButton.setEnabled(self.selected_task_id in self.orders_in_progress)

    def update_order_status(self, task_id, new_status):
        # '배송중' 상태로 변경 (진행 -> 완료 탭으로 이동)
        if new_status == "배송중" and task_id in self.orders_in_progress:
            data = self.orders_in_progress.pop(task_id)
            
            row = self.listWidget_in_progress.row(data['list_item'])
            self.listWidget_in_progress.takeItem(row)
            
            list_item = QListWidgetItem()
            
            # [수정] 삭제된 함수 대신 통합된 함수를 호출합니다.
            item_widget = self.create_order_item_widget(task_id, data['payload'], "배송중")
            list_item.setSizeHint(item_widget.sizeHint())
            
            self.listWidget_completed.insertItem(0, list_item)
            self.listWidget_completed.setItemWidget(list_item, item_widget)

            data['list_item'] = list_item
            self.orders_completed[task_id] = data

        # '완료' 상태로 변경 (완료 탭 내에서 업데이트)
        elif new_status == "완료" and task_id in self.orders_completed:
            data = self.orders_completed[task_id]
            
            row = self.listWidget_completed.row(data['list_item'])
            item = self.listWidget_completed.item(row) # takeItem 대신 item을 사용해야 위젯이 삭제되지 않음
            
            # [수정] 삭제된 함수 대신 통합된 함수를 호출합니다.
            new_widget = self.create_order_item_widget(task_id, data['payload'], "완료")
            item.setSizeHint(new_widget.sizeHint())
            
            self.listWidget_completed.setItemWidget(item, new_widget)
        
        self.update_ui_labels()
        # 주문 상세 정보는 선택이 해제될 때만 초기화하도록 로직 변경 가능 (현재는 유지)
        if self.selected_task_id == task_id:
            self.clear_order_details()

    def create_order_item_widget(self, task_id, payload, status):
        """모든 주문 항목 위젯을 생성하는 통합 함수"""
        widget = QWidget()
        widget.setStyleSheet("background-color: white; border-radius: 9px; padding: 10px;")

        main_layout = QVBoxLayout(widget)
        main_layout.setContentsMargins(12, 8, 12, 8)
        main_layout.setSpacing(4)

        timestamp = payload.get('timestamp', datetime.now())
        time_text = timestamp.strftime('%H:%M')
        location = payload.get('request_location', 'N/A').replace("ROOM_", "") + "호"
        
        # 상태별로 다른 HTML과 상세 텍스트를 설정
        status_html = ""
        details_text = location
        if status == "준비중":
            status_html = "<span style='background-color: #3498db; color: white; border-radius: 5px; padding: 3px 8px;'>준비중</span>"
            items = payload.get('order_details', {}).get('items', [])
            menu_count = sum(item.get('quantity', 0) for item in items)
            details_text = f"{location} | 메뉴 {menu_count}개"
        elif status == "픽업 대기중":
            status_html = "<span style='background-color: #95a5a6; color: white; border-radius: 5px; padding: 3px 8px;'>픽업 대기중</span>"
            items = payload.get('order_details', {}).get('items', [])
            menu_count = sum(item.get('quantity', 0) for item in items)
            details_text = f"{location} | 메뉴 {menu_count}개"
        elif status == "배송중":
            status_html = "<span style='background-color: #27ae60; color: white; border-radius: 5px; padding: 3px 8px;'>배송중</span>"
        else: # 완료
            status_html = "<span style='background-color: #7f8c8d; color: white; border-radius: 5px; padding: 3px 8px;'>완료</span>"

        # 첫 번째 줄
        top_layout = QHBoxLayout()
        top_layout.addWidget(QLabel(f"<b>주문 #{task_id}</b>"))
        top_layout.addStretch()
        top_layout.addWidget(QLabel(time_text))

        # 두 번째 줄
        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(QLabel(details_text))
        bottom_layout.addStretch()
        
        status_label = QLabel(status_html)
        status_label.setTextFormat(Qt.TextFormat.RichText)
        bottom_layout.addWidget(status_label)
        
        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)
        
        return widget

    def handle_websocket_message(self, data):
        action = data.get('action')
        payload = data.get('payload', {})
        if not payload: return

        task_id = payload.get('task_id')
        if not task_id: return

        if action in ['food_order_creation', 'supply_order_creation']:
            self.comm.new_order_received.emit(payload)
        elif action == 'food_pickup_arrival':
            self.comm.pickup_arrival_received.emit(payload)
        elif action == 'food_delivery_arrival':
            self.update_order_status(task_id, "완료")

    def start_websocket_connection(self):
        thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        thread.start()

    def run_websocket_client(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.websocket_handler())

    async def websocket_handler(self):
        while True:
            try:
                async with websockets.connect(RMS_WS_URL) as websocket:
                    logger.info(f"✅ WebSocket 서버에 연결되었습니다: {RMS_WS_URL}")
                    self.comm.connection_status.emit("서버에 연결되었습니다.")
                    async for message in websocket:
                        try:
                            self.comm.message_received.emit(json.loads(message))
                        except json.JSONDecodeError:
                            logger.warning(f"⚠️ 잘못된 JSON 형식의 메시지: {message}")
            except Exception as e:
                logger.error(f"❌ WebSocket 연결 오류: {e}")
                self.comm.connection_status.emit(f"서버 연결 끊김. 5초 후 재시도...")
                await asyncio.sleep(5)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StaffGUI()
    window.show()
    sys.exit(app.exec())