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

<<<<<<< HEAD
class StaffGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROOMIE")
        self.root.geometry("1200x800")
        self.root.configure(bg="#34495e")
        
        # 데이터 저장
        self.orders = {}  # task_id: order_data
        self.ready_orders = {}  # 준비완료된 주문들
        self.selected_order = None
        
        # WebSocket 연결 상태
        self.websocket = None
        self.ws_connected = False
        
        self.setup_ui()
        self.start_websocket_connection()
    
    def setup_ui(self):
        # 메인 프레임
        main_frame = tk.Frame(self.root, bg="#34495e")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 헤더
        header_frame = tk.Frame(main_frame, bg="#34495e", height=80)
        header_frame.pack(fill=tk.X, padx=20, pady=20)
        header_frame.pack_propagate(False)
        
        title_label = tk.Label(header_frame, text="ROOMIE", font=("Arial", 28, "bold"), 
                              fg="white", bg="#34495e")
        title_label.pack(side=tk.LEFT, anchor="w")
        
        restaurant_label = tk.Label(header_frame, text="Restaurant", font=("Arial", 18), 
                                   fg="#3498db", bg="#34495e")
        restaurant_label.pack(side=tk.RIGHT, anchor="e")
        
        # 컨텐츠 프레임
        content_frame = tk.Frame(main_frame, bg="#34495e")
        content_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=(0, 20))
        
        # 탭 프레임 (위쪽)
        tab_frame = tk.Frame(content_frame, bg="#34495e", height=60)
        tab_frame.pack(fill=tk.X, pady=(0, 10))
        tab_frame.pack_propagate(False)
        
        # 탭 버튼들
        self.tab_buttons_frame = tk.Frame(tab_frame, bg="#34495e")
        self.tab_buttons_frame.pack(side=tk.LEFT, anchor="w")
        
        self.orders_tab_btn = tk.Button(self.tab_buttons_frame, text="신청품목", 
                                       font=("Arial", 14, "bold"), bg="#3498db", 
                                       fg="white", relief="flat", padx=30, pady=10,
                                       command=lambda: self.switch_tab("orders"))
        self.orders_tab_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        self.ready_tab_btn = tk.Button(self.tab_buttons_frame, text="완료", 
                                      font=("Arial", 14), bg="#7f8c8d", 
                                      fg="white", relief="flat", padx=30, pady=10,
                                      command=lambda: self.switch_tab("ready"))
        self.ready_tab_btn.pack(side=tk.LEFT)
        
        # 메인 컨텐츠 영역 (아래쪽)
        self.content_area = tk.Frame(content_frame, bg="white")
        self.content_area.pack(fill=tk.BOTH, expand=True)
        
        # 초기 탭 설정
        self.current_tab = "orders"
        self.setup_orders_view()
        
    def switch_tab(self, tab_name):
        """탭 전환"""
        self.current_tab = tab_name
        
        # 탭 버튼 스타일 변경
        if tab_name == "orders":
            self.orders_tab_btn.config(bg="#3498db", font=("Arial", 14, "bold"))
            self.ready_tab_btn.config(bg="#7f8c8d", font=("Arial", 14))
        else:
            self.orders_tab_btn.config(bg="#7f8c8d", font=("Arial", 14))
            self.ready_tab_btn.config(bg="#3498db", font=("Arial", 14, "bold"))
        
        # 기존 위젯들 제거
        for widget in self.content_area.winfo_children():
            widget.destroy()
        
        # 해당 탭 뷰 생성
        if tab_name == "orders":
            self.setup_orders_view()
        else:
            self.setup_ready_view()
    
    def setup_orders_view(self):
        """신청품목 탭 뷰"""
        # 위쪽 주문 리스트 영역
        list_frame = tk.Frame(self.content_area, bg="#ecf0f1", height=200)
        list_frame.pack(fill=tk.X, padx=0, pady=0)
        list_frame.pack_propagate(False)
        
        # 주문 리스트 헤더
        list_header = tk.Frame(list_frame, bg="#bdc3c7", height=40)
        list_header.pack(fill=tk.X)
        list_header.pack_propagate(False)
        
        tk.Label(list_header, text="신청품목", font=("Arial", 14, "bold"), 
                bg="#bdc3c7", fg="#2c3e50").pack(side=tk.LEFT, padx=20, pady=10)
        
        # 주문 리스트
        list_content = tk.Frame(list_frame, bg="#ecf0f1")
        list_content.pack(fill=tk.BOTH, expand=True)
        
        # 스크롤 가능한 리스트박스
        scrollbar = tk.Scrollbar(list_content)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 20), pady=20)
        
        self.orders_listbox = tk.Listbox(list_content, font=("Arial", 12), 
                                        bg="#ecf0f1", selectmode=tk.SINGLE,
                                        yscrollcommand=scrollbar.set, relief="flat",
                                        selectbackground="#3498db", selectforeground="white")
        self.orders_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, 
                                padx=(20, 0), pady=20)
        scrollbar.config(command=self.orders_listbox.yview)
        self.orders_listbox.bind('<<ListboxSelect>>', self.on_order_select)
        
        # 아래쪽 주문 상세 정보 영역
        detail_frame = tk.Frame(self.content_area, bg="white")
        detail_frame.pack(fill=tk.BOTH, expand=True, padx=0, pady=0)
        
        # 주문 번호 헤더
        detail_header = tk.Frame(detail_frame, bg="white", height=80)
        detail_header.pack(fill=tk.X, padx=30, pady=(20, 0))
        detail_header.pack_propagate(False)
        
        self.order_title = tk.Label(detail_header, text="주문을 선택하세요", 
                                   font=("Arial", 20, "bold"), bg="white", fg="#2c3e50")
        self.order_title.pack(anchor="w", pady=20)
        
        # 상세 정보 컨테이너
        detail_content = tk.Frame(detail_frame, bg="white")
        detail_content.pack(fill=tk.BOTH, expand=True, padx=30, pady=(0, 20))
        
        # 왼쪽 영역 (주문정보, 배송정보)
        left_detail = tk.Frame(detail_content, bg="white")
        left_detail.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 20))
        
        # 주문정보 섹션
        order_info_frame = tk.Frame(left_detail, bg="white")
        order_info_frame.pack(fill=tk.X, pady=(0, 30))
        
        tk.Label(order_info_frame, text="주문정보", font=("Arial", 14, "bold"),
                bg="white", fg="#2c3e50").pack(anchor=tk.W, pady=(0, 10))
        
        # 구분선
        separator1 = tk.Frame(order_info_frame, height=2, bg="#ecf0f1")
        separator1.pack(fill=tk.X, pady=(0, 15))
        
        self.order_details_frame = tk.Frame(order_info_frame, bg="white")
        self.order_details_frame.pack(fill=tk.X)
        
        # 배송정보 섹션
        delivery_info_frame = tk.Frame(left_detail, bg="white")
        delivery_info_frame.pack(fill=tk.X)
        
        tk.Label(delivery_info_frame, text="배송정보", font=("Arial", 14, "bold"),
                bg="white", fg="#2c3e50").pack(anchor=tk.W, pady=(0, 10))
        
        # 구분선
        separator2 = tk.Frame(delivery_info_frame, height=2, bg="#ecf0f1")
        separator2.pack(fill=tk.X, pady=(0, 15))
        
        self.delivery_info_frame = tk.Frame(delivery_info_frame, bg="white")
        self.delivery_info_frame.pack(fill=tk.X)
        
        # 오른쪽 영역 (준비완료 버튼)
        right_detail = tk.Frame(detail_content, bg="white", width=200)
        right_detail.pack(side=tk.RIGHT, fill=tk.Y)
        right_detail.pack_propagate(False)
        
        # 준비완료 버튼
        button_container = tk.Frame(right_detail, bg="white")
        button_container.pack(expand=True, fill=tk.X)
        
        self.ready_button = tk.Button(button_container, text="준비완료", 
                                     font=("Arial", 16, "bold"), bg="#27ae60", 
                                     fg="white", relief="flat", pady=15, 
                                     command=self.mark_ready)
        self.ready_button.pack(fill=tk.X, pady=50)
        
        self.clear_order_details()
    
    def setup_ready_view(self):
        """완료 탭 뷰"""
        # 완료된 주문 리스트
        ready_header = tk.Frame(self.content_area, bg="#bdc3c7", height=40)
        ready_header.pack(fill=tk.X)
        ready_header.pack_propagate(False)
        
        tk.Label(ready_header, text="준비완료된 주문", font=("Arial", 14, "bold"),
                bg="#bdc3c7", fg="#2c3e50").pack(side=tk.LEFT, padx=20, pady=10)
        
        ready_content = tk.Frame(self.content_area, bg="#ecf0f1")
        ready_content.pack(fill=tk.BOTH, expand=True)
        
        # 준비완료 리스트박스
        ready_scrollbar = tk.Scrollbar(ready_content)
        ready_scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 20), pady=20)
        
        self.ready_listbox = tk.Listbox(ready_content, font=("Arial", 12),
                                       bg="#ecf0f1", yscrollcommand=ready_scrollbar.set,
                                       relief="flat", selectbackground="#27ae60",
                                       selectforeground="white")
        self.ready_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True,
                               padx=(20, 0), pady=20)
        ready_scrollbar.config(command=self.ready_listbox.yview)
    
    def clear_order_details(self):
        self.order_title.config(text="주문을 선택하세요")
        
        # 기존 위젯들 제거
        for widget in self.order_details_frame.winfo_children():
            widget.destroy()
        for widget in self.delivery_info_frame.winfo_children():
            widget.destroy()
            
        self.ready_button.config(state="disabled")
    
    def display_order_details(self, order_data):
        self.clear_order_details()
        
        logger.info(f"🔍 display_order_details 호출됨: {order_data}")
        
        task_id = order_data['task_id']
        self.order_title.config(text=f"주문 #{task_id}")
        
        # 주문 항목들 표시
        items = order_data['order_details']['items']
        logger.info(f"📦 주문 항목들: {items}")
        total_amount = 0
        
        for item in items:
            item_frame = tk.Frame(self.order_details_frame, bg="white")
            item_frame.pack(fill=tk.X, pady=5)
            
            name_label = tk.Label(item_frame, text=item['name'], 
                                 font=("Arial", 12), bg="white", fg="#2c3e50")
            name_label.pack(side=tk.LEFT)
            
            quantity_label = tk.Label(item_frame, text=str(item['quantity']), 
                                     font=("Arial", 12), bg="white", fg="#2c3e50")
            quantity_label.pack(side=tk.RIGHT, padx=(0, 100))
            
            price_label = tk.Label(item_frame, text=f"{item['price']:,}원", 
                                  font=("Arial", 12, "bold"), bg="white", fg="#e74c3c")
            price_label.pack(side=tk.RIGHT)
            
            total_amount += item['price'] * item['quantity']
        
        # 총액 표시
        total_frame = tk.Frame(self.order_details_frame, bg="white")
        total_frame.pack(fill=tk.X, pady=(15, 0))
        
        # 굵은 선
        separator = tk.Frame(total_frame, height=2, bg="#2c3e50")
        separator.pack(fill=tk.X, pady=(0, 10))
        
        tk.Label(total_frame, text=f"총 {len(items)}개", font=("Arial", 12, "bold"), 
                bg="white", fg="#2c3e50").pack(side=tk.RIGHT, padx=(0, 100))
        tk.Label(total_frame, text=f"{total_amount:,}원", font=("Arial", 14, "bold"), 
                bg="white", fg="#e74c3c").pack(side=tk.RIGHT)
        
        # 배송정보 표시
        room_frame = tk.Frame(self.delivery_info_frame, bg="white")
        room_frame.pack(fill=tk.X, pady=5)
        tk.Label(room_frame, text="호실", font=("Arial", 12), 
                bg="white", fg="#2c3e50").pack(side=tk.LEFT)
        location = order_data.get('request_location', 'N/A')
        tk.Label(room_frame, text=location, 
                font=("Arial", 12, "bold"), bg="white", fg="#2c3e50").pack(side=tk.RIGHT)
        
        # 주문 시간 표시
        time_frame = tk.Frame(self.delivery_info_frame, bg="white")
        time_frame.pack(fill=tk.X, pady=5)
        tk.Label(time_frame, text="주문 일시", font=("Arial", 12), 
                bg="white", fg="#2c3e50").pack(side=tk.LEFT)
        current_time = datetime.now().strftime("%Y.%m.%d %H:%M")
        tk.Label(time_frame, text=current_time, font=("Arial", 12), 
                bg="white", fg="#2c3e50").pack(side=tk.RIGHT)
        
        # 상태 표시
        status_frame = tk.Frame(self.delivery_info_frame, bg="white")
        status_frame.pack(fill=tk.X, pady=5)
        tk.Label(status_frame, text="상태", font=("Arial", 12), 
                bg="white", fg="#2c3e50").pack(side=tk.LEFT)
        tk.Label(status_frame, text="픽업 대기중", font=("Arial", 12, "bold"), 
                bg="white", fg="#f39c12").pack(side=tk.RIGHT)
        
        self.ready_button.config(state="normal")
    
    def on_order_select(self, event):
        selection = self.orders_listbox.curselection()
        if selection:
            index = selection[0]
            order_text = self.orders_listbox.get(index)
            # 주문 번호 추출 (주문 #TASK_001 11:42 형식에서)
            task_id = order_text.split()[1].replace('#', '')
            if task_id in self.orders:
                self.selected_order = task_id
                self.display_order_details(self.orders[task_id])
    
    def add_new_order(self, order_data):
        """새 주문 추가"""
        task_id = str(order_data['task_id']) 
        self.orders[task_id] = order_data
        
        # 주문 시간 (현재 시간으로 설정)
        current_time = datetime.now().strftime("%H:%M")
        order_text = f"주문 #{task_id}        {current_time}"
        
        self.orders_listbox.insert(tk.END, order_text)
        
        # 새로 추가된 주문을 자동으로 선택하고 세부 정보를 표시
        new_order_index = self.orders_listbox.size() - 1
        if new_order_index >= 0:
            self.orders_listbox.selection_clear(0, tk.END)
            self.orders_listbox.selection_set(new_order_index)
            self.orders_listbox.see(new_order_index)
            self.on_order_select(None)

        # 알림 표시
        messagebox.showinfo("새 주문", f"새로운 주문이 접수되었습니다!\n주문 번호: {task_id}")
    
    def mark_ready(self):
        """준비완료 처리"""
        if not self.selected_order:
            logger.warning("선택된 주문이 없어 준비완료 처리를 건너뜁니다.")
            return
            
        task_id = self.selected_order
=======
# --- 통신 및 알림 클래스 ---
class Communicate(QObject):
    message_received = pyqtSignal(dict)
    connection_status = pyqtSignal(str)
    new_order_received = pyqtSignal(dict)
    pickup_arrival_received = pyqtSignal(dict)

class CustomNotification(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint | Qt.WindowType.Tool | Qt.WindowType.WindowStaysOnTopHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setStyleSheet("background-color: rgba(30, 30, 30, 0.85); color: white; border-radius: 8px; padding: 15px;")

        # ## [수정] 레이아웃을 먼저 생성합니다. ##
        self.layout = QHBoxLayout(self)
        self.setLayout(self.layout)

        # ## [수정] 그 다음 위젯을 생성하고 레이아웃에 추가합니다. ##
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(40, 40)
        self.image_label.hide()

        self.text_label = QLabel(self)
        self.text_label.setWordWrap(True)
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
        
        self.layout.addWidget(self.image_label)
        self.layout.addWidget(self.text_label)
        
        # --- 이하 애니메이션 설정은 동일 ---
        self.opacity_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self.opacity_effect)
        self.animation = QPropertyAnimation(self.opacity_effect, b"opacity")
        self.hide_timer = QtCore.QTimer(self)
        self.hide_timer.setSingleShot(True)
        self.hide_timer.timeout.connect(self._start_fade_out)

    def show_notification(self, message, image_path=None, duration=4000):
        self.text_label.setText(message)

        if image_path and os.path.exists(image_path):
            from PyQt6.QtGui import QPixmap
            pixmap = QPixmap(image_path).scaled(40, 40, Qt.AspectRatioMode.KeepAspectRatio)
            self.image_label.setPixmap(pixmap)
            self.image_label.show()
        else:
            self.image_label.hide()

        self.adjustSize()
        
        if QApplication.primaryScreen():
            screen_geometry = QApplication.primaryScreen().availableGeometry()
            x = screen_geometry.width() - self.width() - 20
            y = screen_geometry.height() - self.height() - 20
            self.move(x, y)

        self.animation.stop()
        self.animation.setDuration(500)
        self.animation.setStartValue(0.0)
        self.animation.setEndValue(1.0)
        self.show()
        self.animation.start()
        self.hide_timer.start(duration)

    def _start_fade_out(self):
        self.animation.setDuration(1000)
        self.animation.setStartValue(1.0)
        self.animation.setEndValue(0.0)
        self.animation.finished.connect(self.hide)
        self.animation.start()

class OrderListItemWidget(QWidget):
    def __init__(self, task_id, location, items, timestamp, status=""):
        super().__init__()
        self.setStyleSheet("""
            background-color: white;
            border-radius: 8px;
            padding: 10px;
        """)
        
        grid_layout = QtWidgets.QGridLayout(self)
        grid_layout.setContentsMargins(12, 8, 12, 8)
        grid_layout.setSpacing(4)

        order_label = QLabel(f"<b>주문 #{task_id}</b>")
        order_label.setStyleSheet("font-size: 16px; background-color: transparent;")
        
        time_label = QLabel(timestamp.strftime("%H:%M"))
        time_label.setStyleSheet("color: #333; font-size: 14px; background-color: transparent;")
        
        grid_layout.addWidget(order_label, 0, 0)
        grid_layout.addWidget(time_label, 0, 1, Qt.AlignmentFlag.AlignRight)

        location_text = location.replace("ROOM_", "") + "호"
        menu_count = sum(item.get('quantity', 0) for item in items)
        details_label = QLabel(f"{location_text} | 메뉴 {menu_count}개")
        details_label.setStyleSheet("color: #555; background-color: transparent;")
        
        status_html = ""
        if status == "준비중":
            status_html = "<span style='background-color: #3498db; color: white; border-radius: 5px; padding: 2px 8px;'>준비중</span>"
        elif status == "픽업 대기중":
            status_html = "<span style='background-color: #95a5a6; color: white; border-radius: 5px; padding: 2px 8px;'>픽업 대기중</span>"
        
        status_label = QLabel(status_html)
        status_label.setTextFormat(Qt.TextFormat.RichText)
        status_label.setStyleSheet("background-color: transparent;")

        grid_layout.addWidget(details_label, 1, 0)
        if status:
            grid_layout.addWidget(status_label, 1, 1, Qt.AlignmentFlag.AlignRight)
        
        grid_layout.setColumnStretch(0, 1)

# --- 메인 GUI 클래스 (이하 동일) ---
class StaffGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi('staff_gui.ui', self)
        self.setWindowTitle("Staff GUI")

        self.orders_in_progress = {}
        self.orders_completed = {}
        self.selected_task_id = None
        self.notification_popup = CustomNotification()
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
        task_id = payload.get('task_id')
        location = payload.get('request_location', 'N/A')
        message = f"<b>신규 주문 접수</b><br>주문번호: #{task_id}<br>요청위치: {location}"
        
        image_file = "./image/order_call.png" 
        self.notification_popup.show_notification(message, image_path=image_file)
        
        self.order_sound_effect.play()

    def handle_pickup_arrival(self, payload):
        task_id = payload.get('task_id')
        robot_id = payload.get('robot_id')
        self.update_order_status(task_id, "배송중")
        message = f"로봇 #{robot_id}이 주문 #{task_id} 픽업을 위해 도착했습니다."
        
        robot_image_file = "./image/robot_call.png"
        self.notification_popup.show_notification(message, image_path=robot_image_file)
        
        # QMessageBox.information(self, "로봇 픽업 도착", message) # 커스텀 팝업으로 대체
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

    def add_new_order(self, payload):
        task_id = payload.get('task_id')
        if task_id in self.orders_in_progress or task_id in self.orders_completed:
            return

        location = payload.get('request_location', 'N/A')
        items = payload.get('order_details', {}).get('items', [])
        timestamp = datetime.now()
        
        item_widget = OrderListItemWidget(task_id, location, items, timestamp, status="준비중")

        list_item = QListWidgetItem()
        list_item.setSizeHint(item_widget.sizeHint())
        
        self.listWidget_in_progress.insertItem(0, list_item)
        self.listWidget_in_progress.setItemWidget(list_item, item_widget)
        
        payload['timestamp'] = timestamp
        payload['status'] = "준비중"
        self.orders_in_progress[task_id] = {'payload': payload, 'list_item': list_item}
        self.update_ui_labels()

    def mark_as_food_ready(self):
        if self.selected_task_id is None:
            QMessageBox.warning(self, "오류", "준비완료 처리할 주문을 선택하세요.")
            return

        task_id = self.selected_task_id
        request_url = f"{RMS_HTTP_URL}/food_order_status_change"
<<<<<<< HEAD
        request_payload = {
            "type": "request",
            "action": "food_order_status_change", 
            "payload": {
                "task_id": int(task_id)  # task_id를 int로 변환
            }
        }
        logger.info(f"⬆️ HTTP 요청 전송: URL='{request_url}', Payload={json.dumps(request_payload, ensure_ascii=False)}")

        try:
            response = requests.post(
                request_url,
                json=request_payload,
                timeout=5
            )
            
            logger.info(f"⬇️ HTTP 응답 수신: Status Code={response.status_code}, Response Body='{response.text}'")

            if response.status_code == 200:
                data = response.json()
                if data.get('payload', {}).get('status_changed') == 'food_ready':
                    # 성공적으로 상태 변경됨
                    self.move_to_ready(task_id)
                    messagebox.showinfo("완료", f"주문 #{task_id}이 준비완료되었습니다!")
                else:
                    messagebox.showerror("오류", f"상태 변경에 실패했습니다. 응답: {data}")
            else:
                messagebox.showerror("오류", f"서버 오류: {response.status_code}. 응답: {response.text}")
=======
        payload = {"type": "request", "action": "food_order_status_change", "payload": {"task_id": task_id}}

        try:
            response = requests.post(request_url, json=payload, timeout=5)
            response.raise_for_status()
            if response.json().get('payload', {}).get('status_changed') == 'food_ready':
                QMessageBox.information(self, "처리 완료", f"주문 #{task_id}이(가) '픽업 대기중' 상태로 변경되었습니다.")
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
                
                if task_id in self.orders_in_progress:
                    order_data = self.orders_in_progress[task_id]
                    order_data['payload']['status'] = "픽업 대기중"

                    payload = order_data['payload']
                    new_widget = OrderListItemWidget(
                        task_id,
                        payload.get('request_location', 'N/A'),
                        payload.get('order_details', {}).get('items', []),
                        payload.get('timestamp'),
                        status="픽업 대기중"
                    )
                    
                    list_item = order_data['list_item']
                    list_item.setSizeHint(new_widget.sizeHint())
                    self.listWidget_in_progress.setItemWidget(list_item, new_widget)
            else:
                raise Exception("서버 응답 오류")
        except requests.RequestException as e:
<<<<<<< HEAD
            logger.error(f"❌ 통신 오류 발생: {str(e)}")
            messagebox.showerror("오류", f"통신 오류: {str(e)}")
    
    def move_to_ready(self, task_id):
        """주문을 준비완료 탭으로 이동"""
        if task_id in self.orders:
            # 준비완료 목록에 추가
            order_data = self.orders[task_id]
            self.ready_orders[task_id] = order_data
            
            # 준비완료 리스트박스에 추가
            current_time = datetime.now().strftime("%H:%M")
            self.ready_listbox.insert(tk.END, f"주문 #{task_id} - 준비완료        {current_time}")
            
            # 신청품목에서 제거
            del self.orders[task_id]
            
            # 리스트박스에서 제거
            for i in range(self.orders_listbox.size()):
                if task_id in self.orders_listbox.get(i):
                    self.orders_listbox.delete(i)
                    break
            
            # 선택 해제
            self.selected_order = None
=======
            QMessageBox.critical(self, "통신 오류", f"서버와 통신할 수 없습니다: {e}")
        except Exception as e:
            QMessageBox.critical(self, "처리 실패", f"서버에서 상태 변경을 실패했습니다: {e}")

    def update_order_details(self):
        if self.selected_task_id is None:
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
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
        if new_status == "배송중" and task_id in self.orders_in_progress:
            data = self.orders_in_progress.pop(task_id)
            
            row = self.listWidget_in_progress.row(data['list_item'])
            self.listWidget_in_progress.takeItem(row)
            
            list_item = QListWidgetItem()
            item_widget = self.create_completed_item_widget(task_id, data['payload'], "배송중")
            list_item.setSizeHint(item_widget.sizeHint())
            
            self.listWidget_completed.insertItem(0, list_item)
            self.listWidget_completed.setItemWidget(list_item, item_widget)

            data['list_item'] = list_item
            self.orders_completed[task_id] = data

        elif new_status == "완료" and task_id in self.orders_completed:
            data = self.orders_completed[task_id]
            
            row = self.listWidget_completed.row(data['list_item'])
            item = self.listWidget_completed.takeItem(row)
            
            new_widget = self.create_completed_item_widget(task_id, data['payload'], "완료")
            item.setSizeHint(new_widget.sizeHint())
            
            self.listWidget_completed.addItem(item)
            self.listWidget_completed.setItemWidget(item, new_widget)
        
        self.update_ui_labels()
        self.clear_order_details()

    def create_completed_item_widget(self, task_id, payload, status):
        location = payload.get('request_location', 'N/A').replace("ROOM_", "") + "호"
        timestamp = payload.get('timestamp')
        time_text = timestamp.strftime('%H:%M') if timestamp else ""

        if status == "배송중":
            status_html = "<span style='background-color: #27ae60; color: white; border-radius: 5px; padding: 2px 8px;'>배송중</span>"
        else:
            status_html = "<span style='background-color: #95a5a6; color: white; border-radius: 5px; padding: 2px 8px;'>완료</span>"

        widget = QWidget()
        widget.setStyleSheet("background-color: white; border-radius: 8px; padding: 10px;")
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(12, 8, 12, 8)
        
        top_layout = QHBoxLayout()
        top_layout.addWidget(QLabel(f"<b>주문 #{task_id}</b>"))
        top_layout.addStretch()
        top_layout.addWidget(QLabel(time_text))
        
        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(QLabel(location))
        bottom_layout.addStretch()
        
        status_label = QLabel(status_html)
        status_label.setTextFormat(Qt.TextFormat.RichText)
        bottom_layout.addWidget(status_label)
        
        layout.addLayout(top_layout)
        layout.addLayout(bottom_layout)
        
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
<<<<<<< HEAD
                            data = json.loads(message)
                            logger.info(f"📨 받은 WebSocket 메시지: {json.dumps(data, ensure_ascii=False, indent=2)}")
                            self.root.after(0, self.handle_websocket_message, data)
=======
                            self.comm.message_received.emit(json.loads(message))
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
                        except json.JSONDecodeError:
                            logger.warning(f"⚠️ 잘못된 JSON 형식의 메시지: {message}")
            except Exception as e:
<<<<<<< HEAD
                logger.error(f"WebSocket 오류: {e}")
                self.ws_connected = False
                await asyncio.sleep(5)  # 5초 후 재연결 시도
    
    def handle_websocket_message(self, data):
        """WebSocket 메시지 처리 (메인 스레드에서 실행)"""
        message_type = data.get('type')
        action = data.get('action')
        payload = data.get('payload', {})
        
        logger.info(f"🔍 메시지 분석: type={message_type}, action={action}")
        
        if message_type == 'event':
            if action == 'food_order_creation':
                logger.info(f"🍽️ 새 주문 접수: {payload}")
                self.add_new_order(payload)
            elif action == 'food_pickup_arrival':
                logger.info(f"🤖 로봇 도착: task_id={payload.get('task_id')}, robot_id={payload.get('robot_id')}")
                task_id = payload.get('task_id')
                robot_id = payload.get('robot_id')
                self.show_robot_arrival(task_id, robot_id)
            else:
                logger.warning(f"❓ 처리되지 않은 이벤트 액션: {action}")
        else:
            logger.warning(f"❓ 처리되지 않은 메시지 타입: {message_type}")
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = StaffGUI()
    app.run()
=======
                logger.error(f"❌ WebSocket 연결 오류: {e}")
                self.comm.connection_status.emit(f"서버 연결 끊김. 5초 후 재시도...")
                await asyncio.sleep(5)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = StaffGUI()
    window.show()
    sys.exit(app.exec())
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
