import sys
import json
import requests
from PyQt6 import QtWidgets, uic, QtCore
from PyQt6.QtCore import (
    QObject, QRunnable, pyqtSignal, pyqtSlot, QThreadPool, Qt
)
from PyQt6.QtWebSockets import QWebSocket
from PyQt6.QtGui import QColor

# --- API URL 및 WebSocket URL 설정 ---
SERVER_IP = "192.168.0.47"
SERVER_PORT = 8000
API_URL = f"http://{SERVER_IP}:{SERVER_PORT}/api/gui"
WEBSOCKET_URL = f"ws://{SERVER_IP}:{SERVER_PORT}/api/gui/ws/admin/admin_01"

# --- 스레드 실행을 위한 Worker 클래스 ---
class WorkerSignals(QObject):
    """
    Worker 스레드의 시그널을 정의합니다.
    - finished: 작업 완료 시
    - error: 에러 발생 시
    - result: 결과 데이터 반환 시
    """
    finished = pyqtSignal()
    error = pyqtSignal(str)
    result = pyqtSignal(dict)

class HttpWorker(QRunnable):
    """
    백그라운드에서 HTTP 요청을 처리하는 Worker
    """
    def __init__(self, endpoint, payload):
        super().__init__()
        self.signals = WorkerSignals()
        self.endpoint = endpoint
        self.payload = payload

    @pyqtSlot()
    def run(self):
        try:
            # API_URL과 엔드포인트(e.g., /task_list)를 조합합니다.
            url = f"{API_URL}/{self.endpoint}"
            response = requests.post(url, json=self.payload, timeout=5)
            response.raise_for_status()
            self.signals.result.emit(response.json())
        except requests.exceptions.RequestException as e:
            self.signals.error.emit(str(e))
        finally:
            self.signals.finished.emit()

# --- WebSocket 클라이언트 ---
class WebSocketClient(QObject):
    """
    서버와 WebSocket 통신을 관리하는 클라이언트
    """
    message_received = pyqtSignal(dict)

    def __init__(self, url):
        super().__init__()
        self.client = QWebSocket()
        self.url = url
        self.client.textMessageReceived.connect(self.on_message_received)
        self.client.connected.connect(self.on_connected)
        self.client.disconnected.connect(self.on_disconnected)

    def connect(self):
        print(f"Connecting to WebSocket at {self.url}...")
        self.client.open(QtCore.QUrl(self.url))

    def on_connected(self):
        print("WebSocket connected successfully.")

    def on_disconnected(self):
        print("WebSocket disconnected. Attempting to reconnect...")
        QtCore.QTimer.singleShot(3000, self.connect) # 3초 후 재연결 시도

    def on_message_received(self, message):
        try:
            data = json.loads(message)
            if data.get("type") == "event":
                self.message_received.emit(data)
        except json.JSONDecodeError:
            print(f"Failed to decode WebSocket message: {message}")

# --- 로봇 정보 카드 위젯 ---
class RobotCard(QtWidgets.QFrame):
    def __init__(self, robot_data):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setObjectName('robot_card')
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # 로봇 ID와 모델명
        header_layout = QtWidgets.QHBoxLayout()
        robot_id_label = QtWidgets.QLabel(f"Robot ID: {robot_data.get('robot_id', 'N/A')}")
        robot_id_label.setObjectName('card_header')
        model_name_label = QtWidgets.QLabel(robot_data.get('model_name', 'N/A'))
        header_layout.addWidget(robot_id_label)
        header_layout.addStretch()
        header_layout.addWidget(model_name_label)
        layout.addLayout(header_layout)

        # 배터리 상태
        battery_level = robot_data.get('battery_level', 0)
        is_charging = robot_data.get('is_charging', False)
        battery_label = QtWidgets.QLabel(f"Battery: {battery_level}% {'(Charging)' if is_charging else ''}")
        layout.addWidget(battery_label)

        # 로봇 상태 및 현재 작업
        status_label = QtWidgets.QLabel(f"Status: {robot_data.get('robot_status', 'Unknown')}")
        task_id = robot_data.get('task_id')
        task_label = QtWidgets.QLabel(f"Current Task: {task_id if task_id is not None else 'None'}")
        layout.addWidget(status_label)
        layout.addWidget(task_label)
        
        # 에러 상태
        if robot_data.get('has_error'):
            error_code = robot_data.get('error_code', 'N/A')
            error_label = QtWidgets.QLabel(f"Error: {error_code}")
            error_label.setStyleSheet("color: red; font-weight: bold;")
            layout.addWidget(error_label)
        
        self.setStyleSheet("""
            QFrame#robot_card { 
                background-color: white; 
                border: 1px solid #dfe1e5; 
                border-radius: 8px; 
                padding: 10px;
                margin-bottom: 10px;
            }
            QLabel#card_header { font-weight: bold; font-size: 16px; }
        """)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi('admin_gui.ui', self)

        self.setWindowTitle("ROOMIE - Admin")
        self.threadpool = QThreadPool()

        self.setup_ui_objects()
        self.apply_styles()
        self.setup_connections()
        
        self.ws_client = WebSocketClient(WEBSOCKET_URL)
        self.ws_client.message_received.connect(self.handle_websocket_message)
        self.ws_client.connect()

        self.init_ui()

    def setup_ui_objects(self):
        sidebar = self.findChild(QtWidgets.QWidget, 'widget')
        if sidebar: sidebar.setObjectName('sidebar')
        frame_rm = self.findChild(QtWidgets.QFrame, 'frame_search_rm')
        if frame_rm: frame_rm.setObjectName('frame_search_rm')
        frame_th = self.findChild(QtWidgets.QFrame, 'frame_search_th')
        if frame_th: frame_th.setObjectName('frame_search_th')
        self.widget_taskinfo.setObjectName('widget_taskinfo')
        self.widget_tasktimeline.setObjectName('widget_tasktimeline')
        self.findChild(QtWidgets.QFrame, 'frame').setObjectName('frame')
        self.findChild(QtWidgets.QFrame, 'frame_2').setObjectName('frame_2')
        self.findChild(QtWidgets.QFrame, 'frame_3').setObjectName('frame_3')
        self.findChild(QtWidgets.QFrame, 'frame_4').setObjectName('frame_4')

    def setup_connections(self):
        self.btn_db.clicked.connect(lambda: self.switch_page(0))
        self.btn_rm.clicked.connect(lambda: self.switch_page(1))
        self.btn_th.clicked.connect(lambda: self.switch_page(2))
        self.btn_detail.clicked.connect(self.show_task_detail)
        self.btn_back.clicked.connect(lambda: self.switch_page(2))
        self.btn_search_rm.clicked.connect(self.search_robots)
        self.btn_search_th.clicked.connect(self.search_tasks)

    def init_ui(self):
        self.btn_db.setChecked(True)
        self.stackedWidget.setCurrentIndex(0)
        self.populate_filter_combos()
        self.search_tasks()
        self.search_robots()
        
        robot_scroll_content = self.findChild(QtWidgets.QWidget, 'robotcard')
        layout = QtWidgets.QVBoxLayout(robot_scroll_content)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        robot_scroll_content.setLayout(layout)

    def populate_filter_combos(self):
        task_types = ["전체", "음식배송", "비품배송", "호출", "길안내"]
        task_statuses = ["전체", "접수됨", "준비 완료", "로봇 할당됨", "픽업 장소로 이동", "픽업 대기 중", "배송 중", "배송 도착", "수령 완료", "호출 접수됨", "호출 로봇 할당됨", "호출 이동 중", "호출 도착", "길안내 접수됨", "길안내 중", "길안내 도착"]
        robot_statuses = ["전체", "초기화", "충전상태", "작업대기", "픽업위치 이동", "픽업대기", "배송장소 이동", "수령대기", "호출위치 이동", "길안내 목적지 입력대기", "길안내 이동", "대상 탐색", "대기위치로 이동", "엘리베이터 탑승", "오류"]
        destinations = ["전체", "ROOM_101", "ROOM_102", "ROOM_201", "ROOM_202"]
        model = ["전체", "ServiceBot_V1", "ServiceBot_V2"]
        
        self.combo_tasktype.addItems(task_types)
        self.combo_taskstatus.addItems(task_statuses)
        self.combo_robotstatus.addItems(robot_statuses)
        self.combo_destination.addItems(destinations)
        self.combo_model.addItems(model)
    
    def switch_page(self, index):
        for btn in (self.btn_db, self.btn_rm, self.btn_th):
            btn.setChecked(False)
        
        if index == 0: self.btn_db.setChecked(True)
        elif index == 1: self.btn_rm.setChecked(True)
        elif index == 2: self.btn_th.setChecked(True)

        self.stackedWidget.setCurrentIndex(index)

    def run_http_worker(self, endpoint, payload, on_success):
        worker = HttpWorker(endpoint, payload)
        worker.signals.result.connect(on_success)
        worker.signals.error.connect(lambda e: print(f"HTTP Error: {e}"))
        self.threadpool.start(worker)

    @pyqtSlot(dict)
    def handle_websocket_message(self, data):
        action = data.get("action")
        payload = data.get("payload", {})
        
        if action == "task_status_update":
            self.label_totaltask.setText(str(payload.get("total_task_count", "N/A")))
            self.label_waitingtask.setText(str(payload.get("waiting_task_count", "N/A")))
            if self.stackedWidget.currentIndex() == 2:
                self.search_tasks()

        elif action == "robot_status_update":
            self.label_totalrobot.setText(str(payload.get("total_robot_count", "N/A")))
            total = payload.get("total_robot_count", 0)
            waiting = payload.get("waiting_robot_count", 0)
            self.label_activerobot.setText(str(total - waiting))
            if self.stackedWidget.currentIndex() == 1:
                self.search_robots()

    def search_robots(self):
        filters = {}
        if self.lineEdit_id.text():
            try:
                filters["robot_id"] = int(self.lineEdit_id.text())
            except ValueError:
                print("Invalid Robot ID. Must be an integer.")
                return
        if self.combo_model.currentText():
            filters["model_name"] = self.combo_model.currentText()
        if self.combo_robotstatus.currentText():
            filters["robot_status"] = self.combo_robotstatus.currentText()
            
        payload = {"type": "request", "action": "robot_list", "payload": {"filters": filters}}
        self.run_http_worker("robot_list", payload, self.populate_robot_list)

    @pyqtSlot(dict)
    def populate_robot_list(self, data):
        robot_list = data.get("payload", {}).get("robots", [])
        
        robot_scroll_content = self.findChild(QtWidgets.QWidget, 'robotcard')
        layout = robot_scroll_content.layout()

        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        
        if not robot_list:
            layout.addWidget(QtWidgets.QLabel("No robots found."))
        else:
            for robot in robot_list:
                card = RobotCard(robot)
                layout.addWidget(card)

    def search_tasks(self):
        filters = {
            "start_date": self.dateEdit_start.date().toString('yyyy-MM-dd'),
            "end_date": self.dateEdit_end.date().toString('yyyy-MM-dd'),
            "task_type": self.combo_tasktype.currentText(),
            "task_status": self.combo_taskstatus.currentText(),
            "destination": self.combo_destination.currentText()
        }
        filters = {k: v for k, v in filters.items() if v}
        payload = {"type": "request", "action": "task_list", "payload": {"filters": filters}}
        self.run_http_worker("task_list", payload, self.populate_task_table)

    @pyqtSlot(dict)
    def populate_task_table(self, data):
        tasks = data.get("payload", {}).get("tasks", [])
        self.tableWidget.setRowCount(len(tasks))
        
        headers = ["Task ID", "Type", "Status", "Destination", "Robot ID", "Created Time", "Completed Time"]
        self.tableWidget.setColumnCount(len(headers))
        self.tableWidget.setHorizontalHeaderLabels(headers)

        for row, task in enumerate(tasks):
            self.tableWidget.setItem(row, 0, QtWidgets.QTableWidgetItem(str(task.get("task_id"))))
            self.tableWidget.setItem(row, 1, QtWidgets.QTableWidgetItem(task.get("task_type")))
            self.tableWidget.setItem(row, 2, QtWidgets.QTableWidgetItem(task.get("task_status")))
            self.tableWidget.setItem(row, 3, QtWidgets.QTableWidgetItem(task.get("destination")))
            self.tableWidget.setItem(row, 4, QtWidgets.QTableWidgetItem(str(task.get("robot_id", ""))))
            self.tableWidget.setItem(row, 5, QtWidgets.QTableWidgetItem(task.get("task_creation_time")))
            self.tableWidget.setItem(row, 6, QtWidgets.QTableWidgetItem(task.get("task_completion_time", "N/A")))
            
        self.tableWidget.resizeColumnsToContents()
        self.tableWidget.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.tableWidget.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)

    def show_task_detail(self):
        selected_rows = self.tableWidget.selectionModel().selectedRows()
        if not selected_rows:
            QtWidgets.QMessageBox.warning(self, "Selection Error", "Please select a task from the list.")
            return
        
        task_id = int(self.tableWidget.item(selected_rows[0].row(), 0).text())
        payload = {"type": "request", "action": "task_detail", "payload": {"task_id": task_id}}
        
        self.run_http_worker("task_detail", payload, self.populate_task_detail)

    @pyqtSlot(dict)
    def populate_task_detail(self, data):
        detail = data.get("payload", {})

        for widget_container in [self.widget_taskinfo, self.widget_tasktimeline]:
            if widget_container.layout():
                while widget_container.layout().count():
                    item = widget_container.layout().takeAt(0)
                    widget = item.widget()
                    if widget: widget.deleteLater()
            else:
                widget_container.setLayout(QtWidgets.QVBoxLayout())
        
        timeline_layout = self.widget_tasktimeline.layout()
        timeline_data = {
            "Robot Assigned": detail.get("robot_assignment_time"),
            "Pickup Completed": detail.get("pickup_completion_time"),
            "Delivery Arrival": detail.get("delivery_arrival_time"),
            "Task Completed": detail.get("task_completion_time"),
        }

        for event, time in timeline_data.items():
            label = QtWidgets.QLabel(f"{event}: {time if time else 'Pending'}")
            timeline_layout.addWidget(label)
        timeline_layout.addStretch()

        self.switch_page(3)

    def apply_styles(self):
        style = """
        QWidget { 
            font-family: 'Malgun Gothic'; 
        }
        QMainWindow {
            background-color: #f2f4f5;
        }
        QWidget#sidebar {
            background-color: #2f3e4e;
        }
        QLabel#label_title {
            color: white;
            font-weight: bold;
            font-size: 24px;
        }
        QLabel#label_admin {
            color: #e04e3e;
            font-weight: bold;
            font-size: 16px;
        }
        QPushButton#btn_db, QPushButton#btn_rm, QPushButton#btn_th {
            color: white;
            background-color: transparent;
            border: none;
            padding: 10px;
            text-align: left;
            font-size: 16px;
            font-weight: bold;
        }
        QPushButton#btn_db:checked, QPushButton#btn_rm:checked, QPushButton#btn_th:checked {
            background-color: #e04e3e;
            border-radius: 4px;
        }
        QPushButton#btn_db:hover, QPushButton#btn_rm:hover, QPushButton#btn_th:hover {
            background-color: #c84331;
            border-radius: 4px;
        }
        QLabel#label_db, QLabel#label_rm, QLabel#label_th, QLabel#label_td {
            color: #2f3e4e;
            font-size: 30px;
            font-weight: bold;
        }
        QFrame#frame, QFrame#frame_2, QFrame#frame_3, QFrame#frame_4 {
            background-color: white;
            border-radius: 8px;
            padding: 10px;
        }
        QLabel#label_totaltask, QLabel#label_waitingtask, QLabel#label_totalrobot, QLabel#label_activerobot {
            color: #2f3e4e;
            font-size: 26px;
            font-weight: bold;
        }
        QLabel#label_totaltask_2, QLabel#label_waitingtask_2, QLabel#label_totalrobot_2, QLabel#label_activerobot_2 {
            color: #4a5b6a;
            font-size: 18px;
        }
        QLabel#label_robotlocation {
            color: #2f3e4e;
            font-size: 14px;
            font-weight: bold;
        }
        QWidget#frame_search_rm, QWidget#frame_search_th {
            background-color: white;
            border-radius: 8px;
            padding: 8px;
        }
        QLineEdit, QComboBox, QDateEdit {        
            border: 1px solid #ccc;
            padding: 4px 4px;
        }
        QPushButton#btn_search_rm, QPushButton#btn_search_th {
            background-color: #e04e3e;
            color: white;
            border-radius: 4px;
        }
        QPushButton#btn_search_rm:hover, QPushButton#btn_search_th:hover {
            background-color: #c84331;
        }
        QHeaderView::section {
            background-color: #2f3e4e;
            color: white;
            padding: 6px;
            font-size: 13px;
        }
        QTableWidget {
            background-color: white;
            alternate-background-color: #f9fafb;
            gridline-color: #dfe1e5;
        }
        QWidget#widget_taskinfo, QWidget#widget_tasktimeline {
            background-color: white;
            border-radius: 8px;
            padding: 12px;
        }
        QPushButton#btn_detail {
            background-color: #e04e3e;
            color: white;
            border-radius: 4px;
        }
        QPushButton#btn_detail:hover {
            background-color: #c84331;
        }
        QPushButton#btn_back {
            background-color: #2f3e4e;
            color: white;
            border-radius: 4px;
            padding: 6px;
        }
        QPushButton#btn_back:hover {
            background-color: #1e293b;
        }
        """
        self.setStyleSheet(style)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())