import sys
import logging
import os
import math
from datetime import datetime

# PyQt6 및 관련 모듈
from PyQt6 import QtWidgets, uic, QtCore, QtGui
from PyQt6.QtCore import pyqtSignal, pyqtSlot, QThreadPool, QThread, Qt
from PyQt6.QtGui import QCloseEvent, QPixmap
from PyQt6.QtWidgets import QGraphicsScene, QGraphicsPixmapItem

# ROS2 관련 모듈
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from roomie_msgs.msg import RoomiePose

# 프로젝트 내부 모듈
from config import *
from communications import *


# --- 로거 설정 ---
def setup_logger():
    """로거 설정"""
    logger = logging.getLogger('roomie_agui')
    logger.setLevel(logging.INFO)
    
    if logger.hasHandlers():
        logger.handlers.clear()
    
    logs_dir = 'logs'
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_filename = f'roomie_agui_{timestamp}.log'
    log_filepath = os.path.join(logs_dir, log_filename)
    
    formatter = logging.Formatter('[%(asctime)s] - [%(levelname)s] - [%(filename)s:%(lineno)d] | %(message)s')
    
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    file_handler = logging.FileHandler(log_filepath, encoding='utf-8')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    logger.info(f"로그 파일 생성: {log_filepath}")
    return logger

logger = setup_logger()


# --- 헬퍼 클래스 ---
class Page:
    """stackedWidget의 페이지 인덱스를 관리하는 클래스"""
    DASHBOARD = 0
    ROBOT_MANAGEMENT = 1
    TASK_HISTORY = 2
    TASK_DETAIL = 3


class RobotCard(QtWidgets.QFrame):
    """로봇 정보를 표시하는 카드 위젯"""
    def __init__(self, robot_data: dict):
        super().__init__()
        self.robot_data = robot_data
        self.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.setObjectName('robot_card')
        self.setup_ui()
        self.apply_styles()
    
    def setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        self.create_header(layout)
        self.create_robot_info(layout)

    def create_header(self, parent_layout: QtWidgets.QLayout):
        header_layout = QtWidgets.QHBoxLayout()
        robot_id_label = QtWidgets.QLabel(f"Robot ID: {self.robot_data.get('robot_id', 'N/A')}")
        robot_id_label.setObjectName('card_header')
        self.battery_bar = self.create_battery_bar()
        header_layout.addWidget(robot_id_label)
        header_layout.addStretch()
        header_layout.addWidget(self.battery_bar)
        parent_layout.addLayout(header_layout)

    def create_battery_bar(self) -> QtWidgets.QProgressBar:
        battery_level = self.robot_data.get('battery_level', 0)
        is_charging = self.robot_data.get('is_charging', False)
        battery_bar = QtWidgets.QProgressBar()
        battery_bar.setRange(0, 100)
        battery_bar.setValue(battery_level)
        battery_bar.setTextVisible(True)
        battery_bar.setFormat(f'{battery_level}%{" (충전중)" if is_charging else ""}')
        battery_bar.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        battery_bar.setFixedWidth(150)
        battery_bar.setFixedHeight(24)
        return battery_bar

    def create_robot_info(self, parent_layout: QtWidgets.QLayout):
        model_name_label = QtWidgets.QLabel(f"모델: {self.robot_data.get('model_name', 'N/A')}")
        parent_layout.addWidget(model_name_label)
        floor_id = self.robot_data.get('floor_id', 'N/A')
        location_label = QtWidgets.QLabel(f"현재 위치: {floor_id + 1}층")
        parent_layout.addWidget(location_label)
        task_id = self.robot_data.get('task_id')
        task_label = QtWidgets.QLabel(f"현재 업무: Task #{task_id}" if task_id is not None else "현재 업무: 없음")
        parent_layout.addWidget(task_label)
        robot_status = self.robot_data.get('robot_status', '알 수 없음')
        if robot_status == "오류":
            error_code = self.robot_data.get('error_id', 'N/A')
            status_text = f"로봇 상태: <span style='color: red; font-weight: bold;'>오류 (코드: {error_code})</span>"
        else:
            status_text = f"로봇 상태: {robot_status}"
        status_label = QtWidgets.QLabel(status_text)
        parent_layout.addWidget(status_label)

    def get_battery_color(self) -> str:
        battery_level = self.robot_data.get('battery_level', 0)
        return "#e74c3c" if battery_level <= 20 else "#2ecc71"

    def apply_styles(self):
        battery_color = self.get_battery_color()
        self.setStyleSheet(f"""
            QFrame#robot_card {{ 
                background-color: white; border: 1px solid #dfe1e5; 
                border-radius: 8px; padding: 10px; margin-bottom: 10px;
            }}
            QLabel#card_header {{ 
                font-weight: bold; font-size: 16px; color: #2f3e4e;
            }}
            QProgressBar {{
                border: 1px solid #bdc3c7; border-radius: 12px;
                text-align: center; background-color: #ecf0f1; color: black;
            }}
            QProgressBar::chunk {{
                background-color: {battery_color}; border-radius: 11px;
            }}
        """)


class MapView(QtWidgets.QGraphicsView):
    """지도 표시를 위한 커스텀 QGraphicsView"""
    map_resized_signal = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.map_item = None
        self.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        self.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)

    def set_map_item(self, map_item: QtWidgets.QGraphicsPixmapItem):
        self.map_item = map_item
        self.fit_map_to_view()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self.fit_map_to_view()

    def fit_map_to_view(self):
        if self.map_item:
            self.fitInView(self.map_item, Qt.AspectRatioMode.KeepAspectRatio)
            resized_rect = self.map_item.sceneBoundingRect()
            width, height = resized_rect.width(), resized_rect.height()
            self.map_resized_signal.emit(width, height)


# --- 메인 윈도우 ---
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        logger.info("ROOMIE Admin GUI 초기화 시작")
        
        uic.loadUi(UI_FILE, self)
        
        self.setWindowTitle("ROOMIE - Admin")
        self.threadpool = QThreadPool()
        self.Page = Page
        
        # 상태 변수 초기화
        self.current_floor = 1
        self.map_item = None
        self.robot_items = {}
        self.robot_floor_map = {}
        self.map_scene = None
        
        # 통신 모듈 초기화
        self.ros_node = None
        self.ros_topic_handler = ROS2TopicHandler(self)
        
        # UI 및 기능 설정
        self.setup_ui_objects()
        self.setup_floor_buttons()
        self.setup_map_view()
        self.setup_ros2()
        self.setup_websocket()
        self.setup_connections()
        self.apply_styles()
        
        # 초기 데이터 로드 및 UI 표시
        self.init_ui()
        logger.info("ROOMIE Admin GUI 초기화 완료")

    # --- 설정 메소드 ---
    def setup_ui_objects(self):
        self.setStatusBar(QtWidgets.QStatusBar(self))
        self.findChild(QtWidgets.QWidget, 'widget').setObjectName('sidebar')
        self.findChild(QtWidgets.QFrame, 'frame_search_rm').setObjectName('frame_search_rm')
        self.findChild(QtWidgets.QFrame, 'frame_search_th').setObjectName('frame_search_th')
        self.widget_taskinfo.setObjectName('widget_taskinfo')
        self.widget_tasktimeline.setObjectName('widget_tasktimeline')
        for frame_name in ['frame', 'frame_2', 'frame_3', 'frame_4']:
            frame = self.findChild(QtWidgets.QFrame, frame_name)
            if frame: frame.setObjectName(frame_name)
        self.lineEdit_id.setObjectName('__temp_line_edit_id')

    def setup_floor_buttons(self):      
        self.floor_button_group = QtWidgets.QButtonGroup(self)
        self.floor_button_group.addButton(self.btn_floor1)
        self.floor_button_group.addButton(self.btn_floor2)
        self.btn_floor1.setCheckable(True)
        self.btn_floor2.setCheckable(True)
        self.btn_floor1.setChecked(True)

    def setup_map_view(self):
        logger.info("Map View 초기화 시작")
        self.map_scene = QGraphicsScene(self)
        self.dashboard_map.setScene(self.map_scene)
        try:
            map_pixmap = QPixmap('assets/hotel_map.pgm')
            if map_pixmap.isNull():
                logger.error("Map 이미지 로드 실패: 'assets/hotel_map.pgm' 파일을 확인하세요.")
                return
            self.map_item = QtWidgets.QGraphicsPixmapItem(map_pixmap)
            self.map_scene.addItem(self.map_item)
            self.dashboard_map.set_map_item(self.map_item)
            logger.info("PGM 지도 이미지 추가 완료")
        except Exception as e:
            logger.error(f"PGM 지도 이미지 추가 오류: {e}")

    def setup_ros2(self):
        logger.info("ROS2 초기화 시작")
        try:
            rclpy.init()
            self.ros_node = Node('roomie_agui_node')
            self.ros_topic_handler.setup_node(self.ros_node)
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
            self.ros_node.create_subscription(
                RoomiePose, '/roomie/status/roomie_pose',
                self.ros_topic_handler.roomie_pose_callback, qos
            )
            self.ros_timer = QtCore.QTimer(self)
            self.ros_timer.timeout.connect(self.process_ros2_messages)
            self.ros_timer.start(100)
            logger.info("ROS2 토픽 구독 설정 완료 - /roomie/status/roomie_pose")
        except Exception as e:
            logger.error(f"ROS2 초기화 오류: {e}")

    def setup_websocket(self):
        self.ws_thread = QThread()
        self.ws_client = WebSocketClient(WEBSOCKET_URL)
        self.ws_client.moveToThread(self.ws_thread)
        self.ws_thread.started.connect(self.ws_client.connect)
        self.ws_client.message_received.connect(self.handle_websocket_message)
        self.ws_client.connection_status_changed.connect(self.update_connection_status)
        self.ws_thread.start()

    def setup_connections(self):
        # 페이지 전환
        self.btn_db.clicked.connect(lambda: self.switch_page(self.Page.DASHBOARD))
        self.btn_rm.clicked.connect(lambda: self.switch_page(self.Page.ROBOT_MANAGEMENT))
        self.btn_th.clicked.connect(lambda: self.switch_page(self.Page.TASK_HISTORY))
        # 상세/뒤로가기
        self.btn_detail.clicked.connect(self.show_task_detail)
        self.btn_back.clicked.connect(lambda: self.switch_page(self.Page.TASK_HISTORY))
        # 검색
        self.btn_search_rm.clicked.connect(self.search_robots)
        self.btn_search_th.clicked.connect(self.search_tasks)
        # 층 선택
        self.floor_button_group.buttonClicked.connect(self.floor_button_clicked)
        # 지도 리사이즈
        self.dashboard_map.map_resized_signal.connect(self.map_size_display)
        # ROS2 시그널
        self.ros_topic_handler.robot_pose_updated.connect(self.handle_robot_pose_update)

    def init_ui(self):
        self.statusBar().showMessage("서버 연결 대기 중...")
        self.switch_page(self.Page.DASHBOARD)
        self.populate_filter_combos()
        self.search_tasks()
        self.search_robots()
        robot_scroll_content = self.findChild(QtWidgets.QWidget, 'robotcard')
        if robot_scroll_content and not robot_scroll_content.layout():
            layout = QtWidgets.QVBoxLayout(robot_scroll_content)
            layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)

    def apply_styles(self):
        self.setStyleSheet(APP_STYLE)

    # --- 이벤트 핸들러 및 슬롯 ---
    def closeEvent(self, event: QCloseEvent):
        logger.info("애플리케이션 종료 중...")
        if hasattr(self, 'ws_client'): self.ws_client.disconnect()
        if hasattr(self, 'ws_thread'):
            self.ws_thread.quit()
            self.ws_thread.wait()
        if hasattr(self, 'ros_timer'): self.ros_timer.stop()
        if hasattr(self, 'ros_node') and self.ros_node:
            self.ros_node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
        super().closeEvent(event)

    def switch_page(self, index: int):
        self.btn_db.setChecked(index == self.Page.DASHBOARD)
        self.btn_rm.setChecked(index == self.Page.ROBOT_MANAGEMENT)
        self.btn_th.setChecked(index == self.Page.TASK_HISTORY)
        self.stackedWidget.setCurrentIndex(index)

    def floor_button_clicked(self, button):
        new_floor = 1 if button.text() == "1층" else 2
        if self.current_floor != new_floor:
            self.current_floor = new_floor
            logger.info(f"지도 표시 층을 {self.current_floor}층으로 변경")
            self.update_all_robot_visibility()

    @pyqtSlot(int, int, object)
    def handle_robot_pose_update(self, robot_id: int, floor: int, pose):
        robot_current_floor = floor + 1
        self.robot_floor_map[robot_id] = robot_current_floor
        
        if robot_id not in self.robot_items:
            try:
                pixmap = QPixmap('assets/robot_icon.png').scaled(12, 12, transformMode=Qt.TransformationMode.SmoothTransformation)
                if pixmap.isNull():
                    logger.error(f"로봇 {robot_id} 아이콘 이미지 로드 실패")
                    return
                robot_item = QGraphicsPixmapItem(pixmap)
                robot_item.setZValue(1)
                robot_item.setTransformOriginPoint(robot_item.boundingRect().center())
                self.map_scene.addItem(robot_item)
                self.robot_items[robot_id] = robot_item
                logger.info(f"로봇 {robot_id} 아이콘을 지도에 새로 추가")
            except Exception as e:
                logger.error(f"로봇 아이콘 생성 중 오류: {e}")
                return

        robot_item = self.robot_items[robot_id]
        robot_item.setPos(pose.position.x, pose.position.y)
        
        q = pose.orientation
        yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_deg = math.degrees(yaw_rad)
        robot_item.setRotation(yaw_deg)
        
        if robot_current_floor == self.current_floor:
            robot_item.setVisible(True)
        else:
            robot_item.setVisible(False)
            
        if self.stackedWidget.currentIndex() == self.Page.ROBOT_MANAGEMENT:
            self.search_robots()

    @pyqtSlot(dict)
    def handle_websocket_message(self, data: dict):
        action = data.get("action")
        payload = data.get("payload", {})
        if action == "task_status_update":
            self.label_totaltask.setText(str(payload.get("total_task_count", "N/A")))
            self.label_waitingtask.setText(str(payload.get("waiting_task_count", "N/A")))
        elif action == "robot_status_update":
            self.label_totalrobot.setText(str(payload.get("total_robot_count", "N/A")))
            self.label_activerobot.setText(str(payload.get("active_robot_count", "N/A")))

    @pyqtSlot(bool)
    def update_connection_status(self, connected: bool):
        self.statusBar().showMessage("서버 연결됨" if connected else "서버 연결 끊김 - 재연결 시도 중...")

    @pyqtSlot(str)
    def handle_http_error(self, error_msg: str):
        logger.error(f"HTTP Error: {error_msg}")

    @pyqtSlot(float, float)
    def map_size_display(self, width: float, height: float):
        logger.info(f"리사이징된 지도 크기: 너비={int(width)}px, 높이={int(height)}px")

    # --- ROS2 처리 ---
    def process_ros2_messages(self):
        if self.ros_node and rclpy.ok():
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.01)
            except Exception as e:
                logger.error(f"ROS2 메시지 처리 오류: {e}")

    # --- UI 업데이트 및 데이터 로드 ---
    def search_robots(self):
        filters = {}
        if self.lineEdit_id.text():
            try:
                filters["robot_id"] = int(self.lineEdit_id.text())
            except ValueError:
                return
        if self.combo_model.currentText() != "전체": filters["model_name"] = self.combo_model.currentText()
        if self.combo_robotstatus.currentText() != "전체": filters["robot_status"] = self.combo_robotstatus.currentText()
        payload = {"type": "request", "action": "robot_list", "payload": {"filters": filters}}
        self.run_http_worker("robot_list", payload, self.populate_robot_list)

    @pyqtSlot(dict)
    def populate_robot_list(self, data: dict):
        robot_list = data.get("payload", {}).get("robots", [])
        logger.info(f"로봇 목록 수신 - {len(robot_list)}개 로봇")
        container = self.findChild(QtWidgets.QWidget, 'robotcard')
        self._clear_layout(container.layout())
        if not robot_list:
            container.layout().addWidget(QtWidgets.QLabel("조건에 맞는 로봇을 찾을 수 없습니다."))
        else:
            for robot in robot_list:
                container.layout().addWidget(RobotCard(robot))

    def search_tasks(self):
        filters = {
            "start_date": self.dateEdit_start.date().toString('yyyy-MM-dd'),
            "end_date": self.dateEdit_end.date().toString('yyyy-MM-dd'),
        }
        if self.combo_tasktype.currentText() != "전체": filters["task_type"] = self.combo_tasktype.currentText()
        if self.combo_taskstatus.currentText() != "전체": filters["task_status"] = self.combo_taskstatus.currentText()
        if self.combo_destination.currentText() != "전체": filters["destination"] = self.combo_destination.currentText()
        payload = {"type": "request", "action": "task_list", "payload": {"filters": filters}}
        self.run_http_worker("task_list", payload, self.populate_task_table)

    @pyqtSlot(dict)
    def populate_task_table(self, data: dict):
        tasks = data.get("payload", {}).get("tasks", [])
        self.tableWidget.setRowCount(len(tasks))
        headers = ["Task ID", "작업 유형", "작업 상태", "목적지", "할당 로봇 ID", "작업 생성 시각", "작업 완료 시각"]
        self.tableWidget.setColumnCount(len(headers))
        self.tableWidget.setHorizontalHeaderLabels(headers)
        self.tableWidget.verticalHeader().setVisible(False)
        for row, task in enumerate(tasks):
            for col, key in enumerate(["task_id", "task_type", "task_status", "destination", "robot_id", "task_creation_time", "task_completion_time"]):
                value = task.get(key, "N/A")
                if "time" in key and value and value != "N/A":
                    value = value.replace("T", " ").replace("Z", "")
                self.tableWidget.setItem(row, col, QtWidgets.QTableWidgetItem(str(value)))
        self.tableWidget.resizeColumnsToContents()
        self.tableWidget.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.tableWidget.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
    
    def show_task_detail(self):
        selected_rows = self.tableWidget.selectionModel().selectedRows()
        if not selected_rows:
            QtWidgets.QMessageBox.warning(self, "선택 오류", "리스트에서 작업을 선택해주세요.")
            return
        task_id = int(self.tableWidget.item(selected_rows[0].row(), 0).text())
        payload = {"type": "request", "action": "task_detail", "payload": {"task_id": task_id}}
        self.run_http_worker("task_detail", payload, self.populate_task_detail)
        logger.info(f"작업 상세 정보 요청 전송 - Task ID: {task_id}")

    @pyqtSlot(dict)
    def populate_task_detail(self, data: dict):
        detail = data.get("payload", {})
        if self.widget_taskinfo.layout(): self._clear_layout(self.widget_taskinfo.layout())
        else: self.widget_taskinfo.setLayout(QtWidgets.QVBoxLayout())
        if self.widget_tasktimeline.layout(): self._clear_layout(self.widget_tasktimeline.layout())
        else: self.widget_tasktimeline.setLayout(QtWidgets.QVBoxLayout())
        self.populate_task_info(detail)
        self.populate_task_timeline(detail)
        self.switch_page(self.Page.TASK_DETAIL)
    
    def populate_task_info(self, detail: dict):
        layout = self.widget_taskinfo.layout()
        selected_row = self.tableWidget.selectionModel().selectedRows()[0].row()
        task_id = self.tableWidget.item(selected_row, 0).text()
        task_type = self.tableWidget.item(selected_row, 1).text()
        task_status = self.tableWidget.item(selected_row, 2).text()
        destination = self.tableWidget.item(selected_row, 3).text()
        robot_id = self.tableWidget.item(selected_row, 4).text()
        creation_time = self.tableWidget.item(selected_row, 5).text()
        completion_time = self.tableWidget.item(selected_row, 6).text()
        actual_duration = ""
        if completion_time != "N/A" and creation_time != "N/A":
            start = datetime.fromisoformat(creation_time.replace(" ", "T"))
            end = datetime.fromisoformat(completion_time.replace(" ", "T"))
            duration = end - start
            actual_duration = f"{int(duration.total_seconds() // 60)}분 {int(duration.total_seconds() % 60)}초"
        estimated_time = detail.get("calculated_estimated_time")
        estimated_time_min = f"{estimated_time}분" if estimated_time is not None else "N/A"
        info_items = [
            ("Task ID", task_id), ("작업 종류", task_type), ("목적지", destination.replace("호", "")),
            ("배정 로봇 ID", robot_id), ("상태", task_status), ("요청 시간", creation_time),
            ("완료 시간", completion_time if completion_time != "N/A" else "진행중"),
            ("예상 소요 시간", estimated_time_min), ("실제 소요 시간", actual_duration)
        ]
        for label_text, value_text in info_items:
            row_layout = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(label_text)
            label.setFixedWidth(100)
            label.setStyleSheet("color: #666;")
            value = QtWidgets.QLabel(str(value_text))
            value.setStyleSheet("color: #2c3e50;")
            row_layout.addWidget(label)
            row_layout.addWidget(value)
            layout.addLayout(row_layout)
        layout.addStretch()
    
    def populate_task_timeline(self, detail: dict):
        layout = self.widget_tasktimeline.layout()
        creation_time = detail.get("task_creation_time")
        timeline_events = [
            {"time": creation_time, "title": "작업 요청 접수"},
            {"time": detail.get("robot_assignment_time"), "title": "로봇 할당 및 출발"},
            {"time": detail.get("pickup_completion_time"), "title": "픽업 완료"},
            {"time": detail.get("delivery_arrival_time"), "title": "목적지 도착"},
            {"time": detail.get("task_completion_time"), "title": "배송 완료"}
        ]
        for i, event in enumerate(timeline_events):
            event["completed"] = bool(event["time"])
            widget = self.create_timeline_event(event, i == 0, i == len(timeline_events) - 1)
            layout.addWidget(widget)
        layout.addStretch()
    
    # --- 헬퍼 메소드 ---
    def run_http_worker(self, endpoint: str, payload: dict, on_success, on_finished=None):
        worker = HttpWorker(endpoint, payload)
        worker.signals.result.connect(on_success)
        worker.signals.error.connect(self.handle_http_error)
        if on_finished: worker.signals.finished.connect(on_finished)
        self.threadpool.start(worker)

    def update_all_robot_visibility(self):
        logger.info(f"{self.current_floor}층의 로봇들을 지도에 표시합니다.")
        for robot_id, item in self.robot_items.items():
            robot_last_floor = self.robot_floor_map.get(robot_id)
            item.setVisible(robot_last_floor == self.current_floor)

    def create_timeline_event(self, event: dict, is_first: bool, is_last: bool):
        container = QtWidgets.QWidget()
        grid_layout = QtWidgets.QGridLayout(container)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        grid_layout.setVerticalSpacing(4)
        def create_line(completed):
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.Shape.VLine)
            line.setLineWidth(2)
            line.setStyleSheet(f"color: {'#27ae60' if completed else '#bdc3c7'};")
            return line
        top_line, bottom_line = create_line(event["completed"]), create_line(event["completed"])
        top_line.setVisible(not is_first)
        bottom_line.setVisible(not is_last)
        circle = QtWidgets.QLabel("●")
        circle.setStyleSheet(f"font-size: 14px; color: {'#27ae60' if event['completed'] else '#bdc3c7'};")
        time_label = QtWidgets.QLabel()
        if event["time"]:
            time_label.setText(event["time"].replace("T", " ").split(" ")[1][:5])
        time_label.setStyleSheet("color: #27ae60; font-weight: bold;")
        title_label = QtWidgets.QLabel(event["title"])
        title_label.setStyleSheet(f"color: {'#2c3e50' if event['completed'] else '#7f8c8d'};")
        grid_layout.addWidget(circle, 1, 0, QtCore.Qt.AlignmentFlag.AlignHCenter)
        grid_layout.addWidget(bottom_line, 2, 0, 1, 1, QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignTop)
        grid_layout.addWidget(top_line, 0, 0, 1, 1, QtCore.Qt.AlignmentFlag.AlignHCenter | QtCore.Qt.AlignmentFlag.AlignBottom)
        grid_layout.addWidget(time_label, 1, 1)
        grid_layout.addWidget(title_label, 1, 2)
        grid_layout.setColumnStretch(2, 1)
        return container
        
    def _clear_layout(self, layout: QtWidgets.QLayout):
        if layout is None: return
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget: widget.deleteLater()
            elif item.layout(): self._clear_layout(item.layout())

    def populate_filter_combos(self):
        self.combo_tasktype.addItems(TASK_TYPES)
        self.combo_taskstatus.addItems(TASK_STATUSES)
        self.combo_robotstatus.addItems(ROBOT_STATUSES)
        self.combo_destination.addItems(DESTINATIONS)
        self.combo_model.addItems(ROBOT_MODELS)


# --- 애플리케이션 실행 ---
def main():
    """메인 함수"""
    logger.info("ROOMIE Admin GUI 애플리케이션 시작")
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("ROOMIE Admin")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()