import sys
from PyQt6 import QtWidgets, uic, QtCore
from PyQt6.QtCore import pyqtSlot, QThreadPool, QThread
from PyQt6.QtGui import QCloseEvent

from config import *
from communications import HttpWorker, WebSocketClient


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
        """UI 구성 요소 설정"""
        layout = QtWidgets.QVBoxLayout(self)

        self.create_header(layout)
        self.create_robot_info(layout)

    def create_header(self, parent_layout: QtWidgets.QLayout):
        """헤더 부분 생성 (로봇 ID + 배터리)"""
        header_layout = QtWidgets.QHBoxLayout()
        
        robot_id_label = QtWidgets.QLabel(f"Robot ID: {self.robot_data.get('robot_id', 'N/A')}")
        robot_id_label.setObjectName('card_header')
        
        self.battery_bar = self.create_battery_bar()
        
        header_layout.addWidget(robot_id_label)
        header_layout.addStretch()
        header_layout.addWidget(self.battery_bar)
        parent_layout.addLayout(header_layout)

    def create_battery_bar(self) -> QtWidgets.QProgressBar:
        """배터리 프로그레스바 생성"""
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
        """로봇 정보 라벨들 생성"""
        model_name_label = QtWidgets.QLabel(f"모델: {self.robot_data.get('model_name', 'N/A')}")
        parent_layout.addWidget(model_name_label)

        floor_id = self.robot_data.get('floor_id', 'N/A')
        location_label = QtWidgets.QLabel(f"현재 위치: {floor_id+1}층")
        parent_layout.addWidget(location_label)

        task_id = self.robot_data.get('task_id')
        if task_id is not None:
            task_label = QtWidgets.QLabel(f"현재 업무: Task #{task_id}")
        else:
            task_label = QtWidgets.QLabel("현재 업무: 없음")
        parent_layout.addWidget(task_label)

        robot_status = self.robot_data.get('robot_status', '알 수 없음')
        if robot_status == "오류":
            error_code = self.robot_data.get('error_id', 'N/A')
            status_text = f"로봇 상태: <span style='color: red; font-weight: bold;'>오류 (오류 코드: {error_code})</span>"
        else:
            status_text = f"로봇 상태: {robot_status}"
        
        status_label = QtWidgets.QLabel(status_text)
        parent_layout.addWidget(status_label)

    def get_battery_color(self) -> str:
        """배터리 레벨에 따른 색상 반환"""
        battery_level = self.robot_data.get('battery_level', 0)
        return "#e74c3c" if battery_level <= 20 else "#2ecc71"

    def apply_styles(self):
        """스타일 적용"""
        battery_color = self.get_battery_color()
        
        battery_style = f"""
            QProgressBar {{
                border: 1px solid #bdc3c7; border-radius: 12px;
                text-align: center; background-color: #ecf0f1; color: black;
            }}
            QProgressBar::groove {{                           
                background-color: #ecf0f1; border: 1px solid #bdc3c7; border-radius: 12px;
            }}
            QProgressBar::chunk {{
                background-color: {battery_color}; border-radius: 11px;
            }}
        """
        
        self.setStyleSheet(f"""
            QFrame#robot_card {{ 
                background-color: white; border: 1px solid #dfe1e5; 
                border-radius: 8px; padding: 10px; margin-bottom: 10px;
            }}
            QLabel#card_header {{ 
                font-weight: bold; font-size: 16px; color: #2f3e4e;
            }}
            {battery_style}
        """)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(UI_FILE, self)
        
        self.setWindowTitle("ROOMIE - Admin")
        self.threadpool = QThreadPool()
        self.Page = Page  # 페이지 상수 클래스 멤버로 추가
        
        self.setup_ui_objects()
        self.apply_styles()
        self.setup_connections()
        self.setup_websocket()
        
        self.init_ui()

    def setup_ui_objects(self):
        """UI 객체 설정 및 상태바 추가"""
        self.setStatusBar(QtWidgets.QStatusBar(self))
        
        # ... 기존 findChild 코드 ...
        sidebar = self.findChild(QtWidgets.QWidget, 'widget')
        if sidebar: sidebar.setObjectName('sidebar')
        frame_rm = self.findChild(QtWidgets.QFrame, 'frame_search_rm')
        if frame_rm: frame_rm.setObjectName('frame_search_rm')
        frame_th = self.findChild(QtWidgets.QFrame, 'frame_search_th')
        if frame_th: frame_th.setObjectName('frame_search_th')
        self.widget_taskinfo.setObjectName('widget_taskinfo')
        self.widget_tasktimeline.setObjectName('widget_tasktimeline')
        frames = ['frame', 'frame_2', 'frame_3', 'frame_4']
        for frame_name in frames:
            frame = self.findChild(QtWidgets.QFrame, frame_name)
            if frame: frame.setObjectName(frame_name)

    def setup_connections(self):
        """버튼 연결 설정 (페이지 상수 사용)"""
        self.btn_db.clicked.connect(lambda: self.switch_page(self.Page.DASHBOARD))
        self.btn_rm.clicked.connect(lambda: self.switch_page(self.Page.ROBOT_MANAGEMENT))
        self.btn_th.clicked.connect(lambda: self.switch_page(self.Page.TASK_HISTORY))
        self.btn_detail.clicked.connect(self.show_task_detail)
        self.btn_back.clicked.connect(lambda: self.switch_page(self.Page.TASK_HISTORY))
        self.btn_search_rm.clicked.connect(self.search_robots)
        self.btn_search_th.clicked.connect(self.search_tasks)

    def setup_websocket(self):
        """WebSocket 클라이언트 설정"""
        self.ws_thread = QThread()
        self.ws_client = WebSocketClient(WEBSOCKET_URL)
        self.ws_client.moveToThread(self.ws_thread)
        self.ws_thread.started.connect(self.ws_client.connect)
        self.ws_client.message_received.connect(self.handle_websocket_message)
        self.ws_client.connection_status_changed.connect(self.update_connection_status)
        self.ws_thread.start()

    def init_ui(self):
        """UI 초기화"""
        self.statusBar().showMessage("서버 연결 대기 중...")
        self.switch_page(self.Page.DASHBOARD)
        self.populate_filter_combos()
        self.search_tasks()
        self.search_robots()
        
        robot_scroll_content = self.findChild(QtWidgets.QWidget, 'robotcard')
        if robot_scroll_content:
            layout = QtWidgets.QVBoxLayout(robot_scroll_content)
            layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
            robot_scroll_content.setLayout(layout)

    def populate_filter_combos(self):
        """필터 콤보박스 항목 추가"""
        self.combo_tasktype.addItems(TASK_TYPES)
        self.combo_taskstatus.addItems(TASK_STATUSES)
        self.combo_robotstatus.addItems(ROBOT_STATUSES)
        self.combo_destination.addItems(DESTINATIONS)
        self.combo_model.addItems(ROBOT_MODELS)

    def switch_page(self, index: int):
        """페이지 전환"""
        self.btn_db.setChecked(index == self.Page.DASHBOARD)
        self.btn_rm.setChecked(index == self.Page.ROBOT_MANAGEMENT)
        self.btn_th.setChecked(index == self.Page.TASK_HISTORY)
        self.stackedWidget.setCurrentIndex(index)

    def run_http_worker(self, endpoint: str, payload: dict, on_success, on_finished=None):
        """HTTP 작업 실행"""
        worker = HttpWorker(endpoint, payload)
        worker.signals.result.connect(on_success)
        worker.signals.error.connect(self.handle_http_error)
        if on_finished:
            worker.signals.finished.connect(on_finished)
        self.threadpool.start(worker)

    @pyqtSlot(str)
    def handle_http_error(self, error_msg: str):
        """HTTP 요청 실패 처리"""
        print(f"HTTP Error: {error_msg}")

    def search_robots(self):
        """로봇 검색"""
        filters = {}
        if self.lineEdit_id.text():
            try:
                filters["robot_id"] = int(self.lineEdit_id.text())
            except ValueError:
                return
        
        if self.combo_model.currentText() != "전체":
            filters["model_name"] = self.combo_model.currentText()
        if self.combo_robotstatus.currentText() != "전체":
            filters["robot_status"] = self.combo_robotstatus.currentText()
            
        payload = {"type": "request", "action": "robot_list", "payload": {"filters": filters}}
        self.run_http_worker("robot_list", payload, self.populate_robot_list)

    @pyqtSlot(dict)
    def populate_robot_list(self, data: dict):
        """로봇 리스트 표시"""
        robot_list = data.get("payload", {}).get("robots", [])
        container = self.findChild(QtWidgets.QWidget, 'robotcard')
        if not container: return
        
        self._clear_layout(container.layout())
        
        if not robot_list:
            container.layout().addWidget(QtWidgets.QLabel("조건에 맞는 로봇을 찾을 수 없습니다."))
        else:
            for robot in robot_list:
                container.layout().addWidget(RobotCard(robot))

    def search_tasks(self):
        """작업 검색"""
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
        """작업 테이블 표시"""
        tasks = data.get("payload", {}).get("tasks", [])
        self.tableWidget.setRowCount(len(tasks))
        
        headers = ["Task ID", "작업 유형", "작업 상태", "목적지", "할당 로봇 ID", "작업 생성 시각", "작업 완료 시각"]
        self.tableWidget.setColumnCount(len(headers))
        self.tableWidget.setHorizontalHeaderLabels(headers)
        
        # 행 번호 숨기기
        self.tableWidget.verticalHeader().setVisible(False)

        for row, task in enumerate(tasks):
            self.tableWidget.setItem(row, 0, QtWidgets.QTableWidgetItem(str(task.get("task_id"))))
            self.tableWidget.setItem(row, 1, QtWidgets.QTableWidgetItem(task.get("task_type", "")))
            self.tableWidget.setItem(row, 2, QtWidgets.QTableWidgetItem(task.get("task_status", "")))
            self.tableWidget.setItem(row, 3, QtWidgets.QTableWidgetItem(task.get("destination", "")))
            self.tableWidget.setItem(row, 4, QtWidgets.QTableWidgetItem(str(task.get("robot_id", ""))))
            
            # 시간 포맷 변경 (T와 Z 제거, 스페이스 추가)
            creation_time = task.get("task_creation_time", "")
            if creation_time:
                creation_time = creation_time.replace("T", " ").replace("Z", "")
            self.tableWidget.setItem(row, 5, QtWidgets.QTableWidgetItem(creation_time))
            
            completion_time = task.get("task_completion_time", "N/A")
            if completion_time and completion_time != "N/A":
                completion_time = completion_time.replace("T", " ").replace("Z", "")
            self.tableWidget.setItem(row, 6, QtWidgets.QTableWidgetItem(completion_time))
            
        self.tableWidget.resizeColumnsToContents()
        self.tableWidget.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.tableWidget.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)

    def show_task_detail(self):
        """작업 상세 정보 표시"""
        selected_rows = self.tableWidget.selectionModel().selectedRows()
        if not selected_rows:
            QtWidgets.QMessageBox.warning(self, "선택 오류", "리스트에서 작업을 선택해주세요.")
            return
        
        task_id = int(self.tableWidget.item(selected_rows[0].row(), 0).text())
        payload = {"type": "request", "action": "task_detail", "payload": {"task_id": task_id}}
        self.run_http_worker("task_detail", payload, self.populate_task_detail)

    @pyqtSlot(dict)
    def populate_task_detail(self, data: dict):
        """작업 상세 정보 표시"""
        detail = data.get("payload", {})
        
        if self.widget_taskinfo.layout() is not None:
            self._clear_layout(self.widget_taskinfo.layout())
        else:
            self.widget_taskinfo.setLayout(QtWidgets.QVBoxLayout())
            
        if self.widget_tasktimeline.layout() is not None:
            self._clear_layout(self.widget_tasktimeline.layout())
        else:
            self.widget_tasktimeline.setLayout(QtWidgets.QVBoxLayout())
        
        # 작업 정보 표시
        self.populate_task_info(detail)
        
        # 작업 타임라인 표시
        self.populate_task_timeline(detail)

        self.switch_page(self.Page.TASK_DETAIL)
    
    def populate_task_info(self, detail: dict):
        """작업 정보 섹션 채우기"""
        info_layout = self.widget_taskinfo.layout()
        
        # 선택된 행에서 task 기본 정보 가져오기
        selected_rows = self.tableWidget.selectionModel().selectedRows()
        if not selected_rows:
            return
            
        row = selected_rows[0].row()
        task_id = self.tableWidget.item(row, 0).text()
        task_type = self.tableWidget.item(row, 1).text()
        task_status = self.tableWidget.item(row, 2).text()
        destination = self.tableWidget.item(row, 3).text()
        robot_id = self.tableWidget.item(row, 4).text()
        creation_time = self.tableWidget.item(row, 5).text()
        completion_time = self.tableWidget.item(row, 6).text()
        
        # 목적지에서 "호" 제거하고 룸 번호만 추출
        room_number = destination.replace("호", "") if destination != "N/A" else "N/A"
        
        # 소요 시간 계산
        actual_duration = ""
        if completion_time != "N/A" and creation_time != "N/A":
            try:
                from datetime import datetime
                start = datetime.fromisoformat(creation_time.replace(" ", "T") + "Z")
                end = datetime.fromisoformat(completion_time.replace(" ", "T") + "Z")
                duration = end - start
                minutes = int(duration.total_seconds() // 60)
                seconds = int(duration.total_seconds() % 60)
                actual_duration = f"{minutes}분 {seconds}초"
            except:
                pass
        
        # 정보 레이블들 생성
        info_items = [
            ("Task ID", task_id),
            ("작업 종류", task_type),
            ("목적지", room_number),
            ("배정 로봇", f"R-{int(robot_id):02d}" if robot_id != "N/A" and robot_id.isdigit() else "N/A"),
            ("상태", task_status),
            ("요청 시간", creation_time),
            ("완료 시간", completion_time if completion_time != "N/A" else "진행중"),
            ("예상 소요 시간", "40분"),  # 이 값은 설정값이므로 유지
            ("실제 소요 시간", actual_duration)
        ]
        
        for label_text, value_text in info_items:
            row_layout = QtWidgets.QHBoxLayout()
            
            label = QtWidgets.QLabel(label_text)
            label.setFixedWidth(100)
            label.setStyleSheet("color: #666; font-weight: normal;")
            
            value = QtWidgets.QLabel(str(value_text))
            # 상태별 색상 설정
            if label_text == "상태":
                if task_status in ["수령 완료", "완료"]:
                    value.setStyleSheet("color: #27ae60; font-weight: bold;")
                elif "진행" in task_status or "대기" in task_status:
                    value.setStyleSheet("color: #3498db; font-weight: bold;")
                elif "픽업" in task_status:
                    value.setStyleSheet("color: #f39c12; font-weight: bold;")
                else:
                    value.setStyleSheet("color: #e74c3c; font-weight: bold;")
            else:
                value.setStyleSheet("color: #2c3e50;")
            
            row_layout.addWidget(label)
            row_layout.addWidget(value)
            row_layout.addStretch()
            
            info_layout.addLayout(row_layout)
        
        info_layout.addStretch()
    
    def populate_task_timeline(self, detail: dict):
        """작업 타임라인 표시"""
        timeline_layout = self.widget_tasktimeline.layout()
        
        # 서버에서 받은 실제 데이터 사용
        robot_assignment_time = detail.get("robot_assignment_time")
        pickup_completion_time = detail.get("pickup_completion_time") 
        delivery_arrival_time = detail.get("delivery_arrival_time")
        task_completion_time = detail.get("task_completion_time")
        
        # 선택된 행에서 기본 정보 가져오기
        selected_rows = self.tableWidget.selectionModel().selectedRows()
        creation_time = None
        if selected_rows:
            row = selected_rows[0].row()
            creation_time = self.tableWidget.item(row, 5).text()
            # 다시 ISO 형식으로 변환
            if creation_time != "N/A":
                creation_time = creation_time.replace(" ", "T") + "Z"
        
        # 타임라인 데이터 (서버 데이터 기반)
        timeline_events = [
            {
                "time": creation_time,
                "title": "작업 요청 접수",
                "completed": bool(creation_time)
            },
            {
                "time": robot_assignment_time,
                "title": "로봇 할당 및 출발",
                "completed": bool(robot_assignment_time)
            },
            {
                "time": pickup_completion_time,
                "title": "픽업 완료",
                "completed": bool(pickup_completion_time)
            },
            {
                "time": delivery_arrival_time,
                "title": "목적지 도착",
                "completed": bool(delivery_arrival_time)
            },
            {
                "time": task_completion_time,
                "title": "배송 완료",
                "completed": bool(task_completion_time)
            }
        ]
        
        for i, event in enumerate(timeline_events):
            is_first = (i == 0)
            is_last = (i == len(timeline_events) - 1)
            event_widget = self.create_timeline_event(event, is_first, is_last)
            timeline_layout.addWidget(event_widget)
        
        timeline_layout.addStretch()
    
    def create_timeline_event(self, event: dict, is_first: bool, is_last: bool):
        """타임라인 이벤트 위젯 생성 (AttributeError 수정)"""
        container = QtWidgets.QWidget()
        
        grid_layout = QtWidgets.QGridLayout(container)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        grid_layout.setVerticalSpacing(4) 
        
        def create_line(completed):
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.Shape.VLine)
            line.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
            line.setLineWidth(2)
            color = "#27ae60" if completed else "#bdc3c7"
            line.setStyleSheet(f"color: {color};")
            return line

        top_line = create_line(event["completed"])
        bottom_line = create_line(event["completed"])
        top_line.setVisible(not is_first)
        bottom_line.setVisible(not is_last)

        circle = QtWidgets.QLabel("●")
        circle_color = "#27ae60" if event["completed"] else "#bdc3c7"
        circle.setStyleSheet(f"background-color: transparent; font-size: 14px; color: {circle_color};")
        
        time_label = QtWidgets.QLabel()
        if event["time"]:
            time_formatted = event["time"].replace("T", " ").replace("Z", "")
            time_parts = time_formatted.split(" ")
            if len(time_parts) >= 2:
                time_label.setText(time_parts[1][:5])
        time_label.setStyleSheet("color: #27ae60; font-size: 13px; font-weight: bold;")
        time_label.setFixedHeight(circle.sizeHint().height())
        time_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignVCenter)

        title_label = QtWidgets.QLabel(event["title"])
        title_label.setStyleSheet(f"color: {'#2c3e50' if event['completed'] else '#7f8c8d'}; font-size: 15px;")

        # grid_layout.addWidget(top_line, 0, 0, QtCore.Qt.AlignmentFlag.AlignHCenter)
        grid_layout.addWidget(circle, 1, 0, QtCore.Qt.AlignmentFlag.AlignHCenter)
        grid_layout.addWidget(bottom_line, 2, 0, 2, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        
        grid_layout.addWidget(time_label, 1, 1) 
        grid_layout.addWidget(title_label, 2, 1)

        grid_layout.setColumnMinimumWidth(0, 20)
        grid_layout.setColumnStretch(1, 1)
        
        grid_layout.setRowStretch(3, 1)
        # grid_layout.setRowStretch(3, 1)

        return container
        
    def _clear_layout(self, layout: QtWidgets.QLayout):
        """레이아웃 내의 모든 위젯을 제거하는 헬퍼 함수"""
        if layout is None: 
            return
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
            elif item.layout():
                self._clear_layout(item.layout())
                item.layout().deleteLater()

    @pyqtSlot(dict)
    def handle_websocket_message(self, data: dict):
        """WebSocket 메시지 처리"""
        action = data.get("action")
        payload = data.get("payload", {})
        
        if action == "task_status_update":
            self.label_totaltask.setText(str(payload.get("total_task_count", "N/A")))
            self.label_waitingtask.setText(str(payload.get("waiting_task_count", "N/A")))
            if self.stackedWidget.currentIndex() == self.Page.TASK_HISTORY:
                self.search_tasks()

        elif action == "robot_status_update":
            self.label_totalrobot.setText(str(payload.get("total_robot_count", "N/A")))
            self.label_activerobot.setText(str(payload.get("active_robot_count", "N/A")))
            if self.stackedWidget.currentIndex() == self.Page.ROBOT_MANAGEMENT:
                self.search_robots()

    @pyqtSlot(bool)
    def update_connection_status(self, connected: bool):
        """연결 상태에 따라 상태바 업데이트"""
        if connected:
            self.statusBar().showMessage("서버 연결됨")
        else:
            self.statusBar().showMessage("서버 연결 끊김 - 재연결 시도 중...")

    def apply_styles(self):
        """스타일시트 적용"""
        self.setStyleSheet(APP_STYLE)

    def closeEvent(self, event: QCloseEvent):
        """앱 종료 시 리소스 정리"""
        if hasattr(self, 'ws_client'): self.ws_client.disconnect()
        if hasattr(self, 'ws_thread'):
            self.ws_thread.quit()
            self.ws_thread.wait()
        super().closeEvent(event)

def main():
    """메인 함수"""
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("ROOMIE Admin")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()