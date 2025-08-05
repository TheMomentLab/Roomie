import json
import requests
from PyQt6.QtCore import QObject, QRunnable, pyqtSignal, pyqtSlot, QTimer, QUrl
from PyQt6.QtWebSockets import QWebSocket
from config import *


# --- HTTP 통신 ---
class WorkerSignals(QObject):
    """Worker의 시그널 정의"""
    finished = pyqtSignal()
    error = pyqtSignal(str)
    result = pyqtSignal(dict)


class HttpWorker(QRunnable):
    """
    백그라운드에서 HTTP 요청을 처리하는 Worker
    """
    def __init__(self, endpoint: str, payload: dict):
        super().__init__()
        self.signals = WorkerSignals()
        self.endpoint = endpoint
        self.payload = payload

    @pyqtSlot()
    def run(self):
        try:
            # API_URL과 엔드포인트(e.g., /task_list)를 조합
            url = f"{API_URL}/{self.endpoint}"
            response = requests.post(url, json=self.payload, timeout=5)
            response.raise_for_status()
            self.signals.result.emit(response.json())
        except requests.exceptions.RequestException as e:
            self.signals.error.emit(str(e))
        finally:
            self.signals.finished.emit()


# --- WebSocket 통신 ---
class WebSocketClient(QObject):
    """
    서버와 WebSocket 통신을 관리하는 클라이언트
    """
    message_received = pyqtSignal(dict)
    raw_message_for_debug = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)  # 연결 상태 변경 시그널

    def __init__(self, url: str):
        super().__init__()
        self.client: QWebSocket = None
        self.url = url
        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.connect)
        self.reconnect_timer.setSingleShot(True)

    @pyqtSlot()
    def connect(self):
        if self.client is None:
            self.client = QWebSocket()
            self.client.textMessageReceived.connect(self.on_message_received)
            self.client.connected.connect(self.on_connected)
            self.client.disconnected.connect(self.on_disconnected)

        print(f"Connecting to WebSocket at {self.url}...")
        # QUrl을 직접 사용하도록 수정
        self.client.open(QUrl(self.url))

    def on_connected(self):        
        print("WebSocket connected successfully.")
        self.raw_message_for_debug.emit("Connected")
        self.connection_status_changed.emit(True)
        self.reconnect_timer.stop()

    def on_disconnected(self):
        print("WebSocket disconnected.")
        self.raw_message_for_debug.emit("Disconnected. Attempting to reconnect...")
        self.connection_status_changed.emit(False)
        # config 파일에 설정된 시간 후 재연결 시도
        self.reconnect_timer.start(WEBSOCKET_RECONNECT_INTERVAL)

    def on_message_received(self, message: str):
        self.raw_message_for_debug.emit(message)
        try:
            data = json.loads(message)
            if data.get("type") == "event":
                self.message_received.emit(data)
        except json.JSONDecodeError:
            self.raw_message_for_debug.emit(f"JSON-ERROR: {message}")

    def disconnect(self):
        """연결 종료"""
        self.reconnect_timer.stop()
        if self.client:
            self.client.close()


# --- ROS2 통신 (추후 구현) ---
class ROS2Manager(QObject):
    """
    ROS2 노드와 통신을 관리하는 클래스 (추후 구현)
    """
    
    # 시그널 정의 (추후 구현 시 사용)
    map_received = pyqtSignal(object)  # 지도 데이터 수신
    robot_position_updated = pyqtSignal(str, float, float, float)  # robot_id, x, y, orientation
    ros2_status_changed = pyqtSignal(bool)  # ROS2 연결 상태
    
    def __init__(self):
        super().__init__()
        # 추후 구현
        pass
    
    def initialize_ros2(self):
        """ROS2 초기화 (추후 구현)"""
        pass
    
    def shutdown(self):
        """ROS2 종료 (추후 구현)"""
        pass