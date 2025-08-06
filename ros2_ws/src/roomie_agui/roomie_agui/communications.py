import json
import requests
import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from roomie_msgs.msg import RoomiePose
from PyQt6.QtCore import QObject, QRunnable, pyqtSignal, pyqtSlot, QTimer, QUrl
from PyQt6.QtWebSockets import QWebSocket
from config import *

# 로거 설정 - 메인 로거 사용
logger = logging.getLogger('roomie_agui.communications')


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
            logger.error(f"HTTP 요청 실패: {self.endpoint} - {e}")
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

        logger.info(f"WebSocket 연결 시도: {self.url}")
        # QUrl을 직접 사용하도록 수정
        self.client.open(QUrl(self.url))

    def on_connected(self):        
        logger.info("WebSocket 연결 성공")
        self.raw_message_for_debug.emit("Connected")
        self.connection_status_changed.emit(True)
        self.reconnect_timer.stop()

    def on_disconnected(self):
        logger.warning("WebSocket 연결 끊김")
        self.raw_message_for_debug.emit("Disconnected. Attempting to reconnect...")
        self.connection_status_changed.emit(False)
        # config 파일에 설정된 시간 후 재연결 시도
        self.reconnect_timer.start(WEBSOCKET_RECONNECT_INTERVAL)

    def on_message_received(self, message: str):
        logger.debug(f"WebSocket 메시지 수신: {message[:100]}...")  # 메시지가 길 수 있으므로 앞부분만 로그
        self.raw_message_for_debug.emit(message)
        try:
            data = json.loads(message)
            if data.get("type") == "event":
                logger.info(f"WebSocket 이벤트 수신: {data.get('action', 'unknown')}")
                self.message_received.emit(data)
        except json.JSONDecodeError:
            logger.error(f"WebSocket JSON 파싱 오류: {message}")
            self.raw_message_for_debug.emit(f"JSON-ERROR: {message}")

    def disconnect(self):
        """연결 종료"""
        logger.info("WebSocket 연결 종료")
        self.reconnect_timer.stop()
        if self.client:
            self.client.close()


# --- ROS2 토픽 핸들러 ---
class ROS2TopicHandler(QObject):
    """ROS2 토픽 콜백 함수들을 관리하는 클래스"""
    
    # 시그널 정의
    robot_pose_updated = pyqtSignal(int, int, object)  # robot_id, floor_id, pose
    
    def __init__(self, main_window=None):
        super().__init__()
        self.main_window = main_window
        self.node = None
        
    def setup_node(self, node):
        """ROS2 노드 설정"""
        self.node = node
        
    def roomie_pose_callback(self, msg):
        """로봇의 현재 위치(Pose) 정보를 처리합니다."""
        logger.info(f"로봇 위치 수신 - Robot ID: {msg.robot_id}, Floor: {msg.floor_id}, Position: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        
        # 시그널을 통해 메인 윈도우에 로봇 위치 정보 전달
        self.robot_pose_updated.emit(msg.robot_id, msg.floor_id, msg.pose)