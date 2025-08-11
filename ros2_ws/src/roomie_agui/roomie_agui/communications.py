import json
import requests
import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from roomie_msgs.msg import RoomiePose
from PyQt6.QtCore import QObject, QRunnable, pyqtSignal, pyqtSlot, QTimer, QUrl, QThread
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


# --- 강화된 WebSocket 통신 ---
class WebSocketClient(QObject):
    """
    서버와 WebSocket 통신을 관리하는 클라이언트
    강화된 재연결 로직 포함
    """
    message_received = pyqtSignal(dict)
    raw_message_for_debug = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)  # 연결 상태 변경 시그널
    connection_attempt_failed = pyqtSignal(int, str)  # 연결 시도 실패 시그널 (시도 횟수, 오류 메시지)

    def __init__(self, url: str):
        super().__init__()
        self.client: QWebSocket = None
        self.url = url
        
        # 연결 상태 관리
        self.is_connected = False
        self.is_connecting = False
        self.connection_attempts = 0
        self.max_reconnect_attempts = WEBSOCKET_MAX_RECONNECT_ATTEMPTS
        self.reconnect_interval = 5000  # 5초마다 재연결 시도
        self.should_reconnect = True  # 재연결 플래그
        
        # 재연결 관련 변수들 - moveToThread 후에 설정
        self.reconnect_timer = None

    @pyqtSlot()
    def connect(self):
        """WebSocket 연결 시도"""
        if self.is_connecting or self.is_connected:
            logger.debug("이미 연결 중이거나 연결된 상태입니다.")
            return
            
        # QTimer 초기화 (스레드에서 실행되므로 여기서 생성)
        if self.reconnect_timer is None:
            self.reconnect_timer = QTimer()
            self.reconnect_timer.timeout.connect(self._attempt_reconnect)
            self.reconnect_timer.setSingleShot(True)
            
        self.is_connecting = True
        self._create_websocket()
        logger.info(f"WebSocket 연결 시도: {self.url}")
        self.client.open(QUrl(self.url))

    def _create_websocket(self):
        """WebSocket 객체 생성 및 시그널 연결"""
        if self.client:
            self.client.deleteLater()
            
        self.client = QWebSocket()
        self.client.textMessageReceived.connect(self.on_message_received)
        self.client.connected.connect(self.on_connected)
        self.client.disconnected.connect(self.on_disconnected)
        self.client.errorOccurred.connect(self.on_error_occurred)

    def on_connected(self):        
        """연결 성공 시 호출"""
        logger.info("WebSocket 연결 성공")
        self.is_connected = True
        self.is_connecting = False
        self.connection_attempts = 0
        self.should_reconnect = True  # 재연결 플래그 리셋
        
        self.raw_message_for_debug.emit("Connected")
        self.connection_status_changed.emit(True)
        if self.reconnect_timer:
            self.reconnect_timer.stop()

    def on_disconnected(self):
        """연결 끊김 시 호출"""
        logger.warning("WebSocket 연결 끊김")
        self.is_connected = False
        self.is_connecting = False
        
        self.raw_message_for_debug.emit("Disconnected. Attempting to reconnect...")
        self.connection_status_changed.emit(False)
        
        # 재연결 시도
        self._schedule_reconnect()

    def on_error_occurred(self, error):
        """WebSocket 오류 발생 시 호출"""
        error_msg = f"WebSocket 오류: {error}"
        logger.error(error_msg)
        self.connection_attempt_failed.emit(self.connection_attempts, error_msg)
        
        # 연결 시도 중이었다면 재연결 스케줄링
        if self.is_connecting:
            self.is_connecting = False
            self._schedule_reconnect()

    def _schedule_reconnect(self):
        """재연결 스케줄링"""
        if not self.should_reconnect:
            logger.info("재연결이 비활성화되어 있습니다.")
            return
            
        if self.connection_attempts >= self.max_reconnect_attempts:
            logger.error(f"최대 재연결 시도 횟수({self.max_reconnect_attempts})에 도달했습니다.")
            self.should_reconnect = False
            self.connection_status_changed.emit(False)
            return
            
        self.connection_attempts += 1
        
        logger.info(f"재연결 시도 {self.connection_attempts}/{self.max_reconnect_attempts} - 5초 후 시도")
        if self.reconnect_timer:
            self.reconnect_timer.start(self.reconnect_interval)

    @pyqtSlot()
    def _attempt_reconnect(self):
        """실제 재연결 시도"""
        if not self.is_connected and not self.is_connecting:
            logger.info(f"재연결 시도 {self.connection_attempts}/{self.max_reconnect_attempts}")
            self.connect()



    def on_message_received(self, message: str):
        """메시지 수신 시 호출"""
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
        self.is_connected = False
        self.is_connecting = False
        self.connection_attempts = 0
        self.should_reconnect = False  # 재연결 중지
        
        # 타이머 정지
        if self.reconnect_timer:
            self.reconnect_timer.stop()
        
        if self.client:
            self.client.close()
            self.client.deleteLater()
            self.client = None

    def get_connection_status(self) -> dict:
        """현재 연결 상태 정보 반환"""
        return {
            "is_connected": self.is_connected,
            "is_connecting": self.is_connecting,
            "connection_attempts": self.connection_attempts,
            "max_attempts": self.max_reconnect_attempts,
            "should_reconnect": self.should_reconnect
        }


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