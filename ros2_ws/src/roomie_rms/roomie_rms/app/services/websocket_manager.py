"""
WebSocket 연결 관리자 모듈

클라이언트 연결 관리, 메시지 전송, 브로드캐스팅 기능을 제공합니다.
"""

from fastapi import WebSocket, WebSocketDisconnect, APIRouter
from typing import Dict, List
import json
import asyncio

from app.utils.logger import get_logger
from app.services import db_manager
from app.utils.error_handler import safe_database_connection
from app.schemas.gui_models import *

logger = get_logger(__name__)

class WebSocketManager:
    """WebSocket 연결을 관리하는 클래스"""
    
    def __init__(self):
        # 연결된 클라이언트들을 저장 {client_type: {client_id: websocket}}
        self.connections: Dict[str, Dict[str, WebSocket]] = {
            "guest": {},
            "staff": {},
            "admin": {}
        }
        self.router = APIRouter()
        self._setup_routes()

        self.loop: asyncio.AbstractEventLoop = None

    def set_rms_node(self, rms_node_instance):
        """RMS 노드 인스턴스를 설정합니다."""
        self.rms_node_instance = rms_node_instance
        logger.info("WebSocketManager에 RMS 노드 인스턴스가 성공적으로 설정되었습니다.")

    def _setup_routes(self):
        """WebSocket 라우터 설정"""
        
        @self.router.websocket("/ws/guest/{client_id}")
        async def guest_websocket_endpoint(websocket: WebSocket, client_id: str):
            await self.connect(websocket, "guest", client_id)
            try:
                while True:
                    data = await websocket.receive_text()
                    logger.debug(f"게스트 메시지 수신: {client_id} | {data}")
            except WebSocketDisconnect:
                pass
            finally:
                self.disconnect("guest", client_id)

        @self.router.websocket("/ws/staff/{client_id}")
        async def staff_websocket_endpoint(websocket: WebSocket, client_id: str):
            await self.connect(websocket, "staff", client_id)
            try:
                while True:
                    data = await websocket.receive_text()
                    logger.debug(f"직원 메시지 수신: {client_id} | {data}")
            except WebSocketDisconnect:
                pass
            finally:
                self.disconnect("staff", client_id)

        @self.router.websocket("/ws/admin/{client_id}")
        async def admin_websocket_endpoint(websocket: WebSocket, client_id: str):
            await self.connect(websocket, "admin", client_id)
            try:
                while True:
                    data = await websocket.receive_text()
                    logger.debug(f"관리자 메시지 수신: {client_id} | {data}")
            except WebSocketDisconnect:
                pass
            finally:
                self.disconnect("admin", client_id)

    async def _get_robot_status_counts(self) -> dict:
        try:
            with safe_database_connection(db_manager.get_connection) as conn:
                with conn.cursor(dictionary=True) as cursor:
                    cursor.execute("SELECT COUNT(*) as total FROM robot")
                    total_robots = cursor.fetchone()['total']
                    query = "SELECT COUNT(*) as active FROM robot_current_state WHERE robot_status_id NOT IN (1, 2, 90);"
                    cursor.execute(query)
                    active_robots = cursor.fetchone()['active']
                    return {"total_robot_count": total_robots, "active_robot_count": active_robots}
        except Exception as e:
            logger.error(f"로봇 상태 카운트 조회 실패: {e}")
            return None
            
    async def _get_task_status_counts(self) -> dict:
        try:
            with safe_database_connection(db_manager.get_connection) as conn:
                with conn.cursor(dictionary=True) as cursor:
                    cursor.execute("SELECT COUNT(*) as total FROM task")
                    total_tasks = cursor.fetchone()['total']
                    query = "SELECT COUNT(*) as waiting FROM task t JOIN task_status ts ON t.task_status_id = ts.id WHERE ts.id IN (0, 1, 2, 3, 4)"
                    cursor.execute(query)
                    waiting_tasks = cursor.fetchone()['waiting']
                    return {"total_task_count": total_tasks, "waiting_task_count": waiting_tasks}
        except Exception as e:
            logger.error(f"작업 상태 카운트 조회 실패: {e}")
            return None
            
    async def broadcast_status_updates(self):
        try:
            robot_counts = await self._get_robot_status_counts()
            if robot_counts:
                payload = RobotStatusUpdateEventPayload(**robot_counts)
                event = RobotStatusUpdateEvent(payload=payload)
                json_payload_to_send = event.model_dump_json()

                logger.info(
                    "Admin GUI로 'robot_status_update' 이벤트를 전송합니다.",
                    category="WEBSOCKET",
                    subcategory="BROADCAST",
                    details={"Payload": json_payload_to_send}
                )
                await self.broadcast_to("admin", json_payload_to_send)

            task_counts = await self._get_task_status_counts()
            if task_counts:
                payload = TaskStatusUpdateEventPayload(**task_counts)
                event = TaskStatusUpdateEvent(payload=payload)
                json_payload_to_send = event.model_dump_json()

                logger.info(
                    "Admin GUI로 'task_status_update' 이벤트를 전송합니다.",
                    category="WEBSOCKET",
                    subcategory="BROADCAST",
                    details={"Payload": json_payload_to_send}
                )
                await self.broadcast_to("admin", json_payload_to_send)

        except Exception as e:
            logger.error(f"상태 업데이트 브로드캐스트 실패: {e}")

    def trigger_broadcast_sync(self):
        """다른 스레드에서 이 함수를 호출하여 안전하게 브로드캐스트를 시작합니다."""
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.broadcast_status_updates(), self.loop)
        else:
            logger.warning("Uvicorn 이벤트 루프가 아직 설정되지 않아 브로드캐스트를 스킵합니다.")

    async def connect(self, websocket: WebSocket, client_type: str, client_id: str):
        await websocket.accept()
        
        if self.loop is None:
            self.loop = asyncio.get_running_loop()

        self.connections[client_type][client_id] = websocket
        
        if client_type == "admin":
            # get_instance() 대신 self.rms_node_instance 사용
            if self.rms_node_instance:
                logger.info("Admin 클라이언트가 연결되어 상태 브로드캐스트 시작을 시도합니다.")
                self.rms_node_instance.start_status_broadcast()
            else:
                logger.error("RMS 노드 인스턴스가 설정되지 않아 브로드캐스트를 시작할 수 없습니다.")

        connected_count = len(self.connections[client_type])
        logger.info(f"WebSocket 연결됨 | 타입: {client_type.upper()} | ID: {client_id} | 현재 연결 수: {connected_count}")

    def disconnect(self, client_type: str, client_id: str):
        if client_id not in self.connections[client_type]:
            return
        
        del self.connections[client_type][client_id]

        if client_type == "admin" and not self.connections["admin"]:
            # get_instance() 대신 self.rms_node_instance 사용
            if self.rms_node_instance:
                logger.info("마지막 admin 클라이언트의 연결이 끊어져 상태 브로드캐스트를 중지합니다.")
                self.rms_node_instance.stop_status_broadcast()

        remaining_count = len(self.connections[client_type])
        logger.info(f"WebSocket 연결 해제됨 | 타입: {client_type.upper()} | ID: {client_id} | 남은 {client_type} 연결 수: {remaining_count}")
    
        
    async def broadcast_to(self, client_type: str, message: str):
        """특정 타입의 모든 클라이언트에게 브로드캐스트합니다."""
        if client_type in self.connections and self.connections[client_type]:
            for client_id, connection in list(self.connections[client_type].items()):
                try:
                    await connection.send_text(message)
                    logger.debug(f"브로드캐스트 성공: {client_type}({client_id})")
                except Exception:
                    self.disconnect(client_type, client_id)  

    def broadcast_to_sync(self, client_type: str, message: str):
        if self.rms_node_instance and self.rms_node_instance.get_loop():
            asyncio.run_coroutine_threadsafe(self.broadcast_to(client_type, message), self.rms_node_instance.get_loop())
    
    def send_to_client_by_location_sync(self, client_type: str, location_name: str, message: str):
        if self.rms_node_instance and self.rms_node_instance.get_loop():
            asyncio.run_coroutine_threadsafe(self.send_to_client(client_type, location_name, message), self.rms_node_instance.get_loop())


# 글로벌 인스턴스 생성
manager = WebSocketManager()