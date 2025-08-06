"""
Roomie RMS 통합 노드

ROS2 노드와 FastAPI 서버를 통합하여 운영하는 메인 모듈입니다.
"""
import asyncio
import threading
import time
import uvicorn
import rclpy
from rclpy.executors import MultiThreadedExecutor
from fastapi import FastAPI

from app.utils.logger import get_logger, log_node_startup, log_node_shutdown
from app.services import http_manager, websocket_manager, db_manager
from ros_core.base_node import RmsBaseNode
from ros_core.business.robot_manager import RobotManager
from ros_core.business.task_manager import TaskManager
from ros_core.handlers.action_handler import ActionHandler
from ros_core.handlers.service_handler import ServiceHandler
from ros_core.handlers.topic_handler import TopicHandler

logger = get_logger(__name__)

class RmsNode(RmsBaseNode, ServiceHandler, TopicHandler, ActionHandler):
    
    def __init__(self) -> None:
        if hasattr(self, '_initialized'):
            return
        super().__init__('rms_node')
        self._initialized = True
        
        self.get_logger().info = logger.info
        self.get_logger().error = logger.error
        self.get_logger().warning = logger.warning
        self.get_logger().debug = logger.debug
        
        self.robot_manager: RobotManager = RobotManager(self)
        self.task_manager: TaskManager = TaskManager(self.robot_manager, self)
        
        ServiceHandler.__init__(self, robot_manager=self.robot_manager, task_manager=self.task_manager)
        TopicHandler.__init__(self, robot_manager=self.robot_manager, task_manager=self.task_manager, node=self)
        ActionHandler.__init__(self, robot_manager=self.robot_manager, task_manager=self.task_manager, db_manager=db_manager, node=self)
        
        try:
            self.loop: asyncio.AbstractEventLoop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

        # 주기적인 작업을 위한 표준 threading.Thread 사용
        self._broadcast_thread: Optional[threading.Thread] = None
        self._broadcast_stop_event = threading.Event()

        websocket_manager.manager.set_rms_node(self)
        
        logger.info("RMS 통합 노드 초기화 완료", category="SYSTEM", subcategory="INIT")

    def get_loop(self) -> asyncio.AbstractEventLoop:
        return self.loop
    
    def _trigger_status_broadcast_periodically(self, stop_event: threading.Event, interval_seconds: int = 5):
        """주기적으로 websocket_manager의 동기 트리거 함수를 호출하는 콜백 함수"""
        while not stop_event.is_set():
            try:
                websocket_manager.manager.trigger_broadcast_sync()
            except Exception as e:
                logger.error(f"상태 업데이트 트리거 실패: {e}")
            # interval 만큼 대기. stop_event.wait()를 사용하면 즉시 종료 가능
            stop_event.wait(interval_seconds)

    def start_status_broadcast(self):
        """상태 업데이트 브로드캐스트 스레드를 시작합니다."""
        if self._broadcast_thread and self._broadcast_thread.is_alive():
            return
        
        self._broadcast_stop_event.clear()
        self._broadcast_thread = threading.Thread(
            target=self._trigger_status_broadcast_periodically,
            args=(self._broadcast_stop_event, 10),
            daemon=True
        )
        self._broadcast_thread.start()

    def stop_status_broadcast(self):
        """상태 업데이트 브로드캐스트 스레드를 중지합니다."""
        if not (self._broadcast_thread and self._broadcast_thread.is_alive()):
            return
        
        self._broadcast_stop_event.set()
        self._broadcast_thread.join(timeout=1) # 스레드가 종료될 때까지 잠시 대기
        self._broadcast_thread = None
    
    # --- TopicHandler 콜백 메서드 오버라이드 ---
    def robot_state_callback(self, msg):
        """로봇 상태 콜백"""
        TopicHandler.robot_state_callback(self, msg)
    
    def task_state_callback(self, msg):
        """작업 상태 콜백"""
        TopicHandler.task_state_callback(self, msg)
    
    def arrival_callback(self, msg):
        """도착 이벤트 콜백"""
        TopicHandler.arrival_callback(self, msg)
    
    def battery_status_callback(self, msg):
        """배터리 상태 콜백"""
        TopicHandler.battery_status_callback(self, msg)
    
    def roomie_pose_callback(self, msg):
        """로봇 포즈 콜백"""
        TopicHandler.roomie_pose_callback(self, msg)
    
    def pickup_completed_callback(self, msg):
        """픽업 완료 콜백"""
        TopicHandler.pickup_completed_callback(self, msg)
    
    def delivery_completed_callback(self, msg):
        """배송 완료 콜백"""
        TopicHandler.delivery_completed_callback(self, msg)
    
    # --- ServiceHandler 콜백 메서드 오버라이드 ---
    def get_locations_callback(self, request, response):
        """위치 조회 서비스 콜백"""
        return ServiceHandler.get_locations_callback(self, request, response)

def create_app(rms_node: RmsNode) -> FastAPI:
    app = FastAPI(title="Roomie RMS API Server")
    from fastapi.middleware.cors import CORSMiddleware
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])
    app.state.rms_node = rms_node
    app.include_router(http_manager.manager.router, prefix="/api/gui")
    app.include_router(websocket_manager.manager.router, prefix="/api/gui")
    @app.get("/")
    async def root(): return {"message": "Roomie RMS API Server is running"}
    @app.get("/health")
    async def health(): return {"status": "healthy", "rms_node": "connected"}
    return app

def main():
    log_node_startup()
    from app.config import settings
    rclpy.init()
    try:
        rms_node = RmsNode()
        app = create_app(rms_node)
        fastapi_thread = threading.Thread(target=lambda: uvicorn.run(app, host=settings.FASTAPI_HOST, port=settings.FASTAPI_PORT, log_level="info"))
        fastapi_thread.daemon = True
        fastapi_thread.start()
        logger.info("RMS 노드 실행 시작", category="SYSTEM", subcategory="RUN")
        executor: MultiThreadedExecutor = MultiThreadedExecutor()
        executor.add_node(rms_node)
        try:
            executor.spin()
        finally:
            logger.info("RMS 노드 종료 중...", category="SYSTEM", subcategory="SHUTDOWN")
            rms_node.stop_status_broadcast()
            executor.shutdown()
            rms_node.destroy_node()
    except KeyboardInterrupt:
        logger.warning("사용자에 의해 RMS 노드가 중단되었습니다.", category="SYSTEM", subcategory="INTERRUPT")
    except Exception as e:
        logger.critical(f"RMS 노드 실행 중 치명적 오류 발생: {e}", category="SYSTEM", subcategory="CRITICAL")
        raise
    finally:
        log_node_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()