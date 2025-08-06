from datetime import datetime

from app.utils.logger import get_logger, log_database_operation, log_websocket_event
from app.utils.error_handler import handle_database_errors, database_transaction, safe_database_connection
from app.utils.exceptions import DatabaseException
from app.config import settings
from app.services import db_manager
from app.services.websocket_manager import manager as websocket_manager

logger = get_logger(__name__)

class TopicHandler:
    """ROS2 Topic 콜백 함수들을 관리하는 클래스"""
    def __init__(self, robot_manager, task_manager, node=None):
        self.robot_manager = robot_manager
        self.task_manager = task_manager
        self.node = node
        self.task_state_pub = node.task_state_pub if node else None

    #------- 상태 다이어그램이 수정됨에 따라 RC에게 작업 상태를 전송하지 않음 -------
    # def publish_task_state(self, task_id, status_id):
    #     """RC에게 작업의 현재 상태를 알립니다."""
    #     if not self.task_state_pub:
    #         logger.warning(f"[ROS2] task_state_pub이 초기화되지 않음. Task ID {task_id} 상태 전송 실패")
    #         return
            
    #     from roomie_msgs.msg import TaskState
    #     msg = TaskState()
    #     msg.task_id = task_id
    #     msg.task_state_id = status_id
    #     self.task_state_pub.publish(msg)
    #     logger.info(
    #         "RC로 작업 상태 전송",
    #         category="ROS2", subcategory="TOPIC-PUB",
    #         details={"Topic": "/roomie/status/task_state", "TaskID": task_id, "StatusID": status_id}
    #     )

    @handle_database_errors
    def robot_state_callback(self, msg):
        """RC로부터 로봇 상태 정보를 받아 DB에 업데이트하고, AGUI/SGUI에 알림 및 로그를 기록합니다."""
        logger.info(
            "로봇 상태 수신",
            category="ROS2", subcategory="TOPIC-SUB",
            details={"Topic": "/roomie/status/robot_state", "RobotID": msg.robot_id, "StateID": msg.robot_state_id}
        )

        try:
            with safe_database_connection(db_manager.get_connection) as conn:
                with database_transaction(conn) as cursor:
                    # 1. robot_current_state 에 현재 상태 업데이트
                    self.robot_manager.update_robot_current_state(cursor, msg.robot_id, status_id=msg.robot_state_id)
            
                    # 2. robot_log 에 현재 상태 기록
                    cursor.execute("SELECT battery_level, is_charging, floor_id, error_id FROM robot_current_state WHERE robot_id = %s", (msg.robot_id,))
                    current_snapshot = cursor.fetchone()
                    
                    self.robot_manager.add_robot_log(
                        cursor,
                        robot_id=msg.robot_id,
                        status_id=msg.robot_state_id,
                        battery_level=current_snapshot['battery_level'],
                        is_charging=current_snapshot['is_charging'],
                        floor_id=current_snapshot['floor_id'],
                        error_id=current_snapshot['error_id']
                    )


            # 복귀 대기 중 상태일 때 복귀 로직 실행
            if msg.robot_state_id == settings.db_consts.robot_status.get('복귀 대기 중'):
                logger.info(f"로봇 {msg.robot_id} 복귀 여부 판단 시작...")
                self.robot_manager.decide_and_execute_return(msg.robot_id)
            elif msg.robot_state_id not in settings.db_consts.robot_status.values():
                logger.warning(
                    f"알 수 없는 로봇 상태 ID: {msg.robot_state_id}",
                    category="ROS2", subcategory="WARNING",
                    details={"RobotID": msg.robot_id, "StateID": msg.robot_state_id}
                )
            
        except (DatabaseException, Exception) as e:
            logger.error(
                f"로봇 상태({msg.robot_id}) 업데이트 중 오류: {e}",
                category="ROS2", subcategory="TOPIC-ERROR"
            )

    def task_state_callback(self, msg):
        """RC로부터 받은 작업 상태를 로깅합니다 (참고용)."""
        logger.info(
            "작업 상태 수신 (RC -> RMS)",
            category="ROS2", subcategory="TOPIC-SUB",
            details={"Topic": "/roomie/status/task_state", "TaskID": msg.task_id, "StateID": msg.task_state_id}
        )

    @handle_database_errors
    def arrival_callback(self, msg):
        """로봇 도착 이벤트를 처리합니다."""
        task_id = msg.task_id
        arrival_location_id = msg.location_id
        logger.info(
            "로봇 도착 이벤트 수신",
            category="ROS2", subcategory="TOPIC-SUB",
            details={"Topic": "/roomie/event/arrival", "TaskID": task_id, "LocationID": arrival_location_id}
        )

        try:
            with safe_database_connection(db_manager.get_connection) as conn:
                with database_transaction(conn) as cursor:
                    # 작업 정보 조회
                    cursor.execute("""
                        SELECT t.robot_id, t.location_id, tt.name as task_type_name 
                        FROM task t
                        JOIN task_type tt ON t.task_type_id = tt.id
                        WHERE t.id = %s
                    """, (task_id,))
                    task_info = cursor.fetchone()
                    
                    if not task_info:
                        logger.error(
                            f"작업 정보를 찾을 수 없음: Task ID {task_id}",
                            category="TASK", subcategory="ERROR",
                            details={"TaskID": task_id, "Event": "Arrival"}
                        )
                        return

                    robot_id = task_info['robot_id']
                    destination_id = task_info['location_id']
                    
                    # 픽업 장소 도착
                    if arrival_location_id != destination_id:
                        logger.info(
                            "픽업 장소 도착",
                            category="TASK", subcategory="STATUS-UPDATE",
                            details={"RobotID": robot_id, "TaskID": task_id, "LocationID": arrival_location_id}
                        )
                        
                        # DB 상태 업데이트
                        cursor.execute("UPDATE task SET task_status_id = %s WHERE id = %s",
                                    (settings.db_consts.task_status['픽업 대기 중'], task_id))
                        log_database_operation("UPDATE", "task", True, f"Task {task_id} 상태 '픽업 대기 중'으로 변경")
                        
                        # SGUI에 픽업 도착 알림 전송
                        action_type = ""
                        if task_info['task_type_name'] == '음식배송':
                            action_type = "food_pickup_arrival"
                        elif task_info['task_type_name'] == '비품배송':
                            action_type = "supply_pickup_arrival"

                        if action_type:
                            event_data = {
                                "type": "event", 
                                "action": action_type,
                                "payload": {
                                    "task_id": str(task_id), 
                                    "robot_id": f"ROBOT_{robot_id:02d}"
                                }
                            }
                            websocket_manager.broadcast_to_sync("staff", json.dumps(event_data))
                            log_websocket_event("BROADCAST", "staff", f"픽업 도착 알림({action_type}) - Task {task_id}")

                    # 최종 목적지 도착
                    else:
                        cursor.execute("SELECT name FROM location WHERE id = %s", (destination_id,))
                        destination_name = cursor.fetchone()['name']
                        logger.info(
                            f"최종 목적지({destination_name}) 도착",
                            category="TASK", subcategory="STATUS-UPDATE",
                            details={"RobotID": robot_id, "TaskID": task_id, "Location": destination_name}
                        )
                        
                        # DB 상태 업데이트
                        cursor.execute("UPDATE task SET task_status_id = %s, delivery_arrival_time = %s WHERE id = %s",
                                     (settings.db_consts.task_status['배송 도착'], datetime.now(), task_id))
                        log_database_operation("UPDATE", "task", True, f"Task {task_id} 상태 '배송 도착'으로 변경")
                        
                        # GGUI에 배송 도착 알림 전송 (해당 위치의 클라이언트에게만)
                        event_data = {
                            "type": "event",
                            "action": "robot_arrival_completion",
                            "payload": {
                                "task_name": f"TASK_{task_id}",
                                "location_name": destination_name
                            }
                        }
                        websocket_manager.send_to_client_by_location_sync("guest", destination_name, json.dumps(event_data))
                        log_websocket_event("SEND", f"guest@{destination_name}", f"배송 도착 알림 - Task {task_id}")

        except (DatabaseException, Exception) as e:
            logger.error(f"도착 이벤트 처리 중 오류: {e}", category="TASK", subcategory="ERROR", details={"Event": "Arrival"})

    @handle_database_errors
    def battery_status_callback(self, msg):
        """수신한 배터리 정보를 DB에 업데이트하고, AGUI에 실시간으로 전송 및 로그를 기록합니다."""
        logger.info(
            "배터리 상태 수신",
            category="ROS2", subcategory="TOPIC-SUB",
            details={"Topic": "/roomie/status/battery_status", "RobotID": msg.robot_id, "BatteryLevel": f"{msg.charge_percentage}%"}
        )
        
        try:
            with safe_database_connection(db_manager.get_connection) as conn:
                with database_transaction(conn) as cursor:
                    # 1. robot_current_state 에 현재 상태 업데이트
                    self.robot_manager.update_robot_current_state(
                        cursor, 
                        msg.robot_id, 
                        battery_level=msg.charge_percentage, 
                        is_charging=msg.is_charging
                    )
                    
                    # 2. robot_log 에 현재 상태 기록
                    cursor.execute("SELECT robot_status_id, floor_id, error_id FROM robot_current_state WHERE robot_id = %s", (msg.robot_id,))
                    current_snapshot = cursor.fetchone()

                    self.robot_manager.add_robot_log(
                        cursor,
                        robot_id=msg.robot_id,
                        status_id=current_snapshot['robot_status_id'],
                        battery_level=int(msg.charge_percentage),
                        is_charging=msg.is_charging,
                        floor_id=current_snapshot['floor_id'],
                        error_id=current_snapshot['error_id']
                    )
            
            # WebSocket 이벤트 전송 - AGUI에 배터리 상태 업데이트 알림
            if websocket_manager:
                event_data = {
                    "type": "event",
                    "action": "battery_status_update",
                    "payload": {
                        "robot_id": msg.robot_id,
                        "battery_level": int(msg.charge_percentage),
                        "is_charging": msg.is_charging,
                        "timestamp": datetime.now().isoformat()
                    }
                }
                import json
                websocket_manager.broadcast_to_sync("admin", json.dumps(event_data))
                log_websocket_event("BROADCAST", "admin", f"배터리 상태 업데이트 - Robot {msg.robot_id}")

        except (DatabaseException, Exception) as e:
            logger.error(
                f"배터리 상태 업데이트 중 오류: {e}",
                category="DB", subcategory="ERROR",
                details={"RobotID": msg.robot_id}
            )
            
    @handle_database_errors
    def roomie_pose_callback(self, msg):
        """로봇의 현재 위치(Pose) 정보를 DB에 업데이트합니다."""
        logger.info(
            "로봇 위치 수신",
            category="ROS2", subcategory="TOPIC-SUB",
            details={"Topic": "/roomie/status/roomie_pose", "RobotID": msg.robot_id, "FloorID": msg.floor_id}
        )

        # floor_id 유효성 검사
        if msg.floor_id < 0:
            logger.warning(
                f"유효하지 않은 floor_id: {msg.floor_id}",
                category="ROS2", subcategory="WARNING",
                details={"RobotID": msg.robot_id, "FloorID": msg.floor_id}
            )
            return

        try:
            with safe_database_connection(db_manager.get_connection) as conn:
                with database_transaction(conn) as cursor:                    
                    self.robot_manager.update_robot_current_state(cursor, msg.robot_id, floor_id=msg.floor_id)    

        except (DatabaseException, Exception) as e:
            logger.error(
                f"로봇 위치(Pose) 업데이트 중 오류 발생: {e}",
                category="DB", subcategory="ERROR",
                details={"RobotID": msg.robot_id}
            )
            
    # @handle_database_errors
    # def pickup_completed_callback(self, msg):
    #     """픽업 완료 이벤트를 처리합니다."""
    #     logger.info(
    #         "픽업 완료 이벤트 수신",
    #         category="ROS2", subcategory="TOPIC-SUB",
    #         details={"Topic": "/roomie/event/pickup_completed", "RobotID": msg.robot_id, "TaskID": msg.task_id}
    #     )
        
    #     try:
    #         with safe_database_connection(db_manager.get_connection) as conn:
    #             with database_transaction(conn) as cursor:
    #                 cursor.execute("UPDATE task SET task_status_id = %s, pickup_completion_time = %s WHERE id = %s",
    #                              (settings.db_consts.task_status['배송 중'], datetime.now(), msg.task_id))
            
    #         log_database_operation("UPDATE", "task", True, f"Task {msg.task_id} 상태 '배송 중'으로 변경 및 픽업 시간 기록")
            
    #         # # ROS2 토픽으로 상태 전파
    #         # self.publish_task_state(msg.task_id, settings.db_consts.task_status['배송 중'])
            
    #         # AGUI에 픽업 완료 알림 전송
    #         from app.services.websocket_manager import manager as websocket_manager
    #         if websocket_manager:
    #             event_data = {
    #                 "type": "event",
    #                 "action": "pickup_completion",
    #                 "payload": {
    #                     "task_id": msg.task_id,
    #                     "robot_id": msg.robot_id,
    #                     "pickup_time": datetime.now().isoformat()
    #                 }
    #             }
    #             import json
    #             websocket_manager.broadcast_to_sync("admin", json.dumps(event_data))
    #             log_websocket_event("BROADCAST", "admin", f"픽업 완료 알림 - Task {msg.task_id}")
            
    #     except (DatabaseException, Exception) as e:
    #         logger.error(
    #             f"픽업 완료 처리 중 오류: {e}",
    #             category="TASK", subcategory="ERROR",
    #             details={"Event": "PickupCompleted", "TaskID": msg.task_id}
    #         )
            
    # @handle_database_errors
    # def delivery_completed_callback(self, msg):
    #     """배송 완료(고객 수령) 이벤트를 처리합니다."""
    #     logger.info(
    #         "배송 완료 이벤트 수신",
    #         category="ROS2", subcategory="TOPIC-SUB",
    #         details={"Topic": "/roomie/event/delivery_completed", "RobotID": msg.robot_id, "TaskID": msg.task_id}
    #     )

    #     try:
    #         with safe_database_connection(db_manager.get_connection) as conn:
    #             with database_transaction(conn) as cursor:
    #                 cursor.execute("UPDATE task SET task_status_id = %s, task_completion_time = %s WHERE id = %s",
    #                              (settings.db_consts.task_status['수령 완료'], datetime.now(), msg.task_id))
    #                 cursor.execute("SELECT l.name as destination_name FROM task t JOIN location l ON t.location_id = l.id WHERE t.id = %s", (msg.task_id,))
    #                 task_info = cursor.fetchone()
    #                 destination_name = task_info['destination_name'] if task_info else ""
                    
    #         log_database_operation("UPDATE", "task", True, f"Task {msg.task_id} 상태 '수령 완료'로 변경")
            
    #         # # ROS2 토픽으로 최종 작업 상태 전파
    #         # self.publish_task_state(msg.task_id, settings.db_consts.task_status['수령 완료'])
            
    #         # WebSocket 이벤트 전송
    #         if destination_name:
    #             event_data = {
    #                 "type": "event",
    #                 "action": "delivery_completion",
    #                 "payload": {
    #                     "task_name": f"TASK_{msg.task_id}",
    #                     "request_location": destination_name
    #                 }
    #             }
    #             websocket_manager.send_to_client_by_location_sync("guest", destination_name, json.dumps(event_data))
    #             log_websocket_event("SEND", f"guest@{destination_name}", f"배송 완료 알림 - Task {msg.task_id}")

    #         # 로봇 복귀 로직 호출
    #         self.robot_manager.decide_and_execute_return(msg.robot_id)

    #     except (DatabaseException, Exception) as e:
    #         logger.error(
    #             f"배송 완료 처리 중 오류: {e}",
    #             category="TASK", subcategory="ERROR",
    #             details={"Event": "DeliveryCompleted", "TaskID": msg.task_id}
    #         ) 