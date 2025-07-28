from datetime import datetime
from app.utils.logger import get_logger, log_database_operation
from app.utils.error_handler import handle_database_errors
from app.config import settings

logger = get_logger(__name__)

class RobotManager:
    """로봇 상태 조회, 업데이트 등 비즈니스 로직을 관리하는 클래스"""
    
    def __init__(self, node):
        self.node = node

    def get_available_robot(self):
        """사용 가능한 로봇을 찾아 반환합니다."""
        # TODO: 실제 로봇 선택 로직 구현
        # 현재는 항상 1번 로봇을 반환하도록 시뮬레이션
        return 1

    @handle_database_errors
    def add_robot_log(self, cursor, robot_id: int, status_id: int, task_id: int = None, battery_level: int = None, is_charging: bool = None, floor_id: int = None, error_id: int = None):
        """
        robot_log 테이블에 새로운 로그를 기록합니다.
        """
        query = """
            INSERT INTO robot_log 
            (robot_id, robot_status_id, task_id, battery_level, is_charging, floor_id, error_id, last_updated_time)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
        """
        current_time = datetime.now()
        params = (robot_id, status_id, task_id, battery_level, is_charging, floor_id, error_id, current_time)
        cursor.execute(query, params)
        log_database_operation("INSERT", "robot_log", True, f"Robot ID {robot_id} 로그 추가")
    
    
    @handle_database_errors
    def update_robot_current_state(self, cursor, robot_id, status_id=None, floor_id=None, battery_level=None, is_charging=None, error_id=None):
        """(Helper) robot_current_state 테이블을 동시성 문제 없이 안전하게 업데이트합니다."""
        
        # 1. SELECT ... FOR UPDATE로 해당 로봇의 현재 상태 레코드를 잠그고 가져옵니다.
        cursor.execute(
            "SELECT robot_id FROM robot_current_state WHERE robot_id = %s FOR UPDATE",
            (robot_id,)
        )
        current_state_record = cursor.fetchone()

        current_time = datetime.now()
        
        if current_state_record:
            # 2a. 레코드가 있으면 UPDATE
            update_fields = ["last_updated_time = %s"]
            values = [current_time]
            
            if status_id is not None:
                update_fields.append("robot_status_id = %s")
                values.append(status_id)
            if floor_id is not None:
                update_fields.append("floor_id = %s")
                values.append(floor_id)
            if battery_level is not None:
                update_fields.append("battery_level = %s")
                values.append(battery_level)
            if is_charging is not None:
                update_fields.append("is_charging = %s")
                values.append(is_charging)
            if error_id is not None:
                update_fields.append("error_id = %s")
                values.append(error_id)
            
            values.append(robot_id)
            
            query = f"UPDATE robot_current_state SET {', '.join(update_fields)} WHERE robot_id = %s"
            cursor.execute(query, tuple(values))

        else:
            # 2b. 레코드가 없으면 INSERT
            query = """
                INSERT INTO robot_current_state 
                (robot_id, robot_status_id, floor_id, battery_level, is_charging, error_id, last_updated_time)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
            """
            cursor.execute(query, (robot_id, status_id, floor_id, battery_level, is_charging, error_id, current_time))
        
        log_database_operation("UPDATE", "robot_current_state", True, f"Robot ID {robot_id} 상태 업데이트") 

    def decide_and_execute_return(self, robot_id: int):
        """로봇 복귀 여부를 판단하고 복귀 작업을 수행합니다."""
        should_return = True # 현재는 항상 복귀하도록 설정

        if should_return:
            # 복귀 결정
            return_location_id = settings.db_consts.location['LOB_WAITING']
            if self.node and self.node.action_handler:
                self.node.action_handler.send_perform_return_goal(robot_id, return_location_id)
            else:
                logger.error(f"ActionHandler를 찾을 수 없어 로봇 {robot_id}의 복귀 명령을 보낼 수 없습니다.")
                return
        else:
            # TODO: 연속 작업 할당 결정
            pass