"""
Roomie RMS 설정 관리 모듈

이 모듈은 애플리케이션의 모든 설정을 중앙집중식으로 관리합니다:
- API 엔드포인트
- ROS2 인터페이스 이름
- 데이터베이스 설정 
- 로깅 설정
- 애플리케이션 상수
"""

from pydantic_settings import BaseSettings
from typing import Dict, Optional
from pathlib import Path

# API 엔드포인트 경로를 관리하는 클래스
class ApiEndpoints:
    # GUEST GUI HTTP Endpoints
    CREATE_CALL_TASK = "/create_call_task"
    GET_CALL_HISTORY = "/get_call_history"
    GET_FOOD_MENU = "/get_food_menu"
    GET_SUPPLY_MENU = "/get_supply_menu"
    CREATE_DELIVERY_TASK = "/create_delivery_task"
    GET_ORDER_HISTORY = "/get_order_history"
    # STAFF GUI HTTP Endpoints
    FOOD_ORDER_STATUS_CHANGE = "/food_order_status_change"
    # ADMIN GUI HTTP Endpoints
    GET_TASK_LIST = "/task_list"
    GET_TASK_DETAIL = "/task_detail"
    GET_ROBOT_LIST = "/robot_list"
    # Websocket Endpoints
    WS_GUEST = "/ws/guest/{location_name}"
    WS_STAFF = "/ws/staff/{staff_id}"
    WS_ADMIN = "/ws/admin/{admin_id}"

# ROS2 인터페이스 이름(토픽, 서비스, 액션)을 관리하는 클래스
class ROSInterfaces:
    # Services
    GET_LOCATIONS_SERVICE = "/roomie/command/get_locations"
    CREATE_TASK_SERVICE = "/roomie/command/create_task"
    # Actions
    PERFORM_TASK_ACTION = "/roomie/action/perform_task"
    PERFORM_RETURN_ACTION = "/roomie/action/perform_return"
    # Publishers
    TASK_STATE_TOPIC = "/roomie/status/task_state"
    # Subscribers
    ROBOT_STATE_TOPIC = "/roomie/status/robot_state"
    ARRIVAL_TOPIC = "/roomie/event/arrival"
    BATTERY_STATUS_TOPIC = "/roomie/status/battery_status"
    ROOMIE_POSE_TOPIC = "/roomie/status/roomie_pose"
    PICKUP_COMPLETED_TOPIC = "/roomie/event/pickup_completed"
    DELIVERY_COMPLETED_TOPIC = "/roomie/event/delivery_completed"

# 데이터베이스에서 동적으로 로드될 상수를 저장하는 클래스
class DatabaseConstants:
    """
    애플리케이션 시작 시 DB에서 로드되는 상수 값을 저장합니다.
    (예: 'name' -> 'id' 매핑)
    """
    task_status: Dict[str, int] = {}
    robot_status: Dict[str, int] = {}
    task_type: Dict[str, int] = {}
    location: Dict[str, int] = {}

# 애플리케이션에서 사용되는 정적 상수 값을 관리하는 클래스
class AppConstants:
    """
    DB에 저장되지 않는 순수 애플리케이션 레벨의 상수.
    """
    # Task & Location Names
    TASK_TYPE_FOOD_DELIVERY: str = "음식배송"
    TASK_TYPE_SUPPLY_DELIVERY: str = "비품배송"
    TASK_TYPE_CALL: str = "호출"
    TASK_TYPE_GUIDANCE: str = "길안내"
    
    # Task Status Names
    TASK_STATUS_RECEIVED: str = "접수됨"
    TASK_STATUS_READY: str = "준비 완료"
    TASK_STATUS_ROBOT_ASSIGNED: str = "로봇 할당됨"
    TASK_STATUS_MOVING_TO_PICKUP: str = "픽업 장소로 이동"
    TASK_STATUS_WAITING_FOR_PICKUP: str = "픽업 대기 중"
    TASK_STATUS_DELIVERING: str = "배송 중"
    TASK_STATUS_DELIVERY_ARRIVED: str = "배송 도착"
    TASK_STATUS_COMPLETED: str = "수령 완료"
    TASK_STATUS_CALL_RECEIVED: str = "호출 접수됨"
    TASK_STATUS_CALL_ROBOT_ASSIGNED: str = "호출 로봇 할당됨"
    TASK_STATUS_CALL_MOVING: str = "호출 이동 중"
    TASK_STATUS_CALL_ARRIVED: str = "호출 도착"
    TASK_STATUS_GUIDANCE_RECEIVED: str = "길안내 접수됨"
    TASK_STATUS_GUIDANCE_IN_PROGRESS: str = "길안내 중"
    TASK_STATUS_GUIDANCE_ARRIVED: str = "길안내 도착"
    
    # Location Names
    LOCATION_LOB_WAITING: str = "LOB_WAITING"
    LOCATION_LOB_CALL: str = "LOB_CALL"
    LOCATION_RES_PICKUP: str = "RES_PICKUP"
    LOCATION_RES_CALL: str = "RES_CALL"
    LOCATION_SUP_PICKUP: str = "SUP_PICKUP"
    LOCATION_ELE_1: str = "ELE_1"
    LOCATION_ELE_2: str = "ELE_2"
    LOCATION_ROOM_101: str = "ROOM_101"
    LOCATION_ROOM_102: str = "ROOM_102"
    LOCATION_ROOM_201: str = "ROOM_201"
    LOCATION_ROOM_202: str = "ROOM_202"
    
    # Food Items
    FOOD_SPAGHETTI: str = "스파게티"
    FOOD_PIZZA: str = "피자"
    FOOD_STEAK: str = "스테이크"
    FOOD_BURGER: str = "버거"
    
    # Supply Items
    SUPPLY_TOOTHBRUSH: str = "칫솔"
    SUPPLY_TOWEL: str = "타월"
    SUPPLY_WATER: str = "생수"
    SUPPLY_SPOON: str = "수저"
    
    # Robot Status Names
    ROBOT_STATUS_INITIALIZING: str = "초기화"
    ROBOT_STATUS_CHARGING: str = "충전상태"
    ROBOT_STATUS_STANDBY: str = "작업대기"
    ROBOT_STATUS_MOVING_TO_PICKUP: str = "픽업위치 이동"
    ROBOT_STATUS_WAITING_FOR_PICKUP: str = "픽업대기"
    ROBOT_STATUS_MOVING_TO_DESTINATION: str = "배송장소 이동"
    ROBOT_STATUS_WAITING_FOR_HANDOVER: str = "수령대기"
    ROBOT_STATUS_MOVING_TO_CALLER: str = "호출위치 이동"
    ROBOT_STATUS_WAITING_FOR_GUIDANCE_INPUT: str = "길안내 목적지 입력대기"
    ROBOT_STATUS_GUIDING: str = "길안내 이동"
    ROBOT_STATUS_SEARCHING_TARGET: str = "대상 탐색"
    ROBOT_STATUS_RETURNING_TO_STATION: str = "대기위치로 이동"
    ROBOT_STATUS_ENTERING_ELEVATOR: str = "엘리베이터 탑승"
    ROBOT_STATUS_ERROR: str = "오류"
    
    LOCATION_NAME_FOOD_PICKUP: str = "RES_PICKUP"
    LOCATION_NAME_SUPPLY_PICKUP: str = "SUP_PICKUP"
    
    # Defaults
    DEFAULT_ESTIMATED_TIME_MINUTES: int = 30
    # Timeouts
    SERVICE_TIMEOUT_SEC: float = 1.0
    ACTION_TIMEOUT_SEC: float = 1.0


class Settings(BaseSettings):
    """
    애플리케이션 메인 설정 클래스
    환경변수나 .env 파일에서 값을 로드할 수 있습니다.
    """
    
    # 데이터베이스 설정
    DB_HOST: str = "localhost"
    DB_USER: str = "root"
    DB_PASSWORD: str = "1234"
    DB_NAME: str = "roomie_db"
    DB_POOL_NAME: str = "roomie_pool"
    DB_POOL_SIZE: int = 5

    # FastAPI 서버 설정
    FASTAPI_HOST: str = "192.168.0.47"
    FASTAPI_PORT: int = 8000

    # 로그 설정
    LOG_DIR: str = "logs"
    LOG_FILE: str = "roomie_rms.log"
    LOG_LEVEL: str = "INFO"
    LOG_MAX_BYTES: int = 10 * 1024 * 1024  # 10 MB
    LOG_BACKUP_COUNT: int = 5

    # 정적 파일 경로
    PROJECT_ROOT: Path = Path(__file__).resolve().parent.parent

    # DB 경로 설정
    DB_SCHEMA_PATH: Path = PROJECT_ROOT / "assets/sql/roomie_db_tables.sql"
    DB_DATA_PATH: Path = PROJECT_ROOT / "assets/sql/roomie_db_data.sql"
    DB_DUMMY_DATA_PATH: Path = PROJECT_ROOT / "assets/sql/roomie_db_dummy.sql"

    # 정적 파일 경로
    STATIC_DIR: Path = PROJECT_ROOT / "assets"
    
    # 클래스로 분리된 설정들을 포함
    api: ApiEndpoints = ApiEndpoints()
    ros: ROSInterfaces = ROSInterfaces()
    const: AppConstants = AppConstants()
    db_consts: DatabaseConstants = DatabaseConstants()  # DB에서 로드될 상수를 위한 공간
    
    class Config:
        """Pydantic 설정"""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True


# 설정 객체 생성
settings = Settings()
