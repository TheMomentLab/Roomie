# services.py (명세서와 100% 일치하도록 수정)

from datetime import datetime, timedelta
import pytz
import uuid
import random

import config
from models import (
    CreateCallTaskPayload, GetHistoryPayload, GetMenuPayload, CreateDeliveryTaskPayload
)

KST = pytz.timezone(config.TIMEZONE)

# --- 서비스 함수들 ---

def get_food_menu(payload: GetMenuPayload) -> dict:
    """ 음식 메뉴를 반환 """
    return {
        "type": "response",
        "action": "get_food_menu",
        "payload": {"food_items": config.FOOD_MENU}
    }

def get_supply_menu(payload: GetMenuPayload) -> dict:
    """ 비품 메뉴를 반환 """
    return {
        "type": "response",
        "action": "get_supply_menu",
        "payload": {"supply_items": config.SUPPLY_MENU}
    }

def create_delivery_task(payload: CreateDeliveryTaskPayload) -> dict:
    """ 배송 작업을 생성 """
    task_id = f"TASK_{str(uuid.uuid4())[:8].upper()}"
    return {
        "type": "response",
        "action": "create_delivery_task",
        "payload": {
            "location_name": payload.location_name,
            "task_name": task_id,
            "success": True,
            "error_code": None,
            "error_message": None,
            "estimated_time": 55,
            "task_creation_time": datetime.now(KST).isoformat()
        }
    }

def create_call_task(payload: CreateCallTaskPayload) -> dict:
    """ 로봇 호출 작업을 생성 """
    task_id = f"TASK_{str(uuid.uuid4())[:8].upper()}"
    return {
        "type": "response",
        "action": "create_call_task",
        "payload": {
            "location_name": payload.location_name,
            "task_name": task_id,
            "success": True,
            "error_code": None,
            "error_message": None,
            "task_creation_time": datetime.now(KST).isoformat()
        }
    }

def get_task_list(payload: GetHistoryPayload) -> dict:
    """ 전체 요청 이력을 반환 (신규 추가) """
    now = datetime.now(KST)
    return {
        "type": "response",
        "action": "get_task_list",
        "payload": {
            "location_name": payload.request_location,
            "order_details": {
                "tasks": [
                    { "task_name": "TASK_FOOD_001", "task_type_name": "음식배달", "created_at": (now - timedelta(minutes=10)).isoformat() },
                    { "task_name": "TASK_CALL_002", "task_type_name": "로봇호출", "created_at": (now - timedelta(minutes=5)).isoformat() }
                ]
            }
        }
    }

def get_order_history(payload: GetHistoryPayload) -> dict:
    """ 주문(배송) 내역을 반환 """
    now = datetime.now(KST)
    creation_time = now - timedelta(minutes=15)
    return {
        "type": "response",
        "action": "get_order_history",
        "payload": {
            "request_location": payload.request_location,
            "task_name": payload.task_name,
            "task_type_name": payload.task_type_name,
            "estimated_time": 55,
            "task_creation_time": creation_time.isoformat(),
            "robot_assignment_time": (creation_time + timedelta(seconds=30)).isoformat(),
            "pickup_completion_time": (creation_time + timedelta(minutes=5)).isoformat(),
            "delivery_arrival_time": None # 아직 도착 안 한 상태
        }
    }

def get_call_history(payload: GetHistoryPayload) -> dict:
    """ 호출 내역을 반환 """
    return {
        "type": "response",
        "action": "get_call_history",
        "payload": {
            "location_name": payload.location_name,
            "task_name": payload.task_name,
            "task_type_name": "호출",
            "estimated_time": 5, # 분 단위
            "robot_status": {
               "x": round(random.uniform(1.0, 100.0), 2),
               "y": round(random.uniform(1.0, 100.0), 2),
               "floor_id": 1
            }
        }
    }