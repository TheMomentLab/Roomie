# services.py (수정 후)

from datetime import datetime, timedelta
import pytz
import uuid
import json
import random

import config
from models import RequestPayload # RequestModel은 서버에서 처리하므로 여기선 payload만 사용

KST = pytz.timezone(config.TIMEZONE)

def get_food_menu(payload: RequestPayload) -> dict:
    """ 음식 메뉴를 반환 (명세서와 일치) """
    return {
        "type": "response",
        "action": "get_food_menu",
        "payload": {"food_items": config.FOOD_MENU}
    }

def get_supply_menu(payload: RequestPayload) -> dict:
    """ 비품 메뉴를 반환 (명세서와 일치) """
    return {
        "type": "response",
        "action": "get_supply_menu",
        "payload": {"supply_items": config.SUPPLY_MENU}
    }

def create_delivery_task(payload: RequestPayload) -> dict:
    """ 배송 작업을 생성 (명세서와 일치) """
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
            "estimated_time": 55, # int
            "task_creation_time": datetime.now(KST).isoformat()
        }
    }

def create_call_task(payload: RequestPayload) -> dict:
    """ 로봇 호출 작업을 생성 (명세서에 맞게 응답 필드 수정) """
    task_id = f"TASK_{str(uuid.uuid4())[:8].upper()}"
    return {
        "type": "response",
        "action": "create_call_task",
        "payload": {
            "location_name": payload.location, # 요청의 location을 location_name으로 반환
            "task_name": task_id,
            "success": True,
            "error_code": None,
            "error_message": None,
            "task_creation_time": datetime.now(KST).isoformat()
        }
    }

def get_order_history(payload: RequestPayload) -> dict:
    """ 주문 내역을 반환 (도착 예정 시간 계산 로직 추가) """
    now = datetime.now(KST)
    task_creation_time = now - timedelta(minutes=1) # 테스트를 위해 1분 전에 생성되었다고 가정
    estimated_seconds = 55 # 예상 소요 시간 (초)

    # ✅ 수정: 도착 예정 시간을 서버에서 직접 계산
    estimated_arrival_time = task_creation_time + timedelta(seconds=estimated_seconds)

    return {
        "type": "response",
        "action": "get_order_history",
        "payload": {
            "request_location": payload.request_location,
            "task_name": payload.task_name,
            "task_type_name": "음식배달",
            "estimated_time": estimated_seconds,
            "task_creation_time":  task_creation_time.isoformat(),
            "robot_assignment_time": task_creation_time.isoformat(), # 예시 데이터
            "pickup_completion_time": now.isoformat(), # 예시 데이터
            # ✅ 수정: 계산된 도착 시간을 ISO 문자열로 전달
            "delivery_arrival_time": estimated_arrival_time.isoformat()
        }
    }

# --- 신규 API 서비스 함수 ---
def get_call_history(payload: RequestPayload) -> dict:
    """ 호출 내역을 반환하는 신규 서비스 """
    return {
        "type": "response",
        "action": "get_call_history",
        "payload": {
            "location_name": payload.location_name,
            "task_name": payload.task_name,
            "task_type_name": "로봇호출", # task_type_name은 "로봇호출"이 더 적절해 보입니다.
            "estimated_time": 5, # 분 단위
            "robot_status" : {
               "x": round(random.uniform(1.0, 100.0), 2),
               "y": round(random.uniform(1.0, 100.0), 2),
               "floorId": 1 # 명세서의 floor_id는 JSON에서 floorId로 변환됩니다.
            }
        }
    }