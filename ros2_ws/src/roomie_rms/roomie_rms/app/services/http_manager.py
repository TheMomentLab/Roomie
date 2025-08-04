"""
HTTP API 통합 관리자
FastAPI의 모든 HTTP 엔드포인트를 클래스 기반으로 관리합니다.
"""

import json
import base64
import os
from datetime import datetime
from typing import Dict, Any, Optional, List
from fastapi import APIRouter, HTTPException, Request, Depends

from app.config import settings
from app.services import db_manager
from app.services.websocket_manager import manager as websocket_manager
from app.utils.logger import get_logger, log_database_operation
from app.utils.error_handler import safe_database_connection, database_transaction
from app.utils.exceptions import RoomieBaseException, raise_validation_error
from app.schemas.gui_models import *

logger = get_logger(__name__)

class HttpManager:
    """HTTP API 관리자"""

    def __init__(self):
        """HTTP Manager 초기화"""
        self.router = APIRouter()
        self._setup_routes()

    def _setup_routes(self):
        """HTTP API 라우터 설정"""

        # --- GGUI HTTP 동기 인터페이스 ---

        @self.router.post("/create_call_task", response_model=CreateCallTaskResponse)
        async def create_call_task(request: CreateCallTaskRequest, request_obj: Request):
            """(GGUI) 호출 작업 생성 요청"""
            payload = request.payload
            location_name = payload.location
            logger.info(
                "GGUI 호출 작업 생성 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "GGUI", "Method": "POST", "Path": "/api/gui/create_call_task", "Location": location_name}
            )

            try:
                # task_manager를 통해 실제 호출 작업 생성 (task_type_name을 "호출"로 설정)
                rms_node = request_obj.app.state.rms_node
                result = rms_node.task_manager.create_delivery_task(
                    location_name=location_name,
                    task_type_name="호출",
                    order_details={}  # 호출 작업은 주문 상세 정보가 없음
                )

                task_id = result["task_id"]

                # WebSocket으로 호출 알림 전송
                event_data = {
                    "type": "event",
                    "action": "call_request_acceptance",
                    "payload": {
                        "task_name": f"TASK_{task_id}",
                        "estimated_wait_time": 15
                    }
                }
                await websocket_manager.broadcast_to("guest", json.dumps(event_data))
                logger.info(
                    "GGUI에 호출 수락 알림 전송",
                    category="API", subcategory="WS-EVENT",
                    details={"Target": "Guest", "Event": "call_request_acceptance", "TaskID": task_id}
                )

                response_payload = CreateCallTaskResponsePayload(
                    location_name=location_name,
                    task_name=f"TASK_{task_id}",
                    success=True,
                    task_creation_time=datetime.now().isoformat() + "+09:00"
                )

                response = CreateCallTaskResponse(payload=response_payload)
                logger.info(
                    "GGUI 호출 작업 생성 응답 전송",
                    category="API", subcategory="HTTP-RES",
                    details={"TaskID": task_id, "Client": "GGUI", "Response": response.model_dump_json()}
                )
                return response

            except RoomieBaseException as e:
                logger.error(f"호출 작업 생성 실패: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=400, detail=e.message)
            except Exception as e:
                logger.error(f"호출 작업 생성 중 예상치 못한 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="호출 작업 생성 실패")

        @self.router.post("/get_food_menu", response_model=GetFoodMenuResponse)
        async def get_food_menu(request: GetFoodMenuRequest):
            """(GGUI) 음식 메뉴 요청"""
            payload = request.payload
            location_name = payload.location_name
            logger.info(
                "GGUI 음식 메뉴 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "GGUI", "Method": "POST", "Path": "/api/gui/get_food_menu", "Location": location_name}
            )

            try:
                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        # 음식 메뉴 조회
                        cursor.execute("SELECT name, price, image FROM food")
                        results = cursor.fetchall()

                        food_items = []
                        for row in results:
                            image_data_url = None
                            # 이미지 경로를 Base64 데이터로 변환하는 로직
                            if row['image']:
                                image_path = os.path.join(settings.STATIC_DIR, row['image'].lstrip('/'))

                                if os.path.exists(image_path):
                                    try:
                                        with open(image_path, "rb") as image_file:
                                            # 파일을 읽고 Base64로 인코딩 후, utf-8 문자열로 변환
                                            encoded_string = base64.b64encode(image_file.read()).decode('utf-8')

                                            # 파일 확장자에 따라 MIME 타입 결정
                                            mime_type = "image/jpeg" if image_path.lower().endswith(('.jpg', '.jpeg')) else "image/png"

                                            # Data URL 형식으로 완성
                                            image_data_url = f"data:{mime_type};base64,{encoded_string}"
                                    except Exception as e:
                                        logger.warning(f"이미지 파일 처리 중 오류 발생: {image_path}, Error: {e}")
                                else:
                                    logger.warning(f"이미지 파일을 찾을 수 없음: {image_path}")

                            food_items.append(FoodMenuItem(
                                food_name=row['name'],
                                price=row['price'],
                                image=image_data_url
                            ))

                        response_payload = GetFoodMenuResponsePayload(food_items=food_items)
                        response = GetFoodMenuResponse(payload=response_payload)
                        logger.info(
                            "GGUI 음식 메뉴 응답 전송",
                            category="API", subcategory="HTTP-RES",
                            details={"Client": "GGUI", "ItemCount": len(response.payload.food_items)}
                        )
                        return response

            except Exception as e:
                logger.error(f"GGUI 음식 메뉴 조회 중 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="음식 메뉴 조회 실패")

        @self.router.post("/get_supply_menu", response_model=GetSupplyMenuResponse)
        async def get_supply_menu(request: GetSupplyMenuRequest):
            """(GGUI) 비품 메뉴 요청"""
            payload = request.payload
            location_name = payload.location_name
            logger.info(
                "GGUI 비품 메뉴 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "GGUI", "Method": "POST", "Path": "/api/gui/get_supply_menu", "Location": location_name}
            )

            try:
                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        # 비품 메뉴 조회
                        cursor.execute("SELECT name, image FROM supply")
                        results = cursor.fetchall()

                        supply_items = []
                        for row in results:
                            image_data_url = None
                            if row['image']:
                                image_path = os.path.join(settings.STATIC_DIR, row['image'].lstrip('/'))

                                if os.path.exists(image_path):
                                    try:
                                        with open(image_path, "rb") as image_file:
                                            encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
                                            mime_type = "image/jpeg" if image_path.lower().endswith(('.jpg', '.jpeg')) else "image/png"
                                            image_data_url = f"data:{mime_type};base64,{encoded_string}"
                                    except Exception as e:
                                        logger.warning(f"이미지 파일 처리 중 오류 발생: {image_path}, Error: {e}")
                                else:
                                    logger.warning(f"이미지 파일을 찾을 수 없음: {image_path}")

                            supply_items.append(SupplyMenuItem(
                                supply_name=row['name'],
                                image=image_data_url
                            ))

                        response_payload = GetSupplyMenuResponsePayload(supply_items=supply_items)
                        response = GetSupplyMenuResponse(payload=response_payload)
                        logger.info(
                            "GGUI 비품 메뉴 응답 전송",
                            category="API", subcategory="HTTP-RES",
                            details={"Client": "GGUI", "ItemCount": len(response.payload.supply_items)}
                        )
                        return response

            except Exception as e:
                logger.error(f"GGUI 비품 메뉴 조회 중 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="비품 메뉴 조회 실패")

        @self.router.post("/create_delivery_task", response_model=CreateDeliveryTaskResponse)
        async def create_delivery_task(request: CreateDeliveryTaskRequest, request_obj: Request):
            """(GGUI) 배송 작업 생성 요청"""
            payload = request.payload
            location_name = payload.location_name
            logger.info(
                "GGUI 배송 작업 생성 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "GGUI", "Method": "POST", "Path": "/api/gui/create_delivery_task", "Payload": payload.model_dump_json()}
            )

            try:
                # task_manager를 통해 실제 배송 작업 생성
                rms_node = request_obj.app.state.rms_node
                result = rms_node.task_manager.create_delivery_task(
                    location_name=location_name,
                    task_type_name=payload.task_type_name,
                    order_details=payload.order_details
                )
                task_id = result["task_id"]

                max_cooking_time = 0
                delivery_time = 0

                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        # 음식 배송일 경우, 최대 조리 시간 계산
                        if payload.task_type_name == "음식배송":
                            food_names = [item.name for item in payload.order_details['items']]
                            if food_names:
                                # IN 절을 사용하여 주문된 모든 음식의 조리 시간을 한 번에 조회
                                query = f"SELECT MAX(cooking_time) as max_time FROM food WHERE name IN ({', '.join(['%s'] * len(food_names))})"
                                cursor.execute(query, tuple(food_names))
                                result = cursor.fetchone()
                                if result and result['max_time']:
                                    max_cooking_time = result['max_time']

                        # 목적지 층수에 따라 배송 시간 계산
                        cursor.execute("SELECT floor_id FROM location WHERE name = %s", (location_name,))
                        location_info = cursor.fetchone()
                        if location_info:
                            if location_info['floor_id'] == 0: # 1층
                                delivery_time = 5
                            elif location_info['floor_id'] == 1: # 2층
                                delivery_time = 10
                            else:
                                pass

                # 최종 예상 시간 계산 (조리시간 + 픽업시간 + 배송시간)
                pickup_time = 5
                calculated_estimated_time = max_cooking_time + pickup_time + delivery_time

                # WebSocket으로 SGUI에 새 주문 알림 전송
                order_items_list = [
                    {"name": item.name, "quantity": item.quantity, "price": item.price}
                    for item in payload.order_details['items']
                ]

                action_type = ""
                if payload.task_type_name == "음식배송":
                    action_type = "food_order_creation"
                elif payload.task_type_name == "비품배송":
                    action_type = "supply_order_creation"

                # event_data 변수 정의
                event_data = {
                    "type": "event",
                    "action": action_type,
                    "payload": {
                        "task_id": task_id,
                        "request_location": location_name,
                        "order_details": {"items": order_items_list}
                    }
                }

                # 로그 기록
                logger.info(
                    f"SGUI로 '{action_type}' 이벤트 전송",
                    category="API", subcategory="WS-EVENT",
                    details={
                        "Target": "staff",
                        "TaskID": task_id,
                        "Message": json.dumps(event_data)
                    }
                )

                # 메시지 전송
                await websocket_manager.broadcast_to("staff", json.dumps(event_data))

                response_payload = CreateDeliveryTaskResponsePayload(
                    location_name=location_name,
                    task_name=f"TASK_{task_id}",
                    success=True,
                    estimated_time=calculated_estimated_time,
                    task_creation_time=datetime.now().isoformat() + "+09:00"
                )

                response = CreateDeliveryTaskResponse(payload=response_payload)
                logger.info(
                    "GGUI 배송 작업 생성 응답 전송",
                    category="API", subcategory="HTTP-RES",
                    details={"TaskID": task_id, "Client": "GGUI", "Response": response.model_dump_json()}
                )
                return response

            except RoomieBaseException as e:
                logger.error(f"GGUI 작업 생성 실패: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=400, detail=e.message)
            except Exception as e:
                logger.error(f"GGUI 배송 작업 생성 중 예상치 못한 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="배송 작업 생성 실패")

        @self.router.post("/get_call_history", response_model=GetCallHistoryResponse)
        async def get_call_history(request: GetCallHistoryRequest):
            """(GGUI) 호출 내역 조회 요청"""
            payload = request.payload
            logger.info(
                "GGUI 호출 내역 조회 요청",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "GGUI", "Method": "POST", "Path": "/api/gui/get_call_history", "TaskName": payload.task_name}
            )

            try:
                # task_name에서 task_id 추출 (TASK_001 → 1)
                task_id = int(payload.task_name.replace("TASK_", ""))

                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        # task 정보와 할당된 로봇 정보 조회
                        query = """
                        SELECT
                            t.id,
                            t.robot_id,
                            l.name as location_name,
                            tt.name as task_type_name,
                            rcs.robot_id as current_robot_id,
                            rcs.floor_id
                        FROM task t
                        JOIN location l ON t.location_id = l.id
                        JOIN task_type tt ON t.task_type_id = tt.id
                        LEFT JOIN robot_current_state rcs ON t.robot_id = rcs.robot_id
                        WHERE t.id = %s
                        """
                        cursor.execute(query, (task_id,))
                        task_result = cursor.fetchone()

                        if not task_result:
                            raise_validation_error(f"작업 ID {task_id}를 찾을 수 없습니다.")

                        # calculated_estimated_time 계산 (호출은 이동시간만)
                        calculated_estimated_time = 5  # 기본 호출 응답 시간

                        # 로봇이 할당된 경우 현재 위치에서 목적지까지의 대략적인 시간 계산
                        if task_result['robot_id'] and task_result['current_robot_id']:
                            # 층수 차이에 따른 이동시간 추가 계산 (간단한 로직)
                            target_floor_query = "SELECT floor_id FROM location WHERE name = %s"
                            cursor.execute(target_floor_query, (task_result['location_name'],))
                            target_location = cursor.fetchone()

                            if target_location and task_result['floor_id'] is not None:
                                floor_diff = abs(target_location['floor_id'] - task_result['floor_id'])
                                calculated_estimated_time = 5 + (floor_diff * 2)  # 층별 2분 추가

                        # 로봇 상태 정보 (현재는 기본값, 향후 실제 위치 데이터 연동 가능)
                        robot_status = RobotStatus(
                            x=0.0,  # 실제 로봇 위치 x 좌표
                            y=0.0,  # 실제 로봇 위치 y 좌표
                            floor_id=task_result['floor_id'] if task_result['floor_id'] is not None else 0
                        )

                        response_payload = GetCallHistoryResponsePayload(
                            location_name=task_result['location_name'],
                            task_name=payload.task_name,
                            task_type_name=task_result['task_type_name'],
                            estimated_time=calculated_estimated_time,
                            robot_status=robot_status
                        )

                response = GetCallHistoryResponse(payload=response_payload)
                logger.info(
                    "GGUI 호출 내역 조회 응답",
                    category="API", subcategory="HTTP-RES",
                    details={"Client": "GGUI", "TaskID": task_id, "Response": response.model_dump_json()}
                )
                return response

            except RoomieBaseException as e:
                logger.error(f"GGUI 호출 내역 조회 실패: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=400, detail=e.message)
            except Exception as e:
                logger.error(f"GGUI 호출 내역 조회 중 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="GGUI 호출 내역 조회 실패")

        @self.router.post("/get_order_history", response_model=GetOrderHistoryResponse)
        async def get_order_history(request: GetOrderHistoryRequest):
            """(GGUI) 주문 내역 조회 요청"""
            payload = request.payload
            logger.info(
                "GGUI 주문 내역 조회 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "GGUI", "Method": "POST", "Path": "/api/gui/get_order_history", "TaskName": payload.task_name}
            )

            try:
                # task_name에서 task_id 추출 (TASK_001 → 1)
                task_id = int(payload.task_name.replace("TASK_", ""))

                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        # task 정보와 관련 데이터 조회
                        query = """
                        SELECT
                            t.id,
                            t.task_creation_time,
                            t.robot_assignment_time,
                            t.pickup_completion_time,
                            t.delivery_arrival_time,
                            t.task_completion_time,
                            l.name as location_name,
                            tt.name as task_type_name,
                            ts.name as task_status_name
                        FROM task t
                        JOIN location l ON t.location_id = l.id
                        JOIN task_type tt ON t.task_type_id = tt.id
                        JOIN task_status ts ON t.task_status_id = ts.id
                        WHERE t.id = %s
                        """
                        # [수정] SQL 쿼리의 t.type_id -> t.task_type_id, tst.id -> ts.id
                        cursor.execute(query, (task_id,))
                        task_result = cursor.fetchone()

                        if not task_result:
                            raise_validation_error(f"작업 ID {task_id}를 찾을 수 없습니다.")

                        # calculated_estimated_time 계산
                        calculated_estimated_time = 0

                        # 음식 배송인 경우 조리시간 + 배송시간 계산
                        if task_result['task_type_name'] == "음식배송":
                            # 주문된 음식들의 최대 조리시간 조회
                            food_query = """
                            SELECT MAX(f.cooking_time) as max_cooking_time
                            FROM `order` o
                            JOIN food_order_item foi ON o.id = foi.order_id
                            JOIN food f ON foi.food_id = f.id
                            WHERE o.task_id = %s
                            """
                            cursor.execute(food_query, (task_id,))
                            food_result = cursor.fetchone()
                            max_cooking_time = food_result['max_cooking_time'] if food_result and food_result['max_cooking_time'] else 0

                            # 목적지 층수에 따른 배송시간 계산
                            location_query = "SELECT floor_id FROM location WHERE name = %s"
                            cursor.execute(location_query, (task_result['location_name'],))
                            location_info = cursor.fetchone()

                            delivery_time = 5  # 기본값
                            if location_info:
                                if location_info['floor_id'] == 0:  # 1층
                                    delivery_time = 5
                                elif location_info['floor_id'] == 1:  # 2층
                                    delivery_time = 10

                            pickup_time = 5  # 픽업 시간
                            calculated_estimated_time = max_cooking_time + pickup_time + delivery_time

                        elif task_result['task_type_name'] == "비품배송":
                            # 비품 배송의 경우 기본 시간 계산
                            delivery_time = 5  # 기본값
                            location_query = "SELECT floor_id FROM location WHERE name = %s"
                            cursor.execute(location_query, (task_result['location_name'],))
                            location_info = cursor.fetchone()
                            if location_info:
                                if location_info['floor_id'] == 0:  # 1층
                                    delivery_time = 5
                                elif location_info['floor_id'] == 1:  # 2층
                                    delivery_time = 10

                            pickup_time = 5
                            calculated_estimated_time = pickup_time + delivery_time

                        elif task_result['task_type_name'] == "호출":
                            # 호출의 경우 단순한 이동 시간
                            calculated_estimated_time = 5

                        # 시간 포맷 함수
                        def format_time(dt):
                            return dt.isoformat() + "+09:00" if dt else None

                        response_payload = GetOrderHistoryResponsePayload(
                            request_location=task_result['location_name'],
                            task_name=payload.task_name,
                            task_type_name=task_result['task_type_name'],
                            estimated_time=calculated_estimated_time,
                            task_creation_time=format_time(task_result['task_creation_time']),
                            robot_assignment_time=format_time(task_result['robot_assignment_time']),
                            pickup_completion_time=format_time(task_result['pickup_completion_time']),
                            delivery_arrival_time=format_time(task_result['delivery_arrival_time'])
                        )

                response = GetOrderHistoryResponse(payload=response_payload)
                logger.info(
                    "GGUI 주문 내역 조회 응답 전송",
                    category="API", subcategory="HTTP-RES",
                    details={"Client": "GGUI", "TaskID": task_id, "Response": response.model_dump_json()}
                )
                return response

            except RoomieBaseException as e:
                logger.error(f"GGUI 주문 내역 조회 실패: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=400, detail=e.message)
            except Exception as e:
                logger.error(f"GGUI 주문 내역 조회 중 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="GGUI 주문 내역 조회 실패")

        # --- SGUI HTTP 동기 인터페이스 ---

        @self.router.post("/food_order_status_change", response_model=FoodOrderStatusChangeResponse)
        async def food_order_status_change(request_data: FoodOrderStatusChangeRequest, request: Request):
            """(SGUI) 음식 주문 작업 상태를 '준비 완료'로 변경합니다."""
            payload = request_data.payload
            task_id_str = payload.task_id

            logger.info(
                "SGUI 음식 주문 상태 변경 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "SGUI", "TaskID": task_id_str}
            )

            try:
                task_id = int(task_id_str.replace("TASK_", ""))
                new_status = "준비 완료"
                rms_node = request.app.state.rms_node

                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        status_id = settings.db_consts.task_status.get(new_status)
                        if not status_id:
                            raise_validation_error(f"알 수 없는 상태 값: {new_status}")

                        cursor.execute("UPDATE task SET task_status_id = %s WHERE id = %s", (status_id, task_id))
                        log_database_operation("UPDATE", "task", True, f"Task {task_id} 상태 변경: {new_status}", "INFO")

                # 로봇 할당 로직 트리거
                if new_status == "준비 완료":
                    robot_id = rms_node.robot_manager.get_available_robot()
                    rms_node.task_manager.execute_task_assignment(robot_id)

                response_payload = FoodOrderStatusChangeResponsePayload(
                    task_id=task_id_str,
                    status_changed="food_ready"
                )
                response = FoodOrderStatusChangeResponse(payload=response_payload)

                logger.info(
                    "SGUI 음식 주문 상태 변경 응답 전송",
                    category="API", subcategory="HTTP-RES",
                    details={"Client": "SGUI", "Response": response.model_dump_json()}
                )
                return response

            except RoomieBaseException as e:
                logger.error(f"SGUI 음식 주문 상태 변경 실패: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=400, detail=e.message)
            except Exception as e:
                logger.error(f"SGUI 음식 주문 상태 변경 중 예상치 못한 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="SGUI 음식 주문 상태 변경 실패")

        # --- AGUI HTTP 동기 인터페이스 ---

        @self.router.post("/task_list", response_model=TaskListResponse)
        async def task_list(request: TaskListRequest):
            """(AGUI) 작업 목록을 조회합니다."""
            filters = request.payload.filters
            logger.info(
                "AGUI 작업 목록 조회 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "AGUI", "Method": "POST", "Path": "/api/gui/task_list", "Filters": filters.model_dump_json()}
            )
            query = """
                SELECT
                    t.id as task_id,
                    tt.name as task_type,
                    ts.name as task_status,
                    l.name as destination,
                    t.robot_id,
                    t.task_creation_time,
                    t.task_completion_time
                FROM task t
                JOIN task_type tt ON t.task_type_id = tt.id
                JOIN task_status ts ON t.task_status_id = ts.id
                JOIN location l ON t.location_id = l.id
                WHERE 1=1
            """
            # SQL 쿼리의 t.type_id -> t.task_type_id
            params = []

            if filters.start_date:
                query += " AND t.task_creation_time >= %s"
                params.append(filters.start_date)
            if filters.end_date:
                query += " AND t.task_creation_time <= %s"
                params.append(filters.end_date)
            if filters.task_type and filters.task_type != "전체":
                query += " AND tt.name = %s"
                params.append(filters.task_type)
            if filters.task_status and filters.task_status != "전체":
                query += " AND ts.name = %s"
                params.append(filters.task_status)
            if filters.destination and filters.destination != "전체":
                query += " AND l.name = %s"
                params.append(filters.destination)

            try:
                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        cursor.execute(query, tuple(params))
                        tasks_from_db = cursor.fetchall()

                        task_list_models = [TaskInDB(
                            task_id=row['task_id'],
                            task_type=row['task_type'],
                            task_status=row['task_status'],
                            destination=row['destination'],
                            robot_id=row['robot_id'],
                            task_creation_time=row['task_creation_time'],
                            task_completion_time=row['task_completion_time']
                        ) for row in tasks_from_db]

                        log_database_operation("SELECT", "task", True, f"{len(task_list_models)}개 작업 목록 조회", "INFO")
                        response_payload = TaskListResponsePayload(tasks=task_list_models)
                        response = TaskListResponse(payload=response_payload)
                        logger.info(
                            "AGUI 작업 목록 응답 전송",
                            category="API", subcategory="HTTP-RES",
                            details={"Client": "AGUI", "Count": len(task_list_models)}
                        )
                        return response

            except RoomieBaseException as e:
                logger.error(f"작업 목록 조회 중 DB 오류: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail=e.message)
            except Exception as e:
                logger.error(f"작업 목록 조회 중 예상치 못한 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="작업 목록 조회 실패")

        @self.router.post("/robot_list", response_model=RobotListResponse)
        async def robot_list(request: RobotListRequest):
            """(AGUI) 로봇 목록을 조회합니다."""
            filters = request.payload.filters
            logger.info(
                "AGUI 로봇 목록 조회 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "AGUI", "Method": "POST", "Path": "/api/gui/robot_list", "Filters": filters.model_dump_json()}
            )
            query = """
                SELECT
                    r.id as robot_id,
                    r.model_name,
                    rcs.battery_level,
                    rcs.is_charging,
                    rs.name as robot_status,
                    ts.name as task_status,
                    t.id as task_id,
                    (CASE WHEN e.id IS NOT NULL THEN TRUE ELSE FALSE END) as has_error
                FROM robot r
                LEFT JOIN robot_current_state rcs ON r.id = rcs.robot_id
                LEFT JOIN robot_status rs ON rcs.robot_status_id = rs.id
                LEFT JOIN error e ON rcs.error_id = e.id
                LEFT JOIN task t ON r.id = t.robot_id AND t.task_status_id NOT IN (7, 13, 22)
                LEFT JOIN task_status ts ON t.task_status_id = ts.id
                WHERE 1=1
            """
            params = []

            if filters.robot_id:
                query += " AND r.id = %s"
                params.append(int(filters.robot_id.replace("ROBOT_", "")))
            if filters.model_name and filters.model_name != "전체":
                query += " AND r.model_name = %s"
                params.append(filters.model_name)
            if filters.robot_status and filters.robot_status != "전체":
                query += " AND rs.name = %s"
                params.append(filters.robot_status)

            try:
                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        cursor.execute(query, tuple(params))
                        robots_from_db = cursor.fetchall()

                        robot_list_models = []
                        for row in robots_from_db:
                            # 작업이 없으면 로봇 상태, 있으면 작업 상태를 반환
                            final_status = row['task_status'] if row['task_status'] else row['robot_status']
                            
                            # RobotInDB 모델 생성 로직
                            robot_list_models.append(RobotInDB(
                                robot_id=row['robot_id'],
                                model_name=row['model_name'],
                                battery_level=row['battery_level'],
                                is_charging=bool(row['is_charging']),
                                robot_status=final_status,
                                task_id=row['task_id'],
                                has_error=bool(row['has_error'])
                            ))

                        response_payload = RobotListResponsePayload(robots=robot_list_models)
                        response = RobotListResponse(payload=response_payload)
                        logger.info(
                           "AGUI 로봇 목록 응답 전송",
                           category="API", subcategory="HTTP-RES",
                           details={"Client": "AGUI", "Count": len(robot_list_models)}
                        )
                        return response

            except RoomieBaseException as e:
                logger.error(f"로봇 목록 조회 중 DB 오류: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail=e.message)
            except Exception as e:
                logger.error(f"로봇 목록 조회 중 예상치 못한 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="로봇 목록 조회 실패")

        @self.router.post("/task_detail", response_model=TaskDetailResponse)
        async def task_detail(request: TaskDetailRequest):
            """(AGUI) 작업 상세 정보를 조회합니다."""
            task_id_str = request.payload.task_id
            logger.info(
                "AGUI 작업 상세 정보 요청 수신",
                category="API", subcategory="HTTP-REQ",
                details={"Client": "AGUI", "Method": "POST", "Path": "/api/gui/task_detail", "TaskID": task_id_str}
            )

            try:
                task_id = int(task_id_str.replace("TASK_", ""))

                query = """
                    SELECT
                        robot_assignment_time,
                        pickup_completion_time,
                        delivery_arrival_time,
                        task_completion_time
                    FROM task
                    WHERE id = %s
                """

                with safe_database_connection(db_manager.get_connection) as conn:
                    with database_transaction(conn) as cursor:
                        cursor.execute(query, (task_id,))
                        task_details = cursor.fetchone()

                        if not task_details:
                            raise HTTPException(status_code=404, detail=f"작업을 찾을 수 없습니다: {task_id_str}")

                        def format_time(dt):
                            return dt.isoformat() + 'Z' if dt else None

                        response_payload = TaskDetailResponsePayload(
                            robot_assignment_time=format_time(task_details.get('robot_assignment_time')),
                            pickup_completion_time=format_time(task_details.get('pickup_completion_time')),
                            delivery_arrival_time=format_time(task_details.get('delivery_arrival_time')),
                            task_completion_time=format_time(task_details.get('task_completion_time'))
                        )

                        response = TaskDetailResponse(payload=response_payload)
                        logger.info(
                            "AGUI 작업 상세 응답 전송",
                            category="API", subcategory="HTTP-RES",
                            details={"Client": "AGUI", "Response": response.model_dump_json()}
                        )
                        return response

            except ValueError:
                raise HTTPException(status_code=400, detail=f"잘못된 형식의 작업 ID: {task_id_str}")
            except RoomieBaseException as e:
                logger.error(f"작업 상세 정보 조회 중 DB 오류: {e.message}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail=e.message)
            except Exception as e:
                logger.error(f"작업 상세 정보 조회 중 예상치 못한 오류: {e}", category="API", subcategory="ERROR")
                raise HTTPException(status_code=500, detail="작업 상세 정보 조회 실패")

# 글로벌 인스턴스 생성
manager = HttpManager()