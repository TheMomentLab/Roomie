# test_server.py (수정 완료)

import asyncio
import json
import random

import uvicorn
from fastapi import APIRouter, FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from starlette.websockets import WebSocketDisconnect

import config
import services
from models import RequestModel

# --- 앱 및 라우터 설정 ---
app = FastAPI()
router = APIRouter(prefix="/api/gui")

# --- CORS 미들웨어 설정 ---
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- WebSocket 연결 관리 ---
room_connections: dict[str, list[WebSocket]] = {}

# --- 서비스 매핑 ---
# action 이름과 실제 실행될 서비스 함수를 연결합니다.
service_map = {
    "get_food_menu": services.get_food_menu,
    "get_supply_menu": services.get_supply_menu,
    "create_delivery_task": services.create_delivery_task,
    "create_call_task": services.create_call_task,
    "get_order_history": services.get_order_history,
    "get_call_history": services.get_call_history,
}

# --- API 동적 엔드포인트 ---
# 모든 POST 요청을 이 하나의 엔드포인트에서 처리합니다.
@router.post("/{action}")
async def handle_dynamic_request(action: str, request: RequestModel):
    # action 이름이 서비스 맵에 있는지 확인합니다.
    if action in service_map:
        # action에 맞는 서비스 함수를 호출하고 payload를 전달합니다.
        service_function = service_map[action]
        response = service_function(request.payload)
        return response
    
    # 정의되지 않은 action일 경우 에러를 반환합니다.
    return {"error": "Unknown action", "status_code": 404}

# 생성한 라우터를 앱에 포함
app.include_router(router)


# --- WebSocket 엔드포인트 ---
@app.websocket("/ws/guest/{room_id}")
async def websocket_endpoint(websocket: WebSocket, room_id: str):
    await websocket.accept()
    if room_id not in room_connections:
        room_connections[room_id] = []
    room_connections[room_id].append(websocket)
    print(f"새로운 WebSocket 연결: {room_id}")

    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        print(f"WebSocket 연결 해제됨: {room_id}")
    finally:
        if room_id in room_connections:
            room_connections[room_id].remove(websocket)
            if not room_connections[room_id]:
                del room_connections[room_id]

async def broadcast_event_to_room(room_id: str, event_data: dict):
    if room_id in room_connections:
        for connection in room_connections[room_id]:
            await connection.send_text(json.dumps(event_data))

# --- 주기적 알림 전송 (테스트용) ---
async def send_periodic_notifications():
    # 명세서에 정의된 이벤트 예시
    event_options = [
        {"type": "event", "action": "call_request_acceptance", "payload": {"task_name": "TASK_WEBSCKT", "estimated_wait_time": 15}},
        {"type": "event", "action": "robot_arrival_completion", "payload": {"task_name": "TASK_WEBSCKT", "location_name": "ROOM_102"}},
        {"type": "event", "action": "delivery_completion", "payload": {"task_name": "TASK_DELIVERY", "request_location": "ROOM_102"}},
        {"type": "event", "action": "task_timeout_return", "payload": {"task_name": "TASK_TIMEOUT", "location_name": "ROOM_102"}}
    ]
    while True:
        await asyncio.sleep(20)
        event_data = random.choice(event_options)
        # ROOM_102 클라이언트에게만 테스트 이벤트 전송
        await broadcast_event_to_room("ROOM_102", event_data)
        print(f"WebSocket 이벤트 전송 to ROOM_102: {event_data['action']}")

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(send_periodic_notifications())

if __name__ == "__main__":
    uvicorn.run(app, host=config.SERVER_HOST, port=config.SERVER_PORT)