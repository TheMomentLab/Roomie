# test_server.py (종료 문제 해결된 최종본)

import asyncio
import json
import random
from contextlib import asynccontextmanager

import uvicorn
from fastapi import APIRouter, FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from starlette.websockets import WebSocketDisconnect

import config
import services
from models import RequestModel

# --- ✅ 1. 백그라운드 작업을 관리할 변수 ---
notification_task = None

# --- ✅ 2. startup/shutdown 이벤트를 관리하는 lifespan 컨텍스트 매니저 ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    global notification_task
    # 서버 시작 시 실행될 로직
    print("서버 시작: 주기적 알림 작업을 시작합니다.")
    notification_task = asyncio.create_task(send_periodic_notifications())
    yield
    # 서버 종료 시 실행될 로직
    print("서버 종료: 주기적 알림 작업을 중단합니다.")
    notification_task.cancel()
    try:
        await notification_task
    except asyncio.CancelledError:
        print("알림 작업이 성공적으로 취소되었습니다.")


# --- 앱 및 라우터 설정 (lifespan 추가) ---
app = FastAPI(lifespan=lifespan) # ⬅️ FastAPI 앱 생성 시 lifespan 등록
router = APIRouter(prefix="/api/gui")

# ... (기존 CORS, WebSocket, 서비스 매핑, API 엔드포인트 코드는 모두 동일) ...
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
service_map = {
    "get_food_menu": services.get_food_menu,
    "get_supply_menu": services.get_supply_menu,
    "create_delivery_task": services.create_delivery_task,
    "create_call_task": services.create_call_task,
    "get_task_list": services.get_task_list,
    "get_order_history": services.get_order_history,
    "get_call_history": services.get_call_history,
}

# --- API 동적 엔드포인트 ---
@router.post("/{action}")
async def handle_dynamic_request(action: str, request: RequestModel):
    if action in service_map:
        service_function = service_map[action]
        response = service_function(request.payload)
        return response
    return {"error": "Unknown action", "status_code": 404}

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
        connections = room_connections.get(room_id, [])
        for connection in connections:
            await connection.send_text(json.dumps(event_data))

# --- 주기적 알림 전송 (테스트용) ---
async def send_periodic_notifications():
    event_options = [
        {"type": "event", "action": "call_request_acceptance", "payload": {"task_name": "TASK_WEBSCKT", "estimated_wait_time": 15}},
        {"type": "event", "action": "robot_arrival_completion", "payload": {"task_name": "TASK_WEBSCKT", "location_name": "ROOM_102"}},
        {"type": "event", "action": "delivery_completion", "payload": {"task_name": "TASK_DELIVERY", "request_location": "ROOM_102"}},
        {"type": "event", "action": "task_timeout_return", "payload": {"task_name": "TASK_TIMEOUT", "location_name": "ROOM_102"}}
    ]
    while True:
        try:
            await asyncio.sleep(20)
            event_data = random.choice(event_options)
            await broadcast_event_to_room("ROOM_102", event_data)
            print(f"WebSocket 이벤트 전송 to ROOM_102: {event_data['action']}")
        except asyncio.CancelledError:
            break

# --- ✅ 3. 기존 @app.on_event("startup") 데코레이터 제거 ---
# 위 lifespan이 이 역할을 대체합니다.

if __name__ == "__main__":
    uvicorn.run(app, host=config.SERVER_HOST, port=config.SERVER_PORT)