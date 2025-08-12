import asyncio
import json
import random
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import APIRouter, FastAPI, WebSocket, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketDisconnect

import config
import services
from models import RequestModel

# === 경로 설정 (절대 경로 안전화) ==========================================
# test_server.py가 gui/guest_gui/test/test_server.py에 있을 때
# guest_gui_dir = .../gui/guest_gui
BASE_DIR = Path(__file__).resolve().parent.parent       # .../gui/guest_gui
ASSETS_DIR = BASE_DIR / "assets"                        # .../gui/guest_gui/assets
# ========================================================================

# --- 백그라운드 작업 관리 변수 ---
notification_task = None

# --- lifespan 컨텍스트 매니저 ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    global notification_task
    print("서버 시작: 주기적 알림 작업을 시작합니다.")
    notification_task = asyncio.create_task(send_periodic_notifications())
    yield
    print("서버 종료: 주기적 알림 작업을 중단합니다.")
    notification_task.cancel()
    try:
        await notification_task
    except asyncio.CancelledError:
        print("알림 작업이 성공적으로 취소되었습니다.")

# --- FastAPI 앱 생성 ---
app = FastAPI(lifespan=lifespan)

# ✅ 정적 파일 서빙(절대 경로 사용)
#   - /assets → 이미지 등 정적 리소스
#   - /front  → 프론트 전체( index.html, js, css )를 동일 오리진으로 서빙
app.mount("/assets", StaticFiles(directory=str(ASSETS_DIR)), name="assets")
app.mount("/front", StaticFiles(directory=str(BASE_DIR), html=True), name="front")

# --- CORS 설정 (이미지 태그에는 영향 적지만 일단 허용) ---
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- API 라우터 ---
router = APIRouter(prefix="/api/gui")

service_map = {
    "get_food_menu": services.get_food_menu,
    "get_supply_menu": services.get_supply_menu,
    "create_delivery_task": services.create_delivery_task,
    "create_call_task": services.create_call_task,
    "get_task_list": services.get_task_list,
    "get_order_history": services.get_order_history,
    "get_call_history": services.get_call_history,
}

@router.post("/{action}")
async def handle_dynamic_request(action: str, req_model: RequestModel, http_req: Request):
    """
    동적 API 액션 처리 + (중요) 이미지 URL 절대화
    - 프론트가 다른 포트에서 열려 있어도 이미지가 항상 8000으로 가도록 만듦
    """
    if action not in service_map:
        return {"error": "Unknown action", "status_code": 404}

    response = service_map[action](req_model.payload)

    # 이미지가 포함된 응답에 대해 절대 URL로 변환
    base_url = f"{http_req.url.scheme}://{http_req.headers.get('host')}"  # e.g., http://localhost:8000
    if action == "get_food_menu" and "payload" in response:
        items = response["payload"].get("food_items", [])
        for it in items:
            img = it.get("image")
            if isinstance(img, str):
                # '/assets/...' 형태면 절대 URL로 바꿔줌
                if img.startswith("/"):
                    it["image"] = base_url + img

    if action == "get_supply_menu" and "payload" in response:
        items = response["payload"].get("supply_items", [])
        for it in items:
            img = it.get("image")
            if isinstance(img, str) and img.startswith("/"):
                it["image"] = base_url + img

    return response

app.include_router(router)

# --- WebSocket 연결 관리 ---
room_connections: dict[str, list[WebSocket]] = {}

@app.websocket("/ws/guest/{room_id}")
async def websocket_endpoint(websocket: WebSocket, room_id: str):
    """WebSocket 연결 관리"""
    await websocket.accept()
    room_connections.setdefault(room_id, []).append(websocket)
    print(f"새로운 WebSocket 연결: {room_id}")
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        print(f"WebSocket 연결 해제됨: {room_id}")
    finally:
        lst = room_connections.get(room_id, [])
        if websocket in lst:
            lst.remove(websocket)
        if not lst and room_id in room_connections:
            del room_connections[room_id]

async def broadcast_event_to_room(room_id: str, event_data: dict):
    """특정 Room에 이벤트 전송"""
    for ws in room_connections.get(room_id, []):
        await ws.send_text(json.dumps(event_data))

async def send_periodic_notifications():
    """테스트용 주기적 WebSocket 이벤트"""
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

if __name__ == "__main__":
    uvicorn.run(app, host=config.SERVER_HOST, port=config.SERVER_PORT)