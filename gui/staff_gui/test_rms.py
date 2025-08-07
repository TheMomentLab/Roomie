from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Path, Query
from fastapi.responses import JSONResponse
import asyncio
import json
import uvicorn
from typing import List, Dict
import threading
import time
import random

# [수정] message.md 명세에 맞게 API 라우터 사용
from fastapi import APIRouter
api_router = APIRouter(prefix="/api/gui")

app = FastAPI(title="Test RMS Server for Staff GUI")

<<<<<<< HEAD
@app.websocket("/api/gui/ws/admin/{staff_id}")
async def websocket_admin(websocket: WebSocket, staff_id: str):
    await websocket.accept()
    connected_clients.append(websocket)
    print(f"Staff '{staff_id}' 연결됨. 총 연결 수: {len(connected_clients)}")

=======
# WebSocket 연결 관리 (staff_id 별로 관리)
connected_clients: Dict[str, WebSocket] = {}

# --- WebSocket 통신 ---
@api_router.websocket("/ws/staff/{staff_id}")
async def websocket_endpoint(websocket: WebSocket, staff_id: str = Path(...)):
    """GUI 클라이언트의 WebSocket 연결을 처리합니다."""
    await websocket.accept()
    connected_clients[staff_id] = websocket
    print(f"✅ Staff GUI 연결됨 (ID: {staff_id}). 총 연결: {len(connected_clients)}")
    
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
<<<<<<< HEAD
        connected_clients.remove(websocket)
        print(f"Staff '{staff_id}' 연결 해제됨. 총 연결 수: {len(connected_clients)}")

        
@app.post("/api/gui/food_order_status_change")
=======
        del connected_clients[staff_id]
        print(f"❌ Staff GUI 연결 해제됨 (ID: {staff_id}). 총 연결: {len(connected_clients)}")

# --- HTTP API ---
@api_router.post("/food_order_status_change")
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
async def food_order_status_change(request_data: dict):
    """'준비완료' 상태 변경 요청을 처리합니다."""
    print(f"🔵 '준비완료' 요청 수신: {request_data}")
    
<<<<<<< HEAD
    # 요청 데이터 검증
    if request_data.get("type") == "request" and request_data.get("action") == "food_order_status_change":
        task_id = request_data.get("payload", {}).get("task_id")
        
        # task_id가 int인지 확인
        if task_id and isinstance(task_id, int):
            # 성공 응답 (인터페이스 명세에 맞춰 수정)
            response = {
                "type": "response",
                "action": "food_order_status_change",
                "payload": {
                    "task_id": task_id,  # int 타입 유지
                    "status_changed": "food_ready"
                }
=======
    payload = request_data.get("payload", {})
    task_id = payload.get("task_id")
    
    if request_data.get("type") == "request" and task_id:
        response = {
            "type": "response",
            "action": "food_order_status_change",
            "payload": {
                "task_id": task_id,
                "status_changed": "food_ready"
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
            }
        }
        print(f"🟢 '준비완료' 처리 완료 (Task ID: {task_id})")
        return JSONResponse(content=response)
    
    return JSONResponse(content={"error": "Invalid request"}, status_code=400)

async def send_to_all_clients(message: dict):
    """연결된 모든 클라이언트에게 메시지를 전송합니다."""
    if connected_clients:
        message_str = json.dumps(message, ensure_ascii=False)
        # 동시에 여러 클라이언트에 전송
        await asyncio.gather(
            *[client.send_text(message_str) for client in connected_clients.values()]
        )

# --- 테스트용 수동 이벤트 트리거 ---
@app.get("/test/create_order", tags=["Manual Testing"])
async def send_test_order():
    """테스트용 음식 주문 이벤트를 수동으로 전송합니다."""
    task_id = int(time.time())
    order_event = {
        "type": "event",
        "action": "food_order_creation",
        "payload": {
<<<<<<< HEAD
            "task_id": int(time.time()),  # int 타입으로 수정
            "request_location": "ROOM_307",
            "order_details": {
                "items": [
                    {
                        "name": "스파게티",
                        "quantity": 2,
                        "price": 15000
                    },
                    {
                        "name": "피자",
                        "quantity": 1,
                        "price": 15000
                    }
                ]
            }
=======
            "task_id": task_id,
            "request_location": "ROOM_505",
            "order_details": {"items": [{"name": "수동 테스트 버거", "quantity": 1, "price": 9900}]}
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
        }
    }
    await send_to_all_clients(order_event)
    return {"message": f"수동 주문 생성됨 (Task ID: {task_id})", "event": order_event}

<<<<<<< HEAD
@app.get("/send_test_supply")
async def send_test_supply():
    """테스트용 비품 요청 이벤트 전송"""
    supply_event = {
        "type": "event",
        "action": "supply_order_creation", 
        "payload": {
            "task_id": int(time.time()),  # int 타입
            "request_location": "ROOM_305",
            "request_details": {
                "items": [
                    {
                        "name": "타월",
                        "quantity": 3
                    },
                    {
                        "name": "생수",
                        "quantity": 2
                    }
                ]
            }
        }
    }
    
    await send_to_all_clients(supply_event)
    return {"message": "테스트 비품 요청 전송됨", "supply": supply_event}

@app.get("/send_robot_arrival")
async def send_robot_arrival():
    """테스트용 로봇 도착 이벤트 전송"""
    arrival_event = {
        "type": "event", 
        "action": "food_pickup_arrival",
        "payload": {
            "task_id": 1,  # int 타입으로 수정
            "robot_id": 1  # int 타입으로 수정 (인터페이스 명세에 맞춰)
        }
=======
@app.get("/test/pickup_arrival", tags=["Manual Testing"])
async def send_pickup_arrival(task_id: int = Query(..., description="픽업 도착 알림을 보낼 주문의 Task ID")):
    """지정된 주문에 대한 '픽업 도착' 이벤트를 전송합니다."""
    arrival_event = {
        "type": "event", 
        "action": "food_pickup_arrival",
        "payload": {"task_id": task_id, "robot_id": random.randint(1, 5)} # [수정] robot_id를 int로
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
    }
    await send_to_all_clients(arrival_event)
    return {"message": f"픽업 도착 알림 전송됨 (Task ID: {task_id})", "event": arrival_event}

<<<<<<< HEAD
@app.get("/send_supply_robot_arrival")
async def send_supply_robot_arrival():
    """테스트용 비품 로봇 도착 이벤트 전송"""
    arrival_event = {
        "type": "event", 
        "action": "supply_pickup_arrival",
        "payload": {
            "task_id": 2,  # int 타입
            "robot_id": 2  # int 타입
        }
    }
    
    await send_to_all_clients(arrival_event)
    return {"message": "비품 로봇 도착 알림 전송됨", "event": arrival_event}

def run_auto_orders():
    """자동으로 주문 생성 (백그라운드)"""
    import asyncio
    
    async def auto_order_loop():
        counter = 1
        while True:
            await asyncio.sleep(15)  # 15초마다
            
            if connected_clients:  # 연결된 클라이언트가 있을 때만
                # 음식 주문과 비품 요청을 번갈아 생성
                if counter % 2 == 1:
                    # 음식 주문
                    order_event = {
                        "type": "event",
                        "action": "food_order_creation", 
                        "payload": {
                            "task_id": counter,  # int 타입으로 수정
                            "request_location": f"ROOM_{300 + (counter % 20)}",
                            "order_details": {
                                "items": [
                                    {
                                        "name": ["스파게티", "피자", "스테이크", "버거"][counter % 4],
                                        "quantity": (counter % 3) + 1,
                                        "price": [15000, 18000, 25000, 12000][counter % 4]
                                    }
                                ]
                            }
                        }
                    }
                    await send_to_all_clients(order_event)
                    print(f"자동 음식 주문 전송: {counter}")
                else:
                    # 비품 요청
                    supply_event = {
                        "type": "event",
                        "action": "supply_order_creation",
                        "payload": {
                            "task_id": counter,  # int 타입
                            "request_location": f"ROOM_{300 + (counter % 20)}",
                            "request_details": {
                                "items": [
                                    {
                                        "name": ["칫솔", "타월", "생수", "수저"][counter % 4],
                                        "quantity": (counter % 3) + 1
                                    }
                                ]
                            }
                        }
                    }
                    await send_to_all_clients(supply_event)
                    print(f"자동 비품 요청 전송: {counter}")
                    
                counter += 1
    
    # 새 이벤트 루프에서 실행
    def run_in_thread():
=======
@app.get("/test/delivery_arrival", tags=["Manual Testing"])
async def send_delivery_arrival(task_id: int = Query(..., description="배달 완료 알림을 보낼 주문의 Task ID")):
    """[추가] 지정된 주문에 대한 '배달 완료' 이벤트를 전송합니다."""
    arrival_event = {
        "type": "event", 
        "action": "food_delivery_arrival", # [추가] 배달 완료 액션
        "payload": {"task_id": task_id, "robot_id": random.randint(1, 5)}
    }
    await send_to_all_clients(arrival_event)
    return {"message": f"배달 완료 알림 전송됨 (Task ID: {task_id})", "event": arrival_event}


# --- 자동 테스트 시나리오 ---
def run_auto_scenario_in_thread():
    """자동 테스트 시나리오를 별도 스레드에서 실행합니다."""
    async def auto_order_lifecycle():
        """주문 생성부터 배달 완료까지의 전체 과정을 자동으로 반복합니다."""
        print("🤖 자동 주문 시나리오 시작...")
        while True:
            await asyncio.sleep(15)  # 15초마다 새 주문 사이클 시작
            
            if not connected_clients:
                continue

            # 1. 주문 생성
            task_id = int(time.time())
            room_number = 300 + random.randint(1, 20)
            items = [{"name": "자동주문 스파게티", "quantity": random.randint(1,2), "price": 15000}]
            
            order_event = {
                "type": "event", "action": "food_order_creation",
                "payload": {"task_id": task_id, "request_location": f"ROOM_{room_number}", "order_details": {"items": items}}
            }
            await send_to_all_clients(order_event)
            print(f"🚀 자동 주문 생성 (Task ID: {task_id})")

            # 2. 픽업 도착 (10초 후)
            await asyncio.sleep(10)
            pickup_event = {
                "type": "event", "action": "food_pickup_arrival",
                "payload": {"task_id": task_id, "robot_id": random.randint(1, 5)}
            }
            await send_to_all_clients(pickup_event)
            print(f"🚚 로봇 픽업 도착 (Task ID: {task_id})")

            # 3. 배달 완료 (15초 후)
            await asyncio.sleep(15)
            delivery_event = {
                "type": "event", "action": "food_delivery_arrival",
                "payload": {"task_id": task_id, "robot_id": random.randint(1, 5)}
            }
            await send_to_all_clients(delivery_event)
            print(f"🎉 배달 완료 (Task ID: {task_id})")

    def run_loop():
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(auto_order_lifecycle())

    thread = threading.Thread(target=run_loop, daemon=True)
    thread.start()

# API 라우터를 메인 앱에 포함
app.include_router(api_router)

if __name__ == "__main__":
<<<<<<< HEAD
    print("🚀 Test RMS Server 시작...")
    print("📱 Staff GUI 테스트 가능:")
    print("   - http://localhost:8000/send_test_order (테스트 주문)")
    print("   - http://localhost:8000/send_test_supply (테스트 비품)")
    print("   - http://localhost:8000/send_robot_arrival (음식 로봇 도착)")
    print("   - http://localhost:8000/send_supply_robot_arrival (비품 로봇 도착)")
    print("   - 15초마다 자동 주문/비품 요청 생성")
    
    # 자동 주문 생성 시작
    run_auto_orders()
    
    # 서버 실행
    uvicorn.run(app, host="0.0.0.0", port=8000)
=======
    print("🚀 Test RMS Server (v2) 시작...")
    print("API 명세(message.md)에 따라 URL 및 이벤트 구조가 업데이트되었습니다.")
    print("이제 GUI와 연결하여 전체 주문 흐름을 자동으로 테스트할 수 있습니다.")
    print("\n🔗 WebSocket URL: ws://{host}:8800/api/gui/ws/staff/{staff_id}")
    print("📄 API 문서 (수동 테스트): http://127.0.0.1:8800/docs\n")
    
    run_auto_scenario_in_thread()
    uvicorn.run(app, host="0.0.0.0", port=8888)
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
