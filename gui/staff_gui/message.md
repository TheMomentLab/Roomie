📡 ROOMIE SGUI 인터페이스 명세서 (Markdown)
📘 HTTP API
🔄 음식 주문 작업 상태 전환
Method: POST

URL: /api/gui/food_order_status_change

📤 요청 데이터 (Request)
json
복사
편집
{
  "type": "request",                         // string
  "action": "food_order_status_change",      // string
  "payload": {
    "task_id": 12                            // int
  }
}
📥 응답 데이터 (Response)
json
복사
편집
{
  "type": "response",                        // string
  "action": "food_order_status_change",      // string
  "payload": {
    "task_id": 12,                           // int
    "status_changed": "food_ready"          // string
  }
}
📡 WebSocket API
🌐 WebSocket URL
swift
복사
편집
/api/gui/ws/staff/{staff_id}
📦 음식 주문 발생 알림
Direction: RMS → SGUI

Event Type: food_order_creation

json
복사
편집
{
  "type": "event",                          // string
  "action": "food_order_creation",         // string
  "payload": {
    "task_id": 12,                          // int
    "request_location": "ROOM_307",        // string
    "order_details": {
      "items": [                            // array of objects
        {
          "name": "스파게티",              // string
          "quantity": 2,                   // int
          "price": 15000                   // int
        },
        {
          "name": "피자",                  // string
          "quantity": 1,                   // int
          "price": 15000                   // int
        }
      ]
    }
  }
}
🤖 음식 픽업 장소 도착 알림
Direction: RMS → SGUI

Event Type: food_pickup_arrival

json
복사
편집
{
  "type": "event",                          // string
  "action": "food_pickup_arrival",          // string
  "payload": {
    "task_id": 12,                          // int
    "robot_id": 1                           // int
  }
}

🤖 비품 배송 장소 도착 알림
Direction: RMS → SGUI

Event Type: supply_delivery_arrival

json
복사
편집
{
  "type": "event",                          // string
  "action": "supply_delivery_arrival",        // string
  "payload": {
    "task_id": 12,                          // int
    "robot_id": 1                           // int
  }
}



📦 비품 요청 발생 알림
Direction: RMS → SGUI

Event Type: supply_order_creation

json
복사
편집
{
  "type": "event",                          // string
  "action": "supply_order_creation",        // string
  "payload": {
    "task_id": 12,                          // int
    "request_location": "ROOM_307",        // string
    "request_details": {
      "items": [                            // array of objects
        {
          "name": "타월",                  // string
          "quantity": 3                    // int
        },
        {
          "name": "생수",                  // string
          "quantity": 2                    // int
        }
      ]
    }
  }
}
🤖 비품 픽업 장소 도착 알림
Direction: RMS → SGUI

Event Type: supply_pickup_arrival

json
복사
편집
{
  "type": "event",                          // string
  "action": "supply_pickup_arrival",        // string
  "payload": {
    "task_id": 12,                          // int
    "robot_id": 1                           // int
  }
}
🧾 참고 데이터 타입
필드명	타입	설명
type	string	"event" 또는 "request"/"response"
action	string	이벤트 또는 요청 종류
task_id	int	작업 고유 ID
robot_id	int	로봇 고유 ID
request_location	string	호실 번호
name	string	항목 이름
quantity	int	수량
price	int	가격

