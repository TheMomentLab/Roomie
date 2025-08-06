# 회원 사용자 GUI 서비스 메시지 메시지 유형

## 형식

* 각 메시지의 구조와 유형을 각 개의 가능성 가정을 고려해서 정보화
* `location_name` 및 `task_type_id/name` 가 규정된 정의가 있음
* ISO 8601 형식의 날짜형식(시간은 KST)

---

## 프로토콜: HTTP

## GUI API Spec Full

---

### 1. 호출 작업 생성

| 항목              | 내용                          |
| --------------- | --------------------------- |
| **HTTP Method** | POST                        |
| **URL**         | `/api/gui/create_call_task` |
| **Request**     |                             |

```json
{
  "type": "request",
  "action": "create_call_task",
  "payload": {
    "location_name": "ROOM_201",   // string
    "task_type_id": 2               // integer (0: 음식, 1: 비품, 2: 호출, 3: 길안내)
  }
}
```

\| **Response** |

```json
{
  "type": "response",
  "action": "create_call_task",
  "payload": {
    "location_name": "ROOM_201",     // string
    "task_name": "TASK_006",         // string
    "success": true,                  // boolean
    "error_code": null,               // integer or null
    "error_message": null,            // string or null
    "task_creation_time": "2025-07-22T16:42:16+09:00"  // string (ISO 8601)
  }
}
```

---

### 2. 호출 내역 조회

| 항목              | 내용                          |
| --------------- | --------------------------- |
| **HTTP Method** | POST                        |
| **URL**         | `/api/gui/get_call_history` |
| **Request**     |                             |

```json
{
  "type": "request",
  "action": "get_call_history",
  "payload": {
    "location_name": "ROOM_102",   // string
    "task_name": "TASK_006"        // string
  }
}
```

\| **Response** |

```json
{
  "type": "response",
  "action": "get_call_history",
  "payload": {
    "location_name": "ROOM_102",     // string
    "task_name": "TASK_006",         // string
    "task_type_name": "호출",         // string
    "estimated_time": 5,              // integer
    "robot_status": {
      "x": 0.2,                        // float
      "y": 1.2,                        // float
      "floor_id": 1                   // integer
    }
  }
}
```

---

### 3. 음식 메뉴 조회

| 항목              | 내용                       |
| --------------- | ------------------------ |
| **HTTP Method** | POST                     |
| **URL**         | `/api/gui/get_food_menu` |
| **Request**     |                          |

```json
{
  "type": "request",
  "action": "get_food_menu",
  "payload": {
    "location_name": "ROOM_201"     // string
  }
}
```

\| **Response** |

```json
{
  "type": "response",
  "action": "get_food_menu",
  "payload": {
    "food_items": [
      {
        "food_name": "스파게티",             // string
        "price": 15000,                     // integer
        "image": "data:image/jpeg;base64,..." // string
      },
      {
        "food_name": "피자",
        "price": 25000,
        "image": "data:image/jpeg;base64,..."
      }
    ]
  }
}
```

---

### 4. 비품 메뉴 조회

| 항목              | 내용                         |
| --------------- | -------------------------- |
| **HTTP Method** | POST                       |
| **URL**         | `/api/gui/get_supply_menu` |
| **Request**     |                            |

```json
{
  "type": "request",
  "action": "get_supply_menu",
  "payload": {
    "location_name": "ROOM_201"     // string
  }
}
```

\| **Response** |

```json
{
  "type": "response",
  "action": "get_supply_menu",
  "payload": {
    "supply_items": [
      {
        "supply_name": "타월",         // string
        "image": "data:image/jpeg;base64,..." // string
      },
      {
        "supply_name": "생수",
        "image": "data:image/jpeg;base64,..."
      }
    ]
  }
}
```

---

### 5. 배송 작업 생성

| 항목              | 내용                              |
| --------------- | ------------------------------- |
| **HTTP Method** | POST                            |
| **URL**         | `/api/gui/create_delivery_task` |
| **Request**     |                                 |

```json
{
  "type": "request",
  "action": "create_delivery_task",
  "payload": {
    "location_name": "ROOM_201",           // string
    "task_type_name": "음식배달",           // string
    "order_details": {
      "items": [
        {
          "name": "스파게티",              // string
          "quantity": 2,                   // integer
          "price": 15000                   // integer
        },
        {
          "name": "피자",
          "quantity": 1,
          "price": 15000
        }
      ]
    }
  }
}
```

\| **Response** |

```json
{
  "type": "response",
  "action": "create_delivery_task",
  "payload": {
    "location_name": "ROOM_201",     // string
    "task_name": "TASK_005",         // string
    "success": true,                  // boolean
    "error_code": null,               // integer or null
    "error_message": null,            // string or null
    "estimated_time": 55,             // integer
    "task_creation_time": "2025-07-22T16:42:16+09:00" // string (ISO 8601)
  }
}
```

---

### 6. 요청 조회

| 항목              | 내용                           |
| --------------- | ---------------------------- |
| **HTTP Method** | POST                         |
| **URL**         | `/api/gui/get_task_list` |
| **Request**     |                              |

```json
{
  "type": "request",
  "action": "get_task_list",
  "payload": {
    "request_location": "ROOM_201"        // string
  }
}
```

\| **Response** |

```json
{
  "type": "response",
  "action": "get_task_list",
  "payload": {
    "location_name": "ROOM_201",           // string
    "order_details": {
      "tasks": [
        {
          "task_name": "TASK_001",      //string
          "task_type_name": "음식배달",   //string
          "created_at": "2025-07-22T16:42:16+09:00"
        },
        {
          "task_name": "TASK_005",      //string
          "task_type_name": "로봇호출",   //string
          "created_at": "2025-07-22T16:42:16+09:00"
        }
      ]
    }
  }
}
```

---

### 7. 주문 내역 조회

| 항목              | 내용                           |
| --------------- | ---------------------------- |
| **HTTP Method** | POST                         |
| **URL**         | `/api/gui/get_order_history` |
| **Request**     |                              |

```json
{
  "type": "request",
  "action": "get_order_history",
  "payload": {
    "request_location": "ROOM_201",        // string
    "task_name": "TASK_001",               // string
    "task_type_name": "음식배달"            // string
  }
}
```

\| **Response** |

```json
{
  "type": "response",
  "action": "get_order_history",
  "payload": {
    "request_location": "ROOM_201",              // string
    "task_name": "TASK_001",                      // string
    "task_type_name": "음식배송",                  // string
    "estimated_time": 55,                          // integer
    "task_creation_time": "2025-07-22T16:42:16+09:00",
    "robot_assignment_time": "2025-07-22T16:42:16+09:00",
    "pickup_completion_time": "2025-07-22T16:42:16+09:00",
    "delivery_arrival_time": null                  // string (nullable)
  }
}
```

---

### 7. WebSocket 알림

#### A. 호출 수락 알림

| 항목            | 내용                                  |
| ------------- | ----------------------------------- |
| **Event**     | `call_request_acceptance`           |
| **Direction** | RMS → GGUI                          |
| **URL**       | `/api/gui/ws/admin/{location_name}` |
| **Payload**   |                                     |

```json
{
  "type": "event",
  "action": "call_request_acceptance",
  "payload": {
    "task_name": "TASK_006",         // string
    "estimated_wait_time": 15         // integer
  }
}
```

#### B. 로봇 도착 완료 알림

```json
{
  "type": "event",
  "action": "robot_arrival_completion",
  "payload": {
    "task_name": "TASK_006",         // string
    "location_name": "ROOM_102"      // string
  }
}
```

#### C. 배송 완료 알림

```json
{
  "type": "event",
  "action": "delivery_completion",
  "payload": {
    "task_name": "TASK_001",         // string
    "request_location": "ROOM_102"   // string
  }
}
```

#### D. 시간초과 복귀 알림

```json
{
  "type": "event",
  "action": "task_timeout_return",
  "payload": {
    "task_name": "TASK_006",         // string
    "location_name": "ROOM_102"      // string
  }
}
```


---

## 보조

### Location Mapping

```text
0: LOB_WAITING
1: LOB_CALL
2: RES_PICKUP
3: RES_CALL
4: SUP_PICKUP
5: ELE_1
6: ELE_2
101: ROOM_101
102: ROOM_102
201: ROOM_201
202: ROOM_202
```

### Task Type Mapping

```text
0: 음식배송
1: 비품배송
2: 호출
3: 길안내
```

### Food Type Mapping

```text
0: 스파게티
1: 피자
2: 스테이크
3: 버거
```
### Supply Type Mapping

```text
0: 칫솔
1: 타월
2: 생수
3: 수저
```