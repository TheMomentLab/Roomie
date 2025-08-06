# Vision Service Interface Specification

## 1. Service Interfaces

### 1.1 인식 모드 전환 요청
- **From**: RC → VS
- **Protocol**: ROS2 Service
- **Topic**: `/vs/command/set_vs_mode`

```srv
# SetVSMode.srv
# Request
int32 robot_id
int32 mode_id
---
# Response
int32 robot_id
bool success
```

**mode_id 값:**
- `0`: 대기모드 (후방 전용)
- `1`: 등록모드 (후방 전용)
- `2`: 추적모드 (후방 전용)
- `3`: 엘리베이터 외부 모드 (전방 전용)
- `4`: 엘리베이터 내부 모드 (전방 전용)
- `5`: 일반 주행모드 (전방 전용)
- `6`: 대기모드 (전방 전용)

**버튼 인식 방식:**
- **button_recog_1 (모드 3 - 엘리베이터 외부)**
  - 상하 위치 기반 분류
  - 위에 있는 버튼: 상행버튼 (button_id: 101)
  - 아래에 있는 버튼: 하행버튼 (button_id: 100)
- **button_recog_2 (모드 4 - 엘리베이터 내부)**
  - 버튼 군집 형성 후 위치에 따른 층수 분류
  - 군집 내 위치 기반으로 층수 버튼 매핑 (button_id: 1~14)
  - 층수 범위: 1~12층, B1층(13), B2층(14)
  - 특수 버튼: 열기(102), 닫기(103)

**사용 모델:**
- **모드 3, 4 (엘리베이터)**: `model_elevator.pt` 또는 `best.pt`
  - 감지 클래스: button, direction_light, display, door
- **모드 5 (일반 주행)**: `model_normal.pt` 또는 COCO 사전훈련 모델
  - 감지 클래스: person, chair, door
- **기타 모드**: 객체 감지 비활성화

**ArUco 마커 위치 감지:**
- **활성화**: 모드 5 (일반 주행) 에서만
- **비활성화**: 모드 0, 1, 2, 3, 4, 6
- **사용 목적**: 로봇 현재 위치 감지 및 location_id 반환

---

### 1.2 버튼 상태 감지 요청
- **From**: RC → VS
- **Protocol**: ROS2 Service
- **Topic**: `/vs/command/button_status`

```srv
# ButtonStatus.srv
# Request
int32 robot_id
int32 button_id
---
# Response
int32 robot_id
int32 button_id
bool success
float32 x
float32 y
float32 size
bool is_pressed
builtin_interfaces/Time timestamp
```

**x**: `0~1`
**y**: `0~1`  
**size**: `0~1`

**button_id 값:**
- `0`: (현재 유일하게 감지되는 버튼)
- 버튼이 2개 이상 감지될 경우 success=false
- `1`: 1층
- `2`: 2층
- `3`: 3층
- `4`: 4층
- `5`: 5층
- `6`: 6층
- `7`: 7층
- `8`: 8층
- `9`: 9층
- `10`: 10층
- `11`: 11층
- `12`: 12층
- `13`: B1층
- `14`: B2층
- `100`: 하행버튼
- `101`: 상행버튼
- `102`: 열기버튼
- `103`: 닫기버튼

---

### 1.3 엘리베이터 위치 및 방향 감지 요청
- **From**: RC → VS
- **Protocol**: ROS2 Service
- **Topic**: `/vs/command/elevator_status`

```srv
# ElevatorStatus.srv
# Request
int32 robot_id
---
# Response
int32 robot_id
bool success
int32 direction
int32 position
```

**direction 값:**
- `0`: upward
- `1`: downward

**position 값:**
- 엘리베이터 현재 층

---

### 1.4 문 열림 감지 요청
- **From**: RC → VS
- **Protocol**: ROS2 Service
- **Topic**: `/vs/command/door_status`

```srv
# DoorStatus.srv
# Request
int32 robot_id
---
# Response
int32 robot_id
bool success
bool door_opened
```

**door_opened 값:**
- `0`: closed
- `1`: opened
- 문 감지 여부로 판단

---

### 1.5 현재 위치 감지 결과
- **From**: RC → VS
- **Protocol**: ROS2 Service
- **Topic**: `/vs/command/location`

```srv
# Location.srv
# Request
int32 robot_id
---
# Response
int32 robot_id
bool success
int32 location_id
```

**location_id 값:**

| id | name |
|----|------|
| 0 | LOB_WAITING |
| 1 | LOB_CALL |
| 2 | RES_PICKUP |
| 3 | RES_CALL |
| 4 | SUP_PICKUP |
| 5 | ELE_1 |
| 6 | ELE_2 |
| 101 | ROOM_101 |
| 102 | ROOM_102 |
| 201 | ROOM_201 |
| 202 | ROOM_202 |

---

## 2. Topic Interfaces

### 2.1 추적 이벤트
- **From**: VS → RC
- **Protocol**: ROS2 Topic
- **Topic**: `/vs/tracking_event`

```msg
# TrackingEvent.msg
int32 robot_id
int32 tracking_event_id
int32 task_id
builtin_interfaces/Time timestamp
```

**tracking_event_id 값:**
- `0`: slow_down
- `1`: maintain
- `2`: lost
- `3`: resume

---

### 2.2 추적 대상 등록됨
- **From**: VS → RC
- **Protocol**: ROS2 Topic
- **Topic**: `/vs/registered`

```msg
# Registered.msg
int32 robot_id
builtin_interfaces/Time timestamp
```

---

### 2.3 장애물 감지 결과
- **From**: VS → RC
- **Protocol**: ROS2 Topic
- **Topic**: `/vs/obstacle`

```msg
# Obstacle.msg
int32 robot_id
bool dynamic
float32 x
float32 y
```

**dynamic 값:**
- `False`: 정적 장애물
- `True`: 동적 장애물

---

## 3. 시스템 아키텍처

### 3.1 멀티 카메라 시스템
Vision Service는 3개의 카메라를 사용합니다:

1. **전방 웹캠** (camera_id=0) - 로봇팔 부착
   - 사용 모드: 3, 4, 6
   - 용도: 엘리베이터 버튼 감지, 전방 대기
   
2. **뎁스 카메라** (OpenNI2/Astra)
   - 사용 모드: 5 (일반 주행에서 웹캠과 함께 사용)
   - 용도: 거리 측정, 장애물 감지
   
3. **후방 웹캠** (camera_id=1) - 후방 부착
   - 사용 모드: 0, 1, 2
   - 용도: 추적 대상 등록 및 추적

### 3.2 다중 YOLO 모델 시스템
모드에 따라 자동으로 적절한 YOLO 모델을 선택합니다:

- **일반 주행용** (`model_normal.pt` 또는 COCO 모델)
  - 감지 객체: 사람, 의자, 유리문
  - 백업: YOLOv8n COCO 사전훈련 모델
  
- **엘리베이터용** (`model_elevator.pt` 또는 `best.pt`)
  - 감지 객체: 버튼, 방향등, 디스플레이, 문

### 3.3 조건부 기능 활성화
리소스 최적화를 위해 모드별로 필요한 기능만 활성화합니다:

- **ArUco 마커 감지**: 모드 5에서만 활성화
- **YOLO 객체 감지**: 모드 3, 4, 5에서만 활성화
- **추적 이벤트 발행**: 모드 2에서만 가능
- **등록 이벤트 발행**: 모드 1에서만 가능

---

## 4. 메시지 파일 위치

모든 서비스 및 메시지 정의는 `roomie_msgs` 패키지에 정의되어 있습니다:

- **Services**: `roomie_msgs/srv/`
  - `robot_control/SetVSMode.srv`
  - `robot_control/Location.srv`
  - `door_elevator/DoorStatus.srv`
  - `door_elevator/ElevatorStatus.srv`
  - `sensor/ButtonStatus.srv`
- **Messages**: `roomie_msgs/msg/`
  - `robot_status/Obstacle.msg`
- **Actions**: `roomie_msgs/action/`
 

