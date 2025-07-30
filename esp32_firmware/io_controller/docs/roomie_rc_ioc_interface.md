# Roomie RC-IOC ROS 2 Interface Specification

> **Robot Controller (RC) = Raspberry Pi**  
> **IO Controller (IOC) = ESP32 with micro-ROS**

---

## 메시지 및 서비스 패키지

- 공용 패키지: `roomie_msgs`

---

## ROS 2 Services (IOC: Server)

### `/ioc/control_lock`  
**Type:** `roomie_msgs/srv/ControlLock`

```srv
# ControlLock.srv

# Request
int32 robot_id
bool locked

---

# Response
int32 robot_id
bool success
```

- 설명: `locked`가 `true`이면 잠금, `false`이면 해제
- 동작: 서보모터 회전으로 물리적 잠금 제어

---

### `/ioc/read_card_info`  
**Type:** `roomie_msgs/srv/ReadCardInfo`

```srv
# ReadCardInfo.srv

# Request
int32 robot_id

---

# Response
int32 robot_id
bool success
int32 location_id
```

- 설명: 카드 리더로부터 UID 읽기 요청
- 동작: RC가 요청하면 IOC가 카드 UID 응답

---

### `/ioc/check_door_state`  
**Type:** `roomie_msgs/srv/CheckDoorState`

```srv
# CheckDoorState.srv

# Request
int32 robot_id

---

# ResponseGetItemState
int32 robot_id
bool is_opened
```

- 설명: 서랍 문이 열려 있는지 여부 확인

---

### `/ioc/check_item_loaded`  
**Type:** `roomie_msgs/srv/CheckItemLoaded`

```srv
# CheckItemLoaded.srv

# Request
int32 robot_id

---

# Response
int32 robot_id
bool item_loaded
```

- 설명: 적재 감지 센서 (IR)로 물건 유무 확인

---

## ROS 2 Topics (IOC: Subscriber)

### `/roomie/status/robot_state`  
**Type:** `roomie_msgs/msg/RobotState`

```msg
# RobotState.msg

int32 robot_id
int32 robot_state_id
```

- 설명: RC의 상태 전파
- 동작: LED 색상 또는 알림 방식 제어

---

## 동작 예시 흐름

1. RC → `/ioc/control_lock` (lock = true) → IOC: 서랍 잠금
2. RC → `/ioc/read_card_info` 요청 → IOC: UID 반환
3. RC → `/roomie/status/robot_state` = "DELIVERING" → IOC: 파란 LED 점등

---

## 의존 패키지

- `roomie_msgs`
  - 포함된 `.msg`, `.srv` 파일
  - ROS 2 및 micro-ROS 양측에서 동일 버전 유지
