# Roomie IOC ESP32 Firmware

ESP32를 사용하여 구현된 micro-ROS Roomie IOC Firmware

## 하드웨어 연결

### 핀 연결
- **서보모터**: GPIO 18
- **상태 LED**: GPIO 2 (내장 LED)
- **RGB LED**: R(GPIO 25), G(GPIO 26), B(GPIO 27)
- **문 상태 IR 센서**: GPIO 21 (디지털 출력)
- **적재 감지 IR 센서**: GPIO 22 (디지털 출력)

### IR 센서 연결
```
IR 센서         ESP32
-----------    -----
VCC            3.3V / 5V
GND            GND
DO (Data Out)  GPIO 21 / 22
```

### 서보모터 연결
```
서보모터    ESP32
--------    -----
빨간선      5V
갈색선      GND
주황선      GPIO 18
```

## 소프트웨어 설정

### 1. 소스 파일 구성
```
src/
├── main.cpp
├── controller/
│   ├── controller.cpp
│   └── controller.h
├── services/
│   ├── service_manager.cpp
│   └── service_manager.h
└── topics/
    ├── topic_manager.cpp
    └── topic_manager.h
```

### 2. 펌웨어 업로드
```bash
pio run --target upload
```

## ROS2 인터페이스

### micro-ROS Agent 실행

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### **서비스 (Service)**

### 1. 잠금 제어
-  `locked`: true (잠금), false (해제)
```bash
ros2 service call /ioc/control_lock roomie_msgs/srv/ControlLock "{robot_id: 1, locked: true}"
```

### 2. 문 상태 확인
```bash
ros2 service call /ioc/check_door_state roomie_msgs/srv/CheckDoorState "{robot_id: 1}"
```

### 3. 적재 상태 확인
```bash
ros2 service call /ioc/check_item_loaded roomie_msgs/srv/CheckItemLoaded "{robot_id: 1}"
```

### **구독 토픽 (Subscribed Topic)**

### 1. 로봇 상태 수신
- **토픽 이름**: `/roomie/status/robot_state`
- **메시지 타입**: `roomie_msgs/msg/RobotState`
- **설명**: 로봇의 메인 시스템으로부터 상태 ID를 받아 Controller의 상태를 변경하고, 이에 맞게 LED를 점등합니다.

- **충전상태** (초록색 점등)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 1, robot_state_id: 1}'
```
- **대기상태** (하늘색 점등)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 1, robot_state_id: 2}'
```
- **이동중** (파란색 점멸)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 1, robot_state_id: 12}'
```
- **에러** (빨간색 점등)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 1, robot_state_id: 90}'
```

## LED 상태 표시

- 로봇의 상태(`RobotState`)에 따라 RGB LED와 상태 LED가 다음과 같이 변경됩니다.

| 상태 ID | 상태 이름 | RGB LED | 상태 LED |
|---|---|---|---|
| 0 | `INITIAL` | 빨간색 | 점등 |
| 1 | `CHARGING` | 초록색 | 점등 |
| 2, 10, 11, 13, 21, 23 | `WAITING`, `PICKUP_WAITING`, `DELIVERY_WAITING`, `GUIDE_WAITING`, `DESTINATION_SEARCHING` | 초록색 | 깜빡임 |
| 12, 20, 22, 30, 31 | `PICKUP_MOVING`, `DELIVERY_MOVING`, `CALL_MOVING`, `GUIDE_MOVING`, `RETURN_MOVING`, `ELEVATOR_RIDING` | 파란색 | 점등 |
| 90 | `ERROR` | 빨간색 | 점등 |

## 문제 해결

### IR 센서 문제
1. 전원 공급 확인 (VCC, GND)
2. 핀 연결 확인
3. 센서 방향 및 장애물 확인

### 서보모터 문제
1. 전원 공급 확인 (5V)
2. PWM 신호 확인
3. 각도 범위 조정

### 통신 문제
1. micro-ROS agent 실행 확인
2. 시리얼 연결 및 Baud rate 확인
3. 네트워크 설정 확인
