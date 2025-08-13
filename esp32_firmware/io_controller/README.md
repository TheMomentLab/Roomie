# Roomie IOC ESP32 Firmware

ESP32를 사용하여 구현된 micro-ROS Roomie IOC Firmware

## 하드웨어 연결

### 핀 연결
- **서보모터**: GPIO 13
- **상태 LED**: GPIO 2 (내장 LED)
- **RGB LED**: R(GPIO 25), G(GPIO 26), B(GPIO 27)
- **문 감지 초음파 센서**: TRIG(GPIO 32), ECHO(GPIO 33)
- **적재 감지 초음파 센서**: TRIG(GPIO 12), ECHO(GPIO 14)
- **RFID 카드 리더**: RST(GPIO 22), SS(GPIO 5), SDA(GPIO 21), SCK(GPIO 18), MOSI(GPIO 23), MISO(GPIO 19)

### 초음파 센서 연결
```
초음파 센서    ESP32
-----------    -----
VCC            5V
GND            GND
TRIG           GPIO 32 / 12
ECHO           GPIO 33 / 14
```

### 서보모터 연결
```
서보모터    ESP32
--------    -----
빨간선      5V
갈색선      GND
주황선      GPIO 13
```

### RFID 카드 리더 연결
```
RFID 리더    ESP32
----------    -----
VCC           3.3V
GND           GND
RST           GPIO 22
SDA           GPIO 5
SCK           GPIO 18
MOSI          GPIO 23
MISO          GPIO 19
```

## 소프트웨어 설정

### 1. 소스 파일 구성
`io_controller` 디렉토리의 전체 구조는 다음과 같습니다.
```
io_controller/
├── platformio.ini         # PlatformIO 프로젝트 설정
├── colcon.meta            # Colcon 빌드 시스템 메타 파일
├── README.md              # 프로젝트 설명 및 가이드
├── src/                   # 메인 소스 코드
│   ├── main.cpp           # 어플리케이션 메인 진입점
│   ├── controller/        # 하드웨어 제어 로직
│   │   ├── controller.h
│   │   └── controller.cpp
│   ├── services/          # ROS 2 서비스 관리
│   │   ├── service_manager.h
│   │   └── service_manager.cpp
│   └── topics/            # ROS 2 토픽 관리
│       ├── topic_manager.h
│       └── topic_manager.cpp
├── include/               # 공용 헤더 파일
├── lib/                   # 외부 라이브러리
├── test/                  # 테스트 코드
└── extra_packages/        # 외부 ROS 2 패키지 (colcon.meta 참조)
```

### 2. 펌웨어 업로드
```bash
pio run --target upload
```

## ROS2 인터페이스

### micro-ROS Agent 실행
터미널을 열고 아래 명령어를 실행하여 ESP32와 ROS2를 연결합니다.
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### **서비스 (Service)**

### 1. 잠금 제어
- `locked`: true (잠금, 서보모터 0도), false (해제, 서보모터 90도)
```bash
ros2 service call /ioc/control_lock roomie_msgs/srv/ControlLock "{robot_id: 0, locked: true}"
```

### 2. 문 상태 확인
- `is_opened`: 문이 열려있으면 true, 닫혀있으면 false
```bash
ros2 service call /ioc/check_door_state roomie_msgs/srv/CheckDoorState "{robot_id: 0}"
```

### 3. 적재 상태 확인
- `item_loaded`: 아이템이 적재되어 있으면 true, 아니면 false
```bash
ros2 service call /ioc/check_item_loaded roomie_msgs/srv/CheckItemLoaded "{robot_id: 0}"
```

### **토픽 (Topics)**

### 1. 로봇 상태 수신 (구독)
- **토픽 이름**: `/roomie/status/robot_state`
- **메시지 타입**: `roomie_msgs/msg/RobotState`
- **설명**: 로봇의 메인 시스템으로부터 상태 ID를 받아 Controller의 상태를 변경하고, 이에 맞게 LED를 점등합니다.

#### **토픽 발행 예시**
- **충전 상태** (초록색 점등)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 0, robot_state_id: 1}'
```
- **작업 대기** (노란색 점등)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 0, robot_state_id: 2}'
```
- **배송 이동** (파란색 점등)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 0, robot_state_id: 12}'
```
- **오류 상태** (빨간색 점등)
```bash
ros2 topic pub --once /roomie/status/robot_state roomie_msgs/msg/RobotState '{robot_id: 0, robot_state_id: 90}'
```

### 2. 카드 읽기 요청 (구독)
- **토픽 이름**: `/ioc/read_card_request`
- **메시지 타입**: `roomie_msgs/msg/ReadCardRequest`
- **설명**: RC(Robot Controller)로부터 카드 읽기 요청을 받아 RFID 카드 리더로 카드를 읽습니다.

#### **토픽 발행 예시**
```bash
ros2 topic pub --once /ioc/read_card_request roomie_msgs/msg/ReadCardRequest '{robot_id: 0}'
```

### 3. 카드 읽기 응답 (발행)
- **토픽 이름**: `/ioc/read_card_response`
- **메시지 타입**: `roomie_msgs/msg/ReadCardResponse`
- **설명**: 카드 읽기 결과를 RC로 전송합니다.

#### **토픽 구독 예시**
```bash
ros2 topic echo /ioc/read_card_response
```

## 센서 동작 방식

### 문 감지 초음파 센서
- **동작**: 센서와 문 사이의 거리를 측정합니다.
- **판단**: 측정된 거리가 `5.0cm`를 초과하면 문이 열린 것으로 판단합니다.

### 적재 감지 초음파 센서
- **동작**: 내부 공간 천장에 설치되어 바닥까지의 거리를 측정합니다.
- **판단**: 측정된 거리가 `25.0cm` 미만이면 물건이 적재된 것으로 판단합니다.

### RFID 카드 리더
- **동작**: MFRC522 모듈을 사용하여 RFID 카드의 UID를 읽습니다.
- **카드 데이터**: 카드의 블록 4에 저장된 4바이트 데이터를 location_id로 해석합니다.
- **응답**: 카드 읽기 성공 시 `success=true`, `location_id=읽은값`, 실패 시 `success=false`, `location_id=-1`

## LED 상태 표시

로봇의 상태(`RobotState`)에 따라 RGB LED와 상태 LED가 다음과 같이 변경됩니다.

| 상태 ID | 상태 이름 | RGB LED | 상태 LED |
|---|---|---|---|
| 0 | `INITIAL` | 청록색 | 점등 |
| 1 | `CHARGING` | 초록색 | 점등 |
| 2, 11, 13, 21, 23 | `WAITING`, `PICKUP_WAITING`, `DELIVERY_WAITING`, `GUIDE_WAITING`, `DESTINATION_SEARCHING` | 노란색 | 점등 |
| 10, 12, 20, 22, 30, 31 | `PICKUP_MOVING`, `DELIVERY_MOVING`, `CALL_MOVING`, `GUIDE_MOVING`, `RETURN_MOVING`, `ELEVATOR_RIDING` | 파란색 | 점등 |
| 90 | `ERROR` | 빨간색 | 점등 |

## 문제 해결

### 초음파 센서 문제
1. 전원 공급 확인 (VCC, GND)
2. TRIG/ECHO 핀 연결 확인
3. 센서 방향 및 장애물 확인
4. 시리얼 모니터에서 거리 측정값 확인

### 서보모터 문제
1. 전원 공급 확인 (5V)
2. PWM 신호선 연결 확인
3. 각도 범위 확인 (잠금 0도, 해제 90도)

### RFID 카드 리더 문제
1. SPI 핀 연결 확인 (SDA, SCK, MOSI, MISO)
2. 전원 공급 확인 (3.3V)
3. 카드 인식 거리 확인 (1-2cm 이내)
4. 카드 데이터 형식 확인 (4바이트 정수)

### 통신 문제
1. micro-ROS agent가 정상적으로 실행 중인지 확인
2. 네트워크 연결 상태 확인
3. 토픽 이름과 메시지 타입 확인
