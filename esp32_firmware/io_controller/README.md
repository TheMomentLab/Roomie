# Roomie IOC ESP32 구현

ESP32를 사용하여 구현된 micro-ROS Roomie IOC Firmware

## 하드웨어 연결

### 핀 연결
- **서보모터**: GPIO 18번 핀
- **상태 LED**: GPIO 2번 핀 (내장 LED)
- **RGB LED**: R(GPIO 25), G(GPIO 26), B(GPIO 27)
- **문 상태 IR 센서**: GPIO 21 (디지털 출력 핀)
- **적재 감지 IR 센서**: GPIO 22 (디지털 출력 핀)

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

### 1. 파일 구성
```
io_controller/
├── platformio.ini                  # PlatformIO 설정
├── README.md
├── .gitignore
│
├── src/                            # 기능별로 모듈화된 ESP32 코드
│   ├── main.cpp
│   ├── controller/
│   │   ├── controller.cpp
│   │   └── controller.h
│   ├── services/
│   │   ├── service_manager.cpp
│   │   └── service_manager.h
│   └── topics/
│       ├── topic_manager.cpp
│       └── topic_manager.h
│
├── include/                        # 직접 작성한 공용 헤더
│   └── controller.h                # (필요 시 사용)
│
├── lib/                            # ROS2에서 생성된 헤더 복사본
│   └── roomie_msgs/
│       └── include/
│           └── roomie_msgs/
│               ├── msg/
│               │   └── robot_state.h
│               └── srv/
│                   ├── control_lock.h
│                   ├── check_door_state.h
│                   └── check_item_loaded.h
│
├── test/                           # 단위 테스트 (선택)
│   └── ...
│
├── docs/                           # 프로젝트 문서
│   └── ...
│
└── .pio/                           # PlatformIO 빌드 출력 디렉토리 (자동 생성)

```

### 2. 펌웨어 업로드
```bash
pio run --target upload
```

## ROS2 인터페이스

### **서비스 (Service)**

### 1. 잠금 제어
-  locked: true (잠금), false (해제)
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
- **메시지타입**: `roomie_msgs/msg/RobotState`
- **설명**: 로봇의 메인 시스템으로부터 상태 ID를 받아 Controller의 상태를 변경하고, 이에 맞게 LED 점등

## 센서 동작 방식

### 문 상태 IR 센서
- **동작**: 센서 앞에 문이 감지되면 LOW 신호 출력, 감지되지 않으면(문이 열리면) HIGH 신호 출력
- **판단**: digitalRead 결과가 HIGH이면 문이 열린 것으로 판단

### 적재 감지 IR 센서
- **설치 위치**: 내부 공간 천장, 바닥을 향해
- **자동 캘리브레이션**: 부팅 시 미적재 상태 기준 거리 측정
- **적재 감지**: 기준 거리보다 5cm 이상 가까워지면 적재 판단

## LED 상태 표시

- 로봇의 상태(`RobotState`)에 따라 RGB LED와 상태 LED가 다음과 같이 변경됨

| 상태 ID | 상태 이름 | RGB LED | 상태 LED |
|-|-|-|-|
| 0 | `INITIAL` | 빨간색 | 점멸 |
| 1 | `CHARGING` | 초록색 | 점등 |
| 2, 10, 11, 13, 21, 23 | `WAITING`, `PICKUP_MOVING`, `PICKUP_WAITING`, `DELIVERY_WAITING`, `GUIDE_WAITING`, `TARGET_SEARCHING` | 파란색 | 점등 |
| 12, 20, 22, 30, 31 | `DELIVERY_MOVING`, `CALL_MOVING`, `GUIDE_MOVING`, `RETURN_MOVING`, `ELEVATOR_RIDING` | 파란색  | 점멸 |
| 90 | `ERROR` | 빨간색 | 점등 |
| - | 알 수 없는 상태 (Default) | 노란색 | 점멸 |


## 캘리브레이션

### 적재 센서 자동 캘리브레이션
부팅 시 자동으로 실행되며, 미적재 상태에서 10회 측정하여 평균값을 기준 거리로 설정합니다.

시리얼 모니터에서 확인 가능:
```
적재 센서 캘리브레이션 완료 - 기준 거리: 25.30cm
```

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