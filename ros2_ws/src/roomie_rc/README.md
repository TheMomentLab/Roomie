# Roomie RC (Robot Control) Module

로봇 제어 Python 모듈 - 서비스 로봇의 중심 제어 노드

## 📁 구조

- `rc_node.py` - 메인 로봇 제어 노드 (중심 허브)
- `rms_client.py` - RMS(Roomie Main Service) 통신 클라이언트
- `gui_client.py` - RGUI(Robot GUI) 통신 클라이언트  
- `nav_client.py` - Nav2 내비게이션 통신 클라이언트
- `vs_client.py` - VS(Vision Service) 통신 클라이언트
- `ioc_client.py` - IOC(IO Controller) 통신 클라이언트
- `arm_client.py` - Arm 제어 통신 클라이언트
- `location_manager.py` - 위치 정보 관리
- `config.py` - 설정 파일

## 🚀 주요 기능

### ✅ 구현 완료
- **완전한 워크플로우**: 시작 → 픽업 → 적재 → 배송 → 수령확인 → 복귀
- **상태 머신**: 30개 이상의 세밀한 상태 관리
- **RMS 통신**: 작업 할당, 상태 발행, 이벤트 처리
- **GUI 통신**: 서비스/토픽 양방향 통신, 사용자 인터랙션
- **하드웨어 통신**: VS(위치 감지), IOC(서랍/센서 제어), Arm(팔 제어)
- **내비게이션**: Nav2 기반 자율 주행
- **에러 처리**: 각 단계별 에러 상황 대응
- **재시작 기능**: 중간 상태에서 작업 재개 가능

### 🔄 TODO
- 실제 센서 데이터 연동
- 배터리 모니터링
- 다중 로봇 지원

## 📊 상태 관리 시스템

### RC 상태 (RCState)
로봇 제어 노드의 내부 상태 머신입니다. 총 30개 이상의 세밀한 상태로 구성됩니다.

#### 1. 시작 단계 (2개)
- `IDLE` - 대기 상태
- `TASK_ASSIGNED` - 작업 할당됨

#### 2. 픽업 단계 (8개)
- `PICKUP_MOVING` - 픽업 이동 중
- `COUNTDOWN_START` - 출발 카운트다운 시작
- `COUNTDOWN_COMPLETE` - 카운트다운 완료
- `NAVIGATION` - 내비게이션 중
- `PICKUP_ARM_ROTATING` - 픽업 시 팔 회전
- `PICKUP_LOCATION_CHECK` - 픽업 시 위치 확인
- `PICKUP_ARM_RETURN` - 픽업 시 팔 복귀
- `ARRIVED` - 목적지 도착

#### 3. 물품 적재 단계 (6개)
- `DRAWER_OPENING` - 서랍 열기 중
- `DRAWER_OPENED` - 서랍 열림 완료
- `DOOR_CHECK` - 서랍 열림 상태 확인 중
- `ITEM_CHECK` - 물품 적재 여부 확인 중
- `DRAWER_CLOSING` - 서랍 닫기 중

#### 4. 배송 단계 (6개)
- `DELIVERY_COUNTDOWN` - 배송 출발 카운트다운
- `DELIVERY_MOVING` - 배송지로 이동 중
- `DELIVERY_ARM_ROTATING` - 배송 시 팔 회전
- `DELIVERY_LOCATION_CHECK` - 배송 시 위치 확인
- `DELIVERY_ARM_RETURN` - 배송 시 팔 복귀
- `DELIVERY_COMPLETE` - 배송 도착 완료

#### 5. 수령 확인 단계 (5개)
- `UNLOAD_DRAWER_OPENING` - 수령 확인용 서랍 열기 중
- `UNLOAD_DRAWER_OPENED` - 수령 확인용 서랍 열림 완료
- `UNLOAD_DOOR_CHECK` - 수령 확인용 서랍 열림 상태 확인 중
- `UNLOAD_ITEM_CHECK` - 수령 확인용 물품 적재 여부 확인 중
- `UNLOAD_DRAWER_CLOSING` - 수령 확인용 서랍 닫기 중

#### 6. 복귀 단계 (3개)
- `RETURN_COUNTDOWN` - 복귀 카운트다운
- `RETURN_MOVING` - 복귀 이동 중
- `RETURN_COMPLETE` - 복귀 완료

### 로봇 상태 (Robot State)
로봇의 작업 할당 가능 여부를 나타내는 상태입니다.

- **2**: 대기 상태 (IDLE)
- **10**: 픽업 위치 이동 중
- **11**: 픽업 완료
- **12**: 배송 중
- **13**: 배송 완료
- **30**: 복귀 중

## 📡 통신 인터페이스

### RMS 통신
- **Action**: `/roomie/action/perform_task` (작업 할당 받기)
- **Publisher**: 로봇 상태, 배터리, 위치

### GUI 통신  
- **Topic**: `/robot_gui/event` (양방향 이벤트)
- **Action**: `/robot_gui/action/start_countdown` (출발 카운트다운)
- **Action**: `/robot_gui/action/return_countdown` (복귀 카운트다운)

### 내비게이션 통신
- **Action**: `/navigate_to_pose` (Nav2 내비게이션)

### 하드웨어 통신
- **VS**: 위치 감지 (`/vs/command/location`)
- **IOC**: 서랍 제어 (`/ioc/control_lock`), 상태 확인 (`/ioc/check_door_state`, `/ioc/check_item_loaded`)
- **Arm**: 팔 제어 (`/arm/action/set_pose`)

## 🏃 실행 방법

```bash
# 빌드
colcon build --packages-select roomie_rc

# 실행  
source install/setup.bash
ros2 run roomie_rc rc_node

# 테스트용 GUI 도구 실행
python3 src/roomie_rc_test/roomie_rc_test/gui_test_tool.py
```

## 🧪 테스트

### IOC 연동 테스트
1. RC 노드 실행: `ros2 run roomie_rc rc_node`
2. GUI 테스트 도구 실행: `python3 src/roomie_rc_test/roomie_rc_test/gui_test_tool.py`
3. GUI에서 버튼 클릭하여 이벤트 전송
4. RC 노드 로그에서 상태 변화 확인

### 상태별 테스트
- **104 버튼**: 서랍 열기 요청
- **105 버튼**: 적재 완료 확인
- **100 버튼**: 수령 확인 