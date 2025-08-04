# Roomie Elevator Controller (EC)

## 📋 **프로젝트 개요**
5층에서 6층으로 이동하는 엘리베이터 제어 로봇 시스템

## 🎯 **시나리오 구조**

### **시나리오 1: 5층 → 엘리베이터 호출** ✅ **완료**
1. **팔 정면 설정** (`SETTING_ARM_FORWARD`)
   - 로봇 팔을 정면으로 설정 (pose_id=3)
   - 위치 확인 전에 팔 방향 조정

2. **위치 확인** (`CHECKING_LOCATION`)
   - VS로부터 현재 위치 확인 (아루코 마커 인식)
   - 목표 위치 도달 시 다음 단계로 진행

3. **엘리베이터 모드 설정** (`SETTING_ELEVATOR_MODE`)
   - VS에 엘리베이터 모드 설정 요청
   - GUI 경고 메시지 발행

4. **버튼 추적** (`TRACKING_BUTTON`)
   - VS로부터 버튼 상태 모니터링
   - 버튼 인식 시 다음 단계로 진행

5. **버튼 클릭** (`CLICKING_BUTTON`)
   - VS에 버튼 클릭 액션 요청
   - 클릭 완료 시 다음 단계로 진행

6. **팔 원위치** (`RETURNING_ARM`)
   - VS에 팔 원위치 액션 요청
   - 원위치 완료 시 다음 단계로 진행

7. **이동 이벤트 발송** (`SENDING_MOVEMENT_EVENT`)
   - GUI에 이동 시작 이벤트 발송 (event_id=2)
   - 사용자에게 이동 시작 알림

8. **후진 이동** (`BACKING_UP`)
   - 전광판 인식을 위한 후진 이동 (0.4m)
   - 시간 기반 거리 계산 (cmd_vel + 시간)
   - 디버그 모드: 시뮬레이션, 실제 모드: 실제 이동

9. **엘리베이터 중앙 이동** (`MOVING_TO_ELEVATOR_CENTER`)
   - simple_navigator2를 사용한 PID 제어 이동
   - 목표: (-2.0, 1.4) 좌표
   - 완료 신호 수신 시 다음 단계로 진행

10. **시나리오 2 준비** (`SCENARIO_2_READY`)
   - 시나리오 1 완료 및 시나리오 2 준비 완료

### **시나리오 2: 엘리베이터 탑승** ✅ **완료**
1. **엘리베이터 도착 판단** (`CHECKING_ELEVATOR_STATUS`)
   - VS로부터 엘리베이터 상태 모니터링 (2분 타임아웃)
   - 현재 층과 엘리베이터 층 일치, 방향 일치 시 다음 단계로 진행

2. **문 열림 대기** (`WAITING_FOR_DOOR_OPEN`)
   - VS로부터 문 상태 모니터링 (30초 타임아웃)
   - 문이 열리면 다음 단계로 진행

3. **탑승 이벤트 발송** (`SENDING_BOARDING_EVENT`)
   - GUI에 엘리베이터 탑승 시작 이벤트 발송 (event_id=3)

4. **엘리베이터 내부 이동** (`MOVING_TO_ELEVATOR_INTERIOR`)
   - Nav2를 사용하여 엘리베이터 내부로 이동
   - 목표: config.py에서 설정 가능한 좌표
   - 디버그 모드: 시뮬레이션, 실제 모드: 실제 Nav2 이동

5. **엘리베이터 내부 도달** (`ELEVATOR_INTERIOR_REACHED`)
   - 엘리베이터 내부 도달

6. **시나리오 3 준비** (`SCENARIO_3_READY`)
   - 시나리오 2 완료 및 시나리오 3 준비 완료

### **시나리오 3: 엘리베이터 내부 버튼 클릭** ✅ **완료**
1. **엘리베이터 내부 모드 설정** (`SETTING_ELEVATOR_INTERIOR_MODE`)
   - VS에 엘리베이터 내부 모드 설정 요청 (mode_id=4)
   - 내부 버튼 인식을 위한 모드 전환

2. **내부 GUI 경고** (`SENDING_INTERIOR_GUI_WARNING`)
   - GUI에 엘리베이터 내부 버튼 조작 시작 이벤트 발송 (event_id=1)
   - 사용자에게 팔 움직임 경고

3. **내부 버튼 클릭** (`CLICKING_INTERIOR_BUTTON`)
   - VS에 6층 버튼 클릭 액션 요청 (button_id=6)
   - 추적 없이 바로 클릭

4. **팔 상향 설정** (`SETTING_ARM_UPWARD`)
   - VS에 팔 상향 액션 요청 (pose_id=4)
   - 전광판 인식을 위한 상향 자세

5. **내부 이동 이벤트 발송** (`SENDING_INTERIOR_MOVEMENT_EVENT`)
   - GUI에 내부 이동 시작 이벤트 발송 (event_id=2)

6. **시나리오 4 준비** (`SCENARIO_4_READY`)
   - 시나리오 3 완료 및 시나리오 4 준비 완료

### **시나리오 4: 목표 층 도착 후 하차** ✅ **완료**
1. **엘리베이터 도착 확인** (`CHECKING_ELEVATOR_ARRIVAL`)
   - VS로부터 엘리베이터 상태 모니터링 (3분 타임아웃)
   - 목적지 층(6층) 도착 확인 시 다음 단계로 진행

2. **문 열림 대기** (`WAITING_FOR_DOOR_OPEN_EXIT`)
   - VS로부터 문 상태 모니터링 (30초 타임아웃)
   - 6층에서 문이 열리면 다음 단계로 진행

3. **하차 이벤트 발송** (`SENDING_EXIT_EVENT`)
   - GUI에 엘리베이터 하차 시작 이벤트 발송 (event_id=5)
   - 사용자에게 하차 시작 알림

4. **엘리베이터 하차** (`EXITING_ELEVATOR`)
   - Nav2를 사용하여 엘리베이터 외부로 이동
   - 목표: config.py에서 설정 가능한 좌표 (6층 외부)
   - 디버그 모드: 시뮬레이션, 실제 모드: 실제 Nav2 이동

5. **VS 모드 복원** (`RESTORING_NORMAL_MODE`)
   - VS를 일반 주행 모드로 복원 (mode_id=5)
   - 시나리오 완료
## 🔧 **디버그 모드**

### **통신 테스트 모드** (`debug_mode:=true`)
- 실제 로봇 이동 없이 통신만 테스트
- 후진 이동: 시뮬레이션 완료
- simple_navigator2: 시뮬레이션 완료
- 빠른 반복 테스트 가능

### **실제 주행 모드** (`debug_mode:=false`)
- 실제 로봇 이동 및 센서 데이터 사용
- 후진 이동: 실제 cmd_vel 발행
- simple_navigator2: 실제 PID 제어
- Nav2: 실제 네비게이션 실행

## 🚀 **실행 방법**

### **전체 시나리오 테스트 (디버그 모드)**
```bash
jazzy && source install/setup.bash && ros2 launch roomie_ec test_communication_launch.py debug_mode:=true start_state:=INIT
```

### **전체 시나리오 테스트 (실제 모드)**
```bash
jazzy && source install/setup.bash && ros2 launch roomie_ec test_communication_launch.py debug_mode:=false start_state:=INIT
```

### **부분 테스트 (후진부터)**
```bash
jazzy && source install/setup.bash && ros2 launch roomie_ec test_communication_launch.py debug_mode:=false start_state:=BACKING_UP
```

### **부분 테스트 (엘리베이터 중앙 이동만)**
```bash
jazzy && source install/setup.bash && ros2 launch roomie_ec test_communication_launch.py debug_mode:=false start_state:=MOVING_TO_ELEVATOR_CENTER
```

## 📁 **주요 파일 구조**

### **roomie_ec_node.py** - 메인 컨트롤러
- 시나리오 상태 머신 관리
- VS 서비스/액션 클라이언트
- 후진 이동 로직 (시간 기반)
- simple_navigator2 모니터링

### **simple_navigator2.py** - PID 제어 이동
- 엘리베이터 중앙으로의 정밀 이동
- 상태: `rotate_to_goal` → `move_to_goal` → `rotate_to_final` → `idle`
- `/simple_nav/status` 토픽으로 완료 신호 발행
- idle 상태에서는 cmd_vel 발행하지 않음

### **test_communication_node.py** - 모의 VS 서버
- VS 서비스/액션 모의 응답
- start_state에 따른 조건부 응답
- 디버그 모드에서만 실행
- 시나리오 2용 ElevatorStatus, DoorStatus 서비스 제공

### **test_communication_launch.py** - 실행 설정
- debug_mode, start_state 파라미터 관리
- 조건부 노드 실행

## 🔄 **상태 머신**

```
INIT → SETTING_ARM_FORWARD → CHECKING_LOCATION → SETTING_ELEVATOR_MODE → 
TRACKING_BUTTON → CLICKING_BUTTON → RETURNING_ARM → SENDING_MOVEMENT_EVENT → 
BACKING_UP → MOVING_TO_ELEVATOR_CENTER → SCENARIO_2_READY → CHECKING_ELEVATOR_STATUS → 
WAITING_FOR_DOOR_OPEN → SENDING_BOARDING_EVENT → MOVING_TO_ELEVATOR_INTERIOR → 
ELEVATOR_INTERIOR_REACHED → SCENARIO_3_READY → SETTING_ELEVATOR_INTERIOR_MODE → 
SENDING_INTERIOR_GUI_WARNING → CLICKING_INTERIOR_BUTTON → SETTING_ARM_UPWARD → 
SENDING_INTERIOR_MOVEMENT_EVENT → SCENARIO_4_READY → CHECKING_ELEVATOR_ARRIVAL → 
ELEVATOR_ARRIVED_EXIT → WAITING_FOR_DOOR_OPEN_EXIT → DOOR_OPENED_EXIT → 
SENDING_EXIT_EVENT → EXITING_ELEVATOR → ELEVATOR_EXIT_COMPLETED → 
RESTORING_NORMAL_MODE → NORMAL_MODE_RESTORED → COMPLETED
```

## ⚠️ **해결된 문제들**

### **cmd_vel 충돌 문제** ✅
- **문제**: roomie_ec_node와 simple_navigator2가 동시에 cmd_vel 발행
- **해결**: simple_navigator2가 idle 상태일 때 cmd_vel 발행하지 않도록 수정

### **완료 신호 수신 문제** ✅
- **문제**: simple_navigator2 완료 시 roomie_ec_node가 신호를 받지 못함
- **해결**: handle_rotate_to_final에서 즉시 "completed" 메시지 발행

### **후진 이동 로직** ✅
- **문제**: TF 기반 복잡한 거리 계산
- **해결**: 시간 기반 간단한 거리 계산 (cmd_vel × 시간)

## 🎯 **다음 단계**

### **테스트 완료된 부분**
- ✅ 시나리오 1 전체 (실제 주행 포함)
- ✅ 시나리오 2 전체 (디버그 모드 포함)
- ✅ 디버그 모드 / 실제 모드 전환
- ✅ 부분 테스트 (start_state 활용)
- ✅ 노드 간 통신 및 동기화
- ✅ Nav2 통합 (엘리베이터 내부 이동)

### **다음 단계**
- ✅ 시나리오 3: 엘리베이터 내부 버튼 클릭 (완료)
- ✅ 시나리오 4: 목표 층 도착 후 하차 (완료)
- 🎉 **전체 시나리오 완료**: 5층 → 6층 엘리베이터 이동 시스템

## 📝 **주요 파라미터**

### **config.py**
```python
DEBUG_MODE = True  # True: 통신 테스트, False: 실제 주행
START_STATE = "INIT"  # 시나리오 시작 상태
```

### **실행 시 파라미터**
```bash
debug_mode:=true/false  # 디버그 모드 설정
start_state:=INIT/BACKING_UP/MOVING_TO_ELEVATOR_CENTER  # 시작 상태
``` 