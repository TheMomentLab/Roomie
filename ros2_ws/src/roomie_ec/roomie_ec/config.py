#!/usr/bin/env python3
"""
Roomie EC Configuration
엘리베이터 컨트롤러 설정값들을 관리합니다.

=== 시나리오 구조 ===
시나리오 1: 위치 확인 ~ 팔 원위치 완료
- INIT → CHECKING_LOCATION → LOCATION_CONFIRMED → SETTING_ELEVATOR_MODE 
→ ELEVATOR_MODE_SET → SENDING_GUI_WARNING → TRACKING_BUTTON → BUTTON_REACHED 
→ CLICKING_BUTTON → BUTTON_CLICKED → RETURNING_ARM → ARM_RETURNED

시나리오 1 → 2 사이: 후진 이동 ~ 엘리베이터 중앙 이동 (주행 부분)
- ARM_RETURNED → BACKING_UP → BACKUP_COMPLETED → MOVING_TO_ELEVATOR_CENTER 
→ ELEVATOR_CENTER_REACHED → SCENARIO_2_READY

시나리오 2: 엘리베이터 도착 판단 및 탑승 (미구현)
- SCENARIO_2_READY → (시나리오 2 로직)

=== 디버그 모드 ===
- debug_mode=true: 통신 테스트 모드 (실제 주행 없이 서비스/액션 호출만)
- debug_mode=false: 실제 주행 모드 (TF 기반 실제 거리 측정 및 주행)
"""

# ===== Debug Configuration =====
DEBUG_MODE = True  # True: 통신 테스트 모드, False: 실제 주행 모드
START_STATE = "INIT"  # 시나리오 시작 상태 설정

# ===== Robot Configuration =====
ROBOT_ID = 0
TASK_ID = 100
SCENARIO_ID = 1

# ===== Location Configuration =====
LOCATION_ID = 5  # 도착지 (ELE_1)
CURRENT_FLOOR = 5
TARGET_FLOOR = 6

# ===== VS Mode Configuration =====
ELEVATOR_EXTERNAL_MODE = 3  # 엘리베이터 외부 모드
ELEVATOR_INTERIOR_MODE = 4  # 엘리베이터 내부 모드
NORMAL_DRIVING_MODE = 5  # 일반 주행 모드 (시나리오 4용)

# ===== Service Topics =====
VS_LOCATION_SERVICE = '/vs/command/location'
VS_SET_MODE_SERVICE = '/vs/command/set_vs_mode'
VS_BUTTON_STATUS_SERVICE = '/vs/command/button_status'
VS_ELEVATOR_STATUS_SERVICE = '/vs/command/elevator_status'  # 시나리오 2용
VS_DOOR_STATUS_SERVICE = '/vs/command/door_status'  # 시나리오 2용

# ===== Action Topics =====
ARM_CLICK_BUTTON_ACTION = '/arm/action/click_button'
ARM_SET_POSE_ACTION = '/arm/action/set_pose'

# ===== Publisher Topics =====
GUI_EVENT_TOPIC = '/robot_gui/event'
CMD_VEL_TOPIC = '/cmd_vel'

# ===== Button Configuration =====
TARGET_BUTTON_ID = 0  # 상행버튼
INTERIOR_BUTTON_ID = 6  # 6층 버튼
BUTTON_TRACKING_TIMER_HZ = 10.0  # 10Hz (0.1초마다)

# ===== Post-Scenario 1 Movement Configuration =====
BACKUP_DISTANCE = 0.3  # 후진 거리 (미터)
BACKUP_SPEED = 0.15     # 후진 속도 (m/s)
BACKUP_TIMEOUT = 10.0  # 후진 타임아웃 (초)

# ===== Simple Navigator Configuration =====
SIMPLE_GOAL_TOPIC = '/simple_goal_pose'  # simple_navigator2 목표 토픽

# ===== 가상 test =====
ELEVATOR_CENTER_X = -2.0  # 엘리베이터 중앙 X 좌표 5.25
ELEVATOR_CENTER_Y = 1.0  # 엘리베이터 중앙 Y 좌표 -3.0
ELEVATOR_CENTER_YAW = 0.0  # 엘리베이터 중앙 방향 (라디안) -1.57
ELEVATOR_INTERIOR_X = 0.0  # 엘리베이터 내부 X 좌표
ELEVATOR_INTERIOR_Y = 1.0   # 엘리베이터 내부 Y 좌표
ELEVATOR_INTERIOR_YAW = 3.14 # 엘리베이터 내부 방향 (라디안)
ELEVATOR_EXIT_X = -2.0  # 엘리베이터 외부 X 좌표 (6층)
ELEVATOR_EXIT_Y = 1.0   # 엘리베이터 외부 Y 좌표 (6층)
ELEVATOR_EXIT_YAW = -1.57 # 엘리베이터 외부 방향 (라디안)

# ===== 실내 test =====
# ELEVATOR_CENTER_X = -0.2  # 엘리베이터 중앙 X 좌표 5.25
# ELEVATOR_CENTER_Y = 1.0  # 엘리베이터 중앙 Y 좌표 -3.0
# ELEVATOR_CENTER_YAW = 1.57  # 엘리베이터 중앙 방향 (라디안) -1.57
# ELEVATOR_INTERIOR_X = -0.2  # 엘리베이터 내부 X 좌표
# ELEVATOR_INTERIOR_Y = 2.0   # 엘리베이터 내부 Y 좌표
# ELEVATOR_INTERIOR_YAW = -1.57 # 엘리베이터 내부 방향 (라디안)
# ELEVATOR_EXIT_X = -0.2  # 엘리베이터 외부 X 좌표 (6층)
# ELEVATOR_EXIT_Y = 1.0   # 엘리베이터 외부 Y 좌표 (6층)
# ELEVATOR_EXIT_YAW = 0.0 # 엘리베이터 외부 방향 (라디안)

# ===== Elevator Center Position (시나리오 0.5용) =====
# ELEVATOR_BUTTON_X = 3.95  # 엘리베이터 중앙 X 좌표 5.25
# ELEVATOR_BUTTON_Y = -4.0  # 엘리베이터 중앙 Y 좌표 -3.0
# ELEVATOR_BUTTON_YAW = -1.57  # 엘리베이터 중앙 방향 (라디안) -1.57

# # ===== Elevator Center Position (시나리오 1.5용) =====
# ELEVATOR_CENTER_X = 5.53  # 엘리베이터 중앙 X 좌표 5.25
# ELEVATOR_CENTER_Y = -4.5  # 엘리베이터 중앙 Y 좌표 -3.0
# ELEVATOR_CENTER_YAW = -1.70  # 엘리베이터 중앙 방향 (라디안) -1.57

# # ===== Elevator Interior Position (시나리오 2용) =====
# ELEVATOR_INTERIOR_X = 5.53  # 엘리베이터 내부 X 좌표
# ELEVATOR_INTERIOR_Y = -7.4   # 엘리베이터 내부 Y 좌표
# ELEVATOR_INTERIOR_YAW = 2.5 # 엘리베이터 내부 방향 (라디안)

# # ===== Elevator Exit Position (시나리오 4용) =====
# ELEVATOR_EXIT_X = -5.53  # 엘리베이터 외부 X 좌표 (6층)
# ELEVATOR_EXIT_Y = -4.0   # 엘리베이터 외부 Y 좌표 (6층)
# ELEVATOR_EXIT_YAW = 1.57 # 엘리베이터 외부 방향 (라디안)



# ===== GUI Event Configuration =====
GUI_EVENT_BUTTON_OPERATION_START = 1  # 엘리베이터 버튼 조작 시작
GUI_EVENT_MOVEMENT_START = 2  # 이동 시작
GUI_EVENT_ELEVATOR_BOARDING_START = 3  # 엘리베이터 탑승 시작 (시나리오 2용)
GUI_EVENT_ELEVATOR_EXIT_START = 5  # 엘리베이터 하차 시작 (시나리오 4용)

# ===== Timeout Configuration =====
LOCATION_CHECK_TIMEOUT = 15.0  # 15초 타임아웃
ELEVATOR_STATUS_TIMEOUT = 120.0  # 엘리베이터 상태 확인 타임아웃 (2분)
DOOR_STATUS_TIMEOUT = 30.0  # 문 상태 확인 타임아웃 (30초)
ELEVATOR_ARRIVAL_TIMEOUT = 180.0  # 엘리베이터 도착 확인 타임아웃 (3분, 시나리오 4용)

# ===== Elevator Status Constants =====
ELEVATOR_DIRECTION_UPWARD = 0
ELEVATOR_DIRECTION_DOWNWARD = 1
DOOR_STATUS_CLOSED = False
DOOR_STATUS_OPENED = True

# ===== Timer Configuration =====
SCENARIO_TIMER_HZ = 1.0  # 1Hz
LOCATION_TIMER_HZ = 1.0  # 1Hz (1초마다)

# ===== Control Parameters =====
TARGET_BUTTON_SIZE = 0.06  # 목표 버튼 크기     # 0.06
TARGET_BUTTON_X = 0.5      # 목표 x좌표 (화면 중앙)
MIN_BUTTON_CONFIDENCE = 0.5  # 최소 신뢰도

# 속도 제한
MAX_LINEAR_SPEED = 0.15   # 최대 선속도 (m/s)
MIN_LINEAR_SPEED = 0.03 # 최소 선속도 (m/s)
MAX_ANGULAR_SPEED = 0.25  # 최대 각속도 (rad/s)

# 제어 게인
KP_X = 0.5    # x좌표 제어 게인
KP_SIZE = 1.0 # 크기 제어 게인

# 목표 달성 조건
TARGET_SIZE_ERROR = 0.005  # 크기 오차 허용 범위
TARGET_X_ERROR = 0.05      # x좌표 오차 허용 범위 