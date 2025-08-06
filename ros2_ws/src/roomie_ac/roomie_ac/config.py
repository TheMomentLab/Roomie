from pathlib import Path
import numpy as np
from enum import IntEnum
from ament_index_python.packages import get_package_share_directory
import os


# 1. Pose 종류를 관리하는 Enum 클래스 정의
class Pose(IntEnum):
    """로봇 팔의 미리 정의된 자세 종류"""
    INIT = 0
    OBSERVE = 1
    LEFT = 2
    RIGHT = 3
    FORWARD = 4
    UP= 5
    OBSERVE1 = 6
    OBSERVE2 = 7
    OBSERVE3 = 8

# 버튼 피드백 상태
class ButtonActionStatus:
    MOVING_TO_TARGET = "MOVING_TO_TARGET"
    ALIGNING_TO_TARGET = "ALIGNING_TO_TARGET"
    PRESSING = "PRESSING"
    RETRACTING = "RETRACTING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"

# 제어 전략 모드 정의
class ControlMode(IntEnum):
    """제어 전략 모드"""
    MODEL_ONLY = 0
    PBVS = 1 
    IBVS = 2

HAND_EYE_UNIT = 'mm'  # 또는 'm'

# 기본 설정 (DEBUG 등)
DEBUG = True

# 파일 경로 설정
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = os.path.join(get_package_share_directory('roomie_ac'), 'data')
URDF_FILE = os.path.join(get_package_share_directory('roomie_ac'), 'urdf', 'roomie2.urdf')
CAMERA_PARAMS_FILE = os.path.join(DATA_DIR, 'camera_params.npz')  
HAND_EYE_MATRIX_FILE = os.path.join(DATA_DIR, 'hand_eye_matrix.npy')
# 시리얼 통신 설정
SERIAL_PORT = "/dev/ttyUSB2"
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT = 5.0 # 시리얼 연결 및 응답 대기 타임아웃

# 서보 모터 및 관절 설정
SERVO_ZERO_OFFSET_DEG = np.array([90, 90, 90, 90])
SERVO_DIRECTION_MULTIPLIER = np.array([1, -1, -1, -1])
JOINT_LIMIT_DEG = np.array([[-90, 90], [-90, 90], [-90, 90], [-90, 90]])
JOINT_LIMIT_RAD = np.deg2rad(JOINT_LIMIT_DEG) # 라디안 변환
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4'] # RViz2 퍼블리싱을 위한 관절 이름

# IK (Inverse Kinematics) 설정
ACTIVE_LINKS_MASK = [False, True, True, True, True, False] # ikpy 체인에서 활성화할 링크 마스크
IK_MAX_ITERATIONS = 20000 # IK 최대 반복 횟수
IK_TOLERANCE_M = 2e-3 # IK 오차 허용 범위 (m)

# 워크스페이스 (작업 공간) 설정
WORKSPACE_R_MIN_M = -0.45 # 로봇 중심으로부터 최소/최대 반경 (m)
WORKSPACE_R_MAX_M = 0.45
WORKSPACE_Z_MIN_M = -0.40 # 로봇팔 끝점의 최소/최대 높이 (m)
WORKSPACE_Z_MAX_M = 0.402

# 동작 및 지연 시간 설정
COMEBACK_DELAY_SEC = 20.0 # 홈 포지션으로 복귀 대기 시간 (초)

# 제어 전략 설정
# ControlMode.MODEL_ONLY : 미리 정의된 좌표로 이동 (이미지 서보잉 없음)
# ControlMode.HYBRID     : 비전 기반 이미지 서보잉 사용 (기존 방식)
CONTROL_STRATEGY = ControlMode.PBVS

# 2. Enum을 키(key)로, 실제 각도값을 값(value)으로 갖는 딕셔너리 생성
POSE_ANGLES_DEG = {
    Pose.INIT: np.array([0, 40, 170, 30]),
    Pose.OBSERVE: np.array([90, 140, 168, 30]),
    Pose.OBSERVE1: np.array([85, 140, 168, 30]),
    Pose.OBSERVE2: np.array([95, 140, 168, 30]),
    Pose.OBSERVE3: np.array([90, 100, 170, 80]),

    Pose.LEFT: np.array([170, 120, 150, 30]),
    Pose.RIGHT: np.array([0, 130, 170, 30]),
    Pose.FORWARD: np.array([90, 140, 170, 30]),
    Pose.UP: np.array([90, 120, 137, 30]),

}
# 기존 홈 포지션 변수도 이 딕셔너리를 활용할 수 있습니다.
HOME_POSITION_SERVO_DEG = POSE_ANGLES_DEG[Pose.INIT]

#  버튼 재시도 횟수
MAX_CLICK_ATTEMPTS = 3


# --- 카메라 및 인식 설정 ---
CAMERA_DEVICE_ID = 8  # 사용자의 카메라 장치 번호
YOLO_MODEL_PATH = '/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/data/best.pt'



# --- 로봇 및 버튼 ---
ROBOT_ID = 0
# 사전 정의된 버튼 위치 (미터 단위, MODEL_ONLY 모드용)
PREDEFINED_BUTTON_POSES_M = {
    0: np.array([0.242, 0.0, 0.305]),  # 예시 좌표 (button_id: 2)
    1: np.array([0.242, 0.0, 0.235]),  
    2: np.array([0.242, 0.0, 0.235]), 
    3: np.array([0.242, 0.0, 0.235]),  
    4: np.array([0.242, 0.0, 0.235]),  
    5: np.array([0.242, 0.0, 0.19]), 
    6: np.array([0.242, 0.0, 0.19]),  
    101: np.array([0.242, 0.0, 0.305]), 
    102: np.array([0.242, 0.0,0.095]),  
    # 다른 버튼 ID와 좌표를 여기에 추가할 수 있습니다.
    # 3: np.array([0.25, -0.1, 0.15]),
}
REAL_BUTTON_DIAMETER_M = 0.035 # 3.5cm
# --- 비전 ---
IMAGE_WIDTH_PX = 800
IMAGE_HEIGHT_PX = 600
# 중요: 실제 보정 파일 경로로 수정하세요
CAMERA_PARAMS_FILE = '/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/data/camera_params.npz'
HAND_EYE_MATRIX_FILE = '/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/data/hand_eye_matrix.npy'
HAND_EYE_UNIT = 'mm' # 핸드-아이 보정 시 사용한 단위, 'mm' 또는 'm'

# PnP를 위한 원형 버튼의 3D 모델 포인트 (대칭성 보장)
OBJECT_POINTS_3D = np.array([
    [ REAL_BUTTON_DIAMETER_M / 2,  0.0, 0.0], # 버튼 오른쪽
    [-REAL_BUTTON_DIAMETER_M / 2,  0.0, 0.0], # 버튼 왼쪽
    [ 0.0,  REAL_BUTTON_DIAMETER_M / 2, 0.0], # 버튼 위쪽
    [ 0.0, -REAL_BUTTON_DIAMETER_M / 2, 0.0], # 버튼 아래쪽
], dtype=np.float32)

# PnP RANSAC 파라미터
PNPR_REPROJ_ERROR_THRESHOLD_PX = 8.0
PNPR_MIN_INLIERS = 3

# --- 이미지 서보잉 ---
SERVOING_STANDBY_DISTANCE_M = 0.09 # 버튼으로부터 5cm 대기 거리
PRESS_FORWARD_DISTANCE_M = 0.09 # 3cm 누르기 거리
SERVOING_MAX_MOVE_M = 0.014# 멀리서 버튼에 접근할 때의 최대 이동 스텝 (2cm)

# 서보잉 시 한 스텝에 이동할 거리의 비율 (0.0 ~ 1.0)
# 0.4는 목표까지 남은 거리의 40%만큼만 이동하라는 의미입니다.
# 이 값을 조절하여 로봇의 접근 속도와 안정성을 튜닝할 수 있습니다.

# [수정] IBVS 모드에서 전/후진 시 사용할 고정 이동 거리
SERVOING_CONSTANT_STEP_M = 0.014 # 매 스텝마다 1cm씩 이동

# [IBVS 전용 설정값]
IBVS_PIXEL_TOLERANCE = 10          # 목표 픽셀 오차 허용 범위 (10픽셀)
IBVS_GAIN_X = 0.00005              # X축 (좌/우) 오차에 대한 이동량 게인
IBVS_GAIN_Y = 0.00005              # Y축 (상/하) 오차에 대한 이동량 게인
# IBVS_GAIN_Z는 더 이상 사용하지 않음
IBVS_TARGET_SIZE_RATIO = 0.35      # 목표로 할 버튼의 크기 (이미지 높이의 25%)