# roomie_arm_control/config.py

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
    UPUP = 6

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
    IBVS = 2 # IBVS 모드 추가 (사실상 새로운 PBVS가 이 역할을 수행)

# 기본 설정
DEBUG = True
CONTROL_STRATEGY = ControlMode.PBVS # 시각 서보잉을 기본 전략으로 사용

# --- [신규] IBVS (Image-Based Visual Servoing) 제어 파라미터 ---
IBVS_MAX_ITERATIONS = 50              # 시각 서보잉 최대 반복 횟수
IBVS_POSITION_TOLERANCE_M = 0.005   # 5mm 이내로 근접 시 성공으로 간주
IBVS_IK_RETRY_COUNT = 3             # IK 실패 시 재시도 횟수
IBVS_ADAPTIVE_GAIN_MIN = 0.1        # 이동 게인 최소값 (오차가 작을 때)
IBVS_ADAPTIVE_GAIN_MAX = 0.4        # 이동 게인 최대값 (오차가 클 때)
IBVS_LOOP_DELAY_SEC = 0.05          # 각 제어 루프 사이의 최소 지연 시간

# 파일 경로 설정
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = os.path.join(get_package_share_directory('roomie_ac'), 'data')
URDF_FILE = os.path.join(get_package_share_directory('roomie_ac'), 'urdf', 'roomie2.urdf')
CAMERA_PARAMS_FILE = os.path.join(DATA_DIR, 'camera_params.npz')
HAND_EYE_MATRIX_FILE = os.path.join(DATA_DIR, 'hand_eye_matrix.npy')

# 시리얼 통신 설정
SERIAL_PORT = "/dev/ttyUSB1"
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT = 10.0

# 서보 모터 및 관절 설정
SERVO_ZERO_OFFSET_DEG = np.array([90, 90, 90, 90])
SERVO_DIRECTION_MULTIPLIER = np.array([1, -1, -1, -1])
JOINT_LIMIT_DEG = np.array([[-90, 90], [-90, 90], [-90, 90], [-90, 90]])
JOINT_LIMIT_RAD = np.deg2rad(JOINT_LIMIT_DEG)
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

# IK (Inverse Kinematics) 설정
ACTIVE_LINKS_MASK = [False, True, True, True, True, False]
IK_MAX_ITERATIONS = 10000 # 반복 횟수를 약간 줄여도 무방
IK_TOLERANCE_M = 6e-3

# 자세 설정
POSE_ANGLES_DEG = {
    Pose.INIT: np.array([0, 40, 170, 30]),
    Pose.OBSERVE: np.array([90, 140, 168, 30]),
    Pose.LEFT: np.array([170, 120, 150, 30]),
    Pose.RIGHT: np.array([0, 130, 170, 30]),
}
HOME_POSITION_SERVO_DEG = POSE_ANGLES_DEG[Pose.INIT]

# --- 로봇 및 버튼 ---
ROBOT_ID = 0
PREDEFINED_BUTTON_POSES_M = {
    0: np.array([0.235, 0.0, 0.305]),
    # ... 다른 버튼 좌표
}
REAL_BUTTON_DIAMETER_M = 0.035

# --- 비전 ---
IMAGE_WIDTH_PX = 800
IMAGE_HEIGHT_PX = 600
CAMERA_PARAMS_FILE = '/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/data/camera_params.npz'
HAND_EYE_MATRIX_FILE = '/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/data/hand_eye_matrix.npy'
HAND_EYE_UNIT = 'm' # Hand-Eye Matrix가 미터 단위라고 가정

# PnP를 위한 3D 모델 포인트
OBJECT_POINTS_3D = np.array([
    [ REAL_BUTTON_DIAMETER_M / 2,  0.0, 0.0],
    [-REAL_BUTTON_DIAMETER_M / 2,  0.0, 0.0],
    [ 0.0,  REAL_BUTTON_DIAMETER_M / 2, 0.0],
    [ 0.0, -REAL_BUTTON_DIAMETER_M / 2, 0.0],
], dtype=np.float32)
PNPR_REPROJ_ERROR_THRESHOLD_PX = 8.0
PNPR_MIN_INLIERS = 3

# --- 이미지 서보잉 ---
SERVOING_STANDBY_DISTANCE_M = 0.08
PRESS_FORWARD_DISTANCE_M = 0.04 # 누르기 거리를 약간 줄여 안정성 확보