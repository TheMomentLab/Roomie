from pathlib import Path
import numpy as np
from enum import IntEnum
from ament_index_python.packages import get_package_share_directory
import os

# 1. Pose 종류를 관리하는 Enum 클래스 정의
class Pose(IntEnum):
    """로봇 팔의 미리 정의된 자세 종류"""
    INIT = 0
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    OBSERVE = 4
    # 새로운 포즈를 추가하고 싶으면 여기에 한 줄만 추가하면 됩니다.
    # 예: READY_FOR_BUTTON = 4

class ControlMode(IntEnum):
    """제어 전략 모드"""
    MODEL_ONLY = 0      # 모델 기반 제어만 사용 (빠름)
    HYBRID = 1          # 모델 기반 + 이미지 서보잉 결합 (정밀, 권장)



# 기본 설정 (DEBUG 등)
DEBUG = True

# 파일 경로 설정
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = os.path.join(get_package_share_directory('roomie_ac'), 'data')
URDF_FILE = os.path.join(get_package_share_directory('roomie_ac'), 'urdf', 'roomie2.urdf')
CAMERA_PARAMS_FILE = os.path.join(DATA_DIR, 'camera_params.npz')  
HAND_EYE_MATRIX_FILE = os.path.join(DATA_DIR, 'hand_eye_matrix.npy')
# 시리얼 통신 설정
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT = 10.0 # 시리얼 연결 및 응답 대기 타임아웃

# 로봇 ID 설정 (로봇의 고유 식별자)
ROBOT_ID = 0

# 서보 모터 및 관절 설정
SERVO_ZERO_OFFSET_DEG = np.array([90, 90, 90, 90])
SERVO_DIRECTION_MULTIPLIER = np.array([1, -1, -1, -1])
JOINT_LIMIT_DEG = np.array([[0, 180], [0, 180], [0, 180], [0, 180]]) # 각 관절의 제한 각도 (deg)
JOINT_LIMIT_RAD = np.deg2rad(JOINT_LIMIT_DEG) # 라디안 변환
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4'] # RViz2 퍼블리싱을 위한 관절 이름

# IK (Inverse Kinematics) 설정
ACTIVE_LINKS_MASK = [False, True, True, True, True, False] # ikpy 체인에서 활성화할 링크 마스크
IK_MAX_ITERATIONS = 10000 # IK 최대 반복 횟수
IK_TOLERANCE_M = 2e-2 # IK 오차 허용 범위 (m)

# 워크스페이스 (작업 공간) 설정
WORKSPACE_R_MIN_M = -0.45 # 로봇 중심으로부터 최소/최대 반경 (m)
WORKSPACE_R_MAX_M = 0.45
WORKSPACE_Z_MIN_M = -0.40 # 로봇팔 끝점의 최소/최대 높이 (m)
WORKSPACE_Z_MAX_M = 0.40

# 동작 및 지연 시간 설정
COMEBACK_DELAY_SEC = 4.0 # 홈 포지션으로 복귀 대기 시간 (초)

# 제어 전략 설정
# 기본 제어 전략은 하이브리드 모드로 설정
# 모델 기반 제어와 이미지 서보잉을 결합하여 정밀한 버튼 조작을 수행
# 필요에 따라 ControlMode.MODEL_ONLY로 변경 가능
# 예: CONTROL_STRATEGY = ControlMode.MODEL_ONLY
CONTROL_STRATEGY = ControlMode.HYBRID

# --- 하이브리드 제어 설정 ---
PRE_PRESS_DISTANCE_M = 0.05 # 버튼 앞에서 대기할 거리 (5cm)

# 2. Enum을 키(key)로, 실제 각도값을 값(value)으로 갖는 딕셔너리 생성
POSE_ANGLES_DEG = {
    Pose.INIT: np.array([90, 90, 90, 90]),
    Pose.LEFT: np.array([180, 130, 170, 30]),
    Pose.RIGHT: np.array([0, 130, 170, 30]),
    Pose.FORWARD: np.array([90, 120, 127, 40]),
    Pose.OBSERVE: np.array([90, 120, 127, 40]),
}

# 기존 홈 포지션 변수도 이 딕셔너리를 활용할 수 있습니다.
HOME_POSITION_SERVO_DEG = POSE_ANGLES_DEG[Pose.INIT]

# --- 카메라 및 인식 설정 ---
IMAGE_WIDTH_PX = 800  # 사용하는 카메라의 가로 해상도 (픽셀)
IMAGE_HEIGHT_PX = 600 # 사용하는 카메라의 세로 해상도 (픽셀)
REAL_BUTTON_DIAMETER_M = 0.035 # 버튼의 실제 지름 (미터)