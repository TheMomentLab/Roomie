from pathlib import Path
import numpy as np

# 기본 설정 (DEBUG 등)
DEBUG = True

# 파일 경로 설정
SCRIPT_DIR = Path(__file__).resolve().parent
URDF_FILE = SCRIPT_DIR / "../urdf/roomie2.urdf" # urdf 디렉토리에 맞춰 경로 조정

# 시리얼 통신 설정
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT = 10.0 # 시리얼 연결 및 응답 대기 타임아웃

# 서보 모터 및 관절 설정
SERVO_ZERO_OFFSET_DEG = np.array([90, 90, 90, 90])
SERVO_DIRECTION_MULTIPLIER = np.array([1, -1, -1, -1])
JOINT_LIMIT_DEG = np.array([[0, 180], [0, 180], [0, 180], [0, 180]]) # 각 관절의 제한 각도 (deg)
JOINT_LIMIT_RAD = np.deg2rad(JOINT_LIMIT_DEG) # 라디안 변환
HOME_POSITION_SERVO_DEG = [90, 90, 90, 90] # 홈 포지션 각도 (서보 각도, deg)
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

