"""
정적 장애물 분류용 설정 파일

설명:
- 웨이포인트 좌표/이웃 정보와, 깊이 → 웨이포인트 매핑 범위를 정의합니다.
- x 정규화 좌표(0~1)를 이용해 "경로 내"로 볼 수 있는 수평 범위를 설정합니다.
- 전방(A/B/C 행 진행)과 세로(열 기준) 분류를 모두 지원합니다.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


# 수평 시야 내로 간주할 x 정규화 범위 [0, 1]
IN_PATH_X_RANGE: Tuple[float, float] = (0.40, 0.60)


# 전방(A 행 기준) 깊이 → 웨이포인트 매핑 (단위: m)
# 예) A1 기준 전방 분류: 0.9~1.8=A2, 1.8~3.6=A3, 3.6~4.5=A4, 4.5~6.3=A5
RANGES_FORWARD_A: List[Tuple[float, float, str]] = [
    (0.9, 1.8, 'A2'),
    (1.8, 3.6, 'A3'),
    (3.6, 4.5, 'A4'),
    (4.5, 6.3, 'A5'),
]

# 필요 시 B/C 행 진행 방향도 별도로 정의 가능(현재는 미정)
RANGES_FORWARD_C: List[Tuple[float, float, str]] = [
    (0.9, 1.8, 'C2'),
    (1.8, 3.6, 'C3'),
    (3.6, 4.5, 'C4'),
    (4.5, 6.3, 'C5'),
]


# 웨이포인트 좌표/그래프 (단위: m) 테스트용
WAYPOINTS: Dict[str, Dict] = {
    'A1': {'x': 0.00, 'y': 1.0, 'neighbors': ['B1', 'A2']},
    'A2': {'x': 1.00, 'y': 1.0, 'neighbors': ['A1', 'A3']},
    'A3': {'x': 2.00, 'y': 1.0, 'neighbors': ['B3', 'A2', 'A4']},
    'A4': {'x': 3.00, 'y': 1.0, 'neighbors': ['A3', 'A5']},
    'A5': {'x': 4.00, 'y': 1.0, 'neighbors': ['B5', 'A4']},

    'B1': {'x': 0.00, 'y': 0.5, 'neighbors': ['A1', 'C1']},
    'B3': {'x': 2.00, 'y': 0.5, 'neighbors': ['A3', 'C3']},
    'B5': {'x': 4.00, 'y': 0.5, 'neighbors': ['A5', 'C5']},

    'C1': {'x': 0.00, 'y': 0.0, 'neighbors': ['B1', 'C2']},
    'C2': {'x': 1.00, 'y': 0.0, 'neighbors': ['C1', 'C3']},
    'C3': {'x': 2.00, 'y': 0.0, 'neighbors': ['B3', 'C2', 'C4']},
    'C4': {'x': 3.00, 'y': 0.0, 'neighbors': ['C3', 'C5']},
    'C5': {'x': 4.00, 'y': 0.0, 'neighbors': ['B5', 'C4']},
}

# 웨이포인트 좌표/그래프 (단위: m)
# WAYPOINTS: Dict[str, Dict] = {
#     'A1': {'x': 0.35, 'y': 3.20, 'neighbors': ['B1', 'A2']},
#     'A2': {'x': 1.70, 'y': 3.20, 'neighbors': ['A1', 'A3']},
#     'A3': {'x': 3.05, 'y': 3.20, 'neighbors': ['B3', 'A2', 'A4']},
#     'A4': {'x': 4.40, 'y': 3.20, 'neighbors': ['A3', 'A5']},
#     'A5': {'x': 5.75, 'y': 3.20, 'neighbors': ['B5', 'A4']},

#     'B1': {'x': 0.35, 'y': 1.65, 'neighbors': ['A1', 'C1']},
#     'B3': {'x': 3.05, 'y': 1.65, 'neighbors': ['A3', 'C3']},
#     'B5': {'x': 5.75, 'y': 1.76, 'neighbors': ['A5', 'C5']},

#     'C1': {'x': 0.35, 'y': 0.08, 'neighbors': ['B1', 'C2']},
#     'C2': {'x': 1.70, 'y': 0.08, 'neighbors': ['C1', 'C3']},
#     'C3': {'x': 3.05, 'y': 0.08, 'neighbors': ['B3', 'C2', 'C4']},
#     'C4': {'x': 4.40, 'y': 0.08, 'neighbors': ['C3', 'C5']},
#     'C5': {'x': 5.75, 'y': 0.38, 'neighbors': ['B5', 'C4']},
# }


@dataclass
class StartPose:
    waypoint_id: str
    yaw_deg: float


# 기본 시작/도착 설정 (필요시 사용)
DEFAULT_START: Optional[StartPose] = None
DEFAULT_GOAL: Optional[str] = None


def classify_forward_waypoint(depth_m: float, heading_row: str = 'A') -> Optional[str]:
    """전방(행 진행) 깊이에 따른 웨이포인트 분류

    heading_row: 'A' | 'B' | 'C'
    반환: 웨이포인트 ID 또는 None
    """
    ranges_map = {
        'A': RANGES_FORWARD_A,
        'C': RANGES_FORWARD_C,
    }
    ranges = ranges_map.get(heading_row.upper(), [])
    for min_d, max_d, wp_id in ranges:
        if min_d <= depth_m < max_d:
            return wp_id
    return None


def _parse_wp_id(wp_id: str) -> Tuple[str, int]:
    """웨이포인트 ID를 (행 문자, 열 번호)로 파싱"""
    if not wp_id or len(wp_id) < 2:
        return ('', 0)
    row = wp_id[0].upper()
    try:
        col = int(wp_id[1:])
    except ValueError:
        col = 0
    return (row, col)


# 전방 이진 분류를 상대 기준으로 수행하기 위한 깊이→오프셋 정의
# 예) 0.9~1.8 → +1칸, 1.8~3.6 → +2칸, 3.6~4.5 → +3칸, 4.5~6.3 → +4칸
FORWARD_DEPTH_OFFSETS: List[Tuple[float, float, int]] = [
    (0.9, 1.8, 1),
    (1.8, 3.6, 2),
    (3.6, 4.5, 3),
    (4.5, 6.3, 4),
]


def classify_forward_waypoint_relative(depth_m: float, from_wp: str, to_wp: str) -> Optional[str]:
    """수평(행) 진행 시, 현재 세그먼트 방향을 고려하여 깊이를 웨이포인트로 매핑

    - from_wp → to_wp 방향으로 진행한다고 가정하고, 깊이에 따른 열 오프셋을 계산해 목표 웨이포인트를 반환
    - 예) A5→A4 진행 중 깊이 1.0m이면 A4 반환
    """
    row_from, col_from = _parse_wp_id(from_wp)
    row_to, col_to = _parse_wp_id(to_wp)
    if row_from != row_to or row_from not in ('A', 'C'):
        return None
    if col_from == 0 or col_to == 0:
        return None

    dir_sign = 1 if col_to > col_from else -1
    offset: Optional[int] = None
    for dmin, dmax, off in FORWARD_DEPTH_OFFSETS:
        if dmin <= depth_m < dmax:
            offset = off
            break
    if offset is None:
        return None

    target_col = col_from + dir_sign * offset
    target_id = f"{row_from}{target_col}"
    return target_id if target_id in WAYPOINTS else None


# 세로(열) 기준 깊이 → 웨이포인트 매핑
# - A에서 내려다볼 때(원점 A): A1→B1→C1, A3→B3→C3, A5→B5→C5
# - C에서 올려볼 때(원점 C): C1→B1→A1, C3→B3→A3, C5→B5→A5
# 열 1과 3은 동일한 y 간격(A↔B≈1.47m, B↔C≈1.57m), 열 5는 서랍 영향으로 간격이 더 짧음(A↔B≈1.36m, B↔C≈1.38m)

# A 원점(위에서 아래) 기준
RANGES_VERTICAL_COL1_FROM_A: List[Tuple[float, float, str]] = [
    (0.9, 2.1, 'B1'),
    (2.1, 3.1, 'C1'),
]
RANGES_VERTICAL_COL3_FROM_A: List[Tuple[float, float, str]] = [
    (0.9, 2.1, 'B3'),
    (2.1, 3.1, 'C3'),
]
RANGES_VERTICAL_COL5_FROM_A: List[Tuple[float, float, str]] = [
    (0.9, 1.9, 'B5'),
    (1.9, 3.3, 'C5'),
]

# C 원점(아래에서 위) 기준
RANGES_VERTICAL_COL1_FROM_C: List[Tuple[float, float, str]] = [
    (0.9, 2.1, 'B1'),
    (2.1, 3.1, 'A1'),
]
RANGES_VERTICAL_COL3_FROM_C: List[Tuple[float, float, str]] = [
    (0.9, 2.1, 'B3'),
    (2.1, 3.1, 'A3'),
]
RANGES_VERTICAL_COL5_FROM_C: List[Tuple[float, float, str]] = [
    (0.9, 1.9, 'B5'),
    (1.9, 3.3, 'A5'),
]


def classify_vertical_waypoint(depth_m: float, column: str = '1', origin: str = 'A') -> Optional[str]:
    """세로(열) 방향 깊이에 따른 웨이포인트 분류

    column: 열 번호 문자열 ('1'|'3'|'5')
    origin: 'A' 또는 'C' (A에서 아래로, C에서 위로)
    반환: 웨이포인트 ID 또는 None
    """
    if origin.upper() == 'A':
        col_map: Dict[str, List[Tuple[float, float, str]]] = {
            '1': RANGES_VERTICAL_COL1_FROM_A,
            '3': RANGES_VERTICAL_COL3_FROM_A,
            '5': RANGES_VERTICAL_COL5_FROM_A,
        }
    else:
        col_map = {
            '1': RANGES_VERTICAL_COL1_FROM_C,
            '3': RANGES_VERTICAL_COL3_FROM_C,
            '5': RANGES_VERTICAL_COL5_FROM_C,
        }
    ranges = col_map.get(column, [])
    for min_d, max_d, wp_id in ranges:
        if min_d <= depth_m < max_d:
            return wp_id
    return None


