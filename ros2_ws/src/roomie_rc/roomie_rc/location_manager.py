#!/usr/bin/env python3

from enum import IntEnum
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf_transformations

class LocationID(IntEnum):
    """위치 ID 정의"""
    # 대기/호출 위치
    LOB_WAITING = 0  # 로비 대기
    LOB_CALL = 1    # 로비 호출
    
    # 픽업 위치
    RES_PICKUP = 2  # 레스토랑 픽업
    RES_CALL = 3    # 레스토랑 호출
    SUP_PICKUP = 4  # 편의점 픽업
    
    # 엘리베이터
    ELE_1 = 5  # 1층 엘리베이터
    ELE_2 = 6  # 2층 엘리베이터
    
    # 객실
    ROOM_101 = 101
    ROOM_102 = 102
    ROOM_201 = 201
    ROOM_202 = 202


# # 위치 데이터베이스 (테스트용)
# LOCATIONS = {
#     LocationID.LOB_WAITING: {"x": 0.0, "y": 0.5, "floor": 0},
#     LocationID.LOB_CALL: {"x": 0.0, "y": 1.5, "floor": 0},
#     LocationID.RES_PICKUP: {"x": -0.3, "y": 1.5, "floor": 0},
#     LocationID.RES_CALL: {"x": -0.3, "y": 3.5, "floor": 0},
#     LocationID.SUP_PICKUP: {"x": 0.0, "y": 2.0, "floor": 0},
#     LocationID.ELE_1: {"x": 2.0, "y": 2.5, "floor": 0},
#     LocationID.ELE_2: {"x": 2.0, "y": 2.5, "floor": 1},
#     LocationID.ROOM_101: {"x": 3.0, "y": 0.0, "floor": 0},
#     LocationID.ROOM_102: {"x": 3.0, "y": 1.0, "floor": 0},
#     LocationID.ROOM_201: {"x": 3.0, "y": 0.0, "floor": 1},
#     LocationID.ROOM_202: {"x": 3.0, "y": 1.0, "floor": 1},
# }

# 위치 데이터베이스
LOCATIONS = {
    LocationID.LOB_WAITING: {"x": 0.0, "y": 1.5, "floor": 0},
    LocationID.LOB_CALL: {"x": 0.0, "y": 1.5, "floor": 0},
    LocationID.RES_PICKUP: {"x": -5.4, "y": 5.9, "floor": 0},
    LocationID.RES_CALL: {"x": -0.3, "y": 3.5, "floor": 0},
    LocationID.SUP_PICKUP: {"x": 0.0, "y": 2.0, "floor": 0},
    LocationID.ELE_1: {"x": 9.25, "y": 1.7, "floor": 0},
    LocationID.ELE_2: {"x": 9.25, "y": 1.7, "floor": 1},
    LocationID.ROOM_101: {"x": 5.9, "y": 3.4, "floor": 0},
    LocationID.ROOM_102: {"x": 7.3, "y": 3.4, "floor": 0},
    LocationID.ROOM_201: {"x": 5.9, "y": 3.4, "floor": 1},
    LocationID.ROOM_202: {"x": 7.3, "y": 3.4, "floor": 1},
}

# # 위치 데이터베이스
# LOCATIONS = {
#     LocationID.LOB_WAITING: {"x": 0.0, "y": 1.5, "floor": 0},
#     LocationID.ROOM_101: {"x": 5.9, "y": 3.4, "floor": 0},
# }

class LocationManager:
    """위치 정보 관리"""
    
    def get_pose(self, location_id: int, yaw: float = 0.0) -> PoseStamped:
        """위치 ID에 해당하는 PoseStamped 메시지 반환"""
        if location_id not in LOCATIONS:
            return None
            
        loc = LOCATIONS[location_id]
        
        # PoseStamped 메시지 생성
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        
        # 위치 설정
        pose_stamped.pose.position = Point(x=loc["x"], y=loc["y"], z=0.0)
        
        # 방향 설정 (yaw 값 사용)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return pose_stamped
    
    def get_floor(self, location_id: int) -> int:
        """위치 ID에 해당하는 층 반환"""
        if location_id not in LOCATIONS:
            return None
        return LOCATIONS[location_id]["floor"]
    
    def need_elevator(self, current_floor: int, target_id: int) -> bool:
        """엘리베이터 필요 여부 확인"""
        target_floor = self.get_floor(target_id)
        if target_floor is None:
            return False
        return current_floor != target_floor