#!/usr/bin/env python3

import numpy as np
from typing import List, Dict

class ObstacleDetector:
    """YOLO 등으로 감지된 2D 객체 정보와 깊이 카메라 데이터를 결합하여 3D 공간상의 장애물을 식별하는 클래스입니다."""
    
    def __init__(self, logger):
        self.logger = logger
        self.robot_id = 0  # 로봇 ID, 현재는 0으로 고정
        
        # 장애물로 판단할 거리 범위 (미터 단위)
        self.min_distance_m = 0.0
        self.max_distance_m = 3.0
        self.confidence_threshold = 0.5
        
    def detect_obstacles_from_objects(self, objects: List[Dict], depth_camera) -> List[Dict]:
        """
        감지된 객체 목록을 순회하며 장애물로 간주될 수 있는 객체를 3D 좌표로 변환하고, 
        장애물 정보를 담은 리스트를 반환합니다.

        Args:
            objects (List[Dict]): YOLO 모델 등에서 감지된 객체 정보 리스트.
            depth_camera: 픽셀 좌표를 3D 월드 좌표로 변환하는 `pixel_to_3d` 메서드를 가진 카메라 객체.

        Returns:
            List[Dict]: 각 장애물에 대한 상세 정보를 담은 딕셔너리 리스트.
        """
        obstacles = []
        
        for obj in objects:
            # 사람(person)과 의자(chair)만 장애물로 간주
            if obj['class_name'] in ['person', 'chair']:
                # 유효한 깊이 값이 있는지 확인
                if 'depth_mm' in obj and obj['depth_mm'] > 0:
                    distance_m = obj['depth_mm'] / 1000.0
                    
                    # 설정된 거리 범위 내에 있는 객체만 처리
                    if self.min_distance_m <= distance_m <= self.max_distance_m:
                        center_x, center_y = obj['center']
                        
                        # 2D 픽셀 좌표(u, v)와 깊이(depth)를 이용해 3D 카메라 좌표(x, y, z)로 변환
                        world_x, world_y, world_z = depth_camera.pixel_to_3d(
                            center_x, center_y, obj['depth_mm'], is_flipped=True
                        )
                        
                        # 이미지 전체 크기를 기준으로 좌표를 정규화 (0.0 ~ 1.0)
                        image_width = 640
                        image_height = 480
                        normalized_x = center_x / image_width
                        normalized_y = center_y / image_height
                        
                        # 사람은 동적 장애물, 그 외는 정적 장애물로 분류
                        is_dynamic = obj['class_name'] == 'person'
                        
                        # 최종 장애물 정보 구성
                        obstacle_info = {
                            'robot_id': self.robot_id,          # 로봇 식별자
                            'dynamic': is_dynamic,              # 동적 장애물 여부 (True/False)
                            'x': normalized_x,                  # 정규화된 x 좌표
                            'y': normalized_y,                  # 정규화된 y 좌표
                            'depth': distance_m,                # 카메라로부터의 거리 (미터)
                            'world_x': world_x,                 # 3D 카메라 좌표계 x
                            'world_y': world_y,                 # 3D 카메라 좌표계 y
                            'world_z': world_z,                 # 3D 카메라 좌표계 z (깊이)
                            'class_name': obj['class_name'],   # 감지된 객체의 클래스명
                            'confidence': obj['confidence'],     # 감지 신뢰도
                            'tracking_id': obj.get('tracking_id') # 추적 ID (있는 경우)
                        }
                        
                        obstacles.append(obstacle_info)
                        
                        self.logger.debug(
                            f"장애물 감지: {obj['class_name']} | "
                            f"종류: {'동적' if is_dynamic else '정적'} | "
                            f"거리: {distance_m:.2f}m | "
                            f"3D 좌표: ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
                        )
        
        return obstacles