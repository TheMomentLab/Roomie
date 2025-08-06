#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Optional, Tuple

class ObstacleDetector:
    """뎁스 카메라 기반 장애물 감지 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.robot_id = 1
        
        # 장애물 감지 설정
        self.min_distance_m = 0.5  # 최소 감지 거리 (0.5m)
        self.max_distance_m = 3.0  # 최대 감지 거리 (3m)
        self.confidence_threshold = 0.5
        
    def detect_obstacles_from_objects(self, objects: List[Dict], depth_camera) -> List[Dict]:
        """YOLO 감지 결과를 3D 좌표로 변환하여 장애물 생성"""
        obstacles = []
        
        for obj in objects:
            if obj['class_name'] in ['person', 'chair']:
                # 뎁스 정보 확인
                if 'depth_mm' in obj and obj['depth_mm'] > 0:
                    distance_m = obj['depth_mm'] / 1000.0  # mm to meters
                    
                    # 거리 필터링
                    if self.min_distance_m <= distance_m <= self.max_distance_m:
                        # 2D 픽셀 좌표를 3D 월드 좌표로 변환
                        center_x, center_y = obj['center']
                        
                        # 뎁스 카메라의 pixel_to_3d 함수 사용
                        world_x, world_y, world_z = depth_camera.pixel_to_3d(
                            center_x, center_y, obj['depth_mm']
                        )
                        
                        # 장애물 타입 결정
                        is_dynamic = obj['class_name'] == 'person'
                        
                        # 기존 객체에 장애물 정보 추가
                        obj['is_obstacle'] = True
                        obj['obstacle_type'] = 'dynamic' if is_dynamic else 'static'
                        obj['world_x'] = world_x
                        obj['world_y'] = world_y
                        obj['world_z'] = world_z
                        obj['distance_m'] = distance_m
                        
                        # 장애물 메시지용 정보
                        obstacle_info = {
                            'robot_id': self.robot_id,
                            'dynamic': is_dynamic,
                            'x': world_x,  # 실제 월드 좌표 (미터)
                            'y': world_y,  # 실제 월드 좌표 (미터)
                            'z': world_z,  # 실제 월드 좌표 (미터)
                            'distance': distance_m,
                            'class_name': obj['class_name'],
                            'confidence': obj['confidence']
                        }
                        
                        obstacles.append(obstacle_info)
                        
                        self.logger.debug(
                            f"🚧 장애물 감지: {obj['class_name']} "
                            f"타입: {'동적' if is_dynamic else '정적'} "
                            f"거리: {distance_m:.2f}m "
                            f"좌표: ({world_x:.2f}, {world_y:.2f})"
                        )
        
        return obstacles 