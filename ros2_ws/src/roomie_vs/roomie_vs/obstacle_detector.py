#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Optional, Tuple

class ObstacleDetector:
    """뎁스 카메라 기반 장애물 감지 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.robot_id = 0
        
        # 장애물 감지 설정
        self.min_distance_m = 0.0  # 최소 감지 거리 (해제)
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
                        
                        # 뎁스 카메라의 pixel_to_3d 함수 사용 (좌우반전 고려)
                        world_x, world_y, world_z = depth_camera.pixel_to_3d(
                            center_x, center_y, obj['depth_mm'], is_flipped=True
                        )
                        
                        # 화면 정규화 좌표 계산 (0~1)
                        image_width = 640  # 카메라 해상도
                        image_height = 480
                        normalized_x = center_x / image_width
                        normalized_y = center_y / image_height
                        
                        # 장애물 타입 결정
                        is_dynamic = obj['class_name'] == 'person'
                        
                        # 기존 객체에 장애물 정보 추가
                        obj['is_obstacle'] = True
                        obj['obstacle_type'] = 'dynamic' if is_dynamic else 'static'
                        obj['world_x'] = world_x
                        obj['world_y'] = world_y
                        obj['world_z'] = world_z
                        obj['distance_m'] = distance_m
                        obj['normalized_x'] = normalized_x
                        obj['normalized_y'] = normalized_y
                        
                        # 장애물 메시지용 정보 (문서 스펙 준수)
                        obstacle_info = {
                            'robot_id': self.robot_id,
                            'dynamic': is_dynamic,
                            'x': normalized_x,  # 화면 상 정규화된 좌표 (0~1)
                            'y': normalized_y,  # 화면 상 정규화된 좌표 (0~1)
                            'depth': distance_m,  # 미터(m) - 뎁스 카메라가 인식하는 depth
                            'world_x': world_x,  # 디버그용 월드 좌표
                            'world_y': world_y,  # 디버그용 월드 좌표
                            'world_z': world_z,  # 디버그용 월드 좌표
                            'distance': distance_m,
                            'class_name': obj['class_name'],
                            'confidence': obj['confidence'],
                            'tracking_id': obj.get('tracking_id')
                        }
                        
                        obstacles.append(obstacle_info)
                        
                        self.logger.debug(
                            f"🚧 장애물 감지: {obj['class_name']} "
                            f"타입: {'동적' if is_dynamic else '정적'} "
                            f"거리: {distance_m:.2f}m "
                            f"좌표: ({world_x:.2f}, {world_y:.2f})"
                        )
        
        return obstacles 