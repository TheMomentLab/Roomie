import numpy as np
import cv2
from . import config

class CoordinateTransformer:
    """
    카메라 2D 좌표를 로봇의 3D 좌표로 변환하는 모든 계산을 담당합니다.
    """
    def __init__(self):
        # 1. config.py에 정의된 경로를 이용해 보정 데이터 로드
        self.hand_eye_matrix = np.load(config.HAND_EYE_MATRIX_FILE)
        
        camera_params = np.load(config.CAMERA_PARAMS_FILE)
        self.camera_matrix = camera_params['mtx']
        self.dist_coeffs = camera_params['dist']
        
        # 2. config로부터 필요한 파라미터 로드
        self.fx = self.camera_matrix[0, 0]  # 초점 거리 (x)
        self.fy = self.camera_matrix[1, 1]  # 초점 거리 (y)
        self.cx = self.camera_matrix[0, 2]  # 주점 (x)
        self.cy = self.camera_matrix[1, 2]  # 주점 (y)
        
        self.real_button_diameter_m = config.REAL_BUTTON_DIAMETER_M

        print("✅ CoordinateTransformer 초기화 완료: 보정 데이터 로드 성공")

    def calculate_target_pose(self, button_center_xy_norm, button_size_norm, robot_fk_transform):
        """
        [Public] 모든 정보를 종합하여 로봇 베이스 기준 최종 3D 목표 지점을 계산합니다.
        
        Args:
            button_center_xy_norm (tuple): (x, y) 정규화된 이미지 좌표
            button_size_norm (float): 정규화된 버튼 크기 (높이 기준)
            robot_fk_transform (np.ndarray): 로봇의 현재 FK 변환 행렬 (4x4)
        
        Returns:
            np.ndarray: 로봇 베이스 기준의 3D 목표 좌표 [x, y, z] 또는 실패 시 None
        """
        try:
            # --- 1. 거리(Depth) 계산 ---
            # 정규화된 크기를 픽셀 크기로 변환 (세로 기준)
            button_size_px = button_size_norm * config.IMAGE_HEIGHT_PX
            if button_size_px < 1: # 크기가 0에 가까우면 계산 불가
                return None
            
            # 핀홀 카메라 모델 공식을 이용해 거리 추정
            # Distance = (Focal_Length_Y * Real_Object_Height) / Image_Object_Height
            distance_m = (self.fy * self.real_button_diameter_m) / button_size_px

            # --- 2. 2D 이미지 좌표의 왜곡 보정 및 3D 변환 (카메라 기준) ---
            # 정규화된 좌표를 픽셀 좌표로 변환
            u_distorted = button_center_xy_norm[0] * config.IMAGE_WIDTH_PX
            v_distorted = button_center_xy_norm[1] * config.IMAGE_HEIGHT_PX
            
            # 3D 공간 상의 X, Y 오프셋 계산
            # X_offset = (u - cx) * Z / fx
            # Y_offset = (v - cy) * Z / fy
            cam_x_offset = (u_distorted - self.cx) * distance_m / self.fx
            cam_y_offset = (v_distorted - self.cy) * distance_m / self.fy
            
            # 카메라 좌표계 기준 3D 포인트 생성
            # 사용자가 정의한 좌표계: +X가 정면(거리), +Y가 가로, +Z가 세로
            # 계산된 오프셋은 표준 이미지 좌표계 기준(X가 가로, Y가 세로)이므로 매핑 필요
            # [가정] 이미지 X(가로) -> 카메라 Y, 이미지 Y(세로) -> 카메라 Z
            p_camera = np.array([
                distance_m,      # 카메라 정면(+X) -> 거리
                cam_x_offset,    # 이미지 가로 -> 카메라 +Y
                cam_y_offset,    # 이미지 세로 -> 카메라 +Z
                1                # 동차 좌표(Homogeneous coordinate)를 위한 1
            ])

            # --- 3. 최종 좌표 변환 (카메라 -> 로봇 베이스) ---
            # 최종 변환 행렬: T_base_to_cam = T_base_to_EE * T_EE_to_cam
            transform_base_to_camera = robot_fk_transform @ self.hand_eye_matrix
            
            # 최종 좌표 계산
            p_base = transform_base_to_camera @ p_camera
            
            # 동차 좌표에서 3D 좌표로 변환하여 반환
            final_target_xyz = p_base[:3]
            
            if config.DEBUG:
                print(f"[CoordinateTransformer] 최종 계산된 목표 좌표: {np.round(final_target_xyz, 4)}")
                
            return final_target_xyz
            
        except Exception as e:
            print(f"❌ 좌표 변환 중 오류 발생: {e}")
            return None
