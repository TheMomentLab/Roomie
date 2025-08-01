import numpy as np
import cv2
from . import config

class CoordinateTransformer:
    # __init__ 메서드는 변경 사항이 없습니다.
    def __init__(self):
        self.hand_eye_matrix = np.load(config.HAND_EYE_MATRIX_FILE)
        camera_params = np.load(config.CAMERA_PARAMS_FILE)
        self.camera_matrix = camera_params['mtx']
        self.dist_coeffs = camera_params['dist']
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        self.real_button_diameter_m = config.REAL_BUTTON_DIAMETER_M
        print("✅ CoordinateTransformer 초기화 완료: 보정 데이터 로드 성공")

    def calculate_target_pose(self, button_center_xy_norm, button_size_norm, robot_fk_transform):
        """
        [Public] 모든 정보를 종합하여 로봇 베이스 기준 최종 3D 목표 지점을 계산합니다.
        """
        if config.DEBUG:
            print("\n--- 좌표 변환 시작 ---")
            print(f"  [입력] 2D 정규화 좌표: ({button_center_xy_norm[0]:.4f}, {button_center_xy_norm[1]:.4f})")
            print(f"  [입력] 2D 정규화 크기: {button_size_norm:.4f}")

        try:
            # --- 1. 거리(Depth) 계산 ---
            button_size_px = button_size_norm * config.IMAGE_HEIGHT_PX
            if button_size_px < 1:
                return None
            distance_m = (self.fy * self.real_button_diameter_m) / button_size_px

            if config.DEBUG:
                print(f"  [1단계: 거리 계산] 버튼 픽셀 크기: {button_size_px:.2f} px, 계산된 거리: {distance_m:.4f} m")

            # --- 2. 2D 이미지 좌표의 왜곡 보정 및 3D 변환 (카메라 기준) ---
            u_distorted = button_center_xy_norm[0] * config.IMAGE_WIDTH_PX
            v_distorted = button_center_xy_norm[1] * config.IMAGE_HEIGHT_PX

            distorted_points = np.array([[[u_distorted, v_distorted]]], dtype=np.float32)
            undistorted_points = cv2.undistortPoints(distorted_points, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
            u_undistorted, v_undistorted = undistorted_points[0][0]

            if config.DEBUG:
                print(f"  [2단계: 왜곡 보정] 왜곡 좌표: ({u_distorted:.2f}, {v_distorted:.2f}) -> 보정 좌표: ({u_undistorted:.2f}, {v_undistorted:.2f})")

            cam_x_offset = (u_undistorted - self.cx) * distance_m / self.fx
            cam_y_offset = (v_undistorted - self.cy) * distance_m / self.fy

            p_camera = np.array([distance_m, cam_x_offset, cam_y_offset, 1])

            if config.DEBUG:
                print(f"  [2단계: 3D 변환] 카메라 기준 좌표 (X,Y,Z): {np.round(p_camera[:3], 4)}")

            # --- 3. 최종 좌표 변환 (카메라 -> 로봇 베이스) ---
            transform_base_to_camera = robot_fk_transform @ self.hand_eye_matrix
            p_base = transform_base_to_camera @ p_camera
            final_target_xyz = p_base[:3]
            
            if config.DEBUG:
                print(f"  [3단계: 최종 변환] 로봇 베이스 기준 목표 좌표: {np.round(final_target_xyz, 4)}")
                print("--- 좌표 변환 종료 ---\n")
                
            return final_target_xyz
            
        except Exception as e:
            print(f"❌ 좌표 변환 중 오류 발생: {e}")
            return None