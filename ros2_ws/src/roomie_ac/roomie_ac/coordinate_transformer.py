import numpy as np
import cv2
from . import config

class CoordinateTransformer:
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
        self._log("CoordinateTransformer 초기화 완료: 보정 데이터 로드 성공", info=True)

    def calculate_target_pose(self, button_center_xy_norm, button_width_norm, robot_fk_transform):
        """
        [Public] 모든 정보를 종합하여 로봇 베이스 기준 최종 3D 목표 지점을 계산합니다.
        button_width_norm: 정규화된 버튼의 '너비' (yolo_vision_service.py에서 보낸 값)
        """
        if button_width_norm <= 0:
            self._log("버튼 너비가 0 이하이므로 좌표 변환을 수행할 수 없습니다.", error=True)
            return None

        self._log("\n--- 좌표 변환 시작 ---", info=True)
        self._log(f"  [입력] 2D 정규화 좌표: ({button_center_xy_norm[0]:.4f}, {button_center_xy_norm[1]:.4f})", info=True)
        self._log(f"  [입력] 2D 정규화 너비: {button_width_norm:.4f}", info=True)

        try:
            # --- 1. 거리(Depth) 계산 ---
            # [수정] 정규화된 '너비'이므로 이미지 '너비(WIDTH)'를 곱해야 합니다.
            button_width_px = button_width_norm * config.IMAGE_WIDTH_PX
            
            # 핀홀 카메라 모델의 거리 추정 공식: Z = (f * W_real) / w_pixel
            # X축 초점거리(fx)를 사용합니다.
            distance_m = (self.fx * self.real_button_diameter_m) / button_width_px
            self._log(f"  [1단계: 거리 계산] 버튼 픽셀 너비: {button_width_px:.2f} px, 계산된 거리: {distance_m:.4f} m", info=True)

            # --- 2. 2D 이미지 좌표의 왜곡 보정 및 3D 변환 (카메라 기준) ---
            u_distorted = button_center_xy_norm[0] * config.IMAGE_WIDTH_PX
            v_distorted = button_center_xy_norm[1] * config.IMAGE_HEIGHT_PX

            distorted_points = np.array([[[u_distorted, v_distorted]]], dtype=np.float32)
            undistorted_points = cv2.undistortPoints(distorted_points, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix)
            u_undistorted, v_undistorted = undistorted_points[0][0]
            self._log(f"  [2단계: 왜곡 보정] 왜곡 좌표: ({u_distorted:.2f}, {v_distorted:.2f}) -> 보정 좌표: ({u_undistorted:.2f}, {v_undistorted:.2f})", info=True)

            # 역투영 (2D -> 3D)
            # [수정] 표준 카메라 좌표계 (Z축이 정면)에 맞게 수정
            # X_cam = (u - cx) * Z / fx
            # Y_cam = (v - cy) * Z / fy
            # Z_cam = Z
            cam_x = (u_undistorted - self.cx) * distance_m / self.fx
            cam_y = (v_undistorted - self.cy) * distance_m / self.fy
            cam_z = distance_m

            # p_camera는 동차좌표 (Homogeneous Coordinate) 입니다. [x, y, z, 1]
            p_camera = np.array([cam_x, cam_y, cam_z, 1])
            self._log(f"  [2단계: 3D 변환] 카메라 기준 좌표 (X,Y,Z): {np.round(p_camera[:3], 4)}", info=True)

            # --- 3. 최종 좌표 변환 (카메라 -> 로봇 베이스) ---
            transform_base_to_camera = robot_fk_transform @ self.hand_eye_matrix
            p_base = transform_base_to_camera @ p_camera
            final_target_xyz = p_base[:3]
            
            self._log(f"  [3단계: 최종 변환] 로봇 베이스 기준 목표 좌표: {np.round(final_target_xyz, 4)}", info=True)
            self._log("--- 좌표 변환 종료 ---\n", info=True)
                
            return final_target_xyz
            
        except Exception as e:
            self._log(f"좌표 변환 중 오류 발생: {e}", error=True)
            return None

    def _log(self, message: str, info: bool = False, error: bool = False):
        if config.DEBUG:
            if info:
                print(f"[CoordinateTransformer][INFO] {message}")
            elif error:
                print(f"[CoordinateTransformer][ERROR] ❌ {message}")