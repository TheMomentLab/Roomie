# coordinate_transformer.py (좌표계 보정 적용)

import numpy as np
import cv2
from . import config

class CoordinateTransformer:
    def __init__(self):
        # Hand-Eye 및 카메라 파라미터 로드
        try:
            self.hand_eye_matrix = np.load(config.HAND_EYE_MATRIX_FILE)
            self._log(f"✅ 성공적으로 Hand-Eye 보정 행렬을 불러왔습니다: {config.HAND_EYE_MATRIX_FILE}", info=True)
        except FileNotFoundError:
            self.hand_eye_matrix = np.eye(4)
            self._log(f"⚠️ Hand-Eye 보정 파일을 찾을 수 없어 단위 행렬을 사용합니다. 경로: {config.HAND_EYE_MATRIX_FILE}", error=True)

        camera_params = np.load(config.CAMERA_PARAMS_FILE) 
        self.camera_matrix = camera_params['mtx']
        self.dist_coeffs = camera_params['dist']
        
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        self.real_button_diameter_m = config.REAL_BUTTON_DIAMETER_M

        # --- [핵심 수정 1] OpenCV 좌표계를 로봇 좌표계로 변환하기 위한 보정 행렬 정의 ---
        # 이 행렬은 OpenCV의 (X:Right, Y:Down, Z:Forward)를
        # 로봇의 (X:Forward, Y:Left, Z:Up)으로 변환하는 역할을 합니다.
        self.T_correction = np.array([
            [0,  0, 1, 0],  # 로봇의 X축은 OpenCV의 Z축
            [1, 0, 0, 0],  # 로봇의 Y축은 OpenCV의 -X축
            [0, -1, 0, 0],  # 로봇의 Z축은 OpenCV의 -Y축
            [0,  0, 0, 1]
        ])
        self._log("좌표계 보정 행렬(T_correction)이 초기화되었습니다.", info=True)
        self._log("CoordinateTransformer 초기화 완료", info=True)


    def estimate_fallback_pose(self, center_x_norm, center_y_norm, size_norm):
        # 이 함수는 수정 없이 그대로 유지합니다.
        IMAGE_WIDTH = self.cx * 2
        pixel_width = size_norm * IMAGE_WIDTH
        if pixel_width < 1e-6: return None, None
        approx_z = (self.real_button_diameter_m * self.fx) / pixel_width
        x = (center_x_norm * IMAGE_WIDTH - self.cx) * approx_z / self.fx
        y = (center_y_norm * (self.cy * 2) - self.cy) * approx_z / self.fy
        z = approx_z
        tvec_estimate = np.array([[x], [y], [z]])
        rvec_estimate = np.zeros((3, 1))
        return rvec_estimate, tvec_estimate


    def get_button_pose_in_base_frame(self, robot_fk_transform: np.ndarray, mode: str, **kwargs):
        T_cam_to_btn = np.eye(4)

        if mode == 'corner':
            image_points_2d = kwargs.get('image_points_2d')
            if image_points_2d is None: return None
            
            # solvePnP는 OpenCV 좌표계 기준으로 rvec, tvec을 반환
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                config.OBJECT_POINTS_3D, image_points_2d,
                self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not success: return None
            
            R_cam_to_btn, _ = cv2.Rodrigues(rvec)
            T_cam_to_btn[:3, :3] = R_cam_to_btn
            T_cam_to_btn[:3, 3] = tvec.flatten()

        elif mode == 'normal':
            center_x_norm = kwargs.get('center_x_norm')
            center_y_norm = kwargs.get('center_y_norm')
            size_norm = kwargs.get('size_norm')
            
            rvec, tvec = self.estimate_fallback_pose(center_x_norm, center_y_norm, size_norm)
            if rvec is None: return None
            
            R_cam_to_btn, _ = cv2.Rodrigues(rvec)
            T_cam_to_btn[:3, :3] = R_cam_to_btn
            T_cam_to_btn[:3, 3] = tvec.flatten()

        else:
            self._log(f"지원하지 않는 계산 모드: {mode}", error=True)
            return None

        # --- 공통 계산 로직 ---
        T_base_to_cam = robot_fk_transform @ self.hand_eye_matrix

        # --- [핵심 수정 2] 최종 변환 행렬 계산 시 보정 행렬(T_correction)을 곱해줍니다. ---
        T_base_to_btn = T_base_to_cam @ T_cam_to_btn @ self.T_correction
        
        return T_base_to_btn

    def _log(self, message: str, info: bool = False, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[CoordinateTransformer][{log_level}] {message}")