import numpy as np
import cv2
from . import config

class CoordinateTransformer:
    def __init__(self):
        # ======================= [수정 시작] =======================
        # 기존 Hand-Eye 행렬 로드 코드를 모두 주석 처리하거나 삭제하고,
        # self.hand_eye_matrix를 단위 행렬로 강제 설정합니다.
        
        self._log("Hand-Eye 보정을 임시로 비활성화하고 단위 행렬을 사용합니다.", info=True)
        self.hand_eye_matrix = np.eye(4) # 4x4 단위 행렬

        # --- 기존 코드 (주석 처리) ---
        # # 1. Hand-Eye 행렬을 우선 로드합니다.
        # hand_eye_matrix_raw = np.load(config.HAND_EYE_MATRIX_FILE)

        # if config.HAND_EYE_UNIT == 'mm':
        #     self.hand_eye_matrix = hand_eye_matrix_raw.copy()
        #     self.hand_eye_matrix[:3, 3] /= 1000.0
        # else:
        #     self.hand_eye_matrix = hand_eye_matrix_raw
        # ======================== [수정 끝] ========================
        
        # 나머지 카메라 파라미터 로드 코드는 그대로 둡니다.
        camera_params = np.load(config.CAMERA_PARAMS_FILE)
        self.camera_matrix = camera_params['mtx']
        self.dist_coeffs = camera_params['dist']
        
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        self.real_button_diameter_m = config.REAL_BUTTON_DIAMETER_M
        self._log("CoordinateTransformer 초기화 완료: 보정 데이터 로드 성공", info=True)


    def estimate_fallback_pose(self, center_x_norm, center_y_norm, size_norm, camera_matrix, dist_coeffs):
        """
        PnP 실패 시 fallback pose를 추정하기 위한 함수
        - center_x_norm, center_y_norm: normalized [0~1]
        - size_norm: normalized bbox width
        """
        IMAGE_WIDTH = camera_matrix[0, 2] * 2
        IMAGE_HEIGHT = camera_matrix[1, 2] * 2

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]


        # center pixel 좌표
        center_px = np.array([
            center_x_norm * IMAGE_WIDTH,
            center_y_norm * IMAGE_HEIGHT
        ])

        # 근사 Z 거리 계산: size로부터 거리 추정
        # 거리 ≈ 실제지름 * focal_length / 픽셀지름
        pixel_width = size_norm * IMAGE_WIDTH
        approx_z = (config.REAL_BUTTON_DIAMETER_M * fx) / pixel_width

        # 중심점으로부터 X, Y 계산
        x = (center_px[0] - cx) * approx_z / fx
        y = (center_px[1] - cy) * approx_z / fy
        z = approx_z

        tvec_estimate = np.array([[x], [y], [z]])  # shape (3, 1)

        # orientation은 default 또는 이전 값 유지
        rvec_estimate = np.zeros((3, 1))  # or 이전 rvec 값 유지

        return rvec_estimate, tvec_estimate


    def get_button_pose_in_base_frame(self, image_points_2d: np.ndarray, robot_fk_transform: np.ndarray):
        """
        [수정됨] PnP와 FK를 이용해, 로봇 베이스 기준의 '버튼' 6D Pose(4x4 행렬)를 계산합니다.
        """
        if image_points_2d is None or len(image_points_2d) != 4:
            self._log("PnP 계산에 필요한 2D 점이 4개가 아닙니다.", error=True)
            return None

        # 중심점 및 너비 추정
        center_x_px = np.mean(image_points_2d[:, 0])
        center_y_px = np.mean(image_points_2d[:, 1])
        width_px = np.max(image_points_2d[:, 0]) - np.min(image_points_2d[:, 0])
        image_width = self.cx * 2
        image_height = self.cy * 2
        center_x_norm = center_x_px / image_width
        center_y_norm = center_y_px / image_height
        size_norm = width_px / image_width

        # 1. PnP로 카메라 기준 버튼의 6D Pose (rvec, tvec) 계산
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            config.OBJECT_POINTS_3D,
            image_points_2d,
            self.camera_matrix,
            self.dist_coeffs,
            reprojectionError=config.PNPR_REPROJ_ERROR_THRESHOLD_PX,
            confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success or inliers is None or len(inliers) < config.PNPR_MIN_INLIERS:
            self._log("PnP 실패 또는 inliers 부족. fallback pose 사용", error=True)
            rvec, tvec = self.estimate_fallback_pose(
                center_x_norm, center_y_norm, size_norm,
                self.camera_matrix, self.dist_coeffs
            )

        # 2. rvec, tvec을 4x4 변환 행렬(T_cam_to_btn)로 변환
        R_cam_to_btn, _ = cv2.Rodrigues(rvec)
        T_cam_to_btn = np.eye(4)
        T_cam_to_btn[:3, :3] = R_cam_to_btn
        T_cam_to_btn[:3, 3] = tvec.flatten()

        # 3. 로봇 베이스 -> 카메라 변환 행렬 계산
        T_base_to_cam = robot_fk_transform @ self.hand_eye_matrix

        # 4. 로봇 베이스 -> 버튼 변환 행렬 계산
        T_base_to_btn = T_base_to_cam @ T_cam_to_btn
        
        # 5. [변경점] '준비 위치' 계산 로직 없이, 버튼의 Pose 행렬을 바로 반환
        return T_base_to_btn


    def _log(self, message: str, info: bool = False, error: bool = False):
        if config.DEBUG:
            if info:
                print(f"[CoordinateTransformer][INFO] {message}")
            elif error:
                print(f"[CoordinateTransformer][ERROR] ❌ {message}")