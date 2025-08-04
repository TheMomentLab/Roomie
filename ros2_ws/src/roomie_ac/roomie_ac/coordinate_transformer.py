import numpy as np
import cv2
from . import config

class CoordinateTransformer:
    def __init__(self):
        # 1. Hand-Eye 행렬을 우선 로드합니다.
        hand_eye_matrix_raw = np.load(config.HAND_EYE_MATRIX_FILE)

        # 2. 행렬의 이동(translation) 벡터를 확인합니다.
        translation_vector = hand_eye_matrix_raw[:3, 3]

        # 3. 이동 벡터의 절대값 중 하나라도 1.0을 초과하면 mm 단위로 간주합니다.
        hand_eye_matrix_raw = np.load(config.HAND_EYE_MATRIX_FILE)
        if config.HAND_EYE_UNIT == 'mm':
            self.hand_eye_matrix = hand_eye_matrix_raw.copy()
            self.hand_eye_matrix[:3, 3] /= 1000.0
        else:
            self.hand_eye_matrix = hand_eye_matrix_raw

        
        camera_params = np.load(config.CAMERA_PARAMS_FILE)
        self.camera_matrix = camera_params['mtx']
        self.dist_coeffs = camera_params['dist']
        
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        self.real_button_diameter_m = config.REAL_BUTTON_DIAMETER_M
        self._log("CoordinateTransformer 초기화 완료: 보정 데이터 로드 성공", info=True)

    def get_target_pose_from_points(self, image_points_2d: np.ndarray, robot_fk_transform: np.ndarray):
        """
        [최종 버전] PnP로 계산된 버튼 Pose와 로봇 FK를 이용해,
        로봇 베이스 기준의 최종 목표 '준비 위치'를 계산합니다.
        """
        if image_points_2d is None or len(image_points_2d) != 4:
            self._log("PnP 계산에 필요한 2D 점이 4개가 아닙니다.", error=True)
            return None

        # 1. PnP로 카메라 기준 버튼의 6D Pose (rvec, tvec) 계산
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            config.OBJECT_POINTS_3D,
            image_points_2d,
            self.camera_matrix,
            self.dist_coeffs,
            reprojectionError=config.PNPR_REPROJ_ERROR_THRESHOLD_PX,
            confidence=0.99,
            flags=cv2.SOLVEPNP_EPNP  # 변경
        )

        if not success or (inliers is not None and len(inliers) < config.PNPR_MIN_INLIERS):
            self._log("PnP 실패 또는 inliers 부족", error=True)
            return None, None

        # reprojection error 확인
        projected, _ = cv2.projectPoints(config.OBJECT_POINTS_3D, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        error = np.linalg.norm(projected.squeeze() - image_points_2d, axis=1).mean()

        if success:
            self._log(f"[PnP 성공] rvec: {rvec.flatten()}, tvec: {tvec.flatten()}", info=True)
            self._log(f"[Reproj Error] 평균 오차: {error:.2f}px", info=True)
        else:
            self._log("solvePnP 실패", error=True)

        if error > config.PNPR_REPROJ_ERROR_THRESHOLD_PX:
            self._log(f"[PnP] Reprojection error too high: {error:.2f}px", error=True)
            return None, None

        
        if not success:
            self._log("solvePnP 계산 실패", error=True)
            return None

        # 2. rvec, tvec을 4x4 변환 행렬(T_cam_to_btn)로 변환
        R_cam_to_btn, _ = cv2.Rodrigues(rvec)
        T_cam_to_btn = np.eye(4)
        T_cam_to_btn[:3, :3] = R_cam_to_btn
        T_cam_to_btn[:3, 3] = tvec.flatten()

        # 3. 로봇 베이스 -> 카메라 변환 행렬 계산
        T_base_to_cam = robot_fk_transform @ self.hand_eye_matrix

        # 4. 로봇 베이스 -> 버튼 변환 행렬 계산
        T_base_to_btn = T_base_to_cam @ T_cam_to_btn
        
        # 5. 최종 목표인 '준비 위치' 계산
        # 버튼 위치에서 Z축 방향으로 설정된 거리만큼 뒤로 물러난 위치가 최종 목표
        offset = np.eye(4)
        offset[2, 3] = -config.SERVOING_STANDBY_DISTANCE_M # 버튼 기준 -Z축 방향
        
        T_final_target = T_base_to_btn @ offset

        # IK가 사용할 XYZ 좌표만 반환
        target_xyz = T_final_target[:3, 3]
        target_orientation = T_final_target[:3, :3]
        return target_xyz, target_orientation

    def _log(self, message: str, info: bool = False, error: bool = False):
        if config.DEBUG:
            if info:
                print(f"[CoordinateTransformer][INFO] {message}")
            elif error:
                print(f"[CoordinateTransformer][ERROR] ❌ {message}")