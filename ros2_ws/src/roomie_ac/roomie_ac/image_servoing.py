import numpy as np
import asyncio
from . import config
from .vision_client import VisionServiceClient
from .motion_controller import MotionController
from .coordinate_transformer import CoordinateTransformer 
import asyncio


class ImageServoing:
    def __init__(self, vision_client: VisionServiceClient, motion_controller: MotionController, coord_transformer: CoordinateTransformer):
        self.vision_client = vision_client
        self.motion_controller = motion_controller
        self.coord_transformer = coord_transformer
        self.robot_id = config.ROBOT_ID
        self.max_attempts = 5 # 최대 5번 시도
        self.position_tolerance_m = 0.01 # 5mm 오차 허용

    async def align_to_standby_pose(self, button_id: int) -> bool:
        """
        [최종 버전] '준비 위치'에 도달할 때까지 '측정->이동'을 반복합니다.
        """
        self._log(f"버튼 ID {button_id}에 대한 시각 정렬(1단계)을 시작합니다.")

        for attempt in range(self.max_attempts):
            self._log(f"--- 정렬 시도 #{attempt + 1} ---")
            
            # 1. 비전 서비스로부터 2D 점 4개 획득
            response = await self.vision_client.request_button_status(self.robot_id, button_id)
            if not response or not response.success or not response.points:
                self._log("버튼의 2D 점 정보를 얻지 못했습니다.", error=True)
                await asyncio.sleep(0.1); continue  # 0.5 → 0.1로 줄임
            
            image_points_2d = np.array([[p.x, p.y] for p in response.points], dtype=np.float32)

            button_center_px = np.mean(image_points_2d, axis=0)
            
            image_center_px = np.array([config.IMAGE_WIDTH_PX / 2, config.IMAGE_HEIGHT_PX / 2])
            center_offset = np.linalg.norm(button_center_px - image_center_px)
            self._log(f"[정렬 오차] 버튼 중심과 이미지 중심 거리: {center_offset:.2f}px", info=True)

            # 2. 현재 로봇팔의 Pose(FK) 획득
            current_robot_transform = self.motion_controller._get_current_transform()

            # 3. 목표 '준비 위치'의 3D 좌표 계산
            target_xyz, target_orientation = self.coord_transformer.get_target_pose_from_points(
                image_points_2d, current_robot_transform)
            
            if target_xyz is None:
                self._log("목표 '준비 위치' 계산 실패.", error=True); continue

            # [수정] 계산된 위치와 방향으로 IK를 이용해 이동
            if not self.motion_controller.move_to_pose_ik(target_xyz, target_orientation):
                self._log("IK 이동 실패.", error=True); continue
            
            # 4. 현재 위치와 목표 위치 사이의 오차 계산
            current_pos = self.motion_controller._get_current_transform()[:3, 3]
            offset_vector = target_xyz - current_pos
            move_vector = config.SERVOING_KP * offset_vector  # P gain
            move_vector = np.clip(move_vector, -config.SERVOING_MAX_MOVE_M, config.SERVOING_MAX_MOVE_M)  # Damping

            if not self.motion_controller.move_relative_cartesian(move_vector):
                self._log("상대 IK 이동 실패.", error=True); continue
            # 5. 정렬 완료 확인
            final_pos = self.motion_controller._get_current_transform()[:3, 3]
            error = np.linalg.norm(final_pos - target_xyz)
            self._log(f"정렬 이동 완료. 최종 오차: {error*1000:.2f} mm")
            await asyncio.sleep(0.1)
            
            # 6. 오차가 허용 범위 내인지 확인
            if error < self.position_tolerance_m:
                self._log("✅ '준비 위치' 정렬 성공.")
                return True

        self._log(f"❌ 최대 시도 횟수 내에 정렬하지 못했습니다.", error=True)
        return False
        
    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")