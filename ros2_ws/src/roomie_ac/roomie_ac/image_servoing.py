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
        self.max_attempts = 10000
        # ➕ [추가] 정렬 성공 시의 최종 목표 Pose와 Orientation을 저장할 속성
        self.last_target_pose = None
        self.last_target_orientation = None

    async def align_to_standby_pose(self, button_id: int) -> bool:
        """
        [수정됨] '비례 이동' 방식으로 정렬하고, 안정성을 위해 목표 방향을 고정합니다.
        """
        self._log(f"버튼 ID {button_id}에 대한 비례 이동 서보잉을 시작합니다.")
        self.last_target_pose = None
        self.last_target_orientation = None

        for attempt in range(self.max_attempts):
            self._log(f"--- 정렬 시도 #{attempt + 1} ---")

            # 1. 비전 서비스로부터 버튼 위치 정보 획득
            response = await self.vision_client.request_button_status(self.robot_id, button_id)
            if not response or not response.success:
                self._log("버튼 정보를 얻지 못했습니다. 시야에 없는지 확인하세요.", error=True)
                await asyncio.sleep(0.2)
                continue
            
            # 2. 2D 이미지 좌표 생성
            center_x_px = response.x * config.IMAGE_WIDTH_PX
            center_y_px = response.y * config.IMAGE_HEIGHT_PX
            pixel_area = response.size * config.IMAGE_WIDTH_PX * config.IMAGE_HEIGHT_PX
            pixel_diameter = np.sqrt(max(0, pixel_area))
            radius_px = pixel_diameter / 2.0
            
            image_points_2d = np.array([
                [center_x_px + radius_px, center_y_px], [center_x_px - radius_px, center_y_px],
                [center_x_px, center_y_px + radius_px], [center_x_px, center_y_px - radius_px]
            ], dtype=np.float32)

            # 3. 현재 로봇팔의 Pose(FK) 획득
            current_robot_transform = self.motion_controller._get_current_transform()

            # 4. PnP를 통해 최종 목표 위치/방향 계산 (로봇 베이스 기준)
            target_xyz, calculated_orientation = self.coord_transformer.get_target_pose_from_points(
                image_points_2d, current_robot_transform)

            if target_xyz is None:
                self._log("목표 '준비 위치' 계산 실패. PnP 에러일 수 있습니다.", error=True)
                continue
            
            # [수정] IK에 전달할 안정적인 방향 '벡터' 정의
            forward_vector = np.array([1.0, 0.0, 0.0])

            # 5. 현재 위치와 최종 목표 사이의 거리 오차 계산
            current_pos = current_robot_transform[:3, 3]
            position_error = np.linalg.norm(target_xyz - current_pos)
            self._log(f"목표까지의 남은 거리: {position_error*1000:.2f} mm")

            # 6. 오차 허용 범위 내이면 성공
            if position_error < config.SERVOING_POSITION_TOLERANCE_M:
                self.last_target_pose = target_xyz
                self.last_target_orientation = forward_vector
                self._log("✅ '준비 위치' 정렬 성공.")
                return True

            # ======================= [핵심 알고리즘 개선] =======================
            # 7. 이번 스텝에서 이동할 거리와 방향 결정
            move_direction = (target_xyz - current_pos) / position_error
            step_distance = min(config.SERVOING_MAX_STEP_M, position_error)
            step_target_xyz = current_pos + move_direction * step_distance
            # ===================================================================
            
            # 8. 계산된 목표 지점으로 한 스텝 이동 (방향 벡터 사용)
            if not self.motion_controller.move_to_pose_ik(step_target_xyz, forward_vector):
                self._log("IK 이동 스텝 실패.", error=True)
                return False

            await asyncio.sleep(0.2)

        self._log(f"❌ 최대 시도 횟수({self.max_attempts}회) 내에 정렬하지 못했습니다.", error=True)
        return False

        
    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")