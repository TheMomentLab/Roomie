import numpy as np
import asyncio
from . import config
from .vision_client import VisionServiceClient
from .motion_controller import MotionController
from .coordinate_transformer import CoordinateTransformer

class ImageServoing:
    def __init__(self, vision_client: VisionServiceClient, motion_controller: MotionController, coord_transformer: CoordinateTransformer):
        self.vision_client = vision_client
        self.motion_controller = motion_controller
        self.coord_transformer = coord_transformer
        self.robot_id = config.ROBOT_ID
        self.max_attempts = 10000
        self.last_target_pose = None
        self.last_target_orientation = None

    async def align_to_standby_pose(self, button_id: int) -> bool:
        """
        [수정됨] 버튼의 정면 방향을 기준으로 '준비 위치'를 계산하여 정렬합니다.
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

            # 4. [수정] 버튼의 6D Pose 행렬(T_base_to_btn) 획득
            T_base_to_btn = self.coord_transformer.get_button_pose_in_base_frame(
                image_points_2d, current_robot_transform)

            if T_base_to_btn is None:
                self._log("버튼 Pose 계산 실패.", error=True)
                continue
            
            # 5. [핵심 수정] 버튼의 정면(Z축)을 기준으로 '준비 위치' 계산
            button_orientation_matrix = T_base_to_btn[:3, :3]
            button_pos = T_base_to_btn[:3, 3]
            
            # 버튼의 Z축 벡터 (버튼 정면 방향) 추출. [:, 2]는 3번째 열을 의미합니다.
            button_z_vector = button_orientation_matrix[:, 2]

            # 이 벡터를 따라 뒤로 물러나 '준비 위치(standby_pos)' 설정
            standby_pos = button_pos - button_z_vector * config.SERVOING_STANDBY_DISTANCE_M
            
            # 6. 현재 위치와 목표 '준비 위치' 사이의 거리 오차 계산
            current_pos = current_robot_transform[:3, 3]
            position_error = np.linalg.norm(standby_pos - current_pos)
            self._log(f"준비 위치까지 남은 거리: {position_error*1000:.2f} mm")

            # 7. 오차 허용 범위 내이면 성공
            if position_error < config.SERVOING_POSITION_TOLERANCE_M:
                self.last_target_pose = standby_pos
                # 나중을 위해 버튼의 방향 행렬을 저장
                self.last_target_orientation = button_orientation_matrix
                self._log("✅ '준비 위치' 정렬 성공.")
                return True

            # 8. 이번 스텝에서 이동할 거리와 방향 결정
            move_direction = (standby_pos - current_pos) / position_error
            step_distance = min(config.SERVOING_MAX_STEP_M, position_error)
            
            if step_distance < config.IK_MIN_STEP_M:
                self._log(f"이동 거리가 최소 스텝({config.IK_MIN_STEP_M*1000:.1f}mm)보다 작아 현재 스텝을 건너뜁니다.")
                await asyncio.sleep(0.1)
                continue
            
            step_target_xyz = current_pos + move_direction * step_distance

            # 9. 계산된 목표 지점으로 '한 스텝' 이동
            if not await self.motion_controller.move_to_pose_ik(step_target_xyz, blocking=True):
                self._log("IK 이동 스텝 실패.", error=True)
                return False

        self._log(f"❌ 최대 시도 횟수({self.max_attempts}회) 내에 정렬하지 못했습니다.", error=True)
        return False

    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")
