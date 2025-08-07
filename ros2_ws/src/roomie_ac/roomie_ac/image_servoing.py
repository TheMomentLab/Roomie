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
        self.position_tolerance_m = 0.002
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
            response = self.vision_client.request_button_status(self.robot_id, button_id)
            if not response or not response.success:
                self._log("버튼 정보를 얻지 못했습니다. 시야에 없는지 확인하세요.", error=True)
                await asyncio.sleep(0.5)
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

            # ======================= [핵심 수정 시작] =======================
            # PBVS 모드의 안정성을 위해, 비전으로 계산된 방향 대신
            # MODEL_ONLY 모드처럼 항상 로봇 베이스와 정렬된 방향을 사용하도록 강제합니다.
            # 이렇게 하면 press_forward_x()가 항상 예측 가능한 방향(로봇의 X축)으로 동작합니다.
            stable_orientation = np.eye(3)
            # ======================== [핵심 수정 끝] ========================

            # 5. 현재 위치와 최종 목표 사이의 거리 오차 계산
            current_pos = current_robot_transform[:3, 3]
            position_error = np.linalg.norm(current_pos - target_xyz)
            self._log(f"목표까지의 남은 거리: {position_error*1000:.2f} mm")

            # 6. 오차가 허용 범위 내이면 성공으로 판단하고 루프 종료
            if position_error < self.position_tolerance_m:
                self.last_target_pose = target_xyz
                # 성공 시에도 안정화된 방향을 저장합니다.
                self.last_target_orientation = stable_orientation
                self._log(f"✅ '준비 위치' 정렬 성공. 최종 목표 Pose를 저장했습니다: {np.round(self.last_target_pose, 4)}")
                return True

            # 7. 이번 스텝에서 이동할 '중간 목표 지점'을 계산 (비례 이동)
            move_vector = target_xyz - current_pos
            step_target_xyz = current_pos + config.SERVOING_MOVE_GAIN * move_vector
            self._log(f"  -> 이번 스텝 목표 위치: {np.round(step_target_xyz, 4)}")
            
            # 8. 계산된 중간 목표 지점으로 한 스텝 이동 (안정화된 방향 사용)
            if not self.motion_controller.move_to_pose_ik(step_target_xyz, stable_orientation):
                self._log("IK 이동 스텝 실패. 목표에 도달할 수 없습니다.", error=True)
                return False

            await asyncio.sleep(0.2)

        self._log(f"❌ 최대 시도 횟수({self.max_attempts}회) 내에 정렬하지 못했습니다.", error=True)
        return False

        
    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")