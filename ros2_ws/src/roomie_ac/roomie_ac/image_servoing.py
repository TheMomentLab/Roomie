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
        self.max_attempts = 20000
        self.position_tolerance_m = 0.001
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
            # stable_orientation = np.eye(3)
            stable_orientation = calculated_orientation

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

            
                # ======================= [핵심 수정 시작] =======================
            # 7. 이번 스텝에서 이동할 '중간 목표 지점'을 고정 거리 방식으로 계산합니다.

            # (1) 현재 위치에서 최종 목표까지의 방향 벡터 계산
            move_vector = target_xyz - current_pos
            direction_vector = move_vector / position_error # 정규화된 방향 벡터

            # (2) 이번 스텝에 이동할 거리는 '고정값'과 '남은 거리' 중 작은 값으로 선택 (오버슈팅 방지)
            step_distance = min(config.SERVOING_CONSTANT_STEP_M, position_error)

            # (3) 중간 목표 지점 계산
            step_target_xyz = current_pos + direction_vector * step_distance
            self._log(f"  -> 이번 스텝 목표 위치 (고정 이동): {np.round(step_target_xyz, 4)}")
            # ======================== [핵심 수정 끝] ========================

                
            # 8. 계산된 중간 목표 지점으로 한 스텝 이동 (안정화된 방향 사용)
            if not self.motion_controller.move_to_pose_ik(step_target_xyz, stable_orientation):
                self._log("IK 이동 스텝 실패. 목표에 도달할 수 없습니다.", error=True)
                return False

            await asyncio.sleep(0.2)

        self._log(f"❌ 최대 시도 횟수({self.max_attempts}회) 내에 정렬하지 못했습니다.", error=True)
        return False

    async def align_with_ibvs(self, button_id: int) -> bool:
        """
        [수정됨] 하이브리드 IBVS 방식으로 버튼을 정렬합니다.
        - 좌/우, 상/하는 픽셀 오차에 비례하여 움직입니다.
        - 전/후는 고정된 거리(SERVOING_CONSTANT_STEP_M)만큼 움직입니다.
        """
        self._log(f"버튼 ID {button_id}에 대한 하이브리드 IBVS 정렬을 시작합니다.")

        for attempt in range(self.max_attempts):
            self._log(f"--- IBVS 정렬 시도 #{attempt + 1} ---")

            # 1. 비전 정보 획득 (기존과 동일)
            response = self.vision_client.request_button_status(self.robot_id, button_id)
            if not response or not response.success:
                self._log("버튼 정보를 얻지 못했습니다. 시야에 없는지 확인하세요.", error=True)
                await asyncio.sleep(0.5)
                continue

            # 2. 픽셀 좌표 및 크기 정의 (기존과 동일)
            target_x_px = config.IMAGE_WIDTH_PX / 2
            target_y_px = config.IMAGE_HEIGHT_PX / 2
            current_x_px = response.x * config.IMAGE_WIDTH_PX
            current_y_px = response.y * config.IMAGE_HEIGHT_PX

            target_size_px = config.IMAGE_HEIGHT_PX * config.IBVS_TARGET_SIZE_RATIO
            current_size_px = np.sqrt(response.size * config.IMAGE_WIDTH_PX * config.IMAGE_HEIGHT_PX)

            # 3. 픽셀 오차 계산 (기존과 동일)
            error_x = current_x_px - target_x_px
            error_y = current_y_px - target_y_px

            # 4. 성공 여부 판단 (기존과 동일)
            is_centered = abs(error_x) < config.IBVS_PIXEL_TOLERANCE and \
                        abs(error_y) < config.IBVS_PIXEL_TOLERANCE
            is_at_correct_distance = abs(current_size_px - target_size_px) < (config.IBVS_PIXEL_TOLERANCE * 2)

            if is_centered and is_at_correct_distance:
                self._log(f"✅ IBVS 정렬 성공.")
                current_transform = self.motion_controller._get_current_transform()
                self.last_target_pose = current_transform[:3, 3]
                self.last_target_orientation = current_transform[:3, :3]
                return True

            # ======================= [핵심 수정 시작] =======================
            # 5. 이동 벡터 계산
            # (1) 좌/우, 상/하 이동량은 픽셀 오차에 비례하여 계산
            move_y = -error_x * config.IBVS_GAIN_X  # Y축(좌/우)
            move_z = error_y * config.IBVS_GAIN_Y   # Z축(상/하)

            # (2) 전/후 이동량은 '고정 거리'를 사용
            move_x = 0.0
            # 어느정도 중앙에 정렬되었을 때만 전/후진 수행 (안정성 확보)
            if abs(error_x) < (config.IBVS_PIXEL_TOLERANCE * 5) and abs(error_y) < (config.IBVS_PIXEL_TOLERANCE * 5):
                if current_size_px < target_size_px: # 너무 멀면
                    move_x = config.SERVOING_CONSTANT_STEP_M # 고정값만큼 전진
                else: # 너무 가까우면
                    move_x = -config.SERVOING_CONSTANT_STEP_M # 고정값만큼 후진

            self._log(f"픽셀 오차: [X:{error_x:.1f}, Y:{error_y:.1f}], 크기: {current_size_px:.1f}/{target_size_px:.1f}")
            move_vector_cam = np.array([move_x, move_y, move_z])
            self._log(f"  -> 계산된 이동 벡터(카메라 기준): {np.round(move_vector_cam*1000, 2)} mm")
            # ======================== [핵심 수정 끝] ========================

            # 6. 계산된 벡터만큼 로봇을 상대 이동 (기존과 동일)
            if not self.motion_controller.move_relative_cartesian(move_vector_cam):
                self._log("IBVS 이동 스텝 실패.", error=True)
                return False

            await asyncio.sleep(0.1)

        self._log(f"❌ 최대 시도 횟수({self.max_attempts}회) 내에 정렬하지 못했습니다.", error=True)
        return False
        
    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")