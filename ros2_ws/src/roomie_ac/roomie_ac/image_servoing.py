import numpy as np
import asyncio
from . import config
from .vision_client import VisionServiceClient
from .motion_controller import MotionController

class ImageServoing:
    """
    카메라 이미지 피드백을 기반으로 로봇 팔을 목표 지점에 정렬하는 클래스 (정렬 전문가).
    PD 제어기를 사용하여 안정성을 향상시켰습니다.
    """
    def __init__(self, vision_client: VisionServiceClient, motion_controller: MotionController):
        self.vision_client = vision_client
        self.motion_controller = motion_controller

        # --- 정렬 제어 파라미터 ---
        self.robot_id = config.ROBOT_ID
        self.image_center_x = 0.5
        self.image_center_y = 0.5
        
        self.align_threshold_x = 0.005 # X축 정렬 완료 허용 오차 (정규화 기준, 0.5% 이내)
        self.align_threshold_y = 0.005 # Y축 정렬 완료 허용 오차
        self.loop_rate_sec = 0.2
        self.max_attempts = 30
        
        # [수정] PD 제어기 게인(gain) 값 (필요시 이 값을 조절하여 성능 튜닝)
        self.K_p = 0.08  # 비례 게인 (P): 오차에 비례하여 반응.
        self.K_d = 0.02  # 미분 게인 (D): 오차의 변화율에 반응. 진동을 억제.

    async def align_to_target(self, button_id: int) -> bool:
        """
        [Public] 특정 버튼 ID를 받아, 해당 버튼이 이미지 중앙에 오도록 팔을 정렬합니다.
        """
        self._log(f"버튼 ID {button_id}에 대한 이미지 서보잉(PD Control) 정렬을 시작합니다.")

        # PD 제어를 위한 이전 오차 값 초기화
        last_error_x = 0.0
        last_error_y = 0.0

        for attempt in range(self.max_attempts):
            self._log(f"--- 정렬 시도 #{attempt + 1} ---")

            # 1. Vision Service로부터 현재 버튼 위치 정보 요청
            response = await self.vision_client.request_button_status(self.robot_id, [button_id])
            if not response or not response.success or response.sizes[0] <= 0:
                self._log("버튼 위치 정보를 얻는 데 실패했습니다. 0.5초 후 재시도합니다.", error=True)
                await asyncio.sleep(0.5)
                continue

            current_x = response.xs[0]
            current_y = response.ys[0]
            
            # 2. 목표 지점(이미지 중앙)과의 오차 계산
            error_x = self.image_center_x - current_x
            error_y = self.image_center_y - current_y
            self._log(f"현재 위치 (x:{current_x:.4f}, y:{current_y:.4f}), 오차 (x:{error_x:.4f}, y:{error_y:.4f})")

            # 3. 정렬 완료 조건 확인
            if abs(error_x) < self.align_threshold_x and abs(error_y) < self.align_threshold_y:
                self._log("✅ 목표 지점에 성공적으로 정렬되었습니다.")
                return True

            # 4. PD 제어 법칙에 따른 3D 공간 이동 벡터 계산
            # 오차의 변화율 (Derivative term)
            error_delta_x = error_x - last_error_x
            error_delta_y = error_y - last_error_y

            # P 제어량 + D 제어량
            control_signal_x = self.K_p * error_x + self.K_d * (error_delta_x / self.loop_rate_sec)
            control_signal_y = self.K_p * error_y + self.K_d * (error_delta_y / self.loop_rate_sec)

            # 현재 오차를 다음 루프를 위해 저장
            last_error_x = error_x
            last_error_y = error_y

            # 핸드-아이 좌표계 관계에 따라 이동 벡터 계산
            move_x_tool = control_signal_y
            move_y_tool = control_signal_x
            move_z_tool = 0.0 # 정렬 단계에서는 전/후진 안함
            
            move_vector_local = np.array([move_x_tool, move_y_tool, move_z_tool])

            # 5. MotionController를 통해 상대 이동 실행
            self._log(f"PD 제어 이동량 (x:{move_x_tool:.4f}, y:{move_y_tool:.4f})")
            success = self.motion_controller.move_relative_cartesian(move_vector_local)
            if not success:
                self._log("MotionController가 상대 이동에 실패했습니다. 다음 시도를 진행합니다.", error=True)

            # 6. 제어 루프 주기만큼 대기
            await asyncio.sleep(self.loop_rate_sec)

        self._log(f"❌ 최대 시도 횟수({self.max_attempts}) 내에 정렬하지 못했습니다.", error=True)
        return False

    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")