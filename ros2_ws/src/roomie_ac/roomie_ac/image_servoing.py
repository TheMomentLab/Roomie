# roomie_arm_control/image_servoing.py

import numpy as np
import asyncio
from . import config
from .vision_client import VisionServiceClient
from .motion_controller import MotionController

class ImageServoing:
    """
    카메라 이미지 피드백을 기반으로 로봇 팔을 목표 지점에 정렬하는 클래스 (정렬 전문가).
    """
    def __init__(self, vision_client: VisionServiceClient, motion_controller: MotionController):
        """
        정렬에 필요한 도구들(vision_client, motion_controller)을 전달받습니다.
        """
        self.vision_client = vision_client
        self.motion_controller = motion_controller

        # --- 정렬 제어 파라미터 ---
        self.robot_id = 1
        self.image_center_x = 0.5  # 목표 이미지의 중심 x 좌표 (정규화)
        self.image_center_y = 0.5  # 목표 이미지의 중심 y 좌표 (정규화)
        
        # 정렬이 완료되었다고 판단할 오차 허용 범위 (정규화 기준)
        self.align_threshold = 0.01 
        # 제어 루프 주기 (초). 너무 빠르면 로봇 움직임이 불안정해질 수 있음
        self.loop_rate_sec = 0.1   
        # 최대 시도 횟수 (무한 루프 방지)
        self.max_attempts = 50     
        # 비례 제어 게인(gain). 로봇의 반응 속도를 결정. 이 값을 조절하며 최적점을 찾아야 함.
        self.K_p = 0.05            

    async def align_to_target(self, button_id: int) -> bool:
        """
        [Public] 특정 버튼 ID를 받아, 해당 버튼이 이미지 중앙에 오도록 팔을 정렬합니다.
        """
        self._log(f"버튼 ID {button_id}에 대한 이미지 서보잉 정렬을 시작합니다.")

        for attempt in range(self.max_attempts):
            # ... (1, 2, 3번 단계는 동일) ...

            # 4. 3D 공간에서의 이동 벡터 계산 (수정된 최종 로직)
            # 이 로직은 사용자가 제공한 좌표계 관계에 따라 작성되었습니다.
            # --------------------------------------------------------------------
            # 이미지 세로 오차(error_y) -> 카메라 +Z축 이동 -> 엔드 이펙터 -X축 이동
            move_x_tool = -self.K_p * error_y
            
            # 이미지 가로 오차(error_x) -> 카메라 +Y축 이동 -> 엔드 이펙터 +Y축 이동
            move_y_tool = self.K_p * error_x
            
            # 정렬 단계에서는 전/후진(Z축)을 하지 않습니다.
            move_z_tool = 0.0
            # --------------------------------------------------------------------
            
            move_vector_local = np.array([move_x_tool, move_y_tool, move_z_tool])

            # 5. MotionController를 통해 상대 이동 실행
            self._log(f"상대 이동 명령 (tool_xyz): {np.round(move_vector_local, 4)}")
            success = self.motion_controller.move_relative_cartesian(move_vector_local)
            if not success:
                self._log("MotionController가 상대 이동에 실패했습니다.", error=True)
                await asyncio.sleep(0.5)

            # 6. 제어 루프 주기만큼 대기
            await asyncio.sleep(self.loop_rate_sec)

        self._log(f"최대 시도 횟수({self.max_attempts}) 내에 정렬하지 못했습니다.", error=True)
        return False


    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")