import numpy as np
import asyncio
from . import config
from .vision_client import VisionServiceClient
from .motion_controller import MotionController
from .coordinate_transformer import CoordinateTransformer
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ImageServoing:
    """
    카메라를 이용한 시각 서보잉을 담당하는 클래스입니다.
    - align_to_standby_pose: 버튼을 정면으로 바라보는 준비 자세까지 로봇팔을 정렬합니다.
    """
    def __init__(self, vision_client: VisionServiceClient, motion_controller: MotionController, coord_transformer: CoordinateTransformer):
        self.vision_client = vision_client
        self.motion_controller = motion_controller
        self.coord_transformer = coord_transformer
        self.robot_id = config.ROBOT_ID
        self.max_attempts = 200  # 최대 시도 횟수
        self.last_target_pose = None
        self.last_target_orientation = None
        self.marker_pub = self.motion_controller.joint_publisher.create_publisher(Marker, '/debug/target_markers', 10)
        self._log("RViz 디버그 마커 퍼블리셔 생성됨.")

    def _set_successful_alignment_state(self, pose: np.ndarray, orientation: np.ndarray):
        """정렬 성공 시, 최종 목표 자세와 방향을 클래스 변수에 저장합니다."""
        self.last_target_pose = pose
        self.last_target_orientation = orientation
        self._log("✅ '준비 위치' 정렬 성공.")

    def _publish_debug_markers(self, button_pos: np.ndarray, standby_pos: np.ndarray):
        """계산된 버튼 위치와 준비 위치를 RViz에 시각화합니다."""
        # 버튼 위치 마커 (빨간색 구)
        marker_btn = Marker()
        marker_btn.header.frame_id = "base_link"
        marker_btn.header.stamp = self.motion_controller.joint_publisher.get_clock().now().to_msg()
        marker_btn.ns = "targets"
        marker_btn.id = 0
        marker_btn.type = Marker.SPHERE
        marker_btn.action = Marker.ADD
        marker_btn.pose.position = Point(x=button_pos[0], y=button_pos[1], z=button_pos[2])
        marker_btn.pose.orientation.w = 1.0
        marker_btn.scale.x = 0.03; marker_btn.scale.y = 0.03; marker_btn.scale.z = 0.03
        marker_btn.color.a = 0.8; marker_btn.color.r = 1.0; marker_btn.color.g = 0.0; marker_btn.color.b = 0.0

        # 준비 위치 마커 (파란색 구)
        marker_sby = Marker(
            header=marker_btn.header, ns=marker_btn.ns, id=1, type=Marker.SPHERE, action=Marker.ADD,
            pose=marker_btn.pose, scale=marker_btn.scale, color=marker_btn.color
        )
        marker_sby.pose.position = Point(x=standby_pos[0], y=standby_pos[1], z=standby_pos[2])
        marker_sby.color.r = 0.0; marker_sby.color.b = 1.0

        self.marker_pub.publish(marker_btn)
        self.marker_pub.publish(marker_sby)

    async def _get_current_button_pose(self, button_id: int) -> np.ndarray | None:
        """비전 데이터를 이용해 현재 프레임에서 버튼의 3D 변환 행렬(T_base_to_btn)을 계산합니다."""
        response = await self.vision_client.request_status(
            mode=config.POSE_ESTIMATION_MODE, robot_id=self.robot_id, button_id=button_id
        )
        if not response or not response.success:
            self._log("버튼 시야 이탈. 자세 계산 실패.", error=True)
            return None

        image_points_2d = None
        if config.POSE_ESTIMATION_MODE == 'corner':
            if hasattr(response, 'corners') and len(response.corners) == 8:
                image_points_2d = np.array(response.corners, dtype=np.float32).reshape((4, 2))
            else:
                self._log("'corner' 모드지만 corners 정보 없음.", error=True)
                return None
        else:
            center_x_px = response.x * config.IMAGE_WIDTH_PX
            center_y_px = response.y * config.IMAGE_HEIGHT_PX
            pixel_area = response.size * config.IMAGE_WIDTH_PX * config.IMAGE_HEIGHT_PX
            pixel_diameter = np.sqrt(max(0, pixel_area))
            radius_px = pixel_diameter / 2.0
            image_points_2d = np.array([
                [center_x_px + radius_px, center_y_px], [center_x_px - radius_px, center_y_px],
                [center_x_px, center_y_px + radius_px], [center_x_px, center_y_px - radius_px]
            ], dtype=np.float32)

        current_robot_transform = self.motion_controller._get_current_transform()
        return self.coord_transformer.get_button_pose_in_base_frame(
            robot_fk_transform=current_robot_transform,
            mode=config.POSE_ESTIMATION_MODE,
            image_points_2d=image_points_2d
        )

    def create_look_at_matrix(self, camera_pos: np.ndarray, target_pos: np.ndarray, world_up: np.ndarray = np.array([0, 0, 1])) -> np.ndarray | None:
        """카메라 위치에서 목표 위치를 바라보는 3x3 회전 행렬을 생성합니다."""
        epsilon = 1e-6

        forward = target_pos - camera_pos
        forward_norm = np.linalg.norm(forward)
        if forward_norm < epsilon: return None
        forward /= forward_norm

        if abs(np.dot(forward, world_up)) > 0.999:
            self._log("카메라가 수직 방향을 향하고 있어 world_up 벡터를 임시 변경합니다.", info=True)
            right = np.cross(np.array([1, 0, 0]), forward)
        else:
            right = np.cross(world_up, forward)

        right_norm = np.linalg.norm(right)
        if right_norm < epsilon: return None
        right /= right_norm

        down = np.cross(forward, right)
        rotation_matrix = np.array([right, down, forward]).T
        return rotation_matrix

    async def align_to_standby_pose(self, button_id: int) -> bool:
        """버튼의 정면 방향을 기준으로 '준비 위치'를 계산하여 로봇팔을 정렬합니다."""
        self._log(f"버튼 ID {button_id}에 대한 비례 이동 서보잉을 시작합니다.")
        self.last_target_pose = None
        self.last_target_orientation = None
        epsilon = 1e-6

        for attempt in range(self.max_attempts):
            self._log(f"--- 정렬 시도 #{attempt + 1} ---")

            T_base_to_btn = await self._get_current_button_pose(button_id)
            if T_base_to_btn is None:
                await asyncio.sleep(0.1)
                continue

            button_orientation_matrix = T_base_to_btn[:3, :3]
            button_pos = T_base_to_btn[:3, 3]
            button_z_vector = button_orientation_matrix[:, 2]
            standby_pos = button_pos - button_z_vector * config.SERVOING_STANDBY_DISTANCE_M

            current_pos = self.motion_controller._get_current_transform()[:3, 3]
            position_error_vec = standby_pos - current_pos
            position_error = np.linalg.norm(position_error_vec)
            self._log(f"준비 위치까지 남은 거리: {position_error * 1000:.2f} mm")

            # --- [디버그용] 계산된 좌표를 RViz에 마커로 표시 ---
            self._publish_debug_markers(button_pos, standby_pos)

            if position_error < config.SERVOING_POSITION_TOLERANCE_M:
                self._set_successful_alignment_state(standby_pos, button_orientation_matrix)
                return True

            move_direction = position_error_vec / (position_error + epsilon)
            step_distance = min(config.SERVOING_MAX_STEP_M, position_error)
            
            if step_distance < config.IK_MIN_STEP_M:
                self._log(f"이동 거리가 최소 스텝보다 작아 성공으로 간주합니다.")
                self._set_successful_alignment_state(standby_pos, button_orientation_matrix)
                return True

            step_target_xyz = current_pos + move_direction * step_distance
            target_orientation = self.create_look_at_matrix(current_pos, button_pos)

            if target_orientation is None:
                self._log("Look-at 방향 행렬 계산 실패.", error=True)
                continue

            # 9. 계산된 위치와 '수정된 방향'으로 로봇팔을 한 스텝 이동
            if not await self.motion_controller.move_to_pose_ik(
                step_target_xyz, orientation=target_orientation, blocking=True
            ):
                self._log("IK 이동 스텝 실패.", error=True)
                return False

            await asyncio.sleep(0.1)

        self._log(f"❌ 최대 시도 횟수({self.max_attempts}회) 내에 정렬하지 못했습니다.", error=True)
        return False

    def _log(self, message: str, info: bool = False, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[ImageServoing][{log_level}] {message}")
