import rclpy
import asyncio
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from roomie_msgs.action import SetPose, ClickButton
from .vision_client import VisionServiceClient
from .serial_manager import SerialManager
from .kinematics_solver import KinematicsSolver
from .motion_controller import MotionController
from .image_servoing import ImageServoing
from .ros_joint_publisher import ROSJointPublisher
from .coordinate_transformer import CoordinateTransformer
from .config import (
    Pose, POSE_ANGLES_DEG,
    ControlMode, CONTROL_STRATEGY,
    ROBOT_ID,
    ButtonActionStatus,
    PREDEFINED_BUTTON_POSES_M, 
    SERVOING_STANDBY_DISTANCE_M,
    PRESS_FORWARD_DISTANCE_M,
    IMAGE_WIDTH_PX,  # config에서 이미지 크기 변수 임포트
    IMAGE_HEIGHT_PX,    
)

class ArmActionServer(Node):
    """
    팔 제어와 관련된 모든 Action 요청을 처리하는 메인 서버 노드
    """
    def __init__(self):
        super().__init__('arm_action_server')

        # --- 모든 객체 생성 ---
        self.serial_manager = SerialManager()
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher()
        self.vision_client = VisionServiceClient()
        self.coord_transformer = CoordinateTransformer()
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        self.image_servo = ImageServoing(self.vision_client, self.motion_controller, self.coord_transformer) # 수정됨
        #  connect()의 반환값을 변수에 저장하고, None인지 명시적으로 확인합니다.
        initial_angles = self.serial_manager.connect()
        if initial_angles is None:
            self.get_logger().fatal("시리얼 연결에 실패하여 노드를 종료합니다.")
            return

        # --- Action 서버 생성 ---
        self._set_pose_server = ActionServer(self, SetPose, '/arm/action/set_pose', self.set_pose_callback)
        self._click_button_server = ActionServer(self, ClickButton, '/arm/action/click_button', self.click_button_callback)

        self.get_logger().info("✅ Arm Action Server가 성공적으로 시작되었습니다.")


    def set_pose_callback(self, goal_handle):
        """[지휘] SetPose Action 요청을 처리합니다."""

        if goal_handle.request.robot_id != ROBOT_ID:
            msg = f"요청된 robot_id({goal_handle.request.robot_id})가 현재 로봇 ID({ROBOT_ID})와 일치하지 않습니다."
            self.get_logger().error(msg)
            goal_handle.abort()
            return SetPose.Result(robot_id=ROBOT_ID, success=False)

        self.get_logger().info(f"[DEBUG] 수신된 pose_id: {goal_handle.request.pose_id}")
        try:
            requested_pose = Pose(goal_handle.request.pose_id)
            self.get_logger().info(f"[DEBUG] Enum 변환 결과: {requested_pose.name}")
        except ValueError:
            self.get_logger().error(f"[ERROR] 유효하지 않은 pose_id 수신: {goal_handle.request.pose_id}")
            goal_handle.abort()
            return SetPose.Result(robot_id=ROBOT_ID, success=False)

        # ➕ 먼저 관측 자세로 이동
        self.get_logger().info("🟡 먼저 관측 자세(OBSERVE)로 이동합니다.")
        self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])

        target_angles_deg = POSE_ANGLES_DEG.get(requested_pose)
        if target_angles_deg is not None:
            self.get_logger().info(f"==> [DEBUG] MotionController에 전달할 목표 각도: {target_angles_deg}")
            success = self.motion_controller.move_to_angles_deg(target_angles_deg)
            self.get_logger().info(f"<== [DEBUG] MotionController로부터 반환된 결과: success={success}")

            if success:
                self.get_logger().info(f"'{requested_pose.name}' 자세로 이동 완료.")
                goal_handle.succeed()
                return SetPose.Result(robot_id=ROBOT_ID, success=True)

        msg = f"'{requested_pose.name}' 자세로 이동 실패."
        self.get_logger().error(msg)
        self.get_logger().info("안전 모드: 이동 실패로 관측 자세로 복귀합니다.")
        self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])

        goal_handle.abort()
        return SetPose.Result(robot_id=ROBOT_ID, success=False)
    
    async def click_button_callback(self, goal_handle):
        """[최종 버전] 두 가지 제어 전략을 모두 지원하는 시나리오 지휘자."""
        result = ClickButton.Result(); result.robot_id = ROBOT_ID
        feedback = ClickButton.Feedback()
        button_id = goal_handle.request.button_id

        self.get_logger().info(f"ClickButton 목표 수신: button_id={button_id} (제어 모드: {CONTROL_STRATEGY.name})")
        
        try:
            # Step 0: 시작 전 항상 관측 자세로 이동
            self.get_logger().info("🟡 시작 전 관측 자세로 이동합니다.")
            self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])

            # =================== HYBRID 모드 (PBVS 사용) ===================
            if CONTROL_STRATEGY == ControlMode.HYBRID:
                self.get_logger().info(">> [하이브리드 제어]를 시작합니다.")
                
                # Step 1H: 정보 수집
                response = await self.vision_client.request_button_status(ROBOT_ID, button_id)
                if not response or not response.success: # 응답 실패 또는 success=False 체크
                    raise RuntimeError("버튼 위치 정보 획득 실패 (응답 없음 또는 success=False)")

                # ✨ [핵심 수정] 정규화된 중심점/크기를 픽셀 좌표로 변환하고 4개의 점을 재구성합니다.
                self.get_logger().info("중심점/크기 정보를 4점 좌표로 재구성합니다.")
                
                # 1. 정규화된 값을 픽셀 값으로 변환
                center_x_px = response.x * IMAGE_WIDTH_PX
                center_y_px = response.y * IMAGE_HEIGHT_PX
                width_px = response.size * IMAGE_WIDTH_PX # size는 너비 기준 정규화였으므로 너비로 복원
                radius_px = width_px / 2.0
                
                # 2. 4개의 점(numpy 배열) 생성
                image_points_2d = np.array([
                    [center_x_px + radius_px, center_y_px], # 오른쪽
                    [center_x_px - radius_px, center_y_px], # 왼쪽
                    [center_x_px, center_y_px - radius_px], # 위쪽
                    [center_x_px, center_y_px + radius_px]  # 아래쪽
                ], dtype=np.float32)

                # 재구성된 4개의 점을 사용해 1차 목표 위치 계산
                current_transform = self.motion_controller._get_current_transform()
                target_xyz, target_orientation = self.coord_transformer.get_target_pose_from_points(image_points_2d, current_transform)
                
                if target_xyz is None:
                    raise RuntimeError("1차 목표 '준비 위치' 계산 실패")

            # =================== MODEL_ONLY 모드 (좌표 직접 사용) ===================
            elif CONTROL_STRATEGY == ControlMode.MODEL_ONLY:
                self.get_logger().info(">> [모델 전용 제어]를 시작합니다.")
                
                # Step 1M: config에서 미리 정의된 3D 목표 좌표 가져오기
                target_3d_pose = PREDEFINED_BUTTON_POSES_M.get(button_id)
                if target_3d_pose is None:
                    raise RuntimeError(f"config에 button_id {button_id}의 좌표가 없습니다.")
                
                # Step 2M: '준비 위치'로 이동
                feedback.status = ButtonActionStatus.MOVING_TO_TARGET
                goal_handle.publish_feedback(feedback)
                
                # 목표 방향은 로봇 베이스를 향하도록 간단히 설정 (수직 하강)
                # 좀 더 복잡한 방향 제어가 필요하면 이 부분을 수정할 수 있음
                target_orientation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) # 예시: 아래를 바라보는 방향

                # 버튼 위치에서 Z축 방향으로 후퇴하여 '준비 위치' 계산
                standby_pose = target_3d_pose.copy()
                standby_pose[2] += SERVOING_STANDBY_DISTANCE_M # Z축으로 들어올림

                self.get_logger().info(f"모델 기반 '준비 위치'({standby_pose})로 이동합니다.")
                if not self.motion_controller.move_to_pose_ik(standby_pose, target_orientation):
                    raise RuntimeError("준비 위치로 이동(MODEL_ONLY) 실패")

            # =================== 공통 실행 단계 (누르기 및 후퇴) ===================
            self.get_logger().info("✅ 정렬 완료. 공통 누르기/후퇴 단계를 시작합니다.")
            await asyncio.sleep(0.5)

            # Step 2: 버튼 누르기
            self.get_logger().info(">> 버튼 누르기 수행")
            feedback.status = ButtonActionStatus.PRESSING
            goal_handle.publish_feedback(feedback)
            if not self.motion_controller.press_forward(distance_m=PRESS_FORWARD_DISTANCE_M):
                raise RuntimeError("누르기 동작 실패")

            # Step 3: 후퇴
            self.get_logger().info(">> 후퇴 동작 수행")
            feedback.status = ButtonActionStatus.RETRACTING
            goal_handle.publish_feedback(feedback)
            if not self.motion_controller.retreat(distance_m=PRESS_FORWARD_DISTANCE_M):
                raise RuntimeError("후퇴 동작 실패")

        except Exception as e:
            # ... (기존 예외 처리 로직과 동일) ...
            self.get_logger().error(f"🔴 작업 실패: {e}")
            goal_handle.abort()
            result.success = True  # 실패 시에도 success=True로 설정
            result.message = str(e)
            self.get_logger().info("→ 안전을 위해 관측 자세로 복귀합니다.")
            self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])
            return result
            
        success_msg = f"🟢 버튼 {button_id} 클릭 임무 성공적으로 완료."
        self.get_logger().info(success_msg)
        goal_handle.succeed()
        result.success = True
        result.message = success_msg
        self.get_logger().info("→ 임무 종료. 관측 자세로 복귀합니다.")
        self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])
        return result
    
def main(args=None):
    rclpy.init(args=args)
    # [수정] 노드 생성 실패 시를 대비하여 main 함수에서 처리
    try:
        arm_action_server = ArmActionServer()
        if arm_action_server.serial_manager.is_ready: # 시리얼 연결 성공 시에만 spin
            executor = MultiThreadedExecutor()
            rclpy.spin(arm_action_server, executor=executor)
    except Exception as e:
        # 노드 생성 중 발생할 수 있는 예외 처리
        print(f"노드 실행 중 심각한 오류 발생: {e}")
    finally:
        if 'arm_action_server' in locals() and rclpy.ok():
            arm_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
