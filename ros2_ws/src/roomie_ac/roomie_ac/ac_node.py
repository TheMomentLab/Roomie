import rclpy
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
    PRE_PRESS_DISTANCE_M,
    ROBOT_ID,
    ButtonActionStatus,
    PREDEFINED_BUTTON_POSES_M  # 미리 정의된 좌표 딕셔너리 임포트  
)

class ArmActionServer(Node):
    """
    팔 제어와 관련된 모든 Action 요청을 처리하는 메인 서버 노드
    """
    def __init__(self):
        super().__init__('arm_action_server')

        # --- 모든 '연주자' 객체 생성 ---
        self.serial_manager = SerialManager()
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher()
        self.vision_client = VisionServiceClient()
        self.coord_transformer = CoordinateTransformer()
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        self.image_servo = ImageServoing(self.vision_client, self.motion_controller)

        # [수정] connect()의 반환값을 변수에 저장하고, None인지 명시적으로 확인합니다.
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
        """[지휘] ClickButton Action의 전체 시나리오를 지휘합니다."""
        result = ClickButton.Result()
        result.robot_id = ROBOT_ID

        if goal_handle.request.robot_id != ROBOT_ID:
            msg = f"요청된 robot_id({goal_handle.request.robot_id})가 현재 로봇 ID({ROBOT_ID})와 일치하지 않습니다."
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.message = msg
            return result

        button_id = goal_handle.request.button_id
        self.get_logger().info(f"ClickButton 목표 수신: button_id={button_id} (제어 모드: {CONTROL_STRATEGY.name})")
        feedback = ClickButton.Feedback()

        try:
            # Step 0: 관측 자세로 이동
            self.get_logger().info("🟡 시작 전에 관측 자세로 이동합니다.")
            self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])

            # Step 1: Vision Service → 버튼 존재 여부 확인
            # [수정] 두 모드 모두 VS에 요청은 보내되, 응답 활용 방식이 달라집니다.
            self.get_logger().info("VS에 버튼 상태를 요청합니다...")
            response = await self.vision_client.request_button_status(ROBOT_ID, button_id)
            if not response or not response.success:
                # `size` 체크는 HYBRID 모드에서만 의미 있으므로 공통 부분에서는 제거
                raise RuntimeError("Vision Service로부터 버튼 정보를 획득하지 못했습니다(not success).")

            # [수정] 제어 전략에 따라 로직 분기
            # =================== MODEL_ONLY 모드 ===================
            if CONTROL_STRATEGY == ControlMode.MODEL_ONLY:
                self.get_logger().info(">> [모델 전용 제어]를 시작합니다.")
                feedback.status = ButtonActionStatus.MOVING_TO_TARGET
                goal_handle.publish_feedback(feedback)

                # Step 2M: config에서 미리 정의된 3D 목표 좌표 가져오기
                target_3d_pose = PREDEFINED_BUTTON_POSES_M.get(button_id)
                if target_3d_pose is None:
                    raise RuntimeError(f"config.py에 button_id {button_id}에 대한 좌표가 정의되지 않았습니다.")
                self.get_logger().info(f"  - 목표 좌표 (정의값): {target_3d_pose}")

                # Step 3M: 준비 위치로 이동 (버튼 바로 앞으로)
                current_transform = self.motion_controller._get_current_transform()
                # Z축 벡터(바라보는 방향)를 사용하여 후퇴할 방향 계산
                forward_vector = current_transform[:3, 2]
                pre_press_pose = target_3d_pose - forward_vector * PRE_PRESS_DISTANCE_M

                if not self.motion_controller.move_to_pose_ik(pre_press_pose):
                    raise RuntimeError("준비 위치(Pre-press)로 이동 실패")
                
                # Step 4M: 이미지 서보잉 건너뛰기
                self.get_logger().info("  - 이미지 서보잉 정렬 단계를 건너뜁니다.")


            # =================== HYBRID 모드 ===================
            elif CONTROL_STRATEGY == ControlMode.HYBRID:
                self.get_logger().info(">> [하이브리드 제어]를 시작합니다.")
                if response.size <= 0:
                    raise RuntimeError("Vision Service 응답의 버튼 크기가 0 이하입니다.")

                button_center_xy_norm = (response.x, response.y)
                button_size_norm = response.size

                # Step 2H: 준비 위치 이동
                feedback.status = ButtonActionStatus.MOVING_TO_TARGET
                goal_handle.publish_feedback(feedback)
                
                current_fk_transform = self.motion_controller._get_current_transform()
                target_3d_pose = self.coord_transformer.calculate_target_pose(
                    button_center_xy_norm, button_size_norm, current_fk_transform
                )
                if target_3d_pose is None:
                    raise RuntimeError("3D 목표 좌표 계산 실패")

                forward_vector = current_fk_transform[:3, 2]
                pre_press_pose = target_3d_pose - forward_vector * PRE_PRESS_DISTANCE_M

                if not self.motion_controller.move_to_pose_ik(pre_press_pose):
                    raise RuntimeError("준비 위치로 이동 실패")

                # Step 3H: 이미지 서보잉 정렬
                self.get_logger().info(">> 이미지 서보잉 정렬 수행")
                feedback.status = ButtonActionStatus.ALIGNING_TO_TARGET
                goal_handle.publish_feedback(feedback)
                if not await self.image_servo.align_to_target(button_id):
                    raise RuntimeError("이미지 정렬 실패")

            # =================== 공통 실행 단계 ===================
            # Step 4: 버튼 누르기
            self.get_logger().info(">> 버튼 누르기 수행")
            feedback.status = ButtonActionStatus.PRESSING
            goal_handle.publish_feedback(feedback)
            if not self.motion_controller.press_forward():
                raise RuntimeError("누르기 동작 실패")

            # Step 5: 후퇴
            self.get_logger().info(">> 후퇴 동작 수행")
            feedback.status = ButtonActionStatus.RETRACTING
            goal_handle.publish_feedback(feedback)
            if not self.motion_controller.retreat():
                raise RuntimeError("후퇴 동작 실패")

            # Step 6: 관측 자세 복귀
            self.get_logger().info("🟢 임무 종료 후 관측 자세로 복귀합니다.")
            self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])

        except Exception as e:
            # ... (기존 예외 처리 로직은 동일) ...
            error_msg = f"ClickButton 처리 중 오류 발생: {e}"
            self.get_logger().error(error_msg)
            feedback.status = ButtonActionStatus.FAILED
            goal_handle.publish_feedback(feedback)

            self.get_logger().info("🛑 작업 실패 → 관측 자세 복귀")
            self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])

            goal_handle.abort()
            result.success = False
            result.message = error_msg
            return result

        # ... (기존 성공 로직은 동일) ...
        success_msg = f"버튼 {button_id} 클릭 임무 성공"
        self.get_logger().info(success_msg)
        feedback.status = ButtonActionStatus.COMPLETED
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.message = success_msg
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
