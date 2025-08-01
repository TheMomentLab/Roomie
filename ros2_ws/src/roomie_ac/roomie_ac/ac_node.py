import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from roomie_msgs.action import SetPose, ClickButton

# 모든 '연주자' 클래스들을 임포트합니다.
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
    ROBOT_ID 
)


class ArmActionServer(Node):
    """
    팔 제어와 관련된 모든 Action 요청을 처리하는 메인 서버 (지휘자).
    """
    def __init__(self):
        super().__init__('arm_action_server')

        # --- 모든 '연주자' 객체 생성 ---
        self.serial_manager = SerialManager()
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher()
        self.vision_client = VisionServiceClient()
        self.coord_transformer = CoordinateTransformer() # CoordinateTransformer 생성
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        self.image_servo = ImageServoing(self.vision_client, self.motion_controller)
        
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
            return SetPose.Result(success=False, message=msg)

        try:
            requested_pose = Pose(goal_handle.request.pose_id)
            self.get_logger().info(f"SetPose 목표 수신: '{requested_pose.name}'")
        except ValueError:
            msg = f"정의되지 않은 Pose ID 수신: {goal_handle.request.pose_id}"
            self.get_logger().error(msg)
            goal_handle.abort()
            return SetPose.Result(success=False, message=msg)

        # 딕셔너리에서 목표 각도를 찾아 MotionController에게 전달합니다.
        target_angles_deg = POSE_ANGLES_DEG.get(requested_pose)

        if target_angles_deg is not None:
            # MotionController에게 최종 실행을 지시합니다.
            success = self.motion_controller.move_to_angles_deg(target_angles_deg)
            if success:
                self.get_logger().info(f"'{requested_pose.name}' 자세로 이동 완료.")
                goal_handle.succeed()
                return SetPose.Result(success=True, message="자세 이동 성공")

        # 딕셔너리에 키가 없거나, 모션 컨트롤러가 실패한 경우
        msg = f"'{requested_pose.name}' 자세로 이동 실패."
        self.get_logger().error(msg)
        goal_handle.abort()
        return SetPose.Result(success=False, message=msg)


    async def click_button_callback(self, goal_handle):
        """[지휘] ClickButton Action의 전체 시나리오를 지휘합니다."""
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
        result = ClickButton.Result()

        try:
            # --- 시나리오 1: Vision Service로부터 2D 버튼 정보 받기 ---
            response = await self.vision_client.request_button_status(ROBOT_ID, [button_id])

            if not response or not response.success or not response.xs:
                raise RuntimeError("Vision Service로부터 버튼 위치 획득 실패")
            
            button_center_xy_norm = (response.xs[0], response.ys[0])
            button_size_norm = response.sizes[0]

            # --- 시나리오 2: 모델 기반으로 '준비 위치'까지 이동 (HYBRID 모드) ---
            if CONTROL_STRATEGY == ControlMode.HYBRID:
                self.get_logger().info(">> [하이브리드 제어] 1단계: 모델 기반으로 준비 위치로 이동")
                
                # 현재 로봇 팔의 FK 행렬을 가져옵니다.
                current_fk_transform = self.motion_controller._get_current_transform()
                
                # 3D 목표 좌표 계산
                target_3d_pose = self.coord_transformer.calculate_target_pose(
                    button_center_xy_norm, button_size_norm, current_fk_transform
                )
                if target_3d_pose is None:
                    raise RuntimeError("3D 목표 좌표 계산 실패")

                # '준비 위치' 계산 (버튼 앞에서 일정 거리만큼 뒤로)
                # 현재 팔의 Z축(정면) 방향 벡터를 구합니다.
                forward_vector = current_fk_transform[:3, 2]
                pre_press_pose = target_3d_pose - forward_vector * PRE_PRESS_DISTANCE_M
                
                # 준비 위치로 이동
                if not self.motion_controller.move_to_pose_ik(pre_press_pose):
                    raise RuntimeError("준비 위치로 이동 실패")

            # --- 시나리오 3: 이미지 서보잉으로 정밀 정렬 ---
            self.get_logger().info(">> [공통] 2단계: 이미지 서보잉으로 정밀 정렬")
            feedback.status = "ALIGNING_TO_TARGET"
            goal_handle.publish_feedback(feedback)
            is_aligned = await self.image_servo.align_to_target(button_id)
            if not is_aligned:
                raise RuntimeError("이미지 정렬 실패")

            # --- 시나리오 4: 버튼 누르기 ---
            self.get_logger().info(">> [공통] 3단계: 버튼 누르기")
            feedback.status = "PRESSING"
            goal_handle.publish_feedback(feedback)
            if not self.motion_controller.press_forward():
                raise RuntimeError("누르기 동작 실패")

            # --- 시나리오 5: 후퇴 ---
            self.get_logger().info(">> [공통] 4단계: 후퇴")
            feedback.status = "RETRACTING"
            goal_handle.publish_feedback(feedback)
            if not self.motion_controller.retreat():
                raise RuntimeError("후퇴 동작 실패")

        except Exception as e:
            error_msg = f"ClickButton 처리 중 오류 발생: {e}"
            self.get_logger().error(error_msg)
            goal_handle.abort()
            result.success = False
            result.message = error_msg
            return result

        success_msg = f"버튼 {button_id} 클릭 임무 성공"
        self.get_logger().info(success_msg)
        goal_handle.succeed()
        result.success = True
        result.message = success_msg
        return result




def main(args=None):
    rclpy.init(args=args)
    arm_action_server = ArmActionServer()
    
    # 여러 콜백(특히 async 포함)을 동시에 처리하기 위해 MultiThreadedExecutor 사용을 권장합니다.
    executor = MultiThreadedExecutor()
    rclpy.spin(arm_action_server, executor=executor)
    
    arm_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()