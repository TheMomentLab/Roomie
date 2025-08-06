# roomie_arm_control/ac_node.py

import rclpy
import asyncio
import time
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
from rclpy.callback_groups import ReentrantCallbackGroup
from .config import (
    Pose, POSE_ANGLES_DEG,
    ControlMode, CONTROL_STRATEGY,
    ROBOT_ID,
    ButtonActionStatus,
    PREDEFINED_BUTTON_POSES_M, 
    SERVOING_STANDBY_DISTANCE_M,
    PRESS_FORWARD_DISTANCE_M,
    IMAGE_WIDTH_PX,
    IMAGE_HEIGHT_PX ,
    MAX_CLICK_ATTEMPTS 
)

class ArmActionServer(Node):
    """
    팔 제어와 관련된 모든 Action 요청을 처리하는 메인 서버 노드
    """
    def __init__(self):
        super().__init__('arm_action_server')

        self.callback_group = ReentrantCallbackGroup()

        # --- 모든 객체 생성 ---
        self.serial_manager = SerialManager()
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher(callback_group=self.callback_group)
        self.vision_client = VisionServiceClient(callback_group=self.callback_group) 
        self.coord_transformer = CoordinateTransformer()
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        self.image_servo = ImageServoing(self.vision_client, self.motion_controller, self.coord_transformer)
        
        initial_angles = self.serial_manager.connect()
        if initial_angles is None:
            self.get_logger().fatal("시리얼 연결에 실패하여 노드를 종료합니다.")
            return

        # --- Action 서버 생성 ---
        self._set_pose_server = ActionServer(
            self, SetPose, '/arm/action/set_pose', self.set_pose_callback,
            callback_group=self.callback_group
        )
        self._click_button_server = ActionServer(
            self, ClickButton, '/arm/action/click_button', self.click_button_callback,
            callback_group=self.callback_group
        )

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
    
    def click_button_callback(self, goal_handle):
        self.get_logger().info(f"ClickButton 목표 수신: button_id={goal_handle.request.button_id} (제어 모드: {CONTROL_STRATEGY.name})")
        
        try:
            return asyncio.run(self._execute_click_button_logic(goal_handle))
        except Exception as e:
            self.get_logger().error(f"🔴 작업 실패 (상위 핸들러): {e}")
            goal_handle.abort()
            result = ClickButton.Result()
            result.robot_id = ROBOT_ID
            result.success = False
            result.message = f"Failed with exception: {e}"
            self.get_logger().info("→ 안전을 위해 관측 자세로 복귀합니다.")
            self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])
            return result

  
    async def _execute_click_button_logic(self, goal_handle):
        result = ClickButton.Result()
        result.robot_id = ROBOT_ID
        feedback = ClickButton.Feedback()
        button_id = goal_handle.request.button_id

        for attempt in range(MAX_CLICK_ATTEMPTS):
            self.get_logger().info(f"==> 버튼 클릭 시도 #{attempt + 1}/{MAX_CLICK_ATTEMPTS} <==")
            
            try:
                # Step 0: 모든 시도의 시작은 관측 자세
                self.get_logger().info("🟡 시작 전 관측 자세로 이동합니다.")
                if not self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE]):
                    self.get_logger().warning("관측 자세로 이동 실패. 재시도합니다.")
                    await asyncio.sleep(1.0) # 재시도 전 잠시 대기
                    continue
                await asyncio.sleep(0.5)
                
                # 변수 초기화
                target_3d_pose = None
                target_orientation = None
                align_success = False

                # Step 1: 제어 전략에 따른 정렬 및 목표 위치 결정
                if CONTROL_STRATEGY == ControlMode.PBVS:
                    self.get_logger().info(">> [PBVS 제어] 시각 서보잉 정렬을 시작합니다.")
                    feedback.status = ButtonActionStatus.ALIGNING_TO_TARGET
                    goal_handle.publish_feedback(feedback)
                    align_success = await self.image_servo.align_to_standby_pose(button_id)
                    if align_success:
                        target_3d_pose = self.image_servo.last_target_pose
                        target_orientation = self.image_servo.last_target_orientation
                
                elif CONTROL_STRATEGY == ControlMode.IBVS:
                    self.get_logger().info(">> [IBVS 제어] 시각 서보잉 정렬을 시작합니다.")
                    feedback.status = ButtonActionStatus.ALIGNING_TO_TARGET
                    goal_handle.publish_feedback(feedback)
                    align_success = await self.image_servo.align_with_ibvs(button_id)
                    if align_success:
                        target_3d_pose = self.image_servo.last_target_pose
                        target_orientation = self.image_servo.last_target_orientation

                elif CONTROL_STRATEGY == ControlMode.MODEL_ONLY:
                    self.get_logger().info(">> [모델 전용 제어]를 시작합니다.")
                    feedback.status = ButtonActionStatus.MOVING_TO_TARGET
                    goal_handle.publish_feedback(feedback)
                    target_3d_pose = PREDEFINED_BUTTON_POSES_M.get(button_id)
                    if target_3d_pose is None:
                        raise RuntimeError(f"button_id {button_id}에 대한 좌표가 config에 없습니다.")
                    target_orientation = np.eye(3)
                    align_success = True

                # 정렬 실패 시 재시도
                if not align_success:
                    self.get_logger().warning(f"시도 #{attempt + 1}: 정렬 단계 실패. 재시도합니다.")
                    await asyncio.sleep(1.0)
                    continue

                # Step 2: 공통 누르기/후퇴 단계 (절대 좌표 기반)
                self.get_logger().info("✅ 정렬/이동 완료. 공통 누르기/후퇴 단계를 시작합니다.")
                if target_3d_pose is None or target_orientation is None:
                    raise RuntimeError("정렬 후 목표 pose 또는 orientation이 설정되지 않았습니다.")

                # 누르기/대기 위치 계산
                if CONTROL_STRATEGY == ControlMode.MODEL_ONLY:
                    ee_x_axis = target_orientation[:, 0]
                    standby_pose = target_3d_pose - ee_x_axis * SERVOING_STANDBY_DISTANCE_M
                    press_pose = target_3d_pose
                else: # PBVS, IBVS
                    standby_pose = target_3d_pose
                    forward_vector_world = target_orientation @ np.array([0, 0, PRESS_FORWARD_DISTANCE_M])
                    press_pose = standby_pose + forward_vector_world
                
                # 1. 대기 위치로 이동
                self.get_logger().info(f">> 대기 위치로 최종 이동: {np.round(standby_pose, 4)}")
                if not self.motion_controller.move_to_pose_ik(standby_pose, target_orientation):
                    self.get_logger().warning(f"시도 #{attempt + 1}: 대기 위치 이동(IK) 실패. 재시도합니다.")
                    await asyncio.sleep(1.0)
                    continue
                await asyncio.sleep(0.5)

                # 2. 누르기
                self.get_logger().info(">> 누르기 동작 수행 (절대 좌표 이동)")
                feedback.status = ButtonActionStatus.PRESSING
                goal_handle.publish_feedback(feedback)
                if not self.motion_controller.move_to_pose_ik(press_pose, target_orientation):
                    self.get_logger().warning(f"시도 #{attempt + 1}: 누르기 동작(IK) 실패. 재시도합니다.")
                    await asyncio.sleep(1.0)
                    continue
                await asyncio.sleep(0.5)

                # 3. 후퇴
                self.get_logger().info(">> 후퇴 동작 수행 (절대 좌표 이동)")
                feedback.status = ButtonActionStatus.RETRACTING
                goal_handle.publish_feedback(feedback)
                if not self.motion_controller.move_to_pose_ik(standby_pose, target_orientation):
                    self.get_logger().warning(f"시도 #{attempt + 1}: 후퇴 동작(IK) 실패. 재시도합니다.")
                    await asyncio.sleep(1.0)
                    continue
                
                # 여기까지 도달하면 현재 시도가 성공한 것임
                self.get_logger().info(f"🟢 버튼 {button_id} 클릭 임무 성공적으로 완료.")
                goal_handle.succeed()
                result.success = True
                result.message = f"버튼 {button_id} 클릭 임무 성공"
                self.get_logger().info("→ 임무 종료. 관측 자세로 복귀합니다.")
                self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])
                return result

            except Exception as e:
                self.get_logger().error(f"🔴 시도 #{attempt + 1} 중 예외 발생: {e}. 재시도합니다.")
                await asyncio.sleep(1.0) # 예외 발생 후 잠시 대기
        
        # for 루프가 모두 실패한 경우
        self.get_logger().error(f"❌ 최대 재시도 횟수({MAX_CLICK_ATTEMPTS}) 내에 임무를 완수하지 못했습니다.")
        goal_handle.abort()
        result.success = True
        result.message = f"Failed to click button after {MAX_CLICK_ATTEMPTS} attempts."
        self.get_logger().info("→ 안전을 위해 관측 자세로 복귀합니다.")
        self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[Pose.OBSERVE])
        return result

    # [수정] click_button_callback은 이제 로직을 직접 실행하지 않고 asyncio.run으로 위임만 합니다.
    def click_button_callback(self, goal_handle):
        self.get_logger().info(f"ClickButton 목표 수신: button_id={goal_handle.request.button_id} (제어 모드: {CONTROL_STRATEGY.name})")
        # _execute_click_button_logic 함수가 최종 결과를 반환하므로, try/except 블록은 더 이상 필요 없습니다.
        # _execute_click_button_logic 내부에서 모든 예외 처리와 액션 상태(succeed, abort) 관리를 담당합니다.
        return asyncio.run(self._execute_click_button_logic(goal_handle))
    
def main(args=None):
    rclpy.init(args=args)
    try:
        arm_action_server = ArmActionServer()
        if arm_action_server.serial_manager.is_ready:
            executor = MultiThreadedExecutor()
            rclpy.spin(arm_action_server, executor=executor)
    except Exception as e:
        print(f"노드 실행 중 심각한 오류 발생: {e}")
    finally:
        if 'arm_action_server' in locals() and rclpy.ok():
            arm_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()