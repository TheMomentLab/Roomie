# roomie_arm_control/ac_node.py

import rclpy
import asyncio
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
import traceback

from roomie_msgs.action import SetPose, ClickButton
from . import config
from .vision_client import VisionServiceClient
from .serial_manager import SerialManager
from .kinematics_solver import KinematicsSolver
from .motion_controller import MotionController
from .image_servoing import ImageServoing
from .ros_joint_publisher import ROSJointPublisher
from .coordinate_transformer import CoordinateTransformer

class ArmActionServer(Node):
    def __init__(self, loop):
        super().__init__('arm_action_server')
        self.loop = loop
        self.callback_group = ReentrantCallbackGroup()

        # --- 의존성 객체 생성 ---
        self.serial_manager = SerialManager()
        if not self.serial_manager.connect():
            # 초기화 실패 시 로깅 후 종료
            self.get_logger().fatal("시리얼 연결에 실패하여 노드를 종료합니다.")
            # rclpy.shutdown()을 직접 호출하기보다 예외를 발생시켜 main에서 처리하도록 함
            raise ConnectionError("Serial connection failed.")
            
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher(callback_group=self.callback_group)
        self.vision_client = VisionServiceClient(callback_group=self.callback_group) 
        self.coord_transformer = CoordinateTransformer()
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        self.image_servo = ImageServoing(self.vision_client, self.motion_controller, self.coord_transformer)
        
        # --- Action 서버 생성 (콜백은 모두 동기 함수) ---
        self._set_pose_server = ActionServer(self, SetPose, '/arm/action/set_pose', self.set_pose_callback, callback_group=self.callback_group)
        self._click_button_server = ActionServer(self, ClickButton, '/arm/action/click_button', self.click_button_callback, callback_group=self.callback_group)

        self.get_logger().info("✅ Arm Action Server가 성공적으로 시작되었습니다.")
        self.goal_handle = None # goal_handle을 클래스 멤버로 관리

    def set_pose_callback(self, goal_handle):
        # SetPose는 간단한 동기 작업이므로 직접 처리
        self.get_logger().info(f"SetPose 목표 수신: pose_id={goal_handle.request.pose_id}")
        result = SetPose.Result(robot_id=config.ROBOT_ID, success=False)
        
        try:
            requested_pose = config.Pose(goal_handle.request.pose_id)
            target_angles = config.POSE_ANGLES_DEG.get(requested_pose)
            if self.motion_controller.move_to_angles_deg(target_angles):
                self.get_logger().info(f"'{requested_pose.name}' 자세로 이동 완료.")
                goal_handle.succeed()
                result.success = True
            else:
                self.get_logger().error(f"'{requested_pose.name}' 자세로 이동 실패.")
                goal_handle.abort()
        except Exception as e:
            self.get_logger().error(f"SetPose 처리 중 예외 발생: {e}")
            goal_handle.abort()
        
        return result

    def click_button_callback(self, goal_handle):
        """
        ROS 스레드에서 호출되는 동기 콜백. 비동기 로직 실행을 예약하고 결과를 기다립니다.
        """
        self.get_logger().info(f"ClickButton 목표 수신 (ROS 스레드: {threading.get_ident()})")
        future = asyncio.run_coroutine_threadsafe(
            self._execute_click_button_logic_async(goal_handle),
            self.loop
        )
        try:
            # 비동기 작업의 최종 결과를 기다림
            return future.result()
        except Exception as e:
            self.get_logger().error(f"비동기 작업 실행 중 최상위 예외 발생: {e}\n{traceback.format_exc()}")
            goal_handle.abort()
            return ClickButton.Result(success=False, message=str(e))

    async def _execute_click_button_logic_async(self, goal_handle):
        """
        Asyncio 스레드에서 실행되는 모든 실제 로직.
        """
        self.get_logger().info(f"비동기 로직 시작 (Asyncio 스레드: {threading.get_ident()})")
        self.goal_handle = goal_handle
        result = ClickButton.Result(robot_id=config.ROBOT_ID, success=False)
        feedback = ClickButton.Feedback()
        button_id = goal_handle.request.button_id

        try:
            # 1. 관측 자세로 이동
            if not self.motion_controller.move_to_angles_deg(config.POSE_ANGLES_DEG[config.Pose.OBSERVE]):
                raise RuntimeError("시작 전 관측 자세로 이동 실패")
            await asyncio.sleep(0.5)

            # 2. 제어 전략에 따라 목표 위치 결정 (PBVS 모드 포함)
            standby_pose, orientation_vector = await self._determine_target_async(button_id, feedback)

            # 3. 물리적 버튼 누르기 실행
            await self._execute_physical_press_async(standby_pose, orientation_vector, feedback)

            # 4. 최종 성공 처리
            self.get_logger().info(f"🟢 버튼 {button_id} 클릭 임무 성공적으로 완료.")
            goal_handle.succeed()
            result.success = True
            result.message = f"버튼 {button_id} 클릭 임무 성공"

        except Exception as e:
            error_message = f"🔴 작업 실패: {e}\n{traceback.format_exc()}"
            self.get_logger().error(error_message)
            goal_handle.abort()
            result.message = str(e)
        
        finally:
            self.get_logger().info("→ 임무 종료. 안전을 위해 관측 자세로 복귀합니다.")
            self.motion_controller.move_to_angles_deg(config.POSE_ANGLES_DEG[config.Pose.OBSERVE])

        return result

    async def _determine_target_async(self, button_id, feedback_handle):
        """
        PBVS와 MODEL_ONLY 모드를 모두 처리하는 비동기 함수.
        """
        if config.CONTROL_STRATEGY == config.ControlMode.PBVS:
            self.get_logger().info(">> [PBVS 제어] 시각 서보잉 정렬을 시작합니다.")
            feedback_handle.status = config.ButtonActionStatus.ALIGNING_TO_TARGET
            self.goal_handle.publish_feedback(feedback_handle)
            
            # 👈 [복원] 비동기 비전 서보잉 로직을 await으로 호출
            align_success = await self.image_servo.align_to_standby_pose(button_id)
            if not align_success:
                raise RuntimeError("시각 서보잉 정렬에 최종 실패했습니다.")
            
            # PBVS 성공 시, 정렬된 위치가 바로 '대기 위치'가 됨
            standby_pose = self.image_servo.last_target_pose
            orientation_vector = self.image_servo.last_target_orientation
            return standby_pose, orientation_vector

        elif config.CONTROL_STRATEGY == config.ControlMode.MODEL_ONLY:
            self.get_logger().info(">> [모델 전용 제어]를 시작합니다.")
            feedback_handle.status = config.ButtonActionStatus.MOVING_TO_TARGET
            self.goal_handle.publish_feedback(feedback_handle)
            
            target_pose = config.PREDEFINED_BUTTON_POSES_M.get(button_id)
            if target_pose is None:
                raise RuntimeError(f"config에 button_id {button_id} 좌표가 없습니다.")
            
            orientation_vector = np.array([1.0, 0.0, 0.0]) # 정면 방향 벡터
            
            # 모델 전용 모드에서는 목표 위치에서 뒤로 물러나 '대기 위치'를 계산
            retreat_vector = orientation_vector * config.SERVOING_STANDBY_DISTANCE_M
            standby_pose = target_pose - retreat_vector
            return standby_pose, orientation_vector
        
        raise NotImplementedError(f"지원하지 않는 제어 전략: {config.CONTROL_STRATEGY.name}")

    async def _execute_physical_press_async(self, standby_pose, orientation_vector, feedback_handle):
        """
        계산된 대기 위치를 기준으로 누르기/후퇴 동작을 수행하는 비동기 함수.
        """
        # 누를 위치 계산
        press_vector = orientation_vector * config.PRESS_FORWARD_DISTANCE_M
        press_pose = standby_pose + press_vector

        self.get_logger().info(f">> 대기 위치로 이동: {np.round(standby_pose, 4)}")
        if not self.motion_controller.move_to_pose_ik(standby_pose, orientation_vector):
            raise RuntimeError("대기 위치 이동에 실패했습니다.")
        await asyncio.sleep(0.5)

        self.get_logger().info(f">> 계산된 누르기 위치로 이동: {np.round(press_pose, 4)}")
        feedback_handle.status = config.ButtonActionStatus.PRESSING
        self.goal_handle.publish_feedback(feedback_handle)
        if not self.motion_controller.move_to_pose_ik(press_pose, orientation_vector):
            raise RuntimeError("누르기 동작에 실패했습니다.")
        await asyncio.sleep(1.0) # 누르는 시간 확보

        self.get_logger().info(">> 대기 위치로 후퇴")
        feedback_handle.status = config.ButtonActionStatus.RETRACTING
        self.goal_handle.publish_feedback(feedback_handle)
        if not self.motion_controller.move_to_pose_ik(standby_pose, orientation_vector):
            raise RuntimeError("후퇴 동작에 실패했습니다.")
        await asyncio.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    loop = asyncio.get_event_loop()
    arm_action_server = None
    
    try:
        arm_action_server = ArmActionServer(loop)
        
        # rclpy.spin을 별도의 스레드에서 실행
        ros_thread = threading.Thread(target=rclpy.spin, args=(arm_action_server,), daemon=True)
        ros_thread.start()
        
        arm_action_server.get_logger().info(f"ROS 스핀 스레드 시작 (스레드 ID: {ros_thread.ident})")
        arm_action_server.get_logger().info(f"Asyncio 이벤트 루프 시작 (메인 스레드 ID: {threading.get_ident()})")

        # 메인 스레드는 asyncio 이벤트 루프를 실행
        loop.run_forever()

    except (KeyboardInterrupt, ConnectionError) as e:
        if isinstance(e, ConnectionError):
             print(f"초기화 실패: {e}")
        else:
             print("키보드 인터럽트로 종료합니다.")
    except Exception as e:
        print(f"예기치 않은 오류 발생: {e}\n{traceback.format_exc()}")
    finally:
        if loop.is_running():
            loop.stop()
        # 노드가 성공적으로 생성되었다면 정리
        if arm_action_server and rclpy.ok():
            arm_action_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()