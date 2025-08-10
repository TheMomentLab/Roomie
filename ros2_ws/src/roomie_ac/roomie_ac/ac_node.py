# roomie_ac/ac_node.py

# --- 기본 및 ROS 라이브러리 임포트 ---
import rclpy
import asyncio
import threading
import abc
import numpy as np
import traceback
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

# --- 사용자 정의 메시지 및 모듈 임포트 ---
from roomie_msgs.action import SetPose, ClickButton
from . import config
from .vision_client import VisionServiceClient
from .serial_manager import SerialManager
from .kinematics_solver import KinematicsSolver
from .motion_controller import MotionController
from .image_servoing import ImageServoing
from .ros_joint_publisher import ROSJointPublisher
from .coordinate_transformer import CoordinateTransformer

# ==============================================================================
# 🌀 제어 전략 클래스 (Strategy Pattern)
# ==============================================================================
class BaseStrategy(abc.ABC):
    """모든 제어 전략의 공통 규격(인터페이스)을 정의하는 추상 기반 클래스입니다."""
    def __init__(self, motion_controller: MotionController, logger):
        self.motion_controller = motion_controller
        self.logger = logger

    @abc.abstractmethod
    async def execute(self, button_id: int, goal_handle):
        """각 전략의 고유한 실행 로직을 구현하는 메소드입니다."""
        pass

class DirectPressStrategy(BaseStrategy):
    """전략 1: 좌표 계산 없이 목표 지점으로 즉시 이동하여 누르는 전략입니다."""
    async def execute(self, button_id: int, goal_handle):
        target_pose = config.PREDEFINED_BUTTON_POSES_M.get(button_id)
        if target_pose is None: raise ValueError(f"Config에 button_id {button_id} 좌표 없음.")
        self.logger.info(f">> 전략 1: 목표 {np.round(target_pose, 3)}로 직접 이동")
        
        # [수정] orientation을 None으로 설정하여 IK가 최적 방향을 찾도록 함
        if not await self.motion_controller.move_to_pose_ik(target_pose, orientation=None, blocking=True):
            raise RuntimeError("직접 누르기 실패")
            
        await asyncio.sleep(1.0)

class StandbyPressStrategy(BaseStrategy):
    """[수정됨] 사전 정의된 좌표를 기준으로 전/후진하여 누르는 전략입니다."""
    async def execute(self, button_id: int, goal_handle):
        target_pose = config.PREDEFINED_BUTTON_POSES_M.get(button_id)
        if target_pose is None: raise ValueError(f"Config에 button_id {button_id} 좌표 없음.")
        
        # [수정] 이 전략은 카메라를 쓰지 않으므로, 로봇의 X축을 정면으로 가정합니다.
        # 이는 로봇이 버튼을 정면으로 바라보고 설치되었다고 가정하는 것입니다.
        forward_vector = np.array([1., 0., 0.]) 
        
        standby_pose = target_pose - forward_vector * config.SERVOING_STANDBY_DISTANCE_M
        
        self.logger.info(">> 전략 2: 대기 위치로 이동")
        if not await self.motion_controller.move_to_pose_ik(standby_pose, orientation=None, blocking=True):
            raise RuntimeError("대기 위치 이동 실패")
            
        self.logger.info(">> 누르기 위치로 이동")
        if not await self.motion_controller.move_to_pose_ik(target_pose, orientation=None, blocking=True):
            raise RuntimeError("누르기 동작 실패")
            
        await asyncio.sleep(1.0)
        
        self.logger.info(">> 대기 위치로 후퇴")
        if not await self.motion_controller.move_to_pose_ik(standby_pose, orientation=None, blocking=True):
            raise RuntimeError("후퇴 동작 실패")
        
class PBVSPressStrategy(BaseStrategy):
    """[수정됨] 카메라로 계산된 버튼의 정면 방향으로 누르는 전략입니다."""
    def __init__(self, motion_controller: MotionController, image_servo: ImageServoing, logger):
        super().__init__(motion_controller, logger)
        self.image_servo = image_servo

    async def execute(self, button_id: int, goal_handle):
        self.logger.info(">> 전략 3: 시각 서보잉 정렬 시작")
        if not await self.image_servo.align_to_standby_pose(button_id):
            raise RuntimeError("시각 서보잉 정렬 실패")
            
        standby_pose = self.image_servo.last_target_pose
        button_orientation_matrix = self.image_servo.last_target_orientation
        
        if standby_pose is None or button_orientation_matrix is None:
            raise RuntimeError("버튼 위치/방향 정보를 얻지 못했습니다.")

        # [핵심 수정] 저장된 버튼의 방향 행렬에서 정면(Z축) 벡터를 추출
        button_z_vector = button_orientation_matrix[:, 2]
        
        # 정면 벡터 방향으로 '누르기 거리'만큼 전진하여 press_pose 계산
        press_pose = standby_pose + button_z_vector * config.PRESS_FORWARD_DISTANCE_M

        self.logger.info(">> PBVS 누르기 위치로 이동")
        if not await self.motion_controller.move_to_pose_ik(press_pose, orientation=None, blocking=True):
            raise RuntimeError("PBVS 누르기 실패")
            
        await asyncio.sleep(1.0)
        
        self.logger.info(">> PBVS 대기 위치로 후퇴")
        if not await self.motion_controller.move_to_pose_ik(standby_pose, orientation=None, blocking=True):
            raise RuntimeError("PBVS 후퇴 실패")

# ==============================================================================
# 🤖 메인 제어 노드 클래스
# ==============================================================================
class ArmActionServer(Node):
    def __init__(self, loop):
        """노드의 모든 구성요소(객체)와 제어 전략을 초기화합니다."""
        super().__init__('arm_action_server')
        self.loop = loop
        self.callback_group = ReentrantCallbackGroup()
        
        # --- 객체 초기화 순서는 그대로 유지 ---
        self.serial_manager = SerialManager()
        if not self.serial_manager.connect():
            raise ConnectionError("Serial connection failed.")
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher(callback_group=self.callback_group)
        self.vision_client = VisionServiceClient(callback_group=self.callback_group) 
        self.coord_transformer = CoordinateTransformer()
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        self.image_servo = ImageServoing(self.vision_client, self.motion_controller, self.coord_transformer)
        
        # --- 전략 선택 로직은 그대로 유지 ---
        self.strategies = {
            config.ControlStrategy.MODEL_DIRECT_PRESS: DirectPressStrategy(self.motion_controller, self.get_logger()),
            config.ControlStrategy.MODEL_STANDBY_PRESS: StandbyPressStrategy(self.motion_controller, self.get_logger()),
            config.ControlStrategy.PBVS_PRESS: PBVSPressStrategy(self.motion_controller, self.image_servo, self.get_logger())
        }
        self.selected_strategy = self.strategies.get(config.CONTROL_STRATEGY)
        if self.selected_strategy is None:
            raise ValueError(f"지원하지 않는 제어 전략입니다: {config.CONTROL_STRATEGY}")
        
        # --- 액션 서버 생성 ---
        self._set_pose_server = ActionServer(self, SetPose, '/arm/action/set_pose', self.set_pose_callback, callback_group=self.callback_group)
        self._click_button_server = ActionServer(self, ClickButton, '/arm/action/click_button', self.click_button_callback, callback_group=self.callback_group)
        
        self.get_logger().info("✅ Arm Action Server가 성공적으로 시작되었습니다.")
        self.goal_handle = None

    # [핵심 수정] set_pose_callback을 비동기 호환 구조로 변경
    def set_pose_callback(self, goal_handle):
        """(ROS 스레드) SetPose 요청을 받아 비동기 로직 실행을 예약합니다."""
        self.get_logger().info(f"SetPose 목표 수신: pose_id={goal_handle.request.pose_id}")
        future = asyncio.run_coroutine_threadsafe(
            self._execute_set_pose_async(goal_handle),
            self.loop
        )
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"SetPose 비동기 작업 실행 중 예외 발생: {e}")
            goal_handle.abort()
            return SetPose.Result(success=False)

    async def _execute_set_pose_async(self, goal_handle):
        """(Asyncio 스레드) SetPose 액션의 실제 로직을 비동기적으로 처리합니다."""
        result = SetPose.Result(robot_id=config.ROBOT_ID, success=False)
        try:
            requested_pose = config.Pose(goal_handle.request.pose_id)
            target_angles = config.POSE_ANGLES_DEG.get(requested_pose)
            
            # [수정] async로 변경된 함수를 await로 호출
            if await self.motion_controller.move_to_angles_deg(target_angles, blocking=True):
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
        """(ROS 스레드) 액션 요청을 받아, 비동기 로직 실행을 예약하고 그 결과를 기다립니다."""
        self.get_logger().info(f"ClickButton 목표 수신 (ROS 스레드: {threading.get_ident()})")
        future = asyncio.run_coroutine_threadsafe(
            self._execute_click_button_logic_async(goal_handle),
            self.loop
        )
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"비동기 작업 실행 중 최상위 예외 발생: {e}\n{traceback.format_exc()}")
            goal_handle.abort()
            return ClickButton.Result(success=False, message=str(e))

    async def _execute_click_button_logic_async(self, goal_handle):
        """(Asyncio 스레드) `ClickButton` 액션의 전체 흐름(준비-실행-마무리)을 관리합니다."""
        self.get_logger().info(f"비동기 로직 시작 (전략: {self.selected_strategy.__class__.__name__})")
        self.goal_handle = goal_handle
        result = ClickButton.Result(robot_id=config.ROBOT_ID, success=False)
        button_id = goal_handle.request.button_id
        
        try:
            # 1. 공통 준비: 관측 자세로 이동
            # [수정] async로 변경된 함수를 await로 호출
            if not await self.motion_controller.move_to_angles_deg(config.POSE_ANGLES_DEG[config.Pose.OBSERVE], blocking=True):
                raise RuntimeError("관측 자세 이동 실패")
            await asyncio.sleep(0.5)

            # 2. 핵심: 선택된 전략 객체에게 실행을 '위임'
            await self.selected_strategy.execute(button_id, goal_handle)

            # 3. 공통 성공 처리
            self.get_logger().info(f"🟢 버튼 {button_id} 클릭 임무 성공.")
            goal_handle.succeed()
            result.success = True
        except Exception as e:
            error_message = f"🔴 작업 실패: {e}\n{traceback.format_exc()}"
            self.get_logger().error(error_message)
            goal_handle.abort()
            result.message = str(e)
        finally:
            # 4. 공통 마무리: 관측 자세로 복귀
            self.get_logger().info("→ 임무 종료. 관측 자세로 복귀합니다.")
            # [수정] async로 변경된 함수를 await로 호출
            await self.motion_controller.move_to_angles_deg(config.POSE_ANGLES_DEG[config.Pose.OBSERVE], blocking=True)
            
        return result

def main(args=None):
    """ROS 2와 asyncio를 함께 실행하기 위한 멀티쓰레드 환경을 설정하고 노드를 실행합니다."""
    rclpy.init(args=args)
    loop = asyncio.get_event_loop()
    arm_action_server = None
    try:
        arm_action_server = ArmActionServer(loop)
        ros_thread = threading.Thread(target=rclpy.spin, args=(arm_action_server,), daemon=True)
        ros_thread.start()
        arm_action_server.get_logger().info(f"ROS 스핀 스레드 시작 (스레드 ID: {ros_thread.ident})")
        arm_action_server.get_logger().info(f"Asyncio 이벤트 루프 시작 (메인 스레드 ID: {threading.get_ident()})")
        loop.run_forever()
    except (KeyboardInterrupt, ConnectionError) as e:
        if isinstance(e, ConnectionError): print(f"초기화 실패: {e}")
        else: print("키보드 인터럽트로 종료합니다.")
    except Exception as e:
        print(f"예기치 않은 오류 발생: {e}\n{traceback.format_exc()}")
    finally:
        if loop.is_running(): loop.stop()
        if arm_action_server and rclpy.ok(): arm_action_server.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    """이 스크립트가 직접 실행될 때 main 함수를 호출합니다."""
    main()