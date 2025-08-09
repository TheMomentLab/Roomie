# roomie_ac/ac_node.py

# --- 기본 및 ROS 라이브러리 임포트 ---
import rclpy
import asyncio
import threading
import abc
import numpy as np
import traceback
import time  # [수정 1-1] 쿨다운 기능에 필요한 time 모듈 임포트
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
# [수정 2-1] 여러 노드를 함께 실행하기 위한 MultiThreadedExecutor 임포트
from rclpy.executors import MultiThreadedExecutor

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

        if not await self.motion_controller.move_to_pose_ik(target_pose, orientation=None, blocking=True):
            raise RuntimeError("직접 누르기 실패")

        await asyncio.sleep(1.0)

class StandbyPressStrategy(BaseStrategy):
    """사전 정의된 좌표를 기준으로 전/후진하여 누르는 전략입니다."""
    async def execute(self, button_id: int, goal_handle):
        target_pose = config.PREDEFINED_BUTTON_POSES_M.get(button_id)
        if target_pose is None: raise ValueError(f"Config에 button_id {button_id} 좌표 없음.")

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
    """카메라로 계산된 버튼의 정면 방향으로 누르는 전략입니다."""
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

        button_z_vector = button_orientation_matrix[:, 2]
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

        # --- 객체 초기화 ---
        self.serial_manager = SerialManager()
        if not self.serial_manager.connect():
            raise ConnectionError("Serial connection failed.")
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher(callback_group=self.callback_group)
        self.vision_client = VisionServiceClient(callback_group=self.callback_group)
        self.coord_transformer = CoordinateTransformer()
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        self.image_servo = ImageServoing(self.vision_client, self.motion_controller, self.coord_transformer)

        # --- 전략 선택 로직 ---
        self.strategies = {
            config.ControlStrategy.MODEL_DIRECT_PRESS: DirectPressStrategy(self.motion_controller, self.get_logger()),
            config.ControlStrategy.MODEL_STANDBY_PRESS: StandbyPressStrategy(self.motion_controller, self.get_logger()),
            config.ControlStrategy.PBVS_PRESS: PBVSPressStrategy(self.motion_controller, self.image_servo, self.get_logger())
        }
        self.selected_strategy = self.strategies.get(config.CONTROL_STRATEGY)
        if self.selected_strategy is None:
            raise ValueError(f"지원하지 않는 제어 전략입니다: {config.CONTROL_STRATEGY}")

        # [수정 1-2] 중복 실행 방지를 위한 상태 변수 초기화
        self._click_action_lock = threading.Lock() # 스레드 동시 접근 방지용 lock
        self._is_click_action_running = False      # 현재 작업 실행 여부 플래그
        self._last_action_info = {"button_id": None, "timestamp": 0.0} # 마지막 작업 정보 (ID, 완료 시간)
        self.CLICK_ACTION_COOLDOWN_S = 2.0  # 동일 버튼에 대한 연속 요청 방지 시간 (초)

        # --- 액션 서버 생성 ---
        self._set_pose_server = ActionServer(self, SetPose, '/arm/action/set_pose', self.set_pose_callback, callback_group=self.callback_group)
        self._click_button_server = ActionServer(self, ClickButton, '/arm/action/click_button', self.click_button_callback, callback_group=self.callback_group)

        self.get_logger().info("✅ Arm Action Server가 성공적으로 시작되었습니다.")
        self.goal_handle = None

    def set_pose_callback(self, goal_handle):
        """(ROS 스레드) SetPose 요청을 받아 비동기 로직 실행을 예약합니다."""
        self.get_logger().info(f"SetPose 목표 수신: pose_id={goal_handle.request.pose_id}")
        future = asyncio.run_coroutine_threadsafe(self._execute_set_pose_async(goal_handle), self.loop)
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
        """(ROS 스레드) 액션 요청을 받아, 중복 여부를 검사하고 비동기 로직 실행을 예약합니다."""
        self.get_logger().info(f"ClickButton 목표 수신 (ROS 스레드: {threading.get_ident()})")

        # [수정 1-3] 중복 실행 방지 가드 로직
        with self._click_action_lock:
            # 1. 현재 다른 클릭 액션이 실행 중인지 확인
            if self._is_click_action_running:
                self.get_logger().warn("이미 ClickButton 작업이 실행 중입니다. 새로운 목표를 거절합니다.")
                goal_handle.abort()
                return ClickButton.Result(success=False, message="Arm is busy with another click action.")

            # 2. 마지막으로 실행된 버튼과 동일하고, 쿨다운 시간 내에 요청되었는지 확인
            if (goal_handle.request.button_id == self._last_action_info["button_id"] and
                time.time() - self._last_action_info["timestamp"] < self.CLICK_ACTION_COOLDOWN_S):
                self.get_logger().warn(f"쿨다운({self.CLICK_ACTION_COOLDOWN_S}초) 시간 내에 동일 버튼({goal_handle.request.button_id})에 대한 요청입니다. 거절합니다.")
                goal_handle.abort()
                return ClickButton.Result(success=False, message="Duplicate request within cooldown period.")
            
            # 모든 검사를 통과하면, 실행 플래그를 올림
            self._is_click_action_running = True

        # 비동기 로직 실행을 예약하고, try/finally 구문으로 실행 플래그를 안전하게 해제
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._execute_click_button_logic_async(goal_handle),
                self.loop
            )
            # future.result()는 비동기 작업이 끝날 때까지 여기서 기다립니다.
            return future.result()
        except Exception as e:
            self.get_logger().error(f"비동기 작업 실행 중 최상위 예외 발생: {e}\n{traceback.format_exc()}")
            goal_handle.abort()
            return ClickButton.Result(success=False, message=str(e))
        finally:
            # 작업이 성공하든 실패하든, 끝나면 반드시 실행 플래그를 내림
            with self._click_action_lock:
                self._is_click_action_running = False


    async def _execute_click_button_logic_async(self, goal_handle):
        """(Asyncio 스레드) `ClickButton` 액션의 전체 흐름(준비-실행-마무리)을 관리합니다."""
        self.get_logger().info(f"비동기 로직 시작 (전략: {self.selected_strategy.__class__.__name__})")
        self.goal_handle = goal_handle
        result = ClickButton.Result(robot_id=config.ROBOT_ID, success=False)
        button_id = goal_handle.request.button_id
        
        try:
            # 1. 공통 준비: 관측 자세로 이동
            if not await self.motion_controller.move_to_angles_deg(config.POSE_ANGLES_DEG[config.Pose.OBSERVE], blocking=True):
                raise RuntimeError("관측 자세 이동 실패")
            await asyncio.sleep(0.5)

            # 2. 핵심: 선택된 전략 객체에게 실행을 '위임'
            await self.selected_strategy.execute(button_id, goal_handle)

            # 3. 공통 성공 처리
            self.get_logger().info(f"🟢 버튼 {button_id} 클릭 임무 성공.")
            goal_handle.succeed()
            result.success = True
            
            # [수정 1-4] 작업 성공 시, 마지막 작업 정보를 현재 시간으로 갱신 (쿨다운용)
            self._last_action_info = {"button_id": button_id, "timestamp": time.time()}

        except Exception as e:
            error_message = f"🔴 작업 실패: {e}\n{traceback.format_exc()}"
            self.get_logger().error(error_message)
            goal_handle.abort()
            result.message = str(e)
        finally:
            # 4. 공통 마무리: 관측 자세로 복귀
            self.get_logger().info("→ 임무 종료. 관측 자세로 복귀합니다.")
            await self.motion_controller.move_to_angles_deg(config.POSE_ANGLES_DEG[config.Pose.OBSERVE], blocking=True)
            
        return result

# [수정 2-2] main 함수를 MultiThreadedExecutor를 사용하도록 전체 수정
def main(args=None):
    """
    ROS 2와 asyncio를 함께 실행하기 위한 멀티쓰레드 환경을 설정하고 노드를 실행합니다.
    VisionServiceClient, ROSJointPublisher 등 모든 노드를 Executor에 등록하여
    각 노드의 콜백(서비스 응답 등)이 올바르게 처리되도록 합니다.
    """
    rclpy.init(args=args)
    
    # Python의 비동기 이벤트 루프
    loop = asyncio.get_event_loop()
    
    # 노드 및 Executor 객체 초기화
    arm_action_server = None
    executor = None

    try:
        # 1. 메인 노드(ArmActionServer)와 그 안의 서브 노드들(VisionClient 등) 초기화
        arm_action_server = ArmActionServer(loop)

        # 2. 여러 노드를 병렬로 처리할 수 있는 멀티스레드 Executor 생성
        executor = MultiThreadedExecutor()

        # 3. Executor에 이 시스템에서 사용하는 *모든* ROS 노드를 등록
        executor.add_node(arm_action_server)              # 메인 액션 서버 노드
        executor.add_node(arm_action_server.vision_client)     # 비전 서비스 클라이언트 노드 (PBVS 타임아웃 해결의 핵심)
        executor.add_node(arm_action_server.joint_publisher)   # 관절 상태 발행 노드

        # 4. ROS 노드들의 콜백 처리를 담당할 스핀을 별도의 스레드에서 실행
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        arm_action_server.get_logger().info(f"ROS 스핀 스레드 시작 (ID: {ros_thread.ident}). {executor.get_nodes()} 노드를 처리합니다.")
        arm_action_server.get_logger().info(f"Asyncio 이벤트 루프 시작 (메인 스레드 ID: {threading.get_ident()})")
        
        # 5. 메인 스레드는 비동기(async) 코드를 실행하는 이벤트 루프를 계속 실행
        loop.run_forever()

    except (KeyboardInterrupt, ConnectionError) as e:
        if isinstance(e, ConnectionError):
            print(f"초기화 실패: {e}")
        else:
            print("키보드 인터럽트로 종료합니다.")
    except Exception as e:
        # 실행 중인 노드가 있으면 로거를 통해 에러를 기록
        if arm_action_server:
            arm_action_server.get_logger().fatal(f"예기치 않은 오류로 노드를 종료합니다: {e}\n{traceback.format_exc()}")
        else:
            print(f"예기치 않은 오류로 노드를 종료합니다: {e}\n{traceback.format_exc()}")
    finally:
        # 프로그램 종료 시 모든 리소스를 순서대로 정리
        if loop.is_running():
            loop.stop()
        if executor:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print("모든 리소스 정리 완료. 프로그램을 종료합니다.")


if __name__ == '__main__':
    """이 스크립트가 직접 실행될 때 main 함수를 호출합니다."""
    main()
