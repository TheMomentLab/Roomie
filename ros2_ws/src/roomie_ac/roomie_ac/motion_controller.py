import numpy as np
from . import config
from .kinematics_solver import KinematicsSolver
from .serial_manager import SerialManager
from .ros_joint_publisher import ROSJointPublisher

class MotionController:
    def __init__(self, kin_solver: KinematicsSolver, serial_manager: SerialManager, joint_publisher: ROSJointPublisher):
        self.kin_solver = kin_solver
        self.serial = serial_manager
        self.joint_publisher = joint_publisher
        self.current_angles_rad = self._convert_servo_deg_to_rad(np.array(config.HOME_POSITION_SERVO_DEG))

    async def move_to_angles_deg(self, target_angles_deg: np.ndarray, blocking: bool = True) -> bool:
        self._log(f"서보 각도 [{target_angles_deg}]로 이동 명령 (Blocking: {blocking})")

        if not self.serial.send_command(target_angles_deg):
            self._log("명령 전송 실패.", error=True)
            return False

        if blocking:
            # [핵심 수정] await를 사용하여 비동기 함수 호출
            if not await self.serial.wait_for_ack(config.SERIAL_TIMEOUT):
                self._log("동작 완료 신호(ACK) 대기 시간 초과.", error=True)
                return False

        self.current_angles_rad = self._convert_servo_deg_to_rad(target_angles_deg)
        self.joint_publisher.publish(config.JOINT_NAMES, self.current_angles_rad)
        return True

    # [수정] move_to_pose_ik를 async def로 변경
    async def move_to_pose_ik(self, target_xyz: np.ndarray, orientation: np.ndarray = None, blocking: bool = True) -> bool:
        """
        [수정됨] IK를 사용하여 목표 위치로 이동합니다.
        orientation이 None이면 IK가 최적의 방향을 자동으로 찾습니다.
        """
        log_msg = f"IK 이동: 좌표 {np.round(target_xyz, 3)}, 방향: {'자동' if orientation is None else '지정됨'} (Blocking: {blocking})"
        self._log(log_msg)
        
        # [수정] orientation 인자를 solver에 그대로 전달
        solution_rad = self.kin_solver.solve_ik(target_xyz, orientation, self.current_angles_rad)
        if solution_rad is None:
            self._log("IK 해를 찾지 못했습니다.", error=True)
            return False
            
        servo_angles_deg = self._convert_rad_to_servo_deg(solution_rad)
        
        return await self.move_to_angles_deg(servo_angles_deg, blocking=blocking)
    
    # [신규] 중복 제거를 위한 내부 헬퍼 함수
    async def _execute_and_update(self, target_angles_deg: np.ndarray) -> bool:
        if not self.serial.send_command(target_angles_deg):
            return False
        
        if not await self.serial.wait_for_ack(config.SERIAL_TIMEOUT):
            self._log("ESP32로부터 동작 완료 신호를 받지 못했습니다.", error=True)
            return False

        self.current_angles_rad = self._convert_servo_deg_to_rad(target_angles_deg)
        self.joint_publisher.publish(config.JOINT_NAMES, self.current_angles_rad)
        return True


    def _get_current_transform(self) -> np.ndarray:
        full_joints = self.kin_solver._get_full_joints(self.current_angles_rad)
        return self.kin_solver.chain.forward_kinematics(full_joints)
    
    def _convert_rad_to_servo_deg(self, angles_rad: np.ndarray) -> np.ndarray:
        angles_deg_ik = np.rad2deg(angles_rad)
        servo_angles = config.SERVO_ZERO_OFFSET_DEG + angles_deg_ik * config.SERVO_DIRECTION_MULTIPLIER
        self._log(f"각도 변환 (Rad->Deg): {np.round(angles_rad, 2)} -> {np.round(servo_angles).astype(int)}")
        return np.round(servo_angles).astype(int)

    def _convert_servo_deg_to_rad(self, angles_deg: np.ndarray) -> np.ndarray:
        angles_deg_ik = (angles_deg - config.SERVO_ZERO_OFFSET_DEG) * config.SERVO_DIRECTION_MULTIPLIER
        rad_angles = np.deg2rad(angles_deg_ik)
        self._log(f"각도 변환 (Deg->Rad): {angles_deg} -> {np.round(rad_angles, 2)}")
        return rad_angles

    def _log(self, message: str, error: bool = False):
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[MotionController][{log_level}] {message}")