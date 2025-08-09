
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

    # [신규] 중복 제거를 위한 내부 헬퍼 함수
    def _execute_and_update(self, target_angles_deg: np.ndarray) -> bool:
        """시리얼 명령을 보내고, 성공 시 내부 상태를 '보낸 명령 기준'으로 업데이트합니다."""
        success = self.serial.send_command(target_angles_deg)
        if success:
            # [핵심 수정] ESP32의 응답 대신, 우리가 보낸 target_angles_deg로 상태 업데이트
            self.current_angles_rad = self._convert_servo_deg_to_rad(target_angles_deg)
            self.joint_publisher.publish(config.JOINT_NAMES, self.current_angles_rad)
            return True
        return False

    def move_to_angles_deg(self, target_angles_deg: np.ndarray) -> bool:
        """미리 정의된 서보 각도로 직접 이동합니다."""
        self._log(f"서보 각도 [{target_angles_deg}]로 직접 이동합니다.")
        if not self._execute_and_update(target_angles_deg):
            self._log("서보 이동 명령 실패.", error=True)
            return False
        return True

    def move_to_pose_ik(self, target_xyz: np.ndarray, target_orientation_vector: np.ndarray) -> bool:
        """IK를 계산하여 목표 3D 좌표와 '방향 벡터'로 이동합니다."""
        self._log(f"IK를 이용해 좌표 {np.round(target_xyz, 3)}, 방향 {np.round(target_orientation_vector, 2)}로 이동합니다.")
        
        solution_rad = self.kin_solver.solve_ik(target_xyz, target_orientation_vector, self.current_angles_rad)
        if solution_rad is None:
            self._log("IK 해를 찾지 못했습니다.", error=True)
            return False
            
        servo_angles_deg = self._convert_rad_to_servo_deg(solution_rad)
        
        if not self._execute_and_update(servo_angles_deg):
            self._log("IK 이동 후 서보 명령 실패.", error=True)
            return False
            
        self._log("IK 이동 성공.")
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