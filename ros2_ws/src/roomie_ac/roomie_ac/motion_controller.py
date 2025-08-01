# roomie_arm_control/motion_controller.py

import numpy as np
from . import config
from .kinematics_solver import KinematicsSolver
from .serial_manager import SerialManager
from .ros_joint_publisher import ROSJointPublisher
class MotionController:
    """
    로봇 팔의 모든 물리적 움직임을 담당하는 클래스 (전문 연주자).
    IK 계산, 시리얼 통신 등의 저수준 제어를 추상화하여,
    ac_node.py(지휘자)가 이해하기 쉬운 고수준의 동작 함수를 제공합니다.
    """
    def __init__(self, kin_solver: KinematicsSolver, serial_manager: SerialManager, joint_publisher: ROSJointPublisher):
        """
        움직임 제어에 필요한 도구들(kin_solver, serial_manager)을 전달받습니다.
        """
        self.kin_solver = kin_solver
        self.serial = serial_manager
        self.joint_publisher = joint_publisher
        # 로봇 팔의 현재 관절 각도를 내부에서 실시간으로 추적합니다. (단위: 라디안)
        # 초기값은 config의 홈 포지션으로 설정합니다.
        self.current_angles_rad = self._convert_servo_deg_to_rad(np.array(config.HOME_POSITION_SERVO_DEG))

    def move_to_angles_deg(self, target_angles_deg: np.ndarray) -> bool:
        """
        [Public] 미리 정의된 서보 각도(0-180)로 직접 이동합니다. (set_pose 용)
        """
        self._log(f"서보 각도 [{target_angles_deg}]로 직접 이동합니다.")
        response = self.serial.send_command(target_angles_deg)
        
        # ======================= [디버깅 코드 추가] =======================
        self._log(f"==> [DEBUG] SerialManager로부터 받은 응답: {response}")
        # =================================================================
        
        if response is not None:
            # 성공 시, 내부 현재 각도를 업데이트합니다.
            self.current_angles_rad = self._convert_servo_deg_to_rad(response)
            self.joint_publisher.publish(config.JOINT_NAMES, self.current_angles_rad)
            return True
        else:
            self._log("서보 이동 명령 실패.", error=True)
            return False

    def move_to_pose_ik(self, target_xyz: np.ndarray) -> bool:
        """
        [Public] IK를 계산하여 목표 3D 좌표로 팔 끝점을 이동시킵니다.
        """
        self._log(f"IK를 이용해 3D 좌표 {np.round(target_xyz, 3)}로 이동합니다.")
        
        # 1. IK 해 찾기
        solution_rad = self.kin_solver.solve_ik(target_xyz, self.current_angles_rad)
        
        # 2. IK 성공 여부 확인
        if solution_rad is None:
            self._log("IK 해를 찾지 못했습니다.", error=True)
            return False
            
        # 3. 실제 서보 각도로 변환
        servo_angles_deg = self._convert_rad_to_servo_deg(solution_rad)
        
        # 4. 시리얼 명령 전송
        response = self.serial.send_command(servo_angles_deg)
        
        # 5. 상태 업데이트 및 결과 반환
        if response is not None:
            self.current_angles_rad = self._convert_servo_deg_to_rad(response)
            self._log("IK 이동 성공.")
            self.joint_publisher.publish(config.JOINT_NAMES, self.current_angles_rad)
            return True
        else:
            self._log("IK 이동 후 서보 명령 실패.", error=True)
            return False

    def move_relative_cartesian(self, move_vector_m: np.ndarray) -> bool:
        """
        [Public] 현재 팔 끝점 기준, 3D 벡터만큼 상대적으로 이동합니다.
        (press_forward, retreat의 기반이 되는 함수)
        """
        # 1. 현재 3D 위치 및 회전 정보 얻기
        try:
            current_transform = self._get_current_transform()
            current_position = current_transform[:3, 3]
            rotation_matrix = current_transform[:3, :3]
        except Exception as e:
            self._log(f"현재 위치 계산 실패: {e}", error=True)
            return False

        # 2. 베이스 좌표계 기준의 최종 이동 벡터 계산
        # (로컬 이동 벡터를 현재 팔의 회전에 맞게 변환)
        global_move_vector = rotation_matrix.dot(move_vector_m)
        
        # 3. 최종 목표 지점 계산
        target_xyz = current_position + global_move_vector
        
        # 4. move_to_pose_ik 함수를 재사용하여 이동
        return self.move_to_pose_ik(target_xyz)

    def press_forward(self, distance_m=0.05) -> bool:
        """
        [Public] 현재 팔이 바라보는 방향(툴의 Z축)으로 전진하여 누릅니다.
        """
        self._log(f"{distance_m*100:.1f}cm 전진하여 누르기 동작을 수행합니다.")
        # 로컬 Z축으로 전진하는 벡터
        forward_vector_local = np.array([0, 0, distance_m])
        return self.move_relative_cartesian(forward_vector_local)

    def retreat(self, distance_m=0.05) -> bool:
        """
        [Public] 현재 팔이 바라보는 방향의 반대 방향으로 후퇴합니다.
        """
        self._log(f"{distance_m*100:.1f}cm 후퇴 동작을 수행합니다.")
        # 로컬 Z축으로 후퇴하는 벡터
        retreat_vector_local = np.array([0, 0, -distance_m])
        return self.move_relative_cartesian(retreat_vector_local)

    # ------------------- Helper / Private Methods -------------------

    def _get_current_transform(self) -> np.ndarray:
        """
        [Private] 현재 관절 각도를 기준으로 FK를 계산하여 전체 변환 행렬을 반환합니다.
        """
        # kinematics_solver의 내부 로직을 활용합니다.
        full_joints = self.kin_solver._get_full_joints(self.current_angles_rad)
        return self.kin_solver.chain.forward_kinematics(full_joints)
    
    def _convert_rad_to_servo_deg(self, angles_rad: np.ndarray) -> np.ndarray:
        angles_deg_ik = np.rad2deg(angles_rad)
        servo_angles = config.SERVO_ZERO_OFFSET_DEG + angles_deg_ik * config.SERVO_DIRECTION_MULTIPLIER
        # [추가] 디버그 로그
        self._log(f"각도 변환 (Rad->Deg): {np.round(angles_rad, 2)} -> {np.round(servo_angles).astype(int)}")
        return np.round(servo_angles).astype(int)


    def _convert_servo_deg_to_rad(self, angles_deg: np.ndarray) -> np.ndarray:
        angles_deg_ik = (angles_deg - config.SERVO_ZERO_OFFSET_DEG) * config.SERVO_DIRECTION_MULTIPLIER
        rad_angles = np.deg2rad(angles_deg_ik)
        # [추가] 디버그 로그
        self._log(f"각도 변환 (Deg->Rad): {angles_deg} -> {np.round(rad_angles, 2)}")
        return rad_angles

    def _log(self, message: str, error: bool = False):
        """
        [Private] 디버깅을 위한 로그 출력 함수
        """
        if config.DEBUG:
            log_level = "ERROR" if error else "INFO"
            print(f"[MotionController][{log_level}] {message}")
