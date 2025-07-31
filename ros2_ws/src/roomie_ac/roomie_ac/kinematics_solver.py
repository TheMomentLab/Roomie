# roomie_arm_control/kinematics_solver.py

import numpy as np
from ikpy.chain import Chain
from . import config # config.py 임포트

class KinematicsSolver:
    """ikpy를 사용하여 Inverse Kinematics 계산을 수행합니다."""
    # __init__ 메서드에서 urdf_path와 active_links_mask를 직접 받지 않고 config에서 가져옵니다.
    def __init__(self):
        # config.py에서 변수 가져오기
        self.chain = Chain.from_urdf_file(str(config.URDF_FILE), active_links_mask=config.ACTIVE_LINKS_MASK)
        self.active_links_mask = config.ACTIVE_LINKS_MASK

    def _get_full_joints(self, active_joints_rad):
        """활성 관절 배열(len=4)을 전체 관절 배열(len=6)로 변환합니다."""
        full_joints = np.zeros(len(self.chain.links))
        full_joints[self.active_links_mask] = active_joints_rad
        return full_joints

    def solve_ik(self, target_pos, current_active_angles_rad):
        """
        목표 3D 위치에 대한 역기구학(IK) 솔루션을 계산합니다.
        target_pos: 로봇 팔 끝점의 목표 3D 좌표 (numpy 배열 [x, y, z], 단위: m)
        current_active_angles_rad: 현재 활성 관절의 각도 (numpy 배열, 단위: 라디안) - IK 계산의 초기 시드
        반환: 성공 시 활성 관절의 해(라디안), 실패 시 None
        """
        if config.DEBUG:
            print("\n--- IK 계산 시작 ---")
            print(f"  - 목표 좌표 (m): {np.round(target_pos, 4)}")
            print(f"  - 현재 활성 관절 각도 (rad): {np.round(current_active_angles_rad, 4)}")

        q_full_seed = self._get_full_joints(current_active_angles_rad)
        
        try:
            q_solution_all = self.chain.inverse_kinematics(
                target_position=target_pos,
                initial_position=q_full_seed,
                max_iter=config.IK_MAX_ITERATIONS # config.py의 IK_MAX_ITERATIONS 사용
            )
        except Exception as e:
            if config.DEBUG:
                print(f"❌ IK 계산 중 오류 발생: {e}")
            return None
        
        final_pos = self.chain.forward_kinematics(q_solution_all)[:3, 3]
        error = np.linalg.norm(final_pos - target_pos)

        active_solution_rad = q_solution_all[self.active_links_mask]
        if config.DEBUG:
            print(f"  - IK 결과 (활성, rad): {np.round(active_solution_rad, 4)}")
            print(f"  - 최종 도달 좌표 (m): {np.round(final_pos, 4)}")
            print(f"  - 오차 (m): {error:.6f}")

        if error > config.IK_TOLERANCE_M: # config.py의 IK_TOLERANCE_M 사용
            if config.DEBUG:
                print(f"⚠️ IK 오차가 허용 범위를 초과합니다 (오차: {error*1000:.2f} mm, 허용: {config.IK_TOLERANCE_M*1000:.2f} mm).")
            return None 
        
        # 계산된 각도가 관절 제한을 벗어나는지 추가 검사
        for i, angle_rad in enumerate(active_solution_rad):
            if not (config.JOINT_LIMIT_RAD[i][0] <= angle_rad <= config.JOINT_LIMIT_RAD[i][1]):
                if config.DEBUG:
                    print(f"🚫 관절 {i+1}의 IK 결과 각도({np.rad2deg(angle_rad):.2f} deg)가 제한({np.rad2deg(config.JOINT_LIMIT_RAD[i][0]):.2f}~{np.rad2deg(config.JOINT_LIMIT_RAD[i][1]):.2f} deg)을 벗어납니다.")
                return None

        return active_solution_rad