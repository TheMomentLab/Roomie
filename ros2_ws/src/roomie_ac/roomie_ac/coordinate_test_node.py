# test_coord_transform.py (여러 자세 테스트 버전)

import rclpy
import asyncio
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# 필요한 모듈들을 가져옵니다.
from roomie_ac.vision_client import VisionServiceClient
from roomie_ac.kinematics_solver import KinematicsSolver
from roomie_ac.motion_controller import MotionController
from roomie_ac.serial_manager import SerialManager
from roomie_ac.ros_joint_publisher import ROSJointPublisher
from roomie_ac.coordinate_transformer import CoordinateTransformer
from roomie_ac.config import Pose, POSE_ANGLES_DEG

class CoordinateTestNode(Node):
    def __init__(self):
        super().__init__('coordinate_test_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.serial_manager = SerialManager()
        self.kin_solver = KinematicsSolver()
        self.joint_publisher = ROSJointPublisher(callback_group=self.callback_group)
        self.vision_client = VisionServiceClient(callback_group=self.callback_group)
        self.coord_transformer = CoordinateTransformer()
        self.motion_controller = MotionController(self.kin_solver, self.serial_manager, self.joint_publisher)
        
        if self.serial_manager.connect() is None:
            self.get_logger().fatal("시리얼 연결 실패. 테스트를 진행할 수 없습니다.")
            rclpy.shutdown()
            return

    async def run_test(self):
        # ======================= [핵심 수정] =======================
        # 1. 테스트하고 싶은 자세들을 리스트로 정의합니다.
        #    config.py에 정의된 Pose Enum 중에서 원하는 자세를 추가/삭제할 수 있습니다.
        poses_to_test = [
            Pose.OBSERVE,
            Pose.OBSERVE1,
            Pose.OBSERVE2,
            Pose.OBSERVE3
        ]
        self.get_logger().info(f"총 {len(poses_to_test)}개의 자세에 대한 좌표 측정을 시작합니다.")
        await asyncio.sleep(1)

        # 2. 리스트에 있는 각 자세를 순서대로 테스트합니다.
        for pose in poses_to_test:
            self.get_logger().info(f"--- {pose.name} 자세로 이동합니다 ---")
            self.motion_controller.move_to_angles_deg(POSE_ANGLES_DEG[pose])
            # 로봇이 움직일 시간을 충분히 줍니다.
            await asyncio.sleep(3.0)
            self.get_logger().info(f"✅ {pose.name} 자세 도착. 5초간 좌표를 측정합니다.")

            # 3. 현재 자세의 로봇 변환 행렬을 계산합니다.
            fixed_robot_transform = self.motion_controller._get_current_transform()

            # 4. 5초 동안 1초 간격으로 좌표를 측정하고 출력합니다.
            for i in range(5):
                response = self.vision_client.request_button_status(robot_id=0, button_id=0)
                
                if response and response.success:
                    center_x_px = response.x * self.coord_transformer.cx * 2
                    center_y_px = response.y * self.coord_transformer.cy * 2
                    pixel_area = response.size * (self.coord_transformer.cx * 2) * (self.coord_transformer.cy * 2)
                    radius_px = np.sqrt(max(0, pixel_area) / np.pi)
                    
                    image_points_2d = np.array([
                        [center_x_px + radius_px, center_y_px], [center_x_px - radius_px, center_y_px],
                        [center_x_px, center_y_px + radius_px], [center_x_px, center_y_px - radius_px]
                    ], dtype=np.float32)

                    target_xyz, _ = self.coord_transformer.get_target_pose_from_points(
                        image_points_2d,
                        fixed_robot_transform
                    )

                    if target_xyz is not None:
                        log_msg = (
                            f"[{pose.name} 자세 측정 #{i+1}] 계산된 XYZ (m): "
                            f"X={target_xyz[0]:.4f}, Y={target_xyz[1]:.4f}, Z={target_xyz[2]:.4f}"
                        )
                        self.get_logger().info(log_msg)
                else:
                    self.get_logger().warn(f"[{pose.name} 자세] 카메라 시야에서 버튼(마커)을 찾을 수 없습니다...")
                await asyncio.sleep(1.0)
            
            self.get_logger().info(f"--- {pose.name} 자세 측정 완료 ---\n")
        # ==========================================================
        
        self.get_logger().info("모든 자세에 대한 테스트가 완료되었습니다. 프로그램을 종료합니다.")
        # 모든 테스트가 끝나면 rclpy를 종료시켜 프로그램을 끝냅니다.
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    test_node = CoordinateTestNode()
    try:
        if test_node.serial_manager.is_ready:
            asyncio.run(test_node.run_test())
    except KeyboardInterrupt:
        pass
    finally:
        if test_node.serial_manager.is_ready and rclpy.ok():
            test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()