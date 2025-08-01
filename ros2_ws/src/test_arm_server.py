#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from roomie_msgs.action import SetPose
import time

class TestArmServer(Node):
    def __init__(self):
        super().__init__('test_arm_server')
        
        # Action server 생성
        self._action_server = ActionServer(
            self,
            SetPose,
            '/arm/action/set_pose',
            self.execute_callback
        )
        
        self.get_logger().info('Test Arm Server started!')
        self.get_logger().info('Waiting for pose requests...')
    
    def execute_callback(self, goal_handle):
        """액션 실행 콜백"""
        self.get_logger().info(f'Received pose request: robot_id={goal_handle.request.robot_id}, pose_id={goal_handle.request.pose_id}')
        
        # 5초 대기 (시뮬레이션)
        for i in range(5):
            self.get_logger().info(f'Processing... {i+1}/5 seconds')
            time.sleep(1)
        
        # 결과 생성
        result = SetPose.Result()
        result.robot_id = goal_handle.request.robot_id
        result.success = True
        
        self.get_logger().info(f'Pose {goal_handle.request.pose_id} completed successfully!')
        
        # 성공 상태로 설정하고 결과 반환
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded, returning result')
        return result

def main(args=None):
    rclpy.init(args=args)
    
    test_server = TestArmServer()
    
    try:
        rclpy.spin(test_server)
    except KeyboardInterrupt:
        test_server.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        test_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 