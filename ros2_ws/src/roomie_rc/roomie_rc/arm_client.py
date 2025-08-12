#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ROS2 메시지 import
from roomie_msgs.action import SetPose


class ArmClient:
    """
    AC(Arm Controller)와의 통신을 담당하는 클라이언트
    """
    
    def __init__(self, parent_node: Node):
        self.node = parent_node
        
        # Arm Controller 액션 클라이언트 초기화
        self._init_action_clients()
        
        self.node.get_logger().info('Arm Client 초기화 완료')
    
    def _init_action_clients(self):
        """Arm Controller 액션 클라이언트 초기화"""
        # 팔 회전 Action Client (RC → AC)
        self.set_pose_client = ActionClient(
            self.node,
            SetPose,
            '/arm/action/set_pose'
        )
        self.node.get_logger().info('Arm Action Client 초기화 완료')
    
    def set_arm_pose(self, pose_id: int, callback=None):
        """
        팔 회전 명령 전송
        Args:
            pose_id (int): 목표 자세 ID
            callback (callable): 동작 완료 시 호출될 콜백 함수
        """
        if not self.set_pose_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn('SetPose 액션 서버를 찾을 수 없습니다')
            return False
        
        goal_msg = SetPose.Goal()
        goal_msg.robot_id = self.node.robot_id
        goal_msg.pose_id = pose_id
        
        self.node.get_logger().info(f'팔 회전 명령 전송: pose_id={pose_id}')
        
        try:
            send_goal_future = self.set_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self.node.get_logger().info(f'팔 회전 진행 중: {feedback.feedback.progress}%')
            )
            
            if callback:
                send_goal_future.add_done_callback(
                    lambda future: self._handle_set_pose_response(future, callback)
                )
            
            self.node.get_logger().info('팔 회전 명령 전송 완료')
            return True
                
        except Exception as e:
            self.node.get_logger().error(f'팔 회전 명령 전송 실패: {e}')
            return False
            
    def _handle_set_pose_response(self, future, callback):
        """팔 회전 응답 처리"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn('팔 회전 명령이 거부됨')
                if callback:
                    callback(None)
                return

            self.node.get_logger().info('팔 회전 명령 수락됨, 결과 대기 중...')
            
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future: self._handle_set_pose_result(future, callback)
            )
            
        except Exception as e:
            self.node.get_logger().error(f'팔 회전 응답 처리 중 오류: {e}')
            if callback:
                callback(None)
    
    def _handle_set_pose_result(self, future, callback):
        """팔 회전 결과 처리"""
        try:
            result = future.result()
            self.node.get_logger().info(f'팔 회전 결과 수신: success={result.result.success}')
            if callback:
                callback(result)
        except Exception as e:
            self.node.get_logger().error(f'팔 회전 결과 처리 중 오류: {e}')
            if callback:
                callback(None)