#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile

# ROS2 메시지 import
from roomie_msgs.action import PerformTask, PerformReturn
from roomie_msgs.msg import RobotState, BatteryStatus, RoomiePose
from geometry_msgs.msg import Pose, Point, Quaternion


class RMSClient:
    """RMS(Roomie Main Service)와의 통신을 담당하는 클라이언트"""
    
    def __init__(self, parent_node: Node):
        self.node = parent_node
        
        # QoS 프로파일 설정
        self.qos_profile = QoSProfile(depth=10)
        
        # Action Server, Publishers 초기화
        self._init_action_servers()
        self._init_publishers()
        
        self.node.get_logger().info('RMS Client 초기화 완료')
    
    def _init_action_servers(self):
        """Action Server 초기화"""
        self.perform_task_server = ActionServer(
            self.node,
            PerformTask,
            '/roomie/action/perform_task',
            self._perform_task_callback
        )
        
        self.perform_return_server = ActionServer(
            self.node,
            PerformReturn,
            '/roomie/action/perform_return',
            self._perform_return_callback
        )
        
        self.node.get_logger().info('Action Servers 초기화 완료')
    
    def _init_publishers(self):
        """Publisher 초기화"""
        # 로봇 상태 Publisher
        self.robot_state_pub = self.node.create_publisher(
            RobotState,
            '/roomie/status/robot_state',
            self.qos_profile
        )
        
        # 배터리 상태 Publisher  
        self.battery_status_pub = self.node.create_publisher(
            BatteryStatus,
            '/roomie/status/battery_status',
            self.qos_profile
        )
        
        # 로봇 위치 Publisher
        self.roomie_pose_pub = self.node.create_publisher(
            RoomiePose,
            '/roomie/status/roomie_pose',
            self.qos_profile
        )
        
        self.node.get_logger().info('Publishers 초기화 완료')
    
    def _perform_task_callback(self, goal_handle):
        """PerformTask Action 콜백 - 작업 수행 중 계속 유지"""
        request = goal_handle.request
        self.node.get_logger().info(f'작업 할당 받음: task_id={request.task_id}')
        self.node.get_logger().info(f'작업 타입: {request.task_type_id}')
        
        # 메인 노드에 작업 할당 알림 (goal_handle 전달하여 feedback/result 처리)
        if hasattr(self.node, 'handle_task_assignment'):
            self.node.handle_task_assignment(request, goal_handle)
        
        # 작업 수락하고 결과 반환
        result = PerformTask.Result()
        result.robot_id = request.robot_id
        result.task_id = request.task_id
        result.success = True
        result.message = "작업 수행 중"
        
        goal_handle.succeed()
        return result
    
    def _perform_return_callback(self, goal_handle):
        """PerformReturn Action 콜백"""
        request = goal_handle.request
        self.node.get_logger().info('복귀 작업 할당 받음')
        
        # 메인 노드에 복귀 작업 알림 (goal_handle 전달)
        if hasattr(self.node, 'handle_return_task'):
            self.node.handle_return_task(request, goal_handle)
        
        # 작업 수락하고 결과 반환
        result = PerformReturn.Result()
        result.robot_id = request.robot_id
        result.success = True
        result.message = "복귀 작업 수행 중"
        
        goal_handle.succeed()
        return result
    
    def publish_robot_state(self, robot_state_id):
        """로봇 상태 발행"""
        msg = RobotState()
        msg.robot_id = self.node.robot_id
        msg.robot_state_id = robot_state_id
        
        self.robot_state_pub.publish(msg)
        # self.node.get_logger().info(f'로봇 상태 발행: {robot_state_id}')
    
    def publish_battery_status(self, charge_percentage=85.0, is_charging=False):
        """배터리 상태 발행"""
        msg = BatteryStatus()
        msg.robot_id = self.node.robot_id
        msg.charge_percentage = charge_percentage
        msg.is_charging = is_charging
        
        self.battery_status_pub.publish(msg)
        # self.node.get_logger().info(f'배터리 상태 발행: {charge_percentage}%')
    
    def publish_roomie_pose(self, floor_id=0, x=0.0, y=0.0, z=0.0):
        """로봇 위치 발행"""
        msg = RoomiePose()
        msg.robot_id = self.node.robot_id
        msg.floor_id = floor_id
        
        msg.pose = Pose()
        msg.pose.position = Point(x=x, y=y, z=z)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.roomie_pose_pub.publish(msg)
        # self.node.get_logger().info(f'위치 발행: floor_id={floor_id}, x={x}, y={y}')