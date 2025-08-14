#!/usr/bin/env python3
"""
Simple Navigator 2 - PID 제어 이동 노드

엘리베이터 중앙으로의 정밀 이동을 위한 PID 제어 노드입니다.

=== 주요 기능 ===
- 목표 위치까지의 PID 제어 이동
- 후진을 위한 180도 반대 방향 회전 후 이동
- 상태 머신: rotate_to_goal → move_to_goal → rotate_to_final → idle
- 완료 신호 발행: /simple_nav/status 토픽으로 "completed" 메시지

=== 해결된 문제 ===
✅ idle 상태에서 cmd_vel 발행하지 않음 (roomie_ec_node와의 충돌 방지)
✅ 완료 시 즉시 "completed" 메시지 발행 (roomie_ec_node 동기화)

=== 사용법 ===
- 목표 위치: /simple_goal_pose 토픽으로 PoseStamped 메시지 수신
- 완료 신호: /simple_nav/status 토픽으로 String 메시지 발행
- 상태 변화: 각 상태 전환 시 로그 메시지 발행
"""

# simple_navigator2.py (후진 PID 제어 및 상태 머신 적용 버전)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import math
from std_msgs.msg import String

class PID:
    """간단한 PID 제어기 클래스"""
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.P = P
        self.I = I
        self.D = D
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error):
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.P * error + self.I * self.integral + self.D * derivative

    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0

def normalize_angle(angle):
    """각도를 -pi ~ pi 범위로 정규화"""
    return math.atan2(math.sin(angle), math.cos(angle))

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator_reverse_pid')

        # --- 제어 관련 변수 
        self.angle_tolerance = 0.05  # 각도 허용 오차 (라디안)
        self.distance_tolerance = 0.10 # 거리 허용 오차 (미터)
        1
        # 각속도 PID 제어기
        self.angular_pid = PID(P=0.8, I=0.0, D=0.2)
        # 선속도 PID 제어기
        self.linear_pid = PID(P=0.4, I=0.0, D=0.1)
        
        # TF 리스너 및 버퍼
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 퍼블리셔 및 서브스크라이버
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            PoseStamped, 'simple_goal_pose', self.goal_callback, 10)
        
        # 완료 상태 발행자
        self.status_publisher = self.create_publisher(String, '/simple_nav/status', 10)
        
        # 0.1초마다 제어 루프 실행
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 상태 변수
        self.simple_goal_pose = None
        self.state = "idle" # 초기 상태
        self.previous_state = "idle"  # 이전 상태 추적
        
        self.get_logger().info('Simple Reverse PID Navigator has been started.')

    def goal_callback(self, msg):
        """새로운 목표 지점을 받으면 상태를 변경하고 PID 리셋"""
        self.simple_goal_pose = msg
        self.state = "rotate_to_goal"
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.get_logger().info(f'New goal received. State: {self.state}')
        
        # 목표 수신 시 상태 발행
        status_msg = String()
        status_msg.data = "goal_received"
        self.status_publisher.publish(status_msg)

    def control_loop(self):
        """상태에 따라 적절한 제어 핸들러를 호출하는 메인 루프"""
        if self.simple_goal_pose is None or self.state == "idle":
            # 목표가 없거나 idle 상태면 cmd_vel 발행하지 않음
            return
        
        try:
            # 로봇의 현재 위치와 방향(yaw)을 TF에서 가져오기
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception as e:
            self.get_logger().warn(f'Could not get robot pose: {e}')
            return

        # 상태 변화 감지 및 발행
        if self.state != self.previous_state:
            self.get_logger().info(f'State changed: {self.previous_state} -> {self.state}')
            
            # 완료 상태 발행
            status_msg = String()
            if self.state == "idle" and self.previous_state != "idle":
                status_msg.data = "completed"
                self.get_logger().info('Navigation completed! Publishing completion status.')
            else:
                status_msg.data = self.state
            self.status_publisher.publish(status_msg)
            
            self.previous_state = self.state
        
        # 상태에 따라 핸들러 함수 호출
        twist_msg = Twist()
        if self.state == "rotate_to_goal":
            twist_msg = self.handle_rotate_to_goal(current_x, current_y, current_yaw)
        elif self.state == "move_to_goal":
            twist_msg = self.handle_move_to_goal(current_x, current_y, current_yaw)
        elif self.state == "rotate_to_final":
            twist_msg = self.handle_rotate_to_final(current_yaw)
        
        self.publisher_.publish(twist_msg)

    def handle_rotate_to_goal(self, current_x, current_y, current_yaw):
        """목표 지점의 180도 반대 방향으로 회전하는 상태 (후진을 위해)"""
        twist_msg = Twist()
        desired_heading = math.atan2(self.simple_goal_pose.pose.position.y - current_y,
                                        self.simple_goal_pose.pose.position.x - current_x)
        # 후진을 위해 180도 반대 방향으로 회전
        desired_heading = normalize_angle(desired_heading + math.pi)
        error_angle = normalize_angle(desired_heading - current_yaw)

        if abs(error_angle) > self.angle_tolerance:
            twist_msg.angular.z = self.angular_pid.update(error_angle)
        else:
            twist_msg.angular.z = 0.0
            self.state = "move_to_goal"
            self.get_logger().info(f'Reverse heading aligned. State: {self.state}')
        return twist_msg

    def handle_move_to_goal(self, current_x, current_y, current_yaw):
        """목표 지점을 향해 후진하는 상태 (동시에 각도 보정)"""
        twist_msg = Twist()
        dx = self.simple_goal_pose.pose.position.x - current_x
        dy = self.simple_goal_pose.pose.position.y - current_y
        distance_error = math.sqrt(dx**2 + dy**2)

        if distance_error > self.distance_tolerance:
            # 거리 오차로 선속도 계산 (후진이므로 음수)
            twist_msg.linear.x = -self.linear_pid.update(distance_error)
            
            # 후진 중에도 계속 목표 지점의 반대 방향을 바라보도록 각도 보정
            desired_heading = math.atan2(dy, dx)
            desired_heading = normalize_angle(desired_heading + math.pi)  # 180도 반대
            error_angle = normalize_angle(desired_heading - current_yaw)
            twist_msg.angular.z = self.angular_pid.update(error_angle)
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.state = "rotate_to_final"
            self.get_logger().info(f'Position reached. State: {self.state}')
        return twist_msg

    def handle_rotate_to_final(self, current_yaw):
        """최종 목표 방향으로 회전하는 상태"""
        twist_msg = Twist()
        q = self.simple_goal_pose.pose.orientation
        _, _, final_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        final_error_angle = normalize_angle(final_yaw - current_yaw)

        if abs(final_error_angle) > self.angle_tolerance:
            twist_msg.angular.z = self.angular_pid.update(final_error_angle)
        else:
            twist_msg.angular.z = 0.0
            self.state = "idle" # 목표 달성 후 idle 상태로 복귀
            self.simple_goal_pose = None
            self.get_logger().info(f'Final orientation reached. Goal achieved! State: {self.state}')
            
            # 완료 상태 즉시 발행
            status_msg = String()
            status_msg.data = "completed"
            self.status_publisher.publish(status_msg)
            self.get_logger().info('Navigation completed!')
        return twist_msg

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()