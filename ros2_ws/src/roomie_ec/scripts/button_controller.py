#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import numpy as np
from collections import deque

class ButtonController(Node):
    def __init__(self):
        super().__init__('button_controller')
        
        # 구독자: 버튼 탐지 결과
        self.button_sub = self.create_subscription(
            Point, 'detected_buttons', self.button_callback, 10)
        
        # 발행자: 로봇 제어 명령
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 제어 파라미터
        self.target_size = 0.06  # 목표 버튼 크기 (증가)
        self.target_x = 0.5      # 목표 x좌표 (화면 중앙)
        self.min_confidence = 0.7  # 최소 신뢰도
        
        # 속도 제한
        self.max_linear_speed = 0.1   # 최대 선속도 (m/s)
        self.min_linear_speed = 0.009 # 최소 선속도 (m/s) - 추가
        self.max_angular_speed = 0.2  # 최대 각속도 (rad/s)
        
        # 제어 게인
        self.kp_x = 0.5    # x좌표 제어 게인
        self.kp_size = 1.0 # 크기 제어 게인 (증가)
        
        # 버튼 데이터 저장
        self.button_data = deque(maxlen=5)  # 최근 5개 데이터
        self.current_target = None
        
        # 제어 주기 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('버튼 제어 노드가 시작되었습니다.')
    
    def button_callback(self, msg):
        """버튼 탐지 결과를 받아서 처리"""
        # 신뢰도가 낮은 버튼 제외
        if msg.z < self.min_confidence:
            return
        
        # 버튼 데이터 저장
        button_info = {
            'x': msg.x,
            'size': msg.y,
            'confidence': msg.z,
            'timestamp': self.get_clock().now()
        }
        
        self.button_data.append(button_info)
    
    def select_best_button(self):
        """가장 적합한 버튼 선택"""
        if not self.button_data:
            return None
        
        # 최근 데이터에서 가장 큰 버튼 선택
        best_button = max(self.button_data, key=lambda b: b['size'])
        
        # 데이터가 너무 오래된 경우 제외
        current_time = self.get_clock().now()
        if (current_time - best_button['timestamp']).nanoseconds > 1e9:  # 1초 이상
            return None
        
        return best_button
    
    def control_loop(self):
        """메인 제어 루프"""
        # 최적의 버튼 선택
        target_button = self.select_best_button()
        
        if target_button is None:
            # 버튼이 감지되지 않으면 정지
            self.stop_robot()
            return
        
        # 현재 버튼 정보
        current_x = target_button['x']
        current_size = target_button['size']
        confidence = target_button['confidence']
        
        # 제어 명령 계산
        cmd_vel = Twist()
        
        # 크기 기반 전진/후진 제어
        size_error = self.target_size - current_size
        linear_speed = self.kp_size * size_error
        
        # x좌표 기반 회전 제어
        x_error = self.target_x - current_x
        angular_speed = self.kp_x * x_error
        
        # 속도 제한 (최소/최대)
        if linear_speed > 0:  # 전진할 때
            linear_speed = np.clip(linear_speed, self.min_linear_speed, self.max_linear_speed)
        else:  # 후진할 때
            linear_speed = np.clip(linear_speed, -self.max_linear_speed, -self.min_linear_speed)
        
        angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)
        
        # 명령 적용
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        
        # 로그 출력
        self.get_logger().info(
            f'버튼: x={current_x:.3f}, size={current_size:.3f}, conf={confidence:.3f} | '
            f'제어: linear={linear_speed:.3f}, angular={angular_speed:.3f}'
        )
        
        # 목표 달성 확인
        if abs(size_error) < 0.005 and abs(x_error) < 0.05:
            self.get_logger().info('목표 위치 도달! 정지합니다.')
            self.stop_robot()
            return
        
        # 명령 발행
        self.cmd_vel_pub.publish(cmd_vel)
    
    def stop_robot(self):
        """로봇 정지"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    
    controller = ButtonController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 