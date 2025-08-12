#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from roomie_msgs.msg import Obstacle
import time


class SafetyMonitorNode(Node):
    """
    개선된 안전 모니터 노드
    1. goal_pose 받아서 저장
    2. 주기적으로 장애물 상태 모니터링
    3. 장애물 감지시 정지, 사라지면 5초 후 재출발
    4. 목적지 도착하면 끝
    """
    
    def __init__(self):
        super().__init__('safety_monitor_node')
        
        # 안전 거리 설정
        self.safety_distance = 2.5
        self.monitor_period = 0.1  # 200ms마다 모니터링
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # cmd_vel 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # 토픽 구독
        self.obstacle_sub = self.create_subscription(
            Obstacle, '/vs/obstacle', self.obstacle_callback, 1)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 1)
        
        # 상태 변수
        self.current_goal = None
        self.goal_handle = None
        self.navigation_state = "IDLE"  # IDLE, NAVIGATING, STOPPED, COMPLETED
        
        # 장애물 상태 관리
        self.current_obstacle = None
        self.last_obstacle_time = None
        self.obstacle_clear_duration = 2.0  # 2초 후 재출발
        self.restart_timer_active = False  # 재출발 타이머 활성화 여부
        self.restart_timer = None # 타이머 객체 초기화
        
        # 주기적 모니터링 타이머
        self.monitor_timer = self.create_timer(
            self.monitor_period, self.monitor_safety_status)
        
        self.get_logger().info('🚀 개선된 안전 모니터 노드 시작')
        self.get_logger().info(f'안전 거리: {self.safety_distance}m, 모니터링 주기: {self.monitor_period}s')
    
    def goal_callback(self, msg):
        """1. goal_pose 받아서 저장"""
        self.current_goal = msg
        self.navigation_state = "IDLE"
        
        # 상태 변수 초기화
        self.current_obstacle = None
        self.last_obstacle_time = None
        self.restart_timer_active = False
        if self.restart_timer:
            self.restart_timer.cancel()
            self.restart_timer = None

        self.get_logger().info(f'🎯 목적지 설정: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        # 바로 출발
        self.start_navigation()
    
    def obstacle_callback(self, msg):
        """장애물 정보 업데이트 (판단은 monitor_safety_status에서)"""
        self.current_obstacle = msg
        self.last_obstacle_time = time.time()
        
    def monitor_safety_status(self):
        """2. 주기적으로 안전 상태 모니터링"""
        if self.navigation_state not in ["NAVIGATING", "STOPPED"]:
            return
        
        # 장애물 데이터가 너무 오래된 경우 (1초 이상)
        current_time = time.time()
        if (self.last_obstacle_time is None or 
            current_time - self.last_obstacle_time > 1.0):
            # 장애물 데이터 없음 - 안전하다고 가정
            if self.current_obstacle is not None:
                self.get_logger().info('✅ 장애물 데이터 소실 - 안전한 것으로 판단')
                self.current_obstacle = None
            self.handle_safe_condition()
            return
        
        # 장애물 데이터 있음 - 위험도 판단
        if self.is_dangerous_obstacle():
            # 5초 타이머가 활성화된 상태에서 위험한 장애물이 재감지되면 타이머를 취소
            if self.restart_timer_active:
                self.get_logger().warn('🚨 재출발 타이머 중, 위험한 장애물 재감지! 타이머를 취소합니다.')
                if self.restart_timer:
                    self.restart_timer.cancel()
                    self.restart_timer = None
                self.restart_timer_active = False
            self.handle_dangerous_obstacle()
        else:
            self.handle_safe_condition()
    
    def is_dangerous_obstacle(self):
        """위험한 장애물인지 판단"""
        if self.current_obstacle is None:
            return False
        
        return (self.current_obstacle.dynamic and 
                self.current_obstacle.depth <= self.safety_distance)
    
    def handle_dangerous_obstacle(self):
        """위험한 장애물 감지시 처리"""
        if self.navigation_state == "NAVIGATING":
            self.navigation_state = "STOPPED"
            self.get_logger().warn(
                f'🛑 위험한 장애물 감지! 거리: {self.current_obstacle.depth:.2f}m')
            self.stop_navigation()
    
    def handle_safe_condition(self):
        """안전한 상황 처리"""
        if self.navigation_state == "STOPPED" and not self.restart_timer_active:
            self.get_logger().info('✅ 안전 상태 확인됨')
            self.restart_after_delay()
    
    def restart_after_delay(self):
        """5초 후 재출발 (중복 타이머 방지)"""
        if self.restart_timer_active:
            return
        
        self.restart_timer_active = True
        self.get_logger().info('🔄 5초 후 재출발 예정!')
        
        # 새 타이머 생성
        self.restart_timer = self.create_timer(
            self.obstacle_clear_duration, self.restart_navigation_once)
    
    def start_navigation(self):
        """목적지로 출발"""
        if not self.current_goal or self.navigation_state == "NAVIGATING":
            return
        
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('❌ Nav2 서버 연결 실패')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_goal
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        self.navigation_state = "NAVIGATING"
        self.get_logger().info('🚀 목적지로 출발!')
    
    def goal_response_callback(self, future):
        """목표 응답 처리"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('❌ 목표 거부됨')
                return
            
            self.goal_handle = goal_handle
            
            # 결과 모니터링
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'목표 설정 오류: {e}')
    
    def navigation_result_callback(self, future):
        """4. 네비게이션 결과 - 도착하면 끝"""
        try:
            result = future.result()
            if result.status == 4:  # SUCCEEDED
                self.navigation_state = "COMPLETED"
                self.get_logger().info('🏁 목적지 도착! 완료!')
                self.current_goal = None
                self.goal_handle = None
            elif result.status == 5:  # CANCELED (정상적인 취소)
                self.get_logger().info('🛑 네비게이션 취소됨 (장애물로 인한)')
            elif result.status == 6:  # CANCELED (중복 요청)
                self.get_logger().warn('⚠️ 네비게이션 중복 취소')
            else:
                self.navigation_state = "IDLE"
                self.get_logger().warn(f'네비게이션 실패: status={result.status}')
        except Exception as e:
            self.navigation_state = "IDLE"
            self.get_logger().error(f'결과 처리 오류: {e}')
    
    def stop_navigation(self):
        """네비게이션 정지"""
        # 액션 취소
        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()
                self.get_logger().info('🛑 액션 취소')
            except Exception as e:
                self.get_logger().warn(f'액션 취소 실패: {e}')
        
        # 정지 명령
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('🛑 정지!')
    
    def restart_navigation_once(self):
        """5초 후 재출발 (한 번만)"""
        # 타이머 상태 리셋
        self.restart_timer_active = False
        if self.restart_timer:
            self.restart_timer.cancel()
            self.restart_timer = None
        
        if (self.current_goal and 
            self.navigation_state == "STOPPED"):
            
            # 마지막 안전 확인
            if not self.is_dangerous_obstacle():
                self.get_logger().info('✅ 장애물이 사라졌으므로 재출발합니다.')
                self.start_navigation()
            else:
                self.get_logger().warn('⚠️ 재출발 시도했지만 여전히 위험한 장애물 존재')
        else:
            state_msg = f'상태: {self.navigation_state}'
            self.get_logger().info(f'재출발 조건 불만족 - {state_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()