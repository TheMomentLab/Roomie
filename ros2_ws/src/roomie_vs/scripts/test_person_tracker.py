#!/usr/bin/env python3
"""
PersonTracker 통합 테스트 스크립트
기존 VS 인터페이스와의 호환성 확인
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from roomie_msgs.srv import SetVSMode
from roomie_msgs.action import Enroll
from roomie_msgs.msg import Tracking
from std_srvs.srv import Trigger
import time

class PersonTrackerTester(Node):
    def __init__(self):
        super().__init__('person_tracker_tester')
        
        # 클라이언트들 생성
        self.set_mode_client = self.create_client(SetVSMode, '/vs/command/set_vs_mode')
        self.enroll_action_client = ActionClient(self, Enroll, '/vs/action/enroll')
        self.stop_tracking_client = self.create_client(Trigger, '/vs/command/stop_tracking')
        
        # 추적 상태 구독
        self.tracking_subscription = self.create_subscription(
            Tracking,
            '/vs/tracking',
            self.tracking_callback,
            10
        )
        
        self.latest_tracking = None
        self.get_logger().info("PersonTracker 테스터 초기화 완료")

    def tracking_callback(self, msg):
        """추적 상태 메시지 수신"""
        self.latest_tracking = msg
        event_name = {0: "NONE", 1: "LOST", 2: "REACQUIRED"}.get(msg.event, "UNKNOWN")
        self.get_logger().info(
            f"📍 추적 상태: id={msg.id}, tracking={msg.tracking}, "
            f"cx={msg.cx:.3f}, cy={msg.cy:.3f}, scale={msg.scale:.3f}, event={event_name}"
        )

    def wait_for_services(self):
        """모든 서비스가 준비될 때까지 대기"""
        self.get_logger().info("VS 서비스들이 준비될 때까지 대기 중...")
        
        if not self.set_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("SetVSMode 서비스를 찾을 수 없습니다")
            return False
            
        if not self.enroll_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Enroll 액션 서버를 찾을 수 없습니다")
            return False
            
        if not self.stop_tracking_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("StopTracking 서비스를 찾을 수 없습니다")
            return False
            
        self.get_logger().info("✅ 모든 VS 서비스 준비 완료!")
        return True

    def set_vs_mode(self, mode_id):
        """VS 모드 설정"""
        request = SetVSMode.Request()
        request.robot_id = 1
        request.mode_id = mode_id
        
        self.get_logger().info(f"🔄 모드 {mode_id} 설정 요청...")
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ 모드 {mode_id} 설정 성공")
                return True
            else:
                self.get_logger().error(f"❌ 모드 {mode_id} 설정 실패")
                return False
        else:
            self.get_logger().error(f"❌ 모드 {mode_id} 설정 타임아웃")
            return False

    def run_enrollment_test(self, duration_sec=3.0):
        """등록 테스트"""
        self.get_logger().info(f"👤 등록 테스트 시작 (duration: {duration_sec}초)")
        
        goal = Enroll.Goal()
        goal.duration_sec = duration_sec
        
        future = self.enroll_action_client.send_goal_async(
            goal,
            feedback_callback=self.enroll_feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ 등록 액션 거부됨")
            return False
        
        self.get_logger().info("✅ 등록 액션 수락됨, 진행 중...")
        
        # 등록 완료까지 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 2.0)
        
        result = result_future.result()
        if result and result.result.success:
            self.get_logger().info("✅ 등록 완료!")
            return True
        else:
            self.get_logger().error("❌ 등록 실패")
            return False

    def enroll_feedback_callback(self, feedback):
        """등록 진행률 피드백"""
        progress = feedback.feedback.progress
        self.get_logger().info(f"📊 등록 진행률: {progress:.1%}")

    def stop_tracking_test(self):
        """추적 중지 테스트"""
        self.get_logger().info("🛑 추적 중지 테스트")
        
        request = Trigger.Request()
        future = self.stop_tracking_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ 추적 중지 성공: {response.message}")
                return True
            else:
                self.get_logger().error(f"❌ 추적 중지 실패: {response.message}")
                return False
        else:
            self.get_logger().error("❌ 추적 중지 타임아웃")
            return False

    def run_full_test(self):
        """전체 워크플로우 테스트"""
        self.get_logger().info("🚀 PersonTracker 전체 테스트 시작!")
        
        if not self.wait_for_services():
            return False
        
        # 1. 대기모드로 설정 (후방)
        if not self.set_vs_mode(0):
            return False
        time.sleep(1)
        
        # 2. 등록모드로 전환
        if not self.set_vs_mode(1):
            return False
        time.sleep(1)
        
        # 3. 등록 실행 (실제 카메라가 없어도 테스트)
        if not self.run_enrollment_test(3.0):
            self.get_logger().warning("⚠️ 등록 실패 (카메라 없을 수 있음) - 계속 진행")
        time.sleep(1)
        
        # 4. 추적모드로 전환
        if not self.set_vs_mode(2):
            return False
        time.sleep(2)
        
        # 5. 추적 상태 모니터링 (5초간)
        self.get_logger().info("📡 추적 상태 모니터링 (5초간)...")
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 6. 추적 중지
        if not self.stop_tracking_test():
            return False
        
        # 7. 대기모드로 복귀
        if not self.set_vs_mode(0):
            return False
        
        self.get_logger().info("🎉 PersonTracker 전체 테스트 완료!")
        return True

def main():
    rclpy.init()
    
    tester = PersonTrackerTester()
    
    try:
        success = tester.run_full_test()
        if success:
            tester.get_logger().info("✅ 모든 테스트 통과!")
        else:
            tester.get_logger().error("❌ 일부 테스트 실패")
    except KeyboardInterrupt:
        tester.get_logger().info("사용자에 의해 중단됨")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 