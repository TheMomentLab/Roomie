#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from roomie_msgs.action import SetPose
import tkinter as tk
from tkinter import ttk
import threading
import os
import struct
import subprocess

# FastRTPS 버퍼 크기 설정
os.environ['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '1'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.join(os.path.dirname(__file__), '..', 'fastrtps_profile.xml')

class ArmPoseGUI(Node):
    def __init__(self):
        super().__init__('arm_pose_gui')
        
        # Action client 생성
        self._action_client = ActionClient(self, SetPose, '/arm/action/set_pose')
        
        # GUI 생성
        self.create_gui()
        
        # Action client 연결 대기를 백그라운드에서 실행
        self.get_logger().info('Waiting for action server...')
        threading.Thread(target=self._wait_for_server, daemon=True).start()
        
        # ROS 이벤트 처리를 백그라운드에서 실행
        threading.Thread(target=self._spin_ros, daemon=True).start()
    
    def _wait_for_server(self):
        """백그라운드에서 액션 서버 연결 대기"""
        self._action_client.wait_for_server()
        self.get_logger().info('Action server connected!')
        # GUI 업데이트
        self.root.after(0, lambda: self.status_label.config(text="✅ 액션 서버 연결됨 - 준비 완료"))
    
    def _spin_ros(self):
        """백그라운드에서 ROS 이벤트 처리"""
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f'ROS spin error: {str(e)}')
    
    def create_gui(self):
        # 메인 윈도우 생성
        self.root = tk.Tk()
        self.root.title("Roomie Arm Pose Controller")
        self.root.geometry("400x300")
        
        # 스타일 설정
        style = ttk.Style()
        style.theme_use('clam')
        
        # 제목
        title_label = ttk.Label(self.root, text="팔 회전 명령", font=('Arial', 16, 'bold'))
        title_label.pack(pady=20)
        
        # 버튼 프레임
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20)
        
        # robot_id 설정
        self.robot_id = 0  # 기본값
        
        # pose_id에 따른 버튼들
        poses = [
            (0, "초기자세 (Init)", "green"),
            (1, "왼쪽 회전 (Left)", "blue"),
            (2, "오른쪽 회전 (Right)", "orange"),
            (3, "전면 회전 (Forward)", "red")
        ]
        
        for pose_id, text, color in poses:
            btn = ttk.Button(
                button_frame, 
                text=f"{pose_id}: {text}",
                command=lambda pid=pose_id: self.send_pose_action(pid),
                style=f"{color}.TButton"
            )
            btn.pack(pady=5, padx=10, fill='x')
        
        # 상태 표시
        self.status_label = ttk.Label(self.root, text="대기 중...", font=('Arial', 10))
        self.status_label.pack(pady=20)
        
        # 종료 버튼
        quit_btn = ttk.Button(self.root, text="종료", command=self.quit_gui)
        quit_btn.pack(pady=10)
    
    def send_pose_action(self, pose_id):
        """액션을 발행하는 함수"""
        # GUI 스레드에서 ROS 스레드로 실행
        threading.Thread(target=self._send_action_thread, args=(pose_id,), daemon=True).start()
    
    def _send_action_thread(self, pose_id):
        """액션을 발행하는 스레드 함수"""
        try:
            # CLI 명령어 실행
            cmd = f'ros2 action send_goal /arm/action/set_pose roomie_msgs/action/SetPose "{{robot_id: {self.robot_id}, pose_id: {pose_id}}}"'
            
            self.get_logger().info(f'🔄 pose_id {pose_id} 액션 발행')
            self.get_logger().info(f'📤 CLI 명령어: {cmd}')
            
            # 상태 업데이트
            self.root.after(0, lambda: self.status_label.config(text=f"🔄 pose_id {pose_id} 액션 발행 중..."))
            
            # CLI 명령어 실행
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info('✅ CLI 명령어 실행 성공')
                self.root.after(0, lambda: self.status_label.config(text="🎉 액션 완료!"))
            else:
                self.get_logger().error(f'❌ CLI 명령어 실행 실패: {result.stderr}')
                self.root.after(0, lambda: self.status_label.config(text="❌ 액션 실패"))
            
        except Exception as e:
            error_msg = str(e)
            self.get_logger().error(f'Action send failed: {error_msg}')
            self.root.after(0, lambda: self.status_label.config(text=f"오류: {error_msg}"))
    
    def goal_response_callback(self, future):
        """Goal 응답 콜백"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('❌ Goal 거부됨')
                self.root.after(0, lambda: self.status_label.config(text="❌ Goal 거부됨"))
                return
            
            self.get_logger().info('✅ Goal 수락됨')
            self.root.after(0, lambda: self.status_label.config(text="✅ Goal 수락됨, 실행 중..."))
            
            # 결과 대기
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'Goal response callback error: {str(e)}')
            self.root.after(0, lambda: self.status_label.config(text=f"Goal 응답 오류: {str(e)}"))
    
    def get_result_callback(self, future):
        """결과 콜백"""
        try:
            result = future.result().result
            self.get_logger().info(f'📥 수신 결과: robot_id={result.robot_id}, success={result.success}')
            self.get_logger().info(f'📥 RAW 결과: {result}')
            if result.success:
                self.get_logger().info('🎉 액션 완료!')
                self.root.after(0, lambda: self.status_label.config(text="🎉 액션 완료!"))
            else:
                self.get_logger().info('❌ 액션 실패')
                self.root.after(0, lambda: self.status_label.config(text="❌ 액션 실패"))
        except Exception as e:
            self.get_logger().error(f'Get result failed: {str(e)}')
            self.root.after(0, lambda: self.status_label.config(text=f"결과 오류: {str(e)}"))
    
    def quit_gui(self):
        """GUI 종료"""
        self.root.quit()
        self.root.destroy()
        rclpy.shutdown()
    
    def run(self):
        """GUI 실행"""
        try:
            # GUI 메인루프 실행
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt, shutting down')
            self.quit_gui()

def main(args=None):
    rclpy.init(args=args)
    
    arm_gui = ArmPoseGUI()
    
    try:
        arm_gui.run()
    except Exception as e:
        arm_gui.get_logger().error(f'Error: {str(e)}')
    finally:
        arm_gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
