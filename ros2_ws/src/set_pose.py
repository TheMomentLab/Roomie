#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from roomie_msgs.action import SetPose
import tkinter as tk
from tkinter import ttk
import threading
class ArmPoseGUI(Node):
    def __init__(self):
        super().__init__('arm_pose_gui')
        self._action_client = ActionClient(self, SetPose, '/arm/action/set_pose')
        self.robot_id = 0
        self.create_gui()
        self.get_logger().info('Waiting for action server...')
        threading.Thread(target=self._wait_for_server, daemon=True).start()
        threading.Thread(target=self._spin_ros, daemon=True).start()
    def _wait_for_server(self):
        self._action_client.wait_for_server()
        self.get_logger().info(':흰색_확인_표시: Action server connected!')
        self.root.after(0, lambda: self.status_label.config(text=":흰색_확인_표시: 액션 서버 연결됨 - 준비 완료"))
    def _spin_ros(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f'ROS spin error: {str(e)}')
    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Roomie Arm Pose Controller")
        self.root.geometry("400x300")
        style = ttk.Style()
        style.theme_use('clam')
        title_label = ttk.Label(self.root, text="팔 회전 명령", font=('Arial', 16, 'bold'))
        title_label.pack(pady=20)
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20)
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
        self.status_label = ttk.Label(self.root, text="대기 중...", font=('Arial', 10))
        self.status_label.pack(pady=20)
        quit_btn = ttk.Button(self.root, text="종료", command=self.quit_gui)
        quit_btn.pack(pady=10)
    def send_pose_action(self, pose_id):
        threading.Thread(target=self._send_action_thread, args=(pose_id,), daemon=True).start()
    def _send_action_thread(self, pose_id):
        try:
            goal_msg = SetPose.Goal()
            goal_msg.robot_id = self.robot_id
            goal_msg.pose_id = pose_id
            self.get_logger().info(f':보낼_편지함_트레이: pose_id {pose_id} 액션 발행')
            self.root.after(0, lambda: self.status_label.config(text=f":시계_반대_방향_화살표: pose_id {pose_id} 액션 발행 중..."))
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f'Goal send error: {str(e)}')
            self.root.after(0, lambda: self.status_label.config(text=f"전송 오류: {str(e)}"))
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info(':x: Goal 거부됨')
                self.root.after(0, lambda: self.status_label.config(text=":x: Goal 거부됨"))
                return
            self.get_logger().info(':흰색_확인_표시: Goal 수락됨')
            self.root.after(0, lambda: self.status_label.config(text=":흰색_확인_표시: Goal 수락됨, 실행 중..."))
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'Goal response callback error: {str(e)}')
            self.root.after(0, lambda: self.status_label.config(text=f"응답 오류: {str(e)}"))
    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f':받은_편지함_트레이: 수신 결과: robot_id={result.robot_id}, success={result.success}')
            if result.success:
                self.root.after(0, lambda: self.status_label.config(text=":짠: 액션 완료!"))
            else:
                self.root.after(0, lambda: self.status_label.config(text=":x: 액션 실패"))
        except Exception as e:
            self.get_logger().error(f'Get result failed: {str(e)}')
            self.root.after(0, lambda: self.status_label.config(text=f"결과 오류: {str(e)}"))
    def quit_gui(self):
        self.root.quit()
        self.root.destroy()
        rclpy.shutdown()
    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt, shutting down')
            self.quit_gui()
def main(args=None):
    rclpy.init(args=args)
    gui = ArmPoseGUI()
    try:
        gui.run()
    except Exception as e:
        gui.get_logger().error(f'Error: {str(e)}')
    finally:
        gui.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()