#!/usr/bin/env python3
"""
Vision Service 인터페이스 테스트 클라이언트
rms_vs_interface.md에 정의된 모든 서비스 및 토픽 인터페이스를 테스트
"""

import rclpy
from rclpy.node import Node
import threading
import time

# ROS2 메시지 및 서비스 타입들
from roomie_msgs.srv import (
    SetVSMode, 
    ButtonStatus,
    ElevatorStatus, 
    DoorStatus,
    Location
)
from roomie_msgs.msg import Obstacle, GlassDoorStatus



class VSInterfaceTestClient(Node):
    def __init__(self):
        super().__init__('vs_interface_test_client')
        
        # 🔧 Service Clients (rms_vs_interface.md 기준)
        self.service_clients = {
            'set_vs_mode': self.create_client(SetVSMode, '/vs/command/set_vs_mode'),
            'button_status': self.create_client(ButtonStatus, '/vs/command/button_status'),
            'elevator_status': self.create_client(ElevatorStatus, '/vs/command/elevator_status'),
            'door_status': self.create_client(DoorStatus, '/vs/command/door_status'),
            'location': self.create_client(Location, '/vs/command/location')
        }
        
        # 🔧 Topic Subscribers (VS → RC)
        self.obstacle_sub = self.create_subscription(
            Obstacle, '/vs/obstacle', self.on_obstacle_detected, 10)
        self.glass_door_sub = self.create_subscription(
            GlassDoorStatus, '/vs/glass_door_status', self.on_glass_door_status, 10)
        
        # 📊 모니터링 상태
        self.obstacle_monitoring = False
        self.glass_door_monitoring = False
        self.last_obstacle_time = None
        self.last_glass_door_time = None
        
        self.get_logger().info("🧪 VS 인터페이스 테스트 클라이언트 시작")
        self.show_menu()
    

    
    def on_obstacle_detected(self, msg):
        """장애물 감지 콜백"""
        self.last_obstacle_time = time.time()
        if self.obstacle_monitoring:
            obstacle_type = "동적" if msg.dynamic else "정적"
            self.get_logger().info(f"🚧 장애물 감지: {obstacle_type} 장애물 (x={msg.x:.3f}, y={msg.y:.3f})")
    
    def on_glass_door_status(self, msg):
        """유리문 상태 콜백"""
        self.last_glass_door_time = time.time()
        if self.glass_door_monitoring:
            door_status = "열림" if msg.door_opened else "닫힘"
            self.get_logger().info(f"🚪 유리문 상태: {door_status}")
    
    def check_service_availability(self):
        """모든 서비스 가용성 확인"""
        self.get_logger().info("🔍 VS 서비스 가용성 확인 중...")
        print("\n" + "="*70)
        print("📋 VS 서비스 인터페이스 구현 상태")
        print("="*70)
        
        for service_name, client in self.service_clients.items():
            try:
                if client.wait_for_service(timeout_sec=2.0):
                    print(f"✅ {service_name:20} | 구현됨")
                else:
                    print(f"❌ {service_name:20} | 미구현 또는 VS 노드 미실행")
            except Exception as e:
                print(f"❌ {service_name:20} | 에러: {e}")
        
        print("="*70)
        print("명령어를 입력하세요: ", end="")
    
    def test_set_vs_mode(self, mode_id=3):
        """VS 모드 설정 테스트"""
        client = self.service_clients['set_vs_mode']
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ SetVSMode 서비스 없음")
            return
            
        request = SetVSMode.Request()
        request.robot_id = 0
        request.mode_id = mode_id
        
        self.get_logger().info(f"📞 VS 모드 설정 호출: mode_id={mode_id}")
        future = client.call_async(request)
        
        def handle_response():
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                response = future.result()
                self.get_logger().info(f"✅ VS 모드 응답: robot_id={response.robot_id}, success={response.success}")
            else:
                self.get_logger().error("❌ VS 모드 설정 실패")
        
        threading.Thread(target=handle_response, daemon=True).start()
    
    def test_button_status(self, button_id=0):
        """버튼 상태 테스트 - 단일 버튼"""
        client = self.service_clients['button_status']
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ ButtonStatus 서비스 없음")
            return
            
        request = ButtonStatus.Request()
        request.robot_id = 0
        request.button_id = button_id  # 단일 버튼 ID
        
        button_names = {
            0: "현재 유일하게 감지되는 버튼", 1: "1층", 2: "2층", 3: "3층", 4: "4층", 5: "5층", 6: "6층",
            7: "7층", 8: "8층", 9: "9층", 10: "10층", 11: "11층", 12: "12층", 13: "B1층", 14: "B2층",
            100: "하행버튼", 101: "상행버튼", 102: "열기버튼", 103: "닫기버튼"
        }
        button_name = button_names.get(button_id, f"버튼{button_id}")
        
        self.get_logger().info(f"📞 버튼 상태 호출: button_id={button_id} ({button_name})")
        future = client.call_async(request)
        
        def handle_response():
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                response = future.result()
                pressed_str = "눌림" if response.is_pressed else "안눌림"
                if response.success:
                    self.get_logger().info(f"✅ 버튼 상태 응답: {button_name}")
                    self.get_logger().info(f"   위치: x={response.x:.3f}, y={response.y:.3f}, size={response.size:.3f}")
                    self.get_logger().info(f"   상태: {pressed_str}")
                else:
                    self.get_logger().info(f"❌ 버튼 상태 실패: {button_name} (버튼 미감지 또는 2개 이상 감지)")
            else:
                self.get_logger().error("❌ 버튼 상태 호출 실패")
        
        threading.Thread(target=handle_response, daemon=True).start()
    
    def test_button_status_sequence(self):
        """주요 버튼들 순차 테스트"""
        self.get_logger().info("🎯 주요 버튼들 순차 테스트 시작!")
        
        def run_button_tests():
            # 주요 버튼들 테스트
            test_buttons = [
                (0, "현재 유일하게 감지되는 버튼"),
                (100, "하행버튼"),
                (101, "상행버튼"),
                (1, "1층 버튼"),
                (2, "2층 버튼"),
                (3, "3층 버튼"),
                (102, "열기버튼"),
                (103, "닫기버튼")
            ]
            
            for i, (button_id, button_name) in enumerate(test_buttons):
                self.get_logger().info(f"🧪 [{i+1}/{len(test_buttons)}] {button_name} 테스트")
                self.test_button_status(button_id)
                time.sleep(1.5)  # 1.5초 간격
            
            self.get_logger().info("🎉 버튼 순차 테스트 완료!")
        
        threading.Thread(target=run_button_tests, daemon=True).start()
    

    
    def test_elevator_status(self):
        """엘리베이터 상태 테스트"""
        client = self.service_clients['elevator_status']
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ ElevatorStatus 서비스 없음")
            return
            
        request = ElevatorStatus.Request()
        request.robot_id = 0
        
        self.get_logger().info("📞 엘리베이터 상태 호출")
        future = client.call_async(request)
        
        def handle_response():
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                response = future.result()
                direction_str = "상행" if response.direction == 0 else "하행"
                self.get_logger().info(f"✅ 엘리베이터 상태 응답: {direction_str}, {response.position}층")
            else:
                self.get_logger().error("❌ 엘리베이터 상태 호출 실패")
        
        threading.Thread(target=handle_response, daemon=True).start()
    
    def test_door_status(self):
        """문 상태 테스트"""
        client = self.service_clients['door_status']
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ DoorStatus 서비스 없음")
            return
            
        request = DoorStatus.Request()
        request.robot_id = 0
        
        self.get_logger().info("📞 문 상태 호출")
        future = client.call_async(request)
        
        def handle_response():
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                response = future.result()
                door_str = "열림" if response.door_opened else "닫힘"
                self.get_logger().info(f"✅ 문 상태 응답: {door_str}")
            else:
                self.get_logger().error("❌ 문 상태 호출 실패")
        
        threading.Thread(target=handle_response, daemon=True).start()
    

    
    def test_location(self):
        """위치 감지 테스트"""
        client = self.service_clients['location']
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Location 서비스 없음")
            return
            
        request = Location.Request()
        request.robot_id = 0
        
        self.get_logger().info("📞 위치 감지 호출")
        future = client.call_async(request)
        
        def handle_response():
            rclpy.spin_until_future_complete(self, future)
            if future.result():
                response = future.result()
                location_names = {
                    0: "LOB_WAITING", 1: "LOB_CALL", 2: "RES_PICKUP", 3: "RES_CALL",
                    4: "SUP_PICKUP", 5: "ELE_1", 6: "ELE_2", 101: "ROOM_101",
                    102: "ROOM_102", 201: "ROOM_201", 202: "ROOM_202"
                }
                location_name = location_names.get(response.location_id, f"UNKNOWN({response.location_id})")
                self.get_logger().info(f"✅ 위치 감지 응답: {location_name}")
            else:
                self.get_logger().error("❌ 위치 감지 호출 실패")
        
        threading.Thread(target=handle_response, daemon=True).start()
    
    def test_all_services(self):
        """모든 서비스 순차 테스트"""
        self.get_logger().info("🎯 모든 VS 서비스 순차 테스트 시작!")
        
        def run_all_tests():
            tests = [
                ("VS 모드 - 대기모드 (후방)", lambda: self.test_set_vs_mode(0)),
                ("VS 모드 - 등록모드 (후방)", lambda: self.test_set_vs_mode(1)),
                ("VS 모드 - 추적모드 (후방)", lambda: self.test_set_vs_mode(2)),
                ("VS 모드 - 엘리베이터 외부 (전방)", lambda: self.test_set_vs_mode(3)),
                ("VS 모드 - 엘리베이터 내부 (전방)", lambda: self.test_set_vs_mode(4)),
                ("VS 모드 - 일반 주행 (전방)", lambda: self.test_set_vs_mode(5)),
                ("VS 모드 - 대기모드 (전방)", lambda: self.test_set_vs_mode(6)),
                ("버튼 상태 - 유일 버튼", lambda: self.test_button_status(0)),
                ("버튼 상태 - 하행버튼", lambda: self.test_button_status(100)),
                ("버튼 상태 - 상행버튼", lambda: self.test_button_status(101)),
                ("엘리베이터 상태", self.test_elevator_status),
                ("문 상태", self.test_door_status),
                ("위치 감지", self.test_location),
            ]
            
            for i, (test_name, test_func) in enumerate(tests):
                self.get_logger().info(f"🧪 [{i+1}/{len(tests)}] {test_name} 테스트")
                test_func()
                time.sleep(2)  # 2초 간격
            
            self.get_logger().info("🎉 모든 테스트 완료!")
        
        threading.Thread(target=run_all_tests, daemon=True).start()
    
    def show_menu(self):
        """사용 가능한 명령어 표시"""
        print("\n" + "="*70)
        print("🧪 VS 인터페이스 테스트 클라이언트 (업데이트됨)")
        print("="*70)
        print("📋 rms_vs_interface.md 기준 전체 인터페이스 (5개 서비스):")
        print()
        print("🔍 상태 확인:")
        print("  check : 모든 서비스 가용성 확인")
        print("  info  : 현재 노드 및 토픽 상태 확인")
        print("  status: 토픽 상태 확인")
        print()
        print("🔧 서비스 인터페이스 테스트 (RC → VS):")
        print("  1  : SetVSMode - 대기모드 (후방 전용, mode_id=0)")
        print("  1r : SetVSMode - 등록모드 (후방 전용, mode_id=1)")
        print("  1t : SetVSMode - 추적모드 (후방 전용, mode_id=2)")
        print("  1e : SetVSMode - 엘리베이터 외부 (전방 전용, mode_id=3)")
        print("  1i : SetVSMode - 엘리베이터 내부 (전방 전용, mode_id=4)")
        print("  1n : SetVSMode - 일반 주행모드 (전방 전용, mode_id=5)")
        print("  1f : SetVSMode - 대기모드 (전방 전용, mode_id=6)")
        print("  2  : ButtonStatus - 유일 버튼 감지 (button_id=0)")
        print("  2d : ButtonStatus - 하행버튼 감지 (button_id=100)")
        print("  2u : ButtonStatus - 상행버튼 감지 (button_id=101)")
        print("  2f : ButtonStatus - 1층버튼 감지 (button_id=1)")
        print("  2s : ButtonStatus - 주요 버튼들 순차 테스트")
        print("  3  : ElevatorStatus - 엘리베이터 상태 감지")
        print("  4  : DoorStatus - 문 상태 감지")
        print("  5  : Location - 위치 감지")
        print()
        print("📡 토픽 인터페이스 테스트 (VS → RC):")
        print("  topics : 모든 토픽 테스트 (60초)")
        print("  obs_on : 장애물 모니터링 시작")
        print("  obs_off: 장애물 모니터링 중지")
        print("  door_on : 유리문 모니터링 시작")
        print("  door_off: 유리문 모니터링 중지")
        print()
        print("🎯 통합 테스트:")
        print("  all    : 모든 서비스 순차 테스트")
        print("  topics : 모든 토픽 테스트")
        print("  full   : 서비스 + 토픽 전체 테스트")
        print()
        print("🛠️ 기타:")
        print("  menu   : 이 메뉴 다시 표시")
        print("  quit   : 종료")
        print("="*70)
        print("💡 실시간 모니터링: 장애물/유리문 토픽 활성화됨")
        print("💡 VS 노드 키보드 제어: 일반 주행모드에서 장애물 감지 가능")
        print("="*70)
        print("명령어를 입력하세요: ", end="")
    
    def check_node_info(self):
        """현재 노드 및 토픽 상태 확인"""
        self.get_logger().info("📊 VS 노드 상태 확인 중...")
        print("\n" + "="*70)
        print("📊 현재 ROS2 환경 상태")
        print("="*70)
        
        # 노드 정보는 직접 출력하기 어려우니 안내만
        print("🔍 수동 확인 명령어:")
        print("  ros2 node list                    # 실행 중인 노드 확인")
        print("  ros2 service list | grep vs       # VS 서비스 확인")  
        print("  ros2 topic list | grep vs         # VS 토픽 확인")
        print("  ros2 topic echo /vs/obstacle      # 장애물 토픽 실시간 확인")
        print("  ros2 topic echo /vs/glass_door_status  # 유리문 토픽 실시간 확인")

        print("="*70)
        print("명령어를 입력하세요: ", end="")
    
    def request_tracking_event(self):
        """단일 추적 이벤트 발행 요청 (삭제됨)"""
        self.get_logger().info("📡 단일 추적 이벤트 발행 요청 (기능이 삭제되었습니다)")
    
    def request_registered_event(self):
        """등록 완료 이벤트 발행 요청 (삭제됨)"""
        self.get_logger().info("📡 등록 완료 이벤트 발행 요청 (기능이 삭제되었습니다)")
    
    def request_tracking_simulation(self):
        """추적 시뮬레이션 시퀀스 요청 (삭제됨)"""
        self.get_logger().info("🎬 추적 시뮬레이션 시퀀스 요청 (기능이 삭제되었습니다)")
    
    def test_all_topics(self):
        """모든 토픽 테스트"""
        self.get_logger().info("📡 모든 토픽 인터페이스 테스트 시작!")
        
        def run_topic_tests():
            # 1. 장애물 모니터링 시작
            self.get_logger().info("🧪 [1/2] 장애물 토픽 모니터링 시작 (30초)")
            self.obstacle_monitoring = True
            time.sleep(30)
            self.obstacle_monitoring = False
            
            # 2. 유리문 모니터링 시작
            self.get_logger().info("🧪 [2/2] 유리문 토픽 모니터링 시작 (30초)")
            self.glass_door_monitoring = True
            time.sleep(30)
            self.glass_door_monitoring = False
            
            self.get_logger().info("🎉 토픽 테스트 완료!")
            self.get_logger().info("💡 VS 노드에서 해당 토픽을 발행하면 실시간으로 확인됩니다.")
        
        threading.Thread(target=run_topic_tests, daemon=True).start()
    
    def start_obstacle_monitoring(self):
        """장애물 모니터링 시작"""
        self.obstacle_monitoring = True
        self.get_logger().info("🚧 장애물 모니터링 시작 (종료하려면 'stop_obs' 입력)")
        self.get_logger().info("💡 VS 노드가 일반 주행모드(mode_id=5)에서 장애물을 감지하면 실시간으로 표시됩니다.")
    
    def stop_obstacle_monitoring(self):
        """장애물 모니터링 중지"""
        self.obstacle_monitoring = False
        self.get_logger().info("🚧 장애물 모니터링 중지")
    
    def start_glass_door_monitoring(self):
        """유리문 모니터링 시작"""
        self.glass_door_monitoring = True
        self.get_logger().info("🚪 유리문 모니터링 시작 (종료하려면 'stop_door' 입력)")
        self.get_logger().info("💡 VS 노드가 유리문을 감지하면 실시간으로 표시됩니다.")
    
    def stop_glass_door_monitoring(self):
        """유리문 모니터링 중지"""
        self.glass_door_monitoring = False
        self.get_logger().info("🚪 유리문 모니터링 중지")
    
    def check_topic_status(self):
        """토픽 상태 확인"""
        current_time = time.time()
        
        print("\n" + "="*70)
        print("📡 토픽 상태 확인")
        print("="*70)
        
        # 장애물 토픽 상태
        if self.last_obstacle_time:
            time_diff = current_time - self.last_obstacle_time
            status = "활성" if time_diff < 10 else "비활성"
            print(f"🚧 장애물 토픽: {status} (마지막 수신: {time_diff:.1f}초 전)")
        else:
            print("🚧 장애물 토픽: 미수신")
        
        # 유리문 토픽 상태
        if self.last_glass_door_time:
            time_diff = current_time - self.last_glass_door_time
            status = "활성" if time_diff < 10 else "비활성"
            print(f"🚪 유리문 토픽: {status} (마지막 수신: {time_diff:.1f}초 전)")
        else:
            print("🚪 유리문 토픽: 미수신")
        
        # 모니터링 상태
        print(f"📊 장애물 모니터링: {'켜짐' if self.obstacle_monitoring else '꺼짐'}")
        print(f"📊 유리문 모니터링: {'켜짐' if self.glass_door_monitoring else '꺼짐'}")
        
        print("="*70)
        print("명령어를 입력하세요: ", end="")
    
    def test_full_interface(self):
        """서비스 + 토픽 전체 인터페이스 테스트"""
        self.get_logger().info("🎯 VS 전체 인터페이스 테스트 시작!")
        
        def run_full_tests():
            # 1. 서비스 테스트
            self.get_logger().info("🧪 [1단계] 모든 서비스 테스트")
            self.test_all_services()
            
            import time
            time.sleep(3)
            
            # 2. 토픽 테스트
            self.get_logger().info("🧪 [2단계] 토픽 테스트")  
            self.test_all_topics()
            
            time.sleep(2)
            
            self.get_logger().info("🎉 전체 인터페이스 테스트 완료!")
            self.get_logger().info("📋 인터페이스 요약:")
            self.get_logger().info("   ✅ 서비스 5개 타입: SetVSMode(7가지모드), ButtonStatus(단일값), ElevatorStatus, DoorStatus, Location")
            self.get_logger().info("   ✅ 토픽 2개: Obstacle, GlassDoorStatus")
            self.get_logger().info("   ✅ 총 테스트 케이스: 13개 서비스")
            self.get_logger().info("   📋 모드: 후방 3개(대기,등록,추적) + 전방 4개(엘리베이터외부,엘리베이터내부,일반주행,대기)")
        
        threading.Thread(target=run_full_tests, daemon=True).start()
    
    def run_interactive(self):
        """대화형 모드 실행"""
        while True:
            try:
                cmd = input().strip()
                
                if cmd == "quit":
                    self.get_logger().info("👋 VS 테스트 클라이언트 종료")
                    break
                elif cmd == "menu":
                    self.show_menu()
                elif cmd == "check":
                    self.check_service_availability()
                elif cmd == "info":
                    self.check_node_info()
                elif cmd == "all":
                    self.test_all_services()
                elif cmd == "status":
                    self.check_topic_status()
                elif cmd == "topics":
                    self.test_all_topics()
                elif cmd == "obs_on":
                    self.start_obstacle_monitoring()
                elif cmd == "obs_off":
                    self.stop_obstacle_monitoring()
                elif cmd == "door_on":
                    self.start_glass_door_monitoring()
                elif cmd == "door_off":
                    self.stop_glass_door_monitoring()
                elif cmd == "full":
                    self.test_full_interface()
                elif cmd == "1":
                    self.test_set_vs_mode(0)  # 대기모드
                elif cmd == "1r":
                    self.test_set_vs_mode(1)  # 등록모드
                elif cmd == "1t":
                    self.test_set_vs_mode(2)  # 추적모드
                elif cmd == "1e":
                    self.test_set_vs_mode(3)  # 엘리베이터 외부 모드
                elif cmd == "1i":
                    self.test_set_vs_mode(4)  # 엘리베이터 내부 모드
                elif cmd == "1n":
                    self.test_set_vs_mode(5)  # 일반모드
                elif cmd == "1f":
                    self.test_set_vs_mode(6)  # 전방 대기모드
                elif cmd == "2":
                    self.test_button_status(0)  # 유일 버튼
                elif cmd == "2d":
                    self.test_button_status(100)  # 하행버튼
                elif cmd == "2u":
                    self.test_button_status(101)  # 상행버튼
                elif cmd == "2f":
                    self.test_button_status(1)  # 1층버튼
                elif cmd == "2s":
                    self.test_button_status_sequence()  # 순차 테스트
                elif cmd == "3":
                    self.test_elevator_status()
                elif cmd == "4":
                    self.test_door_status()
                elif cmd == "5":
                    self.test_location()
                else:
                    print(f"❌ 알 수 없는 명령어: {cmd}")
                    print("'menu'를 입력하면 사용 가능한 명령어를 볼 수 있습니다.")
                
                print("명령어를 입력하세요: ", end="")
                
            except KeyboardInterrupt:
                self.get_logger().info("👋 Ctrl+C로 종료")
                break
            except Exception as e:
                self.get_logger().error(f"❌ 오류: {e}")


def main():
    rclpy.init()
    
    try:
        client = VSInterfaceTestClient()
        
        # ROS2 스핀을 백그라운드에서 실행
        spin_thread = threading.Thread(target=lambda: rclpy.spin(client), daemon=True)
        spin_thread.start()
        
        # 대화형 모드 실행
        client.run_interactive()
        
    except Exception as e:
        print(f"❌ 오류: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 