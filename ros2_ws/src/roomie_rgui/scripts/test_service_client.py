#!/usr/bin/env python3
"""
Robot GUI 테스트 클라이언트
외부 시스템(RC)을 시뮬레이션하여 Robot GUI와 ROS2 통신으로 상호작용
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import time
from roomie_msgs.msg import RobotGuiEvent
from roomie_msgs.action import StartCountdown, ReturnCountdown


class TestServiceClient(Node):
    def __init__(self):
        super().__init__('test_service_client')
        
        # Publisher - Robot GUI로 이벤트 발행
        self.event_pub = self.create_publisher(RobotGuiEvent, '/robot_gui/event', 10)
        
        # Action Clients
        self.departure_cli = ActionClient(self, StartCountdown, '/robot_gui/action/start_countdown')
        self.return_cli = ActionClient(self, ReturnCountdown, '/robot_gui/action/return_countdown')
        
        self.get_logger().info("🧪 Robot GUI 테스트 클라이언트 시작")
        self.show_menu()
    
    def publish_event(self, event_id: int, robot_id: int = 98, task_id: int = 1, detail: str = ""):
        """Robot GUI로 이벤트 발행"""
        from builtin_interfaces.msg import Time
        from rclpy.clock import Clock
        
        msg = RobotGuiEvent()
        msg.robot_id = robot_id
        msg.task_id = task_id
        msg.rgui_event_id = event_id
        msg.detail = detail
        msg.timestamp = Clock().now().to_msg()
        
        self.event_pub.publish(msg)
        self.get_logger().info(f"📤 이벤트 발행: ID={event_id}, detail='{detail}'")
    
    def call_departure_countdown(self, task_type_id: int = 0):
        """출발 카운트다운 액션 호출"""
        if not self.departure_cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ 출발 카운트다운 액션 서버를 찾을 수 없습니다")
            return
        
        task_types = {0: "음식배송", 1: "비품배송", 2: "호출", 3: "길안내"}
        task_name = task_types.get(task_type_id, "알 수 없음")
        
        goal = StartCountdown.Goal()
        goal.robot_id = 98
        goal.task_id = 1
        goal.task_type_id = task_type_id
        
        self.get_logger().info(f"📞 {task_name} 카운트다운 액션 호출 중...")
        
        def feedback_callback(feedback):
            self.get_logger().info(f"⏰ 액션 피드백: 남은 시간 {feedback.feedback.remaining_time}초")
        
        def done_callback(future):
            result = future.result().result
            self.get_logger().info(f"✅ {task_name} 카운트다운 완료: success={result.success}, robot_id={result.robot_id}")
        
        send_goal_future = self.departure_cli.send_goal_async(goal, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(lambda future: future.result().get_result_async().add_done_callback(done_callback))
    
    def call_return_countdown(self):
        """복귀 카운트다운 액션 호출"""
        if not self.return_cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ 복귀 카운트다운 액션 서버를 찾을 수 없습니다")
            return
        
        goal = ReturnCountdown.Goal()
        goal.robot_id = 98
        
        self.get_logger().info("📞 복귀 카운트다운 액션 호출 중...")
        
        def feedback_callback(feedback):
            self.get_logger().info(f"⏰ 액션 피드백: 남은 시간 {feedback.feedback.remaining_time}초")
        
        def done_callback(future):
            result = future.result().result
            self.get_logger().info(f"✅ 복귀 카운트다운 완료: success={result.success}, robot_id={result.robot_id}")
        
        send_goal_future = self.return_cli.send_goal_async(goal, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(lambda future: future.result().get_result_async().add_done_callback(done_callback))
    
    def show_menu(self):
        """사용 가능한 명령어 표시"""
        print("\n" + "="*60)
        print("🧪 Robot GUI 테스트 클라이언트")
        print("="*60)
        print("📋 사용 가능한 명령어:")
        print()
        print("🔧 액션 호출:")
        print("  start0: 음식배송 출발 카운트다운 액션 호출")
        print("  start1: 비품배송 출발 카운트다운 액션 호출")
        print("  start2: 호출 출발 카운트다운 액션 호출")
        print("  start3: 길안내 출발 카운트다운 액션 호출")
        print("  return: 복귀 카운트다운 액션 호출")
        print()
        print("📡 이벤트 발행 (RC → Robot GUI):")
        print("🛗 엘리베이터:")
        print("  1  : 엘리베이터 버튼 조작 시작")
        print("  2  : 엘리베이터 버튼 조작 종료")
        print("  3  : 엘리베이터 탑승 시작")
        print("  4  : 엘리베이터 탑승 종료")
        print("  5  : 엘리베이터 하차 시작")
        print("  6  : 엘리베이터 하차 종료")
        print("🚶 이동 관련:")
        print("  7  : 호출 이동 시작")
        print("  8  : 호출 이동 종료")
        print("  9  : 호실 번호 인식 완료")
        print("  10 : 길안내 이동 시작")
        print("  11 : 길안내 이동 종료")
        print("  12 : 픽업장소 이동 시작")
        print("  13 : 픽업장소 이동 종료 (도착)")
        print("  14 : 배송장소 이동 시작") 
        print("  15 : 배송장소 도착 완료")
        print("📦 서랍/물품:")
        print("  16 : 서랍 열림")
        print("  17 : 서랍 닫힘")
        print("  18 : 서랍 잠금")
        print("  26 : 적재 감지")
        print("  27 : 적재 미감지")
        print("🔋 충전:")
        print("  19 : 충전 시작")
        print("  20 : 충전 종료")
        print("👤 사용자:")
        print("  21 : 투숙객 이탈")
        print("  22 : 투숙객 이탈 후 재등록")
        print("  23 : 투숙객 등록")
        print("  24 : 배송 수령 완료")
        print("  25 : 배송 수령 미완료")
        print()
        print("🎮 GUI 이벤트 (Robot GUI → RC):")
        print("  100 : [수령 완료] 클릭")
        print("  101 : 목적지 입력 완료")
        print("  102 : 사용자 점유 상태")
        print("  103 : [카드키로 입력] 선택")
        print("  104 : [서랍 열기] 클릭")
        print("  105 : [적재 완료] 클릭")
        print("  106 : 인식모드 전환 요청")
        print()
        print("🎯 시나리오 자동 실행:")
        print("  auto     : 전체 배송 시나리오 자동 실행")
        print("  elevator : 엘리베이터 시나리오 자동 실행")
        print("  menu     : 이 메뉴 다시 표시")
        print("  quit     : 종료")
        print("="*60)
        print("명령어를 입력하세요: ", end="")
    
    def run_auto_scenario(self):
        """전체 배송 시나리오 자동 실행"""
        self.get_logger().info("🎬 자동 시나리오 시작!")
        
        scenarios = [
            (12, "픽업장소 이동 시작", ""),
            (13, "픽업장소 이동 종료", ""),
            (16, "서랍 열림 (주문 확인)", ""),
            (16, "서랍 열림 (픽업 서랍)", ""),
            (14, "배송장소 이동 시작", ""),
            (15, "배송장소 도착 완료", ""),
            (16, "서랍 열림 (배송)", ""),
            (24, "배송 수령 완료", ""),
        ]
        
        def auto_runner():
            for i, (event_id, desc, detail) in enumerate(scenarios):
                time.sleep(3)  # 3초 간격
                self.get_logger().info(f"🎬 [{i+1}/{len(scenarios)}] {desc}")
                self.publish_event(event_id, detail=detail)
            
            self.get_logger().info("🎉 자동 시나리오 완료!")
        
        threading.Thread(target=auto_runner, daemon=True).start()
    
    def run_elevator_scenario(self):
        """엘리베이터 시나리오 자동 실행"""
        self.get_logger().info("🛗 엘리베이터 시나리오 시작!")
        
        scenarios = [
            (1, "엘리베이터 버튼 조작 시작", ""),
            (2, "엘리베이터 버튼 조작 종료", ""),
            (3, "엘리베이터 탑승 시작", ""),
            (4, "엘리베이터 탑승 종료", ""),
            (5, "엘리베이터 하차 시작", ""),
            (6, "엘리베이터 하차 종료", ""),
        ]
        
        def elevator_runner():
            for i, (event_id, desc, detail) in enumerate(scenarios):
                time.sleep(2)  # 2초 간격
                self.get_logger().info(f"🛗 [{i+1}/{len(scenarios)}] {desc}")
                self.publish_event(event_id, detail=detail)
            
            self.get_logger().info("🎉 엘리베이터 시나리오 완료!")
        
        threading.Thread(target=elevator_runner, daemon=True).start()
    
    def run_interactive(self):
        """대화형 모드 실행"""
        while True:
            try:
                cmd = input().strip()
                
                if cmd == "quit":
                    self.get_logger().info("👋 테스트 클라이언트 종료")
                    break
                elif cmd == "menu":
                    self.show_menu()
                elif cmd == "auto":
                    self.run_auto_scenario()
                elif cmd == "elevator":
                    self.run_elevator_scenario()
                elif cmd == "start0":
                    self.call_departure_countdown(task_type_id=0)  # 음식배송
                elif cmd == "start1":
                    self.call_departure_countdown(task_type_id=1)  # 비품배송
                elif cmd == "start2":
                    self.call_departure_countdown(task_type_id=2)  # 호출
                elif cmd == "start3":
                    self.call_departure_countdown(task_type_id=3)  # 길안내
                elif cmd == "return":
                    self.call_return_countdown()
                elif cmd in ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", 
                           "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", 
                           "24", "25", "26", "27", "100", "101", "102", "103", "104", "105", "106"]:
                    event_id = int(cmd)
                    # 13번(픽업장소 이동 종료) 이벤트는 주문 내역 detail 포함
                    if event_id == 13:
                        import json
                        import random
                        
                        # 랜덤 메뉴 목록
                        menu_list = [
                            "스파게티", "피자", "햄버거", "치킨", "샐러드", 
                            "파스타", "스테이크", "초밥", "라면", "김치찌개",
                            "된장찌개", "비빔밥", "불고기", "갈비찜", "삼겹살"
                        ]
                        
                        # 랜덤으로 1~4개 메뉴 선택
                        num_items = random.randint(1, 4)
                        random_items = []
                        
                        for _ in range(num_items):
                            menu_name = random.choice(menu_list)
                            quantity = random.randint(1, 3)  # 1~3개
                            random_items.append({
                                "name": menu_name,
                                "quantity": quantity
                            })
                        
                        detail = json.dumps({
                            "room_number": str(random.randint(101, 999)),  # 101~999호 랜덤
                            "items": random_items
                        }, ensure_ascii=False)
                        
                        self.get_logger().info(f"🎲 랜덤 주문 내역 생성: {len(random_items)}개 메뉴")
                        room_num = json.loads(detail)["room_number"]
                        self.get_logger().info(f"   🏠 호실: {room_num}호")
                        for item in random_items:
                            self.get_logger().info(f"   - {item['name']} {item['quantity']}개")
                        
                        self.publish_event(event_id, detail=detail)
                    else:
                        # 특정 이벤트들에 대한 detail 처리
                        detail = ""
                        if event_id == 9:  # 호실 번호 인식 완료
                            import random
                            room_number = str(random.randint(101, 999))
                            detail = room_number
                            self.get_logger().info(f"🏠 랜덤 호실 번호: {room_number}호")
                        elif event_id == 101:  # 목적지 입력 완료
                            locations = ["LOB_1", "LOB_2", "RES_1", "RES_2", "SUP_1", "ELE_1", "ELE_2", "ROOM_101", "ROOM_201"]
                            import random
                            location = random.choice(locations)
                            detail = location
                            self.get_logger().info(f"📍 랜덤 목적지: {location}")
                        elif event_id == 102:  # 사용자 점유 상태
                            import random
                            status = random.choice(["OCCUPIED", "VACANT"])
                            detail = status
                            self.get_logger().info(f"👤 사용자 점유 상태: {status}")
                        elif event_id == 106:  # 인식모드 전환 요청
                            import random
                            mode = random.choice(["0", "1", "2", "3"])
                            mode_names = {"0": "대기모드", "1": "등록모드", "2": "추적모드", "3": "엘리베이터모드"}
                            detail = mode
                            self.get_logger().info(f"👁️ 랜덤 인식모드: {mode_names[mode]}")
                        
                        self.publish_event(event_id, detail=detail)
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
        client = TestServiceClient()
        
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