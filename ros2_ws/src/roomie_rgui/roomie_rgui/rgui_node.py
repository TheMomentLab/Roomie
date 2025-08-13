import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from PyQt6.QtWidgets import QApplication
import threading
from roomie_msgs.msg import RobotGuiEvent

from roomie_msgs.action import StartCountdown, ReturnCountdown

from .screen_manager import ScreenManager
from .service_client import call_service
from .task_state import DeliveryState

class RobotGuiNode(Node):
    def __init__(self, app):
        super().__init__('robot_gui_node')
        self.app = app
        self.screen = ScreenManager(self)

        # Publisher
        self.event_pub = self.create_publisher(RobotGuiEvent, '/robot_gui/event', 10)
        
        # Subscriber
        self.event_sub = self.create_subscription(RobotGuiEvent, '/robot_gui/event', self.on_robot_event, 10)


        
        # Action Servers
        self.departure_action_srv = ActionServer(
            self, 
            StartCountdown, 
            '/robot_gui/action/start_countdown', 
            self.handle_start_departure_countdown
        )
        self.return_action_srv = ActionServer(
            self, 
            ReturnCountdown, 
            '/robot_gui/action/return_countdown', 
            self.handle_start_return_countdown
        )
        
        # 내부 카운트다운 관련 변수 (기존 호환성용)
        self.countdown_timer = None
        self.countdown_remaining = 0
        self.countdown_action_text = ""
        self.is_delivery_countdown = False
        
        # 엘리베이터 사용 전 화면 상태 저장
        self.screen_before_elevator = None
        
        # 목적지(호실 번호 등) 저장
        self.current_destination: str | None = None
        
        # 카운트다운 시작 전 화면 상태 저장 (픽업/배송 단계 구분용)
        self.screen_before_countdown = None

    def publish_event(self, event_id: int, robot_id: int, task_id: int = 0, detail: str = ""):
        from builtin_interfaces.msg import Time
        from rclpy.clock import Clock

        msg = RobotGuiEvent()
        msg.robot_id = robot_id
        msg.task_id = task_id
        msg.rgui_event_id = event_id
        msg.detail = detail
        msg.timestamp = Clock().now().to_msg()
        self.event_pub.publish(msg)

    def handle_start_departure_countdown(self, goal_handle):
        """
        출발 카운트다운 시작 요청 처리 (액션)
        
        배송 시나리오:
        1. 초기 대기상태(TOUCH_SCREEN)에서 StartCountdown 액션 수신
        2. 카운트다운 화면(COUNTDOWN) 표시 및 5초 카운트다운 진행
        3. 카운트다운 완료 후 픽업장소 이동중(PICKUP_MOVING) 화면으로 전환
        4. 액션 성공 응답 반환
        """
        goal = goal_handle.request
        self.get_logger().info(f"📞 출발 카운트다운 액션 요청: robot_id={goal.robot_id}, task_id={goal.task_id}, task_type_id={goal.task_type_id}")
        
        # 카운트다운 시작 전 현재 화면 저장 (픽업/배송 단계 구분용)
        self.screen_before_countdown = self.screen.get_current_screen_name()
        self.get_logger().info(f"📱 카운트다운 시작 전 화면: {self.screen_before_countdown}")
        
        # 카운트다운 화면으로 전환
        self.screen.show_screen("COUNTDOWN")
        
        # 이전 화면과 task_type_id에 따라 카운트다운 텍스트 결정
        if goal.task_type_id in [0, 1]:  # 배송 작업 (음식배송, 비품배송)
            if self.screen_before_countdown in ["PICKUP_DRAWER_CONTROL", "CHECKING_ORDER", "PICKUP_ARRIVED", "DELIVERY_ARRIVED"]:
                action_text = "배송지로 이동"  # 픽업 완료 후 배송지로
            else:
                action_text = "픽업장소로 이동"  # 초기 출발
        elif goal.task_type_id == 2:  # 호출
            action_text = "호출장소로 이동"
        elif goal.task_type_id == 3:  # 길안내
            action_text = "길안내 시작"
        else:
            action_text = "이동"
        
        self.get_logger().info(f"⏰ 출발 카운트다운 시작: 5초 ({action_text})")
        
        # 액션에서 직접 카운트다운 처리 (5초)
        import time
        from roomie_msgs.action import StartCountdown
        
        for remaining in range(5, 0, -1):
            # 화면 업데이트
            self.update_countdown_display_direct(remaining, action_text)
            
            # 피드백 발송
            feedback = StartCountdown.Feedback()
            feedback.remaining_time = remaining
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"📤 액션 피드백 발송: remaining_time={remaining}")
            
            # 1초 대기
            time.sleep(1.0)
        
        # 카운트다운 완료
        self.get_logger().info("🎉 카운트다운 완료!")
        
        # 화면 전환
        self.handle_countdown_completed_direct(goal.task_type_id)
        
        # 결과 반환
        result = StartCountdown.Result()
        result.robot_id = goal.robot_id
        result.success = True
        
        goal_handle.succeed()
        self.get_logger().info(f"📤 액션 완료: success=True, robot_id={result.robot_id}")
        
        return result
    
    def update_countdown_display_direct(self, remaining_time, action_text):
        """카운트다운 화면 직접 업데이트 (메인 스레드로 위임)"""
        try:
            # ScreenManager의 스레드 안전 메서드 사용
            self.screen.update_countdown_display(remaining_time, action_text)
        except Exception as e:
            self.get_logger().error(f"카운트다운 화면 업데이트 실패: {e}")
    
    def handle_countdown_completed_direct(self, task_type_id):
        """카운트다운 완료 후 화면 전환 처리"""
        if task_type_id in [0, 1]:  # 배송 작업 (음식배송, 비품배송)
            # 카운트다운 시작 전 화면에 따라 다음 화면 결정
            if self.screen_before_countdown in ["PICKUP_DRAWER_CONTROL", "CHECKING_ORDER", "PICKUP_ARRIVED", "DELIVERY_ARRIVED"]:
                self.get_logger().info("🚚 배송 출발 카운트다운 완료 - 배송장소 이동중 화면으로 전환")
                self.screen.show_screen("DELIVERY_MOVING")
            else:
                self.get_logger().info("🚚 픽업 출발 카운트다운 완료 - 픽업장소 이동중 화면으로 전환")
                self.screen.show_screen("PICKUP_MOVING")
            
            # 상태 초기화
            self.screen_before_countdown = None
        elif task_type_id == 2:  # 호출
            self.get_logger().info("📞 호출 작업 카운트다운 완료 - 대기 화면으로 전환")
            self.screen.show_screen("TOUCH_SCREEN")
        elif task_type_id == 3:  # 길안내
            self.get_logger().info("🗺️ 길안내 작업 카운트다운 완료 - 대기 화면으로 전환")
            self.screen.show_screen("TOUCH_SCREEN")
        else:
            self.get_logger().warn(f"알 수 없는 task_type_id: {task_type_id} - 대기 화면으로 전환")
            self.screen.show_screen("TOUCH_SCREEN")
    
    # DEPRECATED: 내부 카운트다운은 더 이상 사용하지 않음
    # 모든 카운트다운은 RC에서 StartCountdown 액션으로 요청
    def start_delivery_countdown(self):
        """
        DEPRECATED: 사용하지 않음
        
        이제 [적재 완료] 클릭 시 event_id=105를 RC로 발송하고,
        RC가 다시 StartCountdown 액션을 보내는 방식으로 변경됨
        """
        self.get_logger().warn("⚠️ start_delivery_countdown()는 더 이상 사용되지 않습니다. RC에서 액션으로 요청해주세요.")
    
    def update_countdown_text(self):
        """카운트다운 화면의 텍스트 업데이트"""
        try:
            # 현재 COUNTDOWN 화면의 위젯 가져오기
            countdown_widget = self.screen.screen_widgets.get("COUNTDOWN")
            if not countdown_widget:
                self.get_logger().warn("COUNTDOWN 화면 위젯을 찾을 수 없음")
                return
            
            from PyQt6.QtWidgets import QLabel
            
            # countdownTitle 라벨 업데이트 (완전한 텍스트로)
            title_label = countdown_widget.findChild(QLabel, "countdownTitle")
            if title_label:
                title_text = f"{self.countdown_remaining}초후에 {self.countdown_action_text}합니다."
                title_label.setText(title_text)
                self.get_logger().info(f"📝 카운트다운 countdownTitle 업데이트: {title_text}")
            else:
                self.get_logger().warn("countdownTitle 라벨을 찾을 수 없음")
            
            # countdownNumber 라벨 업데이트
            countdown_label = countdown_widget.findChild(QLabel, "countdownNumber")
            if countdown_label:
                countdown_label.setText(str(self.countdown_remaining))
                self.get_logger().info(f"📝 카운트다운 countdownNumber 업데이트: {self.countdown_remaining}")
            else:
                self.get_logger().warn("countdownNumber 라벨을 찾을 수 없음")
                
        except Exception as e:
            self.get_logger().error(f"카운트다운 텍스트 업데이트 실패: {e}")

    def start_countdown_timer(self):
        """카운트다운 타이머 시작"""
        if self.countdown_remaining > 0:
            # 1초 후에 on_countdown_tick 호출
            self.countdown_timer = threading.Timer(1.0, self.on_countdown_tick)
            self.countdown_timer.start()
    
    def on_countdown_tick(self):
        """카운트다운 타이머 틱 (1초마다 호출) - 내부 카운트다운용"""
        self.countdown_remaining -= 1
        self.get_logger().info(f"⏰ 내부 카운트다운: {self.countdown_remaining}초 남음")
        
        if self.countdown_remaining > 0:
            # 남은 시간 표시 업데이트
            self.update_countdown_display()
            # 다음 타이머 시작
            self.start_countdown_timer()
        else:
            # 카운트다운 완료
            self.get_logger().info("🎉 내부 카운트다운 완료!")
            
            # 내부 카운트다운 완료 후 화면 전환
            self.handle_internal_countdown_completed()
            
    def update_countdown_display(self):
        """카운트다운 화면의 시간 표시 업데이트 (메인 스레드로 위임)"""
        try:
            self.screen.update_countdown_display(self.countdown_remaining, self.countdown_action_text)
        except Exception as e:
            self.get_logger().error(f"카운트다운 화면 업데이트 실패: {e}")
    
    # DEPRECATED: 내부 카운트다운은 더 이상 사용하지 않음
    def handle_internal_countdown_completed(self):
        """DEPRECATED: 사용하지 않음"""
        self.get_logger().warn("⚠️ handle_internal_countdown_completed()는 더 이상 사용되지 않습니다.")
    
    def handle_start_return_countdown(self, goal_handle):
        """
        복귀 카운트다운 시작 요청 처리 (액션)
        
        복귀 시나리오:
        1. [수령 완료] 버튼 클릭 → event_id=100 발송  
        2. event_id=18 수신 → THANK_YOU 감사화면 전환
        3. ReturnCountdown 액션 수신 → 카운트다운 후 복귀화면 전환
        """
        goal = goal_handle.request
        self.get_logger().info(f"🏠 복귀 카운트다운 액션 요청: robot_id={goal.robot_id}")
        
        # 카운트다운 화면으로 전환 (감사화면은 18번 이벤트에서 이미 처리됨)
        self.screen.show_screen("COUNTDOWN")
        
        action_text = "대기장소로 복귀"
        self.get_logger().info(f"⏰ 복귀 카운트다운 시작: 10초 ({action_text})")
        
        # 액션에서 직접 카운트다운 처리 (10초)
        import time
        from roomie_msgs.action import ReturnCountdown
        
        for remaining in range(10, 0, -1):
            # 화면 업데이트
            self.update_countdown_display_direct(remaining, action_text)
            
            # 피드백 발송
            feedback = ReturnCountdown.Feedback()
            feedback.remaining_time = remaining
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"📤 복귀 액션 피드백 발송: remaining_time={remaining}")
            
            # 1초 대기
            time.sleep(1.0)
        
        # 카운트다운 완료
        self.get_logger().info("🎉 복귀 카운트다운 완료!")
        
        # 화면 전환
        self.get_logger().info("🏠 복귀 카운트다운 완료 - 대기장소 복귀 화면으로 전환")
        self.screen.show_screen("RETURN_TO_BASE")
        
        # 결과 반환
        result = ReturnCountdown.Result()
        result.robot_id = goal.robot_id
        result.success = True
        
        goal_handle.succeed()
        self.get_logger().info(f"📤 복귀 액션 완료: success=True, robot_id={result.robot_id}")
        
        return result

    def on_robot_event(self, msg):
        """RC로부터 받은 이벤트 처리"""
        event_id = msg.rgui_event_id
        self.get_logger().info(f"🔔 이벤트 수신: ID={event_id}, robot_id={msg.robot_id}, detail={msg.detail}")
        
        # 이벤트 ID에 따른 화면 전환 (순서대로)
        # 엘리베이터 관련 이벤트 처리
        if event_id == 1:  # 엘리베이터 버튼 조작 시작
            # 현재 화면 상태 저장 (엘리베이터 사용 후 복원용)
            self.screen_before_elevator = self.screen.get_current_screen_name()
            self.get_logger().info(f"🛗 엘리베이터 사용 전 화면 저장: {self.screen_before_elevator}")
            
            self.get_logger().info("🛗 엘리베이터 버튼 조작 시작 - ELEVATOR_MANIPULATING 화면으로 전환")
            self.screen.show_screen("ELEVATOR_MANIPULATING")
        elif event_id == 2:  # 엘리베이터 버튼 조작 종료
            self.get_logger().info("🛗 엘리베이터 버튼 조작 종료 - ELEVATOR_CALLING 화면으로 전환")
            self.screen.show_screen("ELEVATOR_CALLING")
        elif event_id == 3:  # 엘리베이터 탑승 시작
            self.get_logger().info("🛗 엘리베이터 탑승 시작 - ELEVATOR_BOARDING 화면으로 전환")
            self.screen.show_screen("ELEVATOR_BOARDING")
        elif event_id == 4:  # 엘리베이터 탑승 종료
            self.get_logger().info("🛗 엘리베이터 탑승 종료 - ELEVATOR_MOVING_TO_TARGET 화면으로 전환")
            self.screen.show_screen("ELEVATOR_MOVING_TO_TARGET")
        elif event_id == 5:  # 엘리베이터 하차 시작
            self.get_logger().info("🛗 엘리베이터 하차 시작 - ELEVATOR_EXITING 화면으로 전환")
            self.screen.show_screen("ELEVATOR_EXITING")
        elif event_id == 6:  # 엘리베이터 하차 종료
            self.get_logger().info("🛗 엘리베이터 하차 종료 - 원래 화면으로 복원")
            
            # 엘리베이터 사용 전 화면으로 복원
            if self.screen_before_elevator:
                self.get_logger().info(f"🛗 엘리베이터 사용 전 화면으로 복원: {self.screen_before_elevator}")
                self.screen.show_screen(self.screen_before_elevator)
                # 상태 초기화
                self.screen_before_elevator = None
            else:
                # 저장된 화면이 없으면 기본 화면으로
                self.get_logger().warn("🛗 저장된 화면이 없어서 TOUCH_SCREEN으로 전환")
                self.screen.show_screen("TOUCH_SCREEN")
        elif event_id == 7:  # 호출 이동 시작
            self.get_logger().info("📞 호출 이동 시작")
            # 호출 작업의 이동은 보통 TOUCH_SCREEN에서 시작
            # 특별한 화면 전환이 필요하면 여기에 구현
        elif event_id == 8:  # 호출 이동 종료
            self.get_logger().info("📞 호출 이동 종료")
            # 호출 완료 후 처리
        elif event_id == 9:  # 호실 번호 인식 완료
            self.get_logger().info(f"🏠 호실 번호 인식 완료: {msg.detail}")
            # 인식된 호실 번호는 detail에 저장됨 (예: "101")
            # REGISTERING 화면으로 전환
            try:
                self.current_destination = str(msg.detail) if msg.detail is not None else None
            except Exception:
                self.current_destination = None
            self.screen.show_screen("REGISTERING")
        elif event_id == 10:  # 길안내 이동 시작
            self.get_logger().info("🗺️ 길안내 이동 시작")
            # 현재 화면이 REGISTERING일 때만 GUIDANCE_SCREEN으로 전환
            current = self.screen.get_current_screen_name() if hasattr(self.screen, "get_current_screen_name") else None
            if current == "REGISTERING":
                self.screen.show_screen("GUIDANCE_SCREEN")
            else:
                self.get_logger().info(f"현재 화면이 {current}이므로 화면 전환 생략")
        elif event_id == 11:  # 길안내 이동 종료
            self.get_logger().info("🗺️ 길안내 이동 종료")
            # 도착 화면으로 전환
            self.screen.show_screen("DESTINATION_ARRIVED")
        elif event_id == 12:  # 픽업장소 이동 시작
            self.screen.show_screen("PICKUP_MOVING")
        elif event_id == 13:  # 픽업장소 이동 종료
            # 주문 내역이 detail에 있으면 파싱해서 화면에 전달
            import json
            items = []
            room_number = "202"  # 기본값
            if msg.detail:
                try:
                    data = json.loads(msg.detail)
                    items = data.get("items", [])
                    room_number = data.get("room_number", "202")
                except Exception as e:
                    self.get_logger().warn(f"주문 내역 detail 파싱 실패: {e}")
            self.screen.show_screen("PICKUP_ARRIVED")
            if items:
                # 주문 내역을 화면에 표시
                delivery_controller = self.screen.get_screen_controller("CHECKING_ORDER")
                if delivery_controller and hasattr(delivery_controller, 'show_pickup_order'):
                    delivery_controller.show_pickup_order(items, room_number)
                else:
                    self.get_logger().info(f"주문 내역: {items}, 호실: {room_number}호")
        elif event_id == 14:  # 배송장소 이동 시작
            self.screen.show_screen("DELIVERY_MOVING")
        elif event_id == 15:  # 배송장소 도착 완료
            self.screen.show_screen("DELIVERY_ARRIVED")
        elif event_id == 16:  # 서랍 열림
            # 현재 화면에 따라 다음 화면으로
            current = self.screen.get_current_screen_name()
            if current == "PICKUP_ARRIVED":
                self.screen.show_screen("CHECKING_ORDER")
            elif current == "CHECKING_ORDER":
                self.screen.show_screen("PICKUP_DRAWER_CONTROL")
            elif current == "DELIVERY_ARRIVED":
                self.screen.show_screen("DELIVERY_DRAWER_CONTROL")
            elif current == "PICKUP_DRAWER_CONTROL":
                # 픽업 서랍 조작 화면에서 서랍이 열렸을 때 적재완료 버튼 활성화
                self.screen.notify_drawer_opened(msg.detail)
            elif current == "DELIVERY_DRAWER_CONTROL":
                # 배송 서랍 조작 화면에서 서랍이 열렸을 때 수령완료 버튼 활성화
                self.screen.notify_drawer_opened(msg.detail)
        elif event_id == 17:  # 서랍 닫힘
            self.get_logger().info("🔒 서랍 닫힘 이벤트 수신")
            # 서랍이 닫혔을 때 처리 로직
        elif event_id == 18:  # 서랍 잠금
            self.get_logger().info("🔐 서랍 잠금 이벤트 수신 - 감사 화면으로 전환")
            # 서랍이 잠긴 후 감사 화면 표시 (수령 완료 시나리오)
            self.screen.show_screen("THANK_YOU")
        elif event_id == 21:  # 투숙객 이탈
            self.get_logger().info("👤 투숙객 이탈 이벤트 수신")
            # GUIDANCE_SCREEN에서 이탈 시 RECHECKING으로 전환
            current = self.screen.get_current_screen_name() if hasattr(self.screen, "get_current_screen_name") else None
            if current == "GUIDANCE_SCREEN":
                self.screen.show_screen("RECHECKING")
            else:
                # 다른 화면에서는 별도 처리 없음
                pass
        elif event_id == 22:  # 투숙객 이탈 후 재등록
            self.get_logger().info("👤 투숙객 이탈 후 재등록 이벤트 수신")
            # RECHECKING에서 재등록 시 GUIDANCE_SCREEN으로 전환
            current = self.screen.get_current_screen_name() if hasattr(self.screen, "get_current_screen_name") else None
            if current == "RECHECKING":
                self.screen.show_screen("GUIDANCE_SCREEN")
            else:
                # 다른 화면에서는 별도 처리 없음
                pass
        elif event_id == 23:  # 투숙객 등록
            self.get_logger().info("👤 투숙객 등록 이벤트 수신")
            # 새로운 투숙객이 등록되었을 때 처리
        elif event_id == 24:  # 배송 수령 완료
            self.screen.show_screen("THANK_YOU")
        elif event_id == 25:  # 배송 수령 미완료
            # 감사 화면 후 초기 화면으로
            self.screen.show_screen("TOUCH_SCREEN")
        elif event_id == 26:  # 적재 감지
            self.get_logger().info("📦 적재 감지 이벤트 수신")
            # 물품이 적재되었을 때 처리
        elif event_id == 27:  # 적재 미감지
            self.get_logger().info("📦 적재 미감지 이벤트 수신")
            # 물품이 적재되지 않았을 때 처리
        elif event_id == 100:  # [수령 완료] 클릭
            self.get_logger().info("✅ 수령 완료 버튼 클릭됨 - RC로 이벤트 발송")
            # 수령 완료 이벤트를 RC로 발송 (RC가 ReturnCountdown 액션을 보낼 것임)
            # 여기서는 이벤트 발송만 하고, 실제 감사 화면과 복귀 카운트다운은 RC에서 액션으로 요청할 때 시작
        elif event_id == 101:  # 목적지 입력 완료
            self.get_logger().info(f"📍 목적지 입력 완료: {msg.detail}")
            # detail에 목적지 정보가 저장됨 (예: "LOCATION_NAME")
        elif event_id == 102:  # 사용자 점유 상태
            self.get_logger().info(f"👤 사용자 점유 상태: {msg.detail}")
            # detail: "OCCUPIED" 또는 "VACANT"
        elif event_id == 103:  # [카드키로 입력] 선택
            self.get_logger().info("🔑 카드키로 입력 선택됨")
            # 카드키 입력 방식 선택 시 처리
        elif event_id == 104:  # 서랍 열기 버튼 클릭
            self.get_logger().info("🔓 서랍 열기 버튼 클릭됨 - 서랍 열림 이벤트 발행")
            # 서랍 열림을 알리는 이벤트 발행 (event_id=16)
            event_msg = RobotGuiEvent()
            event_msg.robot_id = 0  # 기본 로봇 ID
            event_msg.rgui_event_id = 16
            event_msg.detail = "drawer_opened"
            self.event_pub.publish(event_msg)
        elif event_id == 105:  # 적재 완료 버튼 클릭
            self.get_logger().info("📦 적재 완료 버튼 클릭됨 - RC로 이벤트 발송")
            # 적재 완료 이벤트를 RC로 발송 (RC가 다시 StartCountdown 액션을 보낼 것임)
            # 여기서는 이벤트 발송만 하고, 실제 카운트다운은 RC에서 액션으로 요청할 때 시작
        elif event_id == 106:  # 인식모드 전환 요청
            self.get_logger().info(f"👁️ 인식모드 전환 요청: {msg.detail}")
            # detail에 모드 값이 저장됨:
            # "0": 대기모드, "1": 등록모드, "2": 추적모드, "3": 엘리베이터모드
            mode_names = {"0": "대기모드", "1": "등록모드", "2": "추적모드", "3": "엘리베이터모드"}
            mode_name = mode_names.get(msg.detail, f"알 수 없는 모드({msg.detail})")
            self.get_logger().info(f"👁️ 인식모드를 {mode_name}으로 전환 요청")
        elif event_id == 19:  # 충전 시작
            self.get_logger().info("🔋 충전 시작 이벤트 수신 - 충전 화면으로 전환")
            # 복귀중 화면에서 충전중 화면으로 전환
            self.screen.show_screen("CHARGING")
        elif event_id == 20:  # 충전 종료
            self.get_logger().info("🔋 충전 완료 이벤트 수신 - 초기 화면으로 전환")
            # 충전 완료 후 초기 화면으로 전환
            self.screen.show_screen("TOUCH_SCREEN")
        else:
            self.get_logger().warn(f"처리되지 않은 이벤트 ID: {event_id}")




def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = RobotGuiNode(app)
    
    # ROS2 스핀을 백그라운드에서 실행
    import threading
    def spin_ros():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # GUI 메인루프 실행
    try:
        sys.exit(app.exec())
    finally:
        rclpy.shutdown()
