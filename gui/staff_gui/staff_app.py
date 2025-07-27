import tkinter as tk
from tkinter import ttk, messagebox
import asyncio
import websockets
import requests
import json
import threading
import logging
from datetime import datetime
from config import RMS_WS_URL, RMS_HTTP_URL, FOOD_TYPES

# 로그 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('staff_gui.log', encoding='utf-8'),
        logging.StreamHandler()  # 콘솔에도 출력
    ]
)
logger = logging.getLogger(__name__)

class StaffGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROOMIE")
        self.root.geometry("800x600")
        self.root.configure(bg="#2c3e50")
        
        # 데이터 저장
        self.orders = {}  # task_id: order_data
        self.ready_orders = {}  # 준비완료된 주문들
        self.selected_order = None
        
        # WebSocket 연결 상태
        self.websocket = None
        self.ws_connected = False
        
        self.setup_ui()
        self.start_websocket_connection()
    
    def setup_ui(self):
        # 메인 프레임
        main_frame = tk.Frame(self.root, bg="#2c3e50")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 헤더
        header_frame = tk.Frame(main_frame, bg="#2c3e50")
        header_frame.pack(fill=tk.X, pady=(0, 20))
        
        title_label = tk.Label(header_frame, text="ROOMIE", font=("Arial", 24, "bold"), 
                              fg="white", bg="#2c3e50")
        title_label.pack(side=tk.LEFT)
        
        restaurant_label = tk.Label(header_frame, text="Restaurant", font=("Arial", 16), 
                                   fg="#3498db", bg="#2c3e50")
        restaurant_label.pack(side=tk.RIGHT)
        
        # 탭 노트북
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # 신청품목 탭
        self.orders_frame = tk.Frame(self.notebook, bg="white")
        self.notebook.add(self.orders_frame, text="신청품목")
        
        # 준비완료 탭  
        self.ready_frame = tk.Frame(self.notebook, bg="white")
        self.notebook.add(self.ready_frame, text="준비완료")
        
        self.setup_orders_tab()
        self.setup_ready_tab()
    
    def setup_orders_tab(self):
        # 왼쪽 주문 목록
        left_frame = tk.Frame(self.orders_frame, bg="#34495e", width=250)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        left_frame.pack_propagate(False)
        
        # 주문 목록 헤더
        orders_header = tk.Label(left_frame, text="신청품목", font=("Arial", 14, "bold"),
                                fg="white", bg="#34495e")
        orders_header.pack(pady=10)
        
        # 주문 리스트박스
        self.orders_listbox = tk.Listbox(left_frame, font=("Arial", 10), 
                                        bg="white", selectmode=tk.SINGLE)
        self.orders_listbox.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        self.orders_listbox.bind('<<ListboxSelect>>', self.on_order_select)
        
        # 오른쪽 상세정보
        right_frame = tk.Frame(self.orders_frame, bg="white")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 주문 번호 표시
        self.order_title = tk.Label(right_frame, text="주문 #", font=("Arial", 18, "bold"),
                                   bg="white")
        self.order_title.pack(pady=20)
        
        # 주문정보 섹션
        order_info_frame = tk.Frame(right_frame, bg="white")
        order_info_frame.pack(fill=tk.X, padx=20, pady=10)
        
        tk.Label(order_info_frame, text="주문정보", font=("Arial", 12, "bold"),
                bg="white").pack(anchor=tk.W)
        
        # 구분선
        separator1 = tk.Frame(order_info_frame, height=1, bg="#bdc3c7")
        separator1.pack(fill=tk.X, pady=5)
        
        self.order_details_frame = tk.Frame(order_info_frame, bg="white")
        self.order_details_frame.pack(fill=tk.X, pady=10)
        
        # 배송정보 섹션
        delivery_info_frame = tk.Frame(right_frame, bg="white")
        delivery_info_frame.pack(fill=tk.X, padx=20, pady=10)
        
        tk.Label(delivery_info_frame, text="배송정보", font=("Arial", 12, "bold"),
                bg="white").pack(anchor=tk.W)
        
        # 구분선
        separator2 = tk.Frame(delivery_info_frame, height=1, bg="#bdc3c7")
        separator2.pack(fill=tk.X, pady=5)
        
        self.delivery_info_frame = tk.Frame(delivery_info_frame, bg="white")
        self.delivery_info_frame.pack(fill=tk.X, pady=10)
        
        # 준비완료 버튼
        self.ready_button = tk.Button(right_frame, text="준비완료", 
                                     font=("Arial", 14, "bold"), bg="#3498db", 
                                     fg="white", pady=10, command=self.mark_ready)
        self.ready_button.pack(side=tk.BOTTOM, fill=tk.X, padx=20, pady=20)
        
        self.clear_order_details()
    
    def setup_ready_tab(self):
        # 준비완료된 주문들 표시
        self.ready_listbox = tk.Listbox(self.ready_frame, font=("Arial", 12))
        self.ready_listbox.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    def clear_order_details(self):
        self.order_title.config(text="주문을 선택하세요")
        
        # 기존 위젯들 제거
        for widget in self.order_details_frame.winfo_children():
            widget.destroy()
        for widget in self.delivery_info_frame.winfo_children():
            widget.destroy()
            
        self.ready_button.config(state="disabled")
    
    def display_order_details(self, order_data):
        self.clear_order_details()
        
        logger.info(f"🔍 display_order_details 호출됨: {order_data}")
        
        task_id = order_data['task_id']
        self.order_title.config(text=f"주문 #{task_id}")
        
        # 주문 항목들 표시
        items = order_data['order_details']['items']
        logger.info(f"📦 주문 항목들: {items}")
        total_amount = 0
        
        for item in items:
            item_frame = tk.Frame(self.order_details_frame, bg="white")
            item_frame.pack(fill=tk.X, pady=2)
            
            name_label = tk.Label(item_frame, text=item['name'], 
                                 font=("Arial", 11), bg="white")
            name_label.pack(side=tk.LEFT)
            
            quantity_label = tk.Label(item_frame, text=str(item['quantity']), 
                                     font=("Arial", 11), bg="white")
            quantity_label.pack(side=tk.RIGHT, padx=(0, 80))
            
            price_label = tk.Label(item_frame, text=f"{item['price']:,}원", 
                                  font=("Arial", 11), bg="white")
            price_label.pack(side=tk.RIGHT)
            
            total_amount += item['price'] * item['quantity']
        
        # 총액 표시
        total_frame = tk.Frame(self.order_details_frame, bg="white")
        total_frame.pack(fill=tk.X, pady=(10, 0))
        
        tk.Label(total_frame, text=f"총 {len(items)}개", font=("Arial", 11, "bold"), 
                bg="white").pack(side=tk.RIGHT, padx=(0, 80))
        tk.Label(total_frame, text=f"{total_amount:,}원", font=("Arial", 11, "bold"), 
                bg="white").pack(side=tk.RIGHT)
        
        # 배송정보 표시
        room_frame = tk.Frame(self.delivery_info_frame, bg="white")
        room_frame.pack(fill=tk.X, pady=2)
        tk.Label(room_frame, text="호실", font=("Arial", 11), bg="white").pack(side=tk.LEFT)
        location = order_data.get('request_location', 'N/A')  # request_location 사용
        tk.Label(room_frame, text=location, 
                font=("Arial", 11), bg="white").pack(side=tk.RIGHT)
        
        # 현재 시간으로 주문 시간 표시 (실제로는 서버에서 받아야 함)
        time_frame = tk.Frame(self.delivery_info_frame, bg="white")
        time_frame.pack(fill=tk.X, pady=2)
        tk.Label(time_frame, text="주문 일시", font=("Arial", 11), bg="white").pack(side=tk.LEFT)
        current_time = datetime.now().strftime("%Y.%m.%d %H:%M")
        tk.Label(time_frame, text=current_time, font=("Arial", 11), bg="white").pack(side=tk.RIGHT)
        
        self.ready_button.config(state="normal")
    
    def on_order_select(self, event):
        selection = self.orders_listbox.curselection()
        if selection:
            index = selection[0]
            order_text = self.orders_listbox.get(index)
            # 주문 번호 추출 (주문 #TASK_001 11:42 형식에서)
            task_id = order_text.split()[1].replace('#', '')
            if task_id in self.orders:
                self.selected_order = task_id
                self.display_order_details(self.orders[task_id])
    
    def add_new_order(self, order_data):
            """새 주문 추가"""
            # task_id를 항상 문자열로 변환하여 저장합니다.
            task_id = str(order_data['task_id']) 
            self.orders[task_id] = order_data
            
            # 주문 시간 (현재 시간으로 설정)
            current_time = datetime.now().strftime("%H:%M")
            order_text = f"주문 #{task_id} {current_time}"
            
            self.orders_listbox.insert(tk.END, order_text)
            
            # 새로 추가된 주문을 자동으로 선택하고 세부 정보를 표시합니다.
            # 새로 추가된 항목의 인덱스를 가져옵니다.
            new_order_index = self.orders_listbox.size() - 1
            if new_order_index >= 0:
                self.orders_listbox.selection_clear(0, tk.END)  # 이전 선택을 모두 지웁니다.
                self.orders_listbox.selection_set(new_order_index)  # 새 항목을 선택합니다.
                self.orders_listbox.see(new_order_index)  # 필요한 경우 새 항목으로 스크롤합니다.
                self.on_order_select(None) # 선택 핸들러를 수동으로 호출합니다.

            # 알림 표시
            messagebox.showinfo("새 주문", f"새로운 주문이 접수되었습니다!\n주문 번호: {task_id}")
    
    def mark_ready(self):
        """준비완료 처리"""
        if not self.selected_order:
            logger.warning("선택된 주문이 없어 준비완료 처리를 건너뜝니다.") # 추가
            return
            
        task_id = self.selected_order
        
        # HTTP 요청 URL과 페이로드 로그 추가
        request_url = f"{RMS_HTTP_URL}/food_order_status_change"
        request_payload = {
            "type": "request",
            "action": "food_order_status_change", 
            "payload": {
                "task_id": task_id
            }
        }
        logger.info(f"⬆️ HTTP 요청 전송: URL='{request_url}', Payload={json.dumps(request_payload, ensure_ascii=False)}") # 추가

        try:
            # HTTP 요청으로 상태 변경
            response = requests.post(
                request_url, # 수정
                json=request_payload, # 수정
                timeout=5
            )
            
            # 서버 응답 상태 코드 및 내용 로그 추가
            logger.info(f"⬇️ HTTP 응답 수신: Status Code={response.status_code}, Response Body='{response.text}'") # 추가

            if response.status_code == 200:
                data = response.json()
                if data.get('payload', {}).get('status_changed') == 'food_ready':
                    # 성공적으로 상태 변경됨
                    self.move_to_ready(task_id)
                    messagebox.showinfo("완료", f"주문 #{task_id}이 준비완료되었습니다!")
                else:
                    messagebox.showerror("오류", f"상태 변경에 실패했습니다. 응답: {data}") # 응답 내용 추가
            else:
                messagebox.showerror("오류", f"서버 오류: {response.status_code}. 응답: {response.text}") # 응답 내용 추가
                
        except requests.RequestException as e:
            logger.error(f"❌ 통신 오류 발생: {str(e)}") # 추가
            messagebox.showerror("오류", f"통신 오류: {str(e)}")
    
    def move_to_ready(self, task_id):
        """주문을 준비완료 탭으로 이동"""
        if task_id in self.orders:
            # 준비완료 목록에 추가
            order_data = self.orders[task_id]
            self.ready_orders[task_id] = order_data
            
            # 준비완료 리스트박스에 추가
            self.ready_listbox.insert(tk.END, f"주문 #{task_id} - 준비완료")
            
            # 신청품목에서 제거
            del self.orders[task_id]
            
            # 리스트박스에서 제거
            for i in range(self.orders_listbox.size()):
                if task_id in self.orders_listbox.get(i):
                    self.orders_listbox.delete(i)
                    break
            
            # 선택 해제
            self.selected_order = None
            self.clear_order_details()
    
    def show_robot_arrival(self, task_id, robot_id):
        """로봇 도착 알림"""
        messagebox.showinfo("로봇 도착", 
                           f"로봇 {robot_id}이 주문 #{task_id} 픽업을 위해 도착했습니다!")
    
    def start_websocket_connection(self):
        """WebSocket 연결 시작"""
        def run_websocket():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.websocket_handler())
        
        ws_thread = threading.Thread(target=run_websocket, daemon=True)
        ws_thread.start()
    
    async def websocket_handler(self):
        """WebSocket 메시지 처리"""
        while True:
            try:
                async with websockets.connect(RMS_WS_URL) as websocket:
                    self.websocket = websocket
                    self.ws_connected = True
                    logger.info("WebSocket 연결됨")
                    
                    async for message in websocket:
                        try:
                            data = json.loads(message)
                            # 받은 데이터 로그 출력
                            logger.info(f"📨 받은 WebSocket 메시지: {json.dumps(data, ensure_ascii=False, indent=2)}")
                            self.root.after(0, self.handle_websocket_message, data)
                        except json.JSONDecodeError:
                            logger.error(f"❌ 잘못된 JSON 메시지: {message}")
                            
            except Exception as e:
                logger.error(f"WebSocket 오류: {e}")
                self.ws_connected = False
                await asyncio.sleep(5)  # 5초 후 재연결 시도
    
    def handle_websocket_message(self, data):
        """WebSocket 메시지 처리 (메인 스레드에서 실행)"""
        message_type = data.get('type')
        action = data.get('action')
        payload = data.get('payload', {})
        
        logger.info(f"🔍 메시지 분석: type={message_type}, action={action}")
        
        if message_type == 'event':
            if action == 'food_order_creation':
                logger.info(f"🍽️ 새 주문 접수: {payload}")
                # 새 주문 접수
                self.add_new_order(payload)
            elif action == 'food_pickup_arrival':
                logger.info(f"🤖 로봇 도착: task_id={payload.get('task_id')}, robot_id={payload.get('robot_id')}")
                # 로봇 도착
                task_id = payload.get('task_id')
                robot_id = payload.get('robot_id')
                self.show_robot_arrival(task_id, robot_id)
            else:
                logger.warning(f"❓ 처리되지 않은 이벤트 액션: {action}")
        else:
            logger.warning(f"❓ 처리되지 않은 메시지 타입: {message_type}")
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = StaffGUI()
    app.run() 