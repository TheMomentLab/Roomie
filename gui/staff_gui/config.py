# RMS 서버 설정
<<<<<<< HEAD
# RMS_HOST = "192.168.0.47"
RMS_HOST = "localhost"
RMS_PORT = 8000
# 인터페이스 명세에 맞춰 수정: ws/admin/{staff_id}
RMS_WS_URL = f"ws://{RMS_HOST}:{RMS_PORT}/api/gui/ws/admin/staff_01"
RMS_HTTP_URL = f"http://{RMS_HOST}:{RMS_PORT}/api/gui"

# # config.py

=======
RMS_HOST = "0.0.0.0"
RMS_PORT = 8888
RMS_WS_URL = f"ws://{RMS_HOST}:{RMS_PORT}/api/gui/ws/staff/staff_01"
RMS_HTTP_URL = f"http://{RMS_HOST}:{RMS_PORT}/api/gui"

# # config.py
RMS_HOST = "localhost"
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
# RMS_PORT = 8000
# RMS_WS_URL = f"ws://{RMS_HOST}:{RMS_PORT}/ws"
# RMS_HTTP_URL = f"http://{RMS_HOST}:{RMS_PORT}"

# 음식 타입 매핑 (인터페이스 명세에 맞춰 수정)
FOOD_TYPES = {
    0: "스파게티",
    1: "피자", 
    2: "스테이크",
    3: "버거"
}

# 비품 타입 매핑 추가
SUPPLY_TYPES = {
    0: "칫솔",
    1: "타월",
    2: "생수", 
    3: "수저"
}