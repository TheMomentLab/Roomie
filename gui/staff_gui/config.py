

# # config.py
RMS_HOST = "192.168.0.5"
RMS_PORT = 8000
RMS_WS_URL = f"ws://{RMS_HOST}:{RMS_PORT}/api/gui/ws/staff/staff_01"
RMS_HTTP_URL = f"http://{RMS_HOST}:{RMS_PORT}/api/gui"

# # config.py
# RMS_HOST = "localhost"
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