# --- 서버 설정 ---
SERVER_IP = "192.168.0.47"
SERVER_PORT = 8000
API_URL = f"http://{SERVER_IP}:{SERVER_PORT}/api/gui"
WEBSOCKET_URL = f"ws://{SERVER_IP}:{SERVER_PORT}/api/gui/ws/admin/admin_01"

# --- 통신 설정 ---
WEBSOCKET_RECONNECT_INTERVAL = 3000  # 재연결 시도 간격 (ms)

# --- UI 설정 ---
UI_FILE = 'admin_gui.ui'

# --- 스타일 설정 ---
APP_STYLE = """
QWidget { 
    font-family: 'Malgun Gothic'; 
}
QMainWindow {
    background-color: #f2f4f5;
}
QWidget#sidebar {
    background-color: #2f3e4e;
}
QLabel#label_title {
    color: white;
    font-weight: bold;
    font-size: 24px;
}
QLabel#label_admin {
    color: #e04e3e;
    font-weight: bold;
    font-size: 16px;
}
QPushButton#btn_db, QPushButton#btn_rm, QPushButton#btn_th {
    color: white;
    background-color: transparent;
    border: none;
    padding: 10px;
    text-align: left;
    font-size: 16px;
    font-weight: bold;
}
QPushButton#btn_db:checked, QPushButton#btn_rm:checked, QPushButton#btn_th:checked {
    background-color: #e04e3e;
    border-radius: 4px;
}
QPushButton#btn_db:hover, QPushButton#btn_rm:hover, QPushButton#btn_th:hover {
    background-color: #c84331;
    border-radius: 4px;
}
QLabel#label_db, QLabel#label_rm, QLabel#label_th, QLabel#label_td {
    color: #2f3e4e;
    font-size: 30px;
    font-weight: bold;
}
QFrame#frame, QFrame#frame_2, QFrame#frame_3, QFrame#frame_4 {
    background-color: white;
    border-radius: 8px;
    padding: 10px;
}
QPushButton#btn_floor1, QPushButton#btn_floor2 {
    background-color: #f0f0f0;
    color: #555;
    border: 1px solid #dcdcdc;
    padding: 4px;
}
QPushButton#btn_floor1:checked, QPushButton#btn_floor2:checked {
    background-color: #4a5a71;
    color: white;
    border: 1px solid #4a5a71;
    font-weight: bold;
    padding: 4px;
}
QLabel#label_totaltask, QLabel#label_waitingtask, QLabel#label_totalrobot, QLabel#label_activerobot {
    color: #2f3e4e;
    font-size: 26px;
    font-weight: bold;
}
QLabel#label_totaltask_2, QLabel#label_waitingtask_2, QLabel#label_totalrobot_2, QLabel#label_activerobot_2 {
    color: #4a5b6a;
    font-size: 18px;
}
QLabel#label_robotlocation {
    color: #2f3e4e;
    font-size: 14px;
    font-weight: bold;
}
QWidget#frame_search_rm, QWidget#frame_search_th {
    background-color: white;
    border-radius: 8px;
    padding: 8px;
}
QLineEdit, QComboBox, QDateEdit {        
    border: 1px solid #ccc;
    padding: 4px 4px;
}
QPushButton#btn_search_rm, QPushButton#btn_search_th {
    background-color: #e04e3e;
    color: white;
    border-radius: 4px;
}
QPushButton#btn_search_rm:hover, QPushButton#btn_search_th:hover {
    background-color: #c84331;
}
QHeaderView::section {
    background-color: #2f3e4e;
    color: white;
    padding: 6px;
    font-size: 13px;
}
QTableWidget {
    background-color: white;
    alternate-background-color: #f9fafb;
    gridline-color: #dfe1e5;
}
QWidget#widget_taskinfo, QWidget#widget_tasktimeline {
    background-color: white;
    border-radius: 8px;
    padding: 12px;
}
QPushButton#btn_detail {
    background-color: #e04e3e;
    color: white;
    border-radius: 4px;
}
QPushButton#btn_detail:hover {
    background-color: #c84331;
}
QPushButton#btn_back {
    background-color: #2f3e4e;
    color: white;
    border-radius: 4px;
    padding: 6px;
}
QPushButton#btn_back:hover {
    background-color: #1e293b;
}
"""

# --- 데이터 설정 ---
TASK_TYPES = ["전체", "음식배송", "비품배송"]
TASK_STATUSES = ["전체", "접수됨", "준비 완료", "로봇 할당됨", "픽업 장소로 이동", "픽업 대기 중", "배송 중", "배송 도착", "수령 완료"]
ROBOT_STATUSES = ["전체", "초기화", "충전상태", "작업대기", "픽업위치 이동", "픽업대기", "배송장소 이동", "수령대기", "대기위치로 이동", "엘리베이터 탑승", "오류"]
DESTINATIONS = ["전체", "ROOM_101", "ROOM_102", "ROOM_201", "ROOM_202"]
ROBOT_MODELS = ["전체", "ServiceBot_V1", "ServiceBot_V2"]

# --- ROS2 설정 (추후 사용) ---
ROS2_NODE_NAME = "roomie_admin_gui"
MAP_TOPIC = "/map"
ROBOT_POSITION_TOPIC = "/robot_position"