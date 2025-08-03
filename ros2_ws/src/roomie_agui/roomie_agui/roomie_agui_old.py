import sys
from PyQt6 import uic
from PyQt6.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsPixmapItem
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QTimer, Qt

# uic.loadUiType("admin_gui.ui") # 이 방법도 있지만, loadUi가 더 직관적입니다.

class AdminWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # .ui 파일을 직접 로드합니다.
        # 이 한 줄이 Ui_MainWindow 클래스와 setupUi(self)를 대체합니다.
        uic.loadUi("admin_gui.ui", self)
        
        # 앱 전체에 적용될 스타일시트 설정
        self.apply_stylesheet()

        # 1. 페이지 전환 기능 연결
        self.init_signals()

        # 2. 지도 및 로봇 실시간 위치 표시 기능 초기화
        self.init_map()

        # 3. 초기 페이지를 Dashboard로 설정
        self.stackedWidget.setCurrentIndex(0)
        self.btn_db.setChecked(True)
        
        # 윈도우 타이틀 설정
        self.setWindowTitle("ROOMIE 어드민")


    def apply_stylesheet(self):
        """UI에 QSS 스타일시트를 적용합니다."""
        style_sheet = """
            QWidget { font-family: 'Malgun Gothic'; }
            #widget { background-color: #2c3e50; }
            #label_title { color: white; font-size: 28px; font-weight: bold; padding-left: 20px; }
            #label_admin { color: #e74c3c; font-size: 16px; font-weight: bold; padding-left: 22px; }
            #verticalLayoutWidget QPushButton { background-color: #2c3e50; color: #bdc3c7; border: none; font-size: 15px; font-weight: bold; text-align: left; padding-left: 25px; height: 50px; }
            #verticalLayoutWidget QPushButton:hover { background-color: #34495e; color: white; }
            #verticalLayoutWidget QPushButton:checked { background-color: #e74c3c; color: white; }
            #stackedWidget > QWidget { background-color: #f0f2f5; }
            #label_db, #label_rm, #label_th, #label_td { font-size: 32px; font-weight: bold; color: #2c3e50; padding: 20px; }
            #page_db QFrame { background-color: white; border-radius: 10px; border: 1px solid #e0e0e0; }
            #label_totaltask, #label_waitingtask, #label_activerobot, #label_error { font-size: 40px; font-weight: bold; color: #2c3e50; }
            #label_totaltask_2, #label_waitingtask_2, #label_activerobot_2, #label_error_2 { font-size: 14px; color: #7f8c8d; }
            #frame_search_rm, #frame_search_th { background-color: white; border-radius: 10px; }
            #frame_search_rm QLabel, #frame_search_th QLabel { font-size: 13px; color: #333; }
            QLineEdit, QComboBox, QDateEdit { border: 1px solid #bdc3c7; border-radius: 5px; padding: 5px; background-color: white; }
            QComboBox::drop-down { border: none; }
            QComboBox::down-arrow { image: url(down_arrow.png); }
            QPushButton#btn_search_rm, QPushButton#btn_search_th, QPushButton#btn_detail { background-color: #e74c3c; color: white; border: none; border-radius: 5px; font-size: 14px; font-weight: bold; padding: 8px; }
            QPushButton#btn_search_rm:hover, QPushButton#btn_search_th:hover, QPushButton#btn_detail:hover { background-color: #c0392b; }
            QTableWidget { background-color: white; border: 1px solid #e0e0e0; border-radius: 5px; gridline-color: #f0f2f5; }
            QHeaderView::section { background-color: #f8f9fa; padding: 8px; border: none; border-bottom: 1px solid #e0e0e0; font-weight: bold; }
            QTableWidget::item { padding: 10px; }
        """
        self.setStyleSheet(style_sheet)
    
    def init_signals(self):
        """UI 요소들의 시그널-슬롯을 연결합니다. 'self.ui' 없이 직접 접근합니다."""
        self.btn_db.clicked.connect(lambda: self.stackedWidget.setCurrentIndex(0))
        self.btn_rm.clicked.connect(lambda: self.stackedWidget.setCurrentIndex(1))
        self.btn_th.clicked.connect(lambda: self.stackedWidget.setCurrentIndex(2))
        self.btn_detail.clicked.connect(lambda: self.stackedWidget.setCurrentIndex(3))
        self.btn_back.clicked.connect(lambda: self.stackedWidget.setCurrentIndex(2))

    def init_map(self):
        """지도와 로봇 아이템을 QGraphicsView에 설정합니다."""
        self.scene = QGraphicsScene()
        self.map.setScene(self.scene)
        self.map.setStyleSheet("border-radius: 10px; border: 1px solid #e0e0e0;")

        self.map_pixmap = QPixmap("map.png")
        if not self.map_pixmap.isNull():
            self.scene.addPixmap(self.map_pixmap)
            self.map.setSceneRect(self.map_pixmap.rect())
        else:
            print("오류: 'map.png' 파일을 찾을 수 없습니다.")

        robot_pixmap = QPixmap("robot.png")
        if not robot_pixmap.isNull():
            self.robot_item = QGraphicsPixmapItem(robot_pixmap.scaled(50, 50, Qt.AspectRatioMode.KeepAspectRatio))
            self.scene.addItem(self.robot_item)
            self.robot_item.setZValue(1)
        else:
            print("오류: 'robot.png' 파일을 찾을 수 없습니다.")
            return

        self.robot_x, self.robot_y = 50, 50
        self.robot_item.setPos(self.robot_x, self.robot_y)

        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_robot_position)
        self.timer.start()

    def update_robot_position(self):
        """타이머에 의해 주기적으로 호출되어 로봇 위치를 업데이트합니다."""
        self.robot_x += 2
        self.robot_y += 1

        if self.robot_x > self.map.width() - 50: self.robot_x = 0
        if self.robot_y > self.map.height() - 50: self.robot_y = 0
        
        self.robot_item.setPos(self.robot_x, self.robot_y)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = AdminWindow()
    window.show()
    sys.exit(app.exec())