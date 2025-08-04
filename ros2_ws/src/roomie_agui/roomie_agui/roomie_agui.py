from PyQt6 import QtWidgets, uic, QtCore
import sys

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # Load UI from Qt Designer file
        uic.loadUi('admin_gui.ui', self)

        # 윈도우 타이틀 설정
        self.setWindowTitle("ROOMIE - Admin")

        # Assign objectNames for styling
        # sidebar widget in .ui is likely named 'widget'; rename to 'sidebar'
        sidebar = self.findChild(QtWidgets.QWidget, 'widget')
        if sidebar:
            sidebar.setObjectName('sidebar')
        # Search frames
        frame_rm = self.findChild(QtWidgets.QFrame, 'frame_search_rm')
        if frame_rm:
            frame_rm.setObjectName('frame_search_rm')
        frame_th = self.findChild(QtWidgets.QFrame, 'frame_search_th')
        if frame_th:
            frame_th.setObjectName('frame_search_th')
        # Task detail containers
        self.widget_taskinfo.setObjectName('widget_taskinfo')
        self.widget_tasktimeline.setObjectName('widget_tasktimeline')

        # Apply custom styles
        self.apply_styles()

        # 초기 페이지 설정
        self.btn_db.setChecked(True)
        self.stackedWidget.setCurrentIndex(0)

        # 네비게이션 버튼 시그널 연결
        self.btn_db.clicked.connect(lambda: self.switch_page(0))
        self.btn_rm.clicked.connect(lambda: self.switch_page(1))
        self.btn_th.clicked.connect(lambda: self.switch_page(2))
        self.btn_detail.clicked.connect(lambda: self.switch_page(3))
        self.btn_back.clicked.connect(lambda: self.switch_page(2))

        # 검색 버튼 시그널 연결
        self.btn_search_rm.clicked.connect(self.search_robot)
        self.btn_search_th.clicked.connect(self.search_tasks)    

    def switch_page(self, index: int):
        for btn in (self.btn_db, self.btn_rm, self.btn_th):
            btn.setChecked(False)
        if index == 0:
            self.btn_db.setChecked(True)
        elif index == 1:
            self.btn_rm.setChecked(True)
        elif index == 2:
            self.btn_th.setChecked(True)
        self.stackedWidget.setCurrentIndex(index)

    def search_robot(self):
        robot_id = self.lineEdit_id.text()
        model = self.combo_model.currentText()
        status = self.combo_robotstatus.currentText()
        print(f'Search Robot - ID: {robot_id}, Model: {model}, Status: {status}')

    def search_tasks(self):
        start_date = self.dateEdit_start.date().toString('yyyy-MM-dd')
        end_date = self.dateEdit_end.date().toString('yyyy-MM-dd')
        task_type = self.combo_tasktype.currentText()
        task_status = self.combo_taskstatus.currentText()
        destination = self.combo_destination.currentText()
        print(f'Search Tasks - {start_date} to {end_date}, Type: {task_type}, Status: {task_status}, Dest: {destination}')

    def apply_styles(self):
        style = """
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
        /* Navigation buttons */
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
        /* Page title labels */
        QLabel#label_db, QLabel#label_rm, QLabel#label_th, QLabel#label_td {
            color: #2f3e4e;
            padding: 20px;
            font-size: 30px;
            font-weight: bold;
        }
        /* Dashboard cards */
        QFrame#frame, QFrame#frame_2, QFrame#frame_3, QFrame#frame_4 {
            background-color: white;
            border-radius: 8px;
            padding: 10px;
        }
        QLabel#label_totaltask, QLabel#label_waitingtask, QLabel#label_activerobot, QLabel#label_error {
            color: #2f3e4e;
            font-size: 26px;
            font-weight: bold;
        }
        QLabel#label_totaltask_2, QLabel#label_waitingtask_2, QLabel#label_activerobot_2, QLabel#label_error_2 {
            color: #4a5b6a;
            font-size: 18px;
        }
        QLabel#label_robotlocation {
            color: #2f3e4e;
            font-size: 14px;
            font-weight: bold;
        }
        /* Search frames and inputs */
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
        /* Table styling */
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
        /* Task detail containers */
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
        self.setStyleSheet(style)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
