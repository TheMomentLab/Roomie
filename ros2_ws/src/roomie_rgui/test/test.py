from PyQt6 import QtWidgets, QtGui
import sys
import os

app = QtWidgets.QApplication(sys.argv)

# 메인 윈도우 생성
window = QtWidgets.QWidget()
window.setWindowTitle("Robot GUI Test")
window.setGeometry(0, 0, 1920, 1080)

# 이미지 라벨 생성
image_label = QtWidgets.QLabel(window)
image_label.setGeometry(0, 0, 1920, 1080)

# 이미지 로드
image_path = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/test/로봇 gui.png"
if os.path.exists(image_path):
    pixmap = QtGui.QPixmap(image_path)
    if not pixmap.isNull():
        image_label.setPixmap(pixmap)
        image_label.setScaledContents(True)
        print(f"이미지 로드 성공: {image_path}")
    else:
        print(f"이미지 로드 실패: {image_path}")
        # 테스트용 색상 배경
        image_label.setStyleSheet("background-color: red;")
else:
    print(f"이미지 파일이 존재하지 않습니다: {image_path}")
    # 테스트용 색상 배경
    image_label.setStyleSheet("background-color: blue;")

window.showFullScreen()
sys.exit(app.exec())
