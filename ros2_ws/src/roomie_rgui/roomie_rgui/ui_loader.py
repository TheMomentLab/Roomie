import os
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget

def load_ui(widget: QWidget, ui_path: str):
    # UI 파일을 여러 경로에서 찾음
    possible_paths = []
    
    # 개발 환경 경로 (src 디렉토리 구조)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(current_dir))  # roomie_rgui 디렉토리
    dev_path = os.path.join(project_root, ui_path)
    possible_paths.append(dev_path)
    
    # 설치된 패키지 경로 (share 디렉토리)
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('roomie_rgui')
        installed_path = os.path.join(package_share_dir, ui_path)
        possible_paths.append(installed_path)
    except:
        pass
    
    # 현재 디렉토리 기준 상대 경로
    relative_path = os.path.join(os.getcwd(), ui_path)
    possible_paths.append(relative_path)
    
    # 존재하는 경로 찾기
    full_ui_path = None
    for path in possible_paths:
        if os.path.exists(path):
            full_ui_path = path
            break
    
    if not full_ui_path:
        print(f"[ui_loader] UI 파일을 찾을 수 없음: {ui_path}")
        print(f"[ui_loader] 시도한 경로들:")
        for path in possible_paths:
            print(f"[ui_loader]    - {path}")
        return
    
    print(f"[ui_loader] UI 파일 로드: {full_ui_path}")
    uic.loadUi(full_ui_path, widget)
    widget.show()
