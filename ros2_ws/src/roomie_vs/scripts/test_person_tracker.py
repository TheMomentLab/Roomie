import cv2
import yaml
import os
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory
from roomie_vs.person_tracking.person_tracker import PersonTracker
from roomie_vs.camera_manager import WebCamCamera
from ultralytics import YOLO

class PersonTrackerTester:
    """ROS2 환경 없이 PersonTracker 클래스를 독립적으로 테스트하기 위한 클래스입니다."""

    def __init__(self):
        self.logger = get_logger('person_tracker_tester')
        
        # 설정 파일 로드
        package_share_directory = get_package_share_directory('roomie_vs')
        config_path = os.path.join(package_share_directory, 'config', 'vision_config.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # YOLO 모델 로드
        yolo_model_path = self.config['yolo']['models']['normal']
        if not os.path.exists(yolo_model_path):
            self.logger.error(f"YOLO 모델을 찾을 수 없습니다: {yolo_model_path}")
            return
        self.yolo_model = YOLO(yolo_model_path)
        self.logger.info("YOLO 모델 로드가 완료되었습니다.")

        # PersonTracker 초기화
        self.person_tracker = PersonTracker(self.logger, self.config, self.yolo_model)

        # 웹캠 초기화
        self.camera = WebCamCamera(self.logger, camera_id=0)
        if not self.camera.initialize():
            self.logger.error("카메라를 열 수 없습니다.")
            return
        
        self.logger.info("테스트 준비 완료. 키 입력을 통해 모드를 변경할 수 있습니다.")
        self.logger.info("  - 'r': 등록 모드 시작 (3초간)")
        self.logger.info("  - 't': 추적 모드 시작")
        self.logger.info("  - 's': 추적 중지 (대기 모드)")
        self.logger.info("  - 'q': 종료")

    def run(self):
        """메인 루프: 키 입력을 받아 PersonTracker의 모드를 변경하고, 결과를 화면에 표시합니다."""
        if not self.camera.is_running:
            return

        while True:
            _, frame = self.camera.get_frames()
            if frame is None:
                self.logger.warning("카메라에서 프레임을 받을 수 없습니다.")
                continue

            # PersonTracker에 프레임 전달 및 이벤트 수신
            tracking_events = self.person_tracker.process_frame(frame)
            for event in tracking_events:
                self.logger.info(f"추적 이벤트 수신: {event}")

            # 시각화된 오버레이 프레임 가져오기
            overlay_frame = self.person_tracker.get_overlay_frame(frame)
            cv2.imshow('Person Tracker Test', overlay_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                self.logger.info("'r' 입력: 등록 모드로 변경합니다.")
                self.person_tracker.set_mode(1)
                self.person_tracker.register_target(duration_sec=3.0)
            elif key == ord('t'):
                self.logger.info("'t' 입력: 추적 모드로 변경합니다.")
                self.person_tracker.set_mode(2)
            elif key == ord('s'):
                self.logger.info("'s' 입력: 대기 모드로 변경합니다.")
                self.person_tracker.set_mode(0)

        self.camera.cleanup()
        cv2.destroyAllWindows()

def main():
    # rclpy.init()을 사용하지 않으므로, get_logger가 기본 로거를 사용하도록 설정
    from rclpy.logging import set_logger_level
    set_logger_level('person_tracker_tester', 10) # DEBUG 레벨

    try:
        tester = PersonTrackerTester()
        tester.run()
    except Exception as e:
        print(f"테스트 실행 중 오류 발생: {e}")

if __name__ == '__main__':
    main()