import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from roomie_vs.model_manager import MultiModelDetector, CNNButtonClassifier, ButtonPressedCNN
from roomie_vs.camera_manager import WebCamCamera

class ButtonOCRTool(Node):
    """웹캠을 사용하여 버튼 감지 및 분류 모델을 테스트하는 도구입니다."""

    def __init__(self):
        super().__init__('button_ocr_tool')
        self.logger = self.get_logger()
        
        # 설정 및 모델 경로 설정
        package_share_directory = get_package_share_directory('roomie_vs')
        config_path = os.path.join(package_share_directory, 'config', 'vision_config.yaml')
        
        # 웹캠 초기화
        self.camera = WebCamCamera(self.logger, camera_id=0)
        if not self.camera.initialize():
            self.logger.error("카메라를 열 수 없습니다. 프로그램을 종료합니다.")
            return

        # 모델 로드
        self.model_detector = MultiModelDetector(self.logger, self._load_config(config_path))
        self.cnn_button_classifier = CNNButtonClassifier(self.logger, 
            os.path.join(package_share_directory, 'training', 'button_cnn', 'best_smart_balanced_model_32px_with_metadata.pth'),
            os.path.join(package_share_directory, 'training', 'button_cnn', 'best_smart_balanced_model_32px_with_metadata_config.yaml'))
        self.button_pressed_cnn = ButtonPressedCNN(self.logger, 
            os.path.join(package_share_directory, 'training', 'button_pressed_cnn', 'best_roomie_button_model_32px_with_metadata.pth'))
        
        # YOLO 모델에 CNN 분류기 연결
        self.model_detector.set_button_pressed_cnn(self.button_pressed_cnn)
        self.model_detector.set_model_for_mode(3) # 엘리베이터 모드로 설정

        self.logger.info("버튼 OCR 테스트 도구가 준비되었습니다. 'q'를 눌러 종료합니다.")

    def _load_config(self, config_path):
        import yaml
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def run(self):
        """메인 루프: 카메라에서 프레임을 받아와 버튼을 감지하고 결과를 화면에 표시합니다."""
        if not self.camera.is_running:
            return

        while rclpy.ok():
            _, color_frame = self.camera.get_frames()
            if color_frame is None:
                self.logger.warning("카메라에서 프레임을 받을 수 없습니다.")
                continue

            # YOLO로 객체(버튼 포함) 감지
            detected_objects = self.model_detector.detect_objects(color_frame, None, conf_threshold=0.6)
            
            # 감지된 각 버튼에 대해 CNN으로 추가 분류 수행
            for obj in detected_objects:
                if obj['is_button']:
                    button_bbox = obj['bbox']
                    # CNN으로 버튼 종류 분류 (예: '1', '열림')
                    cnn_result = self.cnn_button_classifier.classify_button(color_frame, button_bbox)
                    
                    # 화면에 결과 표시
                    self._draw_button_info(color_frame, button_bbox, cnn_result, obj)

            cv2.imshow('Button OCR Test', color_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.camera.cleanup()
        cv2.destroyAllWindows()

    def _draw_button_info(self, frame, bbox, cnn_result, yolo_obj):
        """감지 및 분류된 버튼 정보를 프레임에 그립니다."""
        x1, y1, x2, y2 = bbox
        
        # YOLO가 감지한 경계 상자
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(frame, f"YOLO: {yolo_obj['confidence']:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        if cnn_result:
            button_id = cnn_result['button_id']
            confidence = cnn_result['confidence']
            text = f"CNN: {button_id} ({confidence:.2f})"
            
            # CNN 분류 결과 표시
            cv2.putText(frame, text, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 버튼 눌림 상태 확인 및 표시
            if yolo_obj.get('is_pressed') is not None:
                pressed_text = "Pressed" if yolo_obj['is_pressed'] else "Unpressed"
                pressed_conf = yolo_obj['pressed_confidence']
                pressed_color = (0, 0, 255) if yolo_obj['is_pressed'] else (255, 255, 0)
                cv2.putText(frame, f"{pressed_text} ({pressed_conf:.2f})", (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, pressed_color, 2)

def main(args=None):
    rclpy.init(args=args)
    try:
        button_ocr_tool = ButtonOCRTool()
        button_ocr_tool.run()
    except Exception as e:
        print(f"프로그램 실행 중 오류 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()