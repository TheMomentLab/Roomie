import rclpy
from rclpy.node import Node
import cv2
from roomie_vs.camera_manager import WebCamCamera
from roomie_vs.display_ocr import DisplayOCR

class OCRTestTool(Node):
    """웹캠을 사용하여 DisplayOCR 클래스의 텍스트 인식 기능을 테스트하는 도구입니다."""

    def __init__(self):
        super().__init__('ocr_test_tool')
        self.logger = self.get_logger()
        
        # 웹캠 초기화 (사용 가능한 첫 번째 카메라 사용)
        self.camera = WebCamCamera(self.logger, camera_id=0)
        if not self.camera.initialize():
            self.logger.error("카메라를 열 수 없습니다. 프로그램을 종료합니다.")
            return

        # DisplayOCR 초기화 및 활성화
        self.display_ocr = DisplayOCR(self.logger)
        self.display_ocr.enable_ocr(use_gpu=True)

        self.logger.info("OCR 테스트 도구가 준비되었습니다. 'q'를 눌러 종료합니다.")

    def run(self):
        """메인 루프: 카메라 프레임을 받아와 OCR을 수행하고 결과를 화면에 표시합니다."""
        if not self.camera.is_running:
            return

        while rclpy.ok():
            _, frame = self.camera.get_frames()
            if frame is None:
                self.logger.warning("카메라에서 프레임을 받을 수 없습니다.")
                continue

            # 현재 프레임에서 텍스트 인식
            ocr_results = self.display_ocr.recognize_text(frame)

            # 인식된 텍스트와 경계 상자를 화면에 그림
            for result in ocr_results:
                bbox = result['bbox']
                text = result['text']
                confidence = result['confidence']
                
                # 경계 상자 그리기
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                # 텍스트와 신뢰도 표시
                label = f"{text} ({confidence:.2f})"
                cv2.putText(frame, label, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow('OCR Test', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.camera.cleanup()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    try:
        ocr_test_tool = OCRTestTool()
        ocr_test_tool.run()
    except Exception as e:
        print(f"프로그램 실행 중 오류 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()