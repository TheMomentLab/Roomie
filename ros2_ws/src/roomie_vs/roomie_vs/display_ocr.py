import cv2
import numpy as np
from rclpy.logging import Logger

try:
    import easyocr
    EASYOCR_AVAILABLE = True
except ImportError:
    EASYOCR_AVAILABLE = False

class DisplayOCR:
    """EasyOCR을 사용하여 이미지에서 텍스트(주로 숫자)를 인식하는 클래스입니다."""

    def __init__(self, logger: Logger):
        """
        DisplayOCR 초기화

        Args:
            logger (Logger): ROS2 로거 객체
        """
        self.logger = logger
        self.reader = None
        self.is_enabled = False

    def enable_ocr(self, use_gpu: bool = True):
        """OCR 기능을 활성화하고, 필요 시 EasyOCR 리더를 초기화합니다."""
        if not EASYOCR_AVAILABLE:
            self.logger.warning("easyocr 라이브러리를 찾을 수 없습니다. OCR 기능이 비활성화됩니다.")
            self.logger.warning("설치 명령어: pip install easyocr")
            return

        if self.reader is None:
            try:
                # 영어와 한국어를 인식 언어로 설정
                self.reader = easyocr.Reader(['en', 'ko'], gpu=use_gpu)
                self.logger.info(f"EasyOCR 리더가 성공적으로 초기화되었습니다 (GPU: {use_gpu}).")
            except Exception as e:
                self.logger.error(f"EasyOCR 리더 초기화에 실패했습니다: {e}")
                return
        
        self.is_enabled = True
        self.logger.info("OCR 기능이 활성화되었습니다.")

    def disable_ocr(self):
        """OCR 기능을 비활성화합니다."""
        self.is_enabled = False
        self.logger.info("OCR 기능이 비활성화되었습니다.")

    def recognize_text(self, image: np.ndarray) -> list:
        """
        주어진 이미지에서 텍스트를 인식하여 경계 상자, 텍스트, 신뢰도를 포함한 리스트를 반환합니다.

        Args:
            image (np.ndarray): 텍스트를 인식할 이미지 (OpenCV BGR 포맷)

        Returns:
            list: 각 텍스트에 대한 정보를 담은 딕셔너리 리스트.
                  예: [{'bbox': [x1, y1, x2, y2], 'text': '12', 'confidence': 0.95}]
        """
        if not self.is_enabled or self.reader is None:
            return []

        try:
            # EasyOCR은 RGB 이미지를 입력으로 받으므로 변환이 필요
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.reader.readtext(rgb_image)
            
            ocr_results = []
            for (bbox, text, confidence) in results:
                # bbox는 [[x1, y1], [x2, y1], [x2, y2], [x1, y2]] 형식이므로, [x1, y1, x2, y2]로 변환
                top_left = bbox[0]
                bottom_right = bbox[2]
                x1, y1 = map(int, top_left)
                x2, y2 = map(int, bottom_right)
                
                ocr_results.append({
                    'bbox': [x1, y1, x2, y2],
                    'text': text,
                    'confidence': float(confidence)
                })
            
            return ocr_results

        except Exception as e:
            self.logger.error(f"OCR 텍스트 인식 중 오류 발생: {e}")
            return []