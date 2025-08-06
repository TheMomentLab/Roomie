#!/usr/bin/env python3
"""
간단한 버튼 OCR 도구 (EasyOCR 전용)
엘리베이터 버튼만 감지하고 EasyOCR로 텍스트를 읽어서 button_id 매핑
"""

import cv2
import numpy as np
import time
import os
import sys
from typing import Optional, List

# EasyOCR import
try:
    import easyocr
    EASYOCR_AVAILABLE = True
except ImportError:
    EASYOCR_AVAILABLE = False
    print("❌ EasyOCR이 설치되지 않았습니다. pip install easyocr 로 설치하세요.")

# YOLO import
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("❌ Ultralytics YOLO가 설치되지 않았습니다.")


class ButtonOCRTool:
    """간단한 버튼 OCR 도구"""
    
    def __init__(self):
        self.cap = None
        self.yolo_model = None
        self.ocr_reader = None
        self.is_running = False
        
        # 설정
        self.confidence_threshold = 0.5
        self.auto_mode = True
        self.last_test_time = 0
        self.test_interval = 1.0  # 1초마다
        
        # YOLO 클래스 (elevator 모델 기준)
        self.model_classes = ['button', 'direction_light', 'display', 'door']
        
    def initialize(self) -> bool:
        """시스템 초기화"""
        print("🚀 Button OCR 도구 초기화...")
        
        # 1. 웹캠 초기화
        if not self._init_webcam():
            return False
        
        # 2. YOLO 모델 초기화
        if not self._init_yolo():
            return False
        
        # 3. EasyOCR 초기화
        if not self._init_easyocr():
            return False
        
        self.is_running = True
        print("✅ 초기화 완료!")
        return True
    
    def _init_webcam(self) -> bool:
        """웹캠 초기화"""
        try:
            # 사용 가능한 카메라 찾기
            for camera_id in [0, 1, 2, 3]:
                cap = cv2.VideoCapture(camera_id)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.cap = cap
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        print(f"✅ 웹캠 초기화 완료 (ID: {camera_id})")
                        return True
                    cap.release()
            
            print("❌ 사용 가능한 웹캠을 찾을 수 없습니다")
            return False
            
        except Exception as e:
            print(f"❌ 웹캠 초기화 실패: {e}")
            return False
    
    def _init_yolo(self) -> bool:
        """YOLO 모델 초기화"""
        if not YOLO_AVAILABLE:
            print("❌ YOLO 사용 불가")
            return False
        
        try:
            # elevator 모델 찾기
            model_paths = [
                "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_vs/training/elevator/best.pt",
                "training/elevator/best.pt",
                "../training/elevator/best.pt"
            ]
            
            model_path = None
            for path in model_paths:
                if os.path.exists(path):
                    model_path = path
                    break
            
            if not model_path:
                print("❌ Elevator 모델을 찾을 수 없습니다")
                return False
            
            self.yolo_model = YOLO(model_path)
            
            # GPU 사용 시도
            try:
                self.yolo_model.to('cuda')
                print(f"✅ YOLO 모델 로딩 (GPU): {model_path}")
            except:
                print(f"✅ YOLO 모델 로딩 (CPU): {model_path}")
            
            return True
            
        except Exception as e:
            print(f"❌ YOLO 초기화 실패: {e}")
            return False
    
    def _init_easyocr(self) -> bool:
        """EasyOCR 초기화"""
        if not EASYOCR_AVAILABLE:
            return False
        
        try:
            print("⏳ EasyOCR 초기화 중... (최초 실행 시 시간이 걸립니다)")
            self.ocr_reader = easyocr.Reader(['en', 'ko'])
            print("✅ EasyOCR 초기화 완료")
            return True
            
        except Exception as e:
            print(f"❌ EasyOCR 초기화 실패: {e}")
            return False
    
    def detect_buttons(self, image: np.ndarray) -> List[dict]:
        """버튼 객체 감지"""
        try:
            if self.yolo_model is None:
                return []
            
            results = self.yolo_model(image, conf=self.confidence_threshold, verbose=False)
            
            buttons = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        cls_id = int(box.cls.cpu().numpy())
                        if cls_id < len(self.model_classes):
                            class_name = self.model_classes[cls_id]
                        else:
                            class_name = f"class_{cls_id}"
                        
                        # 버튼만 필터링
                        if class_name == 'button':
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            buttons.append({
                                'bbox': (int(x1), int(y1), int(x2-x1), int(y2-y1)),
                                'confidence': float(box.conf.cpu().numpy())
                            })
            
            return buttons
            
        except Exception as e:
            print(f"❌ 버튼 감지 실패: {e}")
            return []
    
    def _preprocess_roi(self, roi: np.ndarray) -> List[np.ndarray]:
        """ROI 이미지 전처리 (여러 버전 생성)"""
        processed_images = []
        
        # 1. 원본
        processed_images.append(('원본', roi))
        
        # 2. 그레이스케일
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        processed_images.append(('그레이', gray))
        
        # 3. 적응적 임계값 (흰색 글씨 → 검은색 글씨)
        adaptive = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        processed_images.append(('적응임계', adaptive))
        
        # 4. 색상 반전 (흰색 글씨 → 검은색 글씨)
        inverted = cv2.bitwise_not(gray)
        processed_images.append(('색상반전', inverted))
        
        # 5. CLAHE (대비 개선)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        clahe_img = clahe.apply(gray)
        processed_images.append(('대비개선', clahe_img))
        
        # 6. 색상반전 + 적응임계
        inv_adaptive = cv2.adaptiveThreshold(inverted, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        processed_images.append(('반전+임계', inv_adaptive))
        
        return processed_images
    
    def ocr_button(self, image: np.ndarray, bbox: tuple) -> Optional[dict]:
        """버튼 영역 OCR (다중 전처리 시도)"""
        try:
            if self.ocr_reader is None:
                return None
            
            # ROI 추출
            x, y, w, h = bbox
            roi = image[y:y+h, x:x+w]
            
            if roi.size == 0:
                return None
            
            # 다양한 전처리 시도
            processed_images = self._preprocess_roi(roi)
            
            best_result = None
            best_confidence = 0.0
            best_method = ""
            
            for method_name, processed_roi in processed_images:
                try:
                    # EasyOCR 실행
                    results = self.ocr_reader.readtext(processed_roi)
                    
                    if results:
                        # 가장 신뢰도 높은 결과
                        current_best = max(results, key=lambda x: x[2])
                        text = current_best[1].strip()
                        confidence = current_best[2]
                        
                        # 더 좋은 결과면 업데이트
                        if confidence > best_confidence and text:
                            best_confidence = confidence
                            best_result = text
                            best_method = method_name
                            
                except Exception as e:
                    continue  # 이 전처리 방법은 실패, 다음으로
            
            if best_result:
                return {
                    'text': best_result,
                    'confidence': best_confidence,
                    'method': best_method
                }
            
            return None
            
        except Exception as e:
            print(f"❌ OCR 실행 실패: {e}")
            return None
    
    def map_text_to_button_id(self, text: str) -> Optional[int]:
        """텍스트를 button_id로 매핑"""
        if not text:
            return None
        
        text = text.strip().upper()
        
        # 층수 버튼
        floor_mapping = {
            '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6,
            '7': 7, '8': 8, '9': 9, '10': 10, '11': 11, '12': 12,
            'B1': 13, 'B2': 14
        }
        
        # 방향 버튼
        direction_mapping = {
            '▼': 100, '↓': 100, 'DOWN': 100,
            '▲': 101, '↑': 101, 'UP': 101,
            '◀': 102, 'OPEN': 102,
            '▶': 103, 'CLOSE': 103
        }
        
        # 매핑 시도
        if text in direction_mapping:
            return direction_mapping[text]
        
        if text in floor_mapping:
            return floor_mapping[text]
        
        # 숫자만 있는 경우
        try:
            num = int(text)
            if 1 <= num <= 12:
                return num
        except:
            pass
        
        return None
    
    def run(self):
        """메인 실행 루프"""
        print("\n" + "="*50)
        print("🔘 Button OCR 도구 실행")
        print("="*50)
        print("키보드 컨트롤:")
        print("  SPACE: 수동 OCR")
        print("  'a': 자동 모드 ON/OFF")
        print("  '+/-': 신뢰도 조절")
        print("  'q': 종료")
        print("="*50)
        
        try:
            while self.is_running:
                # 프레임 획득
                ret, frame = self.cap.read()
                if not ret:
                    continue
                
                # 버튼 감지
                buttons = self.detect_buttons(frame)
                
                # 시각화
                display_frame = self._draw_buttons(frame, buttons)
                
                # 자동 모드 OCR
                current_time = time.time()
                if (self.auto_mode and buttons and 
                    current_time - self.last_test_time > self.test_interval):
                    
                    self._process_buttons(frame, buttons)
                    self.last_test_time = current_time
                
                # 화면 출력
                cv2.imshow('Button OCR Tool', display_frame)
                
                # 키보드 처리
                key = cv2.waitKey(1) & 0xFF
                if not self._handle_keyboard(key, frame, buttons):
                    break
                    
        except KeyboardInterrupt:
            print("\n사용자 중단")
        except Exception as e:
            print(f"❌ 실행 오류: {e}")
        finally:
            self.cleanup()
    
    def _draw_buttons(self, image: np.ndarray, buttons: List[dict]) -> np.ndarray:
        """버튼 시각화"""
        display_image = image.copy()
        
        for button in buttons:
            x, y, w, h = button['bbox']
            conf = button['confidence']
            
            # 바운딩박스 (마젠타색)
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
            
            # 라벨
            label = f"BUTTON {conf:.2f}"
            cv2.putText(display_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # 상태 정보
        info_y = 30
        cv2.putText(display_image, f"Confidence: {self.confidence_threshold:.2f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        mode_text = "AUTO" if self.auto_mode else "MANUAL"
        cv2.putText(display_image, f"Mode: {mode_text}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        cv2.putText(display_image, f"Buttons: {len(buttons)}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return display_image
    
    def _handle_keyboard(self, key: int, frame: np.ndarray, buttons: List[dict]) -> bool:
        """키보드 입력 처리"""
        if key == ord('q'):
            return False
        
        elif key == ord(' '):  # SPACE
            if buttons:
                self._process_buttons(frame, buttons)
            else:
                print("버튼이 감지되지 않았습니다")
        
        elif key == ord('a'):
            self.auto_mode = not self.auto_mode
            print(f"자동 모드: {'ON' if self.auto_mode else 'OFF'}")
        
        elif key == ord('+') or key == ord('='):
            self.confidence_threshold = min(0.95, self.confidence_threshold + 0.05)
            print(f"신뢰도: {self.confidence_threshold:.2f}")
        
        elif key == ord('-'):
            self.confidence_threshold = max(0.1, self.confidence_threshold - 0.05)
            print(f"신뢰도: {self.confidence_threshold:.2f}")
        
        return True
    
    def _process_buttons(self, image: np.ndarray, buttons: List[dict]):
        """버튼 OCR 처리"""
        if not buttons:
            return
        
        print(f"\n🔘 {len(buttons)}개 버튼 OCR 처리 중...")
        
        results = []
        for i, button in enumerate(buttons):
            bbox = button['bbox']
            conf = button['confidence']
            
            # OCR 실행
            ocr_result = self.ocr_button(image, bbox)
            
            if ocr_result:
                text = ocr_result['text']
                ocr_conf = ocr_result['confidence']
                method = ocr_result.get('method', '기본')
                button_id = self.map_text_to_button_id(text)
                
                results.append({
                    'index': i + 1,
                    'text': text,
                    'button_id': button_id,
                    'detection_conf': conf,
                    'ocr_conf': ocr_conf,
                    'method': method
                })
            else:
                results.append({
                    'index': i + 1,
                    'text': '?',
                    'button_id': None,
                    'detection_conf': conf,
                    'ocr_conf': 0.0,
                    'method': '실패'
                })
        
        # 결과 출력 (층수 버튼만)
        floor_results = [r for r in results if r['button_id'] is not None and 1 <= r['button_id'] <= 14]
        
        if floor_results:
            print("📋 층수 버튼 OCR 결과:")
            for result in floor_results:
                idx = result['index']
                text = result['text']
                btn_id = result['button_id']
                det_conf = result['detection_conf']
                ocr_conf = result['ocr_conf']
                method = result['method']
                
                print(f"  #{idx}: '{text}' → button_id={btn_id} (감지:{det_conf:.2f}, OCR:{ocr_conf:.2f}, {method})")
        else:
            print("📋 층수 버튼 인식 결과 없음")
        
        print("-" * 40)
    
    def cleanup(self):
        """리소스 정리"""
        self.is_running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("✅ 정리 완료")


def main():
    if not EASYOCR_AVAILABLE or not YOLO_AVAILABLE:
        print("❌ 필요한 라이브러리가 설치되지 않았습니다")
        return -1
    
    print("🚀 간단한 Button OCR 도구 시작")
    print("   - EasyOCR 전용")
    print("   - Button 객체만 감지")
    print("   - 자동 button_id 매핑")
    
    tool = ButtonOCRTool()
    
    if not tool.initialize():
        print("❌ 초기화 실패")
        return -1
    
    tool.run()
    print("👋 종료")
    return 0


if __name__ == "__main__":
    exit(main()) 