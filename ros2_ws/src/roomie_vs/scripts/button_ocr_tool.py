#!/usr/bin/env python3
"""
Button OCR Tool (Monitoring Screen Version)
- YOLO-based elevator button detection
- EasyOCR text reading (multiple preprocessing methods applied)
- Real-time preprocessing results monitoring screen
- Combined original video and preprocessing results display
- Automatic button_id mapping
"""

import cv2
import numpy as np
import time
import os
import sys
from typing import Optional, List, Tuple

# EasyOCR import
try:
    import easyocr
    EASYOCR_AVAILABLE = True
except ImportError:
    EASYOCR_AVAILABLE = False
    print("EasyOCR is not installed. Please install it with: pip install easyocr")

# YOLO import
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Ultralytics YOLO is not installed.")


class ButtonOCRTool:
    """Simple Button OCR Tool"""
    
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
        
        # 모니터링 화면 설정
        self.show_preprocessing = True
        self.current_button_rois = []  # 현재 프레임의 버튼 ROI들
        self.preprocessed_results = []  # 전처리 결과들
        
    def initialize(self) -> bool:
        """시스템 초기화"""
        print("Initializing Button OCR Tool...")
        
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
        print("Initialization completed!")
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
                        print(f"Webcam initialized successfully (ID: {camera_id})")
                        return True
                    cap.release()
            
            print("No available webcam found")
            return False
            
        except Exception as e:
            print(f"Webcam initialization failed: {e}")
            return False
    
    def _init_yolo(self) -> bool:
        """YOLO 모델 초기화"""
        if not YOLO_AVAILABLE:
            print("YOLO not available")
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
                print("Elevator model not found")
                return False
            
            self.yolo_model = YOLO(model_path)
            
            # GPU 사용 시도
            try:
                self.yolo_model.to('cuda')
                print(f"YOLO model loaded (GPU): {model_path}")
            except:
                print(f"YOLO model loaded (CPU): {model_path}")
            
            return True
            
        except Exception as e:
            print(f"YOLO initialization failed: {e}")
            return False
    
    def _init_easyocr(self) -> bool:
        """EasyOCR 초기화"""
        if not EASYOCR_AVAILABLE:
            return False
        
        try:
            print("Initializing EasyOCR... (This may take a while on first run)")
            self.ocr_reader = easyocr.Reader(['en', 'ko'])
            print("EasyOCR initialization completed")
            return True
            
        except Exception as e:
            print(f"EasyOCR initialization failed: {e}")
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
            print(f"Button detection failed: {e}")
            return []
    
    def _preprocess_roi(self, roi: np.ndarray) -> List[np.ndarray]:
        """ROI 이미지 전처리 (여러 버전 생성) - 원형 버튼 감지 포함"""
        processed_images = []
        
        # 원형 버튼 감지
        circle_info = self._detect_circle_in_roi(roi)
        
        # 1. 원본
        processed_images.append(('Original', roi))
        
        # 2. 원형 ROI 적용된 원본 (감지된 경우만)
        if circle_info:
            circular_roi = self._apply_circular_roi(roi, circle_info)
            processed_images.append(('Circular_ROI', circular_roi))
            
            # 원형 ROI에 대한 이진화 처리들
            circular_binary_results = self._apply_circular_binarization(circular_roi)
            processed_images.extend(circular_binary_results)
        
        # 3. 그레이스케일
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        processed_images.append(('Grayscale', gray))
        
        # 4. 기본 이진화 방법들
        basic_binary_results = self._apply_basic_binarization(gray)
        processed_images.extend(basic_binary_results)
        
        # 최대 8개만 반환 (화면 공간 고려)
        return processed_images[:8]
    
    def _apply_circular_binarization(self, circular_roi: np.ndarray) -> List[tuple]:
        """원형 ROI에 특화된 이진화 처리"""
        binary_results = []
        
        # 그레이스케일 변환
        if len(circular_roi.shape) == 3:
            gray = cv2.cvtColor(circular_roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = circular_roi.copy()
        
        # 1. Otsu 이진화
        try:
            _, otsu_binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            binary_results.append(('Otsu_Binary', otsu_binary))
        except:
            pass
        
        # 2. 반전 + Otsu 이진화 (흰색 배경에 검은 글씨로)
        try:
            inverted = cv2.bitwise_not(gray)
            _, inv_otsu = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            binary_results.append(('Inv_Otsu', inv_otsu))
        except:
            pass
        
        # 3. 점자 제거 + Otsu (상단 영역만)
        try:
            top_region = self._extract_number_region(gray)
            _, top_otsu = cv2.threshold(top_region, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            binary_results.append(('Top_Otsu', top_otsu))
        except:
            pass
            
        # 4. 점자 제거 + 반전 Otsu (상단 영역만)
        try:
            top_region = self._extract_number_region(gray)
            top_inverted = cv2.bitwise_not(top_region)
            _, top_inv_otsu = cv2.threshold(top_inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            binary_results.append(('Top_Inv_Otsu', top_inv_otsu))
        except:
            pass
        
        # 5. 모폴로지 연산으로 점자 제거
        try:
            braille_removed = self._remove_braille_noise(gray)
            _, morph_otsu = cv2.threshold(braille_removed, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            binary_results.append(('Morph_Clean', morph_otsu))
        except:
            pass
        
        # 6. 가우시안 블러 + 적응적 임계값
        try:
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            adaptive_gauss = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            binary_results.append(('Blur_Adaptive', adaptive_gauss))
        except:
            pass
        
        return binary_results
    
    def _apply_basic_binarization(self, gray: np.ndarray) -> List[tuple]:
        """기본 이진화 처리"""
        binary_results = []
        
        # 1. 적응적 임계값
        try:
            adaptive = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            binary_results.append(('Adaptive', adaptive))
        except:
            pass
        
        # 2. 색상 반전
        try:
            inverted = cv2.bitwise_not(gray)
            binary_results.append(('Inverted', inverted))
        except:
            pass
        
        # 3. CLAHE (대비 개선)
        try:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            clahe_img = clahe.apply(gray)
            binary_results.append(('CLAHE', clahe_img))
        except:
            pass
        
        return binary_results
    
    def _update_button_preprocessing(self, image: np.ndarray, buttons: List[dict]):
        """Update preprocessing results for detected buttons"""
        self.current_button_rois = []
        self.preprocessed_results = []
        
        for i, button in enumerate(buttons):
            x, y, w, h = button['bbox']
            roi = image[y:y+h, x:x+w]
            
            if roi.size > 0:
                # Save ROI
                self.current_button_rois.append(roi)
                
                # Save preprocessing results
                processed_images = self._preprocess_roi(roi)
                self.preprocessed_results.append({
                    'index': i,
                    'bbox': button['bbox'],
                    'confidence': button['confidence'],
                    'processed': processed_images
                })
    
    def _resize_for_display(self, image: np.ndarray, target_size: Tuple[int, int]) -> np.ndarray:
        """Resize image for display"""
        target_w, target_h = target_size
        h, w = image.shape[:2]
        
        # Resize while maintaining aspect ratio
        scale = min(target_w / w, target_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        
        resized = cv2.resize(image, (new_w, new_h))
        
        # Add padding to center the image
        if len(image.shape) == 3:
            padded = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        else:
            padded = np.zeros((target_h, target_w), dtype=np.uint8)
        
        y_offset = (target_h - new_h) // 2
        x_offset = (target_w - new_w) // 2
        
        if len(image.shape) == 3:
            padded[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        else:
            padded[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        
        return padded
    
    def _create_monitoring_display(self, main_frame: np.ndarray) -> np.ndarray:
        """Create simple display - just return main frame"""
        return main_frame
    
    def ocr_button(self, image: np.ndarray, bbox: tuple) -> Optional[dict]:
        """버튼 영역 OCR (Otsu 방법 우선 시도)"""
        try:
            if self.ocr_reader is None:
                return None
            
            # ROI 추출
            x, y, w, h = bbox
            roi = image[y:y+h, x:x+w]
            
            if roi.size == 0:
                return None
            
            # 원형 ROI 감지
            circle_info = self._detect_circle_in_roi(roi)
            
            best_result = None
            best_confidence = 0.0
            best_method = ""
            
            # 1차: 원형 ROI가 감지된 경우 Otsu 방법들 우선 시도
            if circle_info:
                circular_roi = self._apply_circular_roi(roi, circle_info)
                otsu_methods = self._get_otsu_processed_images(circular_roi)
                
                for method_name, processed_roi in otsu_methods:
                    try:
                        results = self.ocr_reader.readtext(processed_roi)
                        
                        if results:
                            current_best = max(results, key=lambda x: x[2])
                            text = current_best[1].strip()
                            confidence = current_best[2]
                            
                            if confidence > best_confidence and text:
                                best_confidence = confidence
                                best_result = text
                                best_method = method_name
                                
                    except Exception as e:
                        continue
                
                # Otsu 방법으로 충분히 좋은 결과가 나왔으면 바로 반환
                if best_confidence > 0.7:  # 70% 이상이면 충분히 좋음
                    return {
                        'text': best_result,
                        'confidence': best_confidence,
                        'method': best_method + '_priority'
                    }
            
            # 2차: 모든 전처리 방법 시도 (백업용)
            processed_images = self._preprocess_roi(roi)
            
            for method_name, processed_roi in processed_images:
                try:
                    results = self.ocr_reader.readtext(processed_roi)
                    
                    if results:
                        current_best = max(results, key=lambda x: x[2])
                        text = current_best[1].strip()
                        confidence = current_best[2]
                        
                        if confidence > best_confidence and text:
                            best_confidence = confidence
                            best_result = text
                            best_method = method_name
                            
                except Exception as e:
                    continue
            
            if best_result:
                return {
                    'text': best_result,
                    'confidence': best_confidence,
                    'method': best_method
                }
            
            return None
            
        except Exception as e:
            print(f"OCR execution failed: {e}")
            return None
    
    def _get_otsu_processed_images(self, circular_roi: np.ndarray) -> List[tuple]:
        """Otsu 방법들만 추출 (점자 제거 포함)"""
        otsu_results = []
        
        # 그레이스케일 변환
        if len(circular_roi.shape) == 3:
            gray = cv2.cvtColor(circular_roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = circular_roi.copy()
        
        # 1. 점자 제거 + Otsu 이진화 (우선순위 1)
        try:
            top_region = self._extract_number_region(gray)
            _, top_otsu = cv2.threshold(top_region, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            otsu_results.append(('Top_Otsu', top_otsu))
        except:
            pass
        
        # 2. 점자 제거 + 반전 Otsu (우선순위 2)
        try:
            top_region = self._extract_number_region(gray)
            top_inverted = cv2.bitwise_not(top_region)
            _, top_inv_otsu = cv2.threshold(top_inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            otsu_results.append(('Top_Inv_Otsu', top_inv_otsu))
        except:
            pass
        
        # 3. 모폴로지 점자 제거 + Otsu (우선순위 3)
        try:
            braille_removed = self._remove_braille_noise(gray)
            _, morph_otsu = cv2.threshold(braille_removed, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            otsu_results.append(('Morph_Clean', morph_otsu))
        except:
            pass
        
        # 4. 기본 Otsu 이진화 (백업용)
        try:
            _, otsu_binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            otsu_results.append(('Otsu_Binary', otsu_binary))
        except:
            pass
        
        # 5. 기본 반전 + Otsu 이진화 (백업용)
        try:
            inverted = cv2.bitwise_not(gray)
            _, inv_otsu = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            otsu_results.append(('Inv_Otsu', inv_otsu))
        except:
            pass
        
        return otsu_results
    
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
        print("Button OCR Tool Running")
        print("="*50)
        print("Keyboard Controls:")
        print("  SPACE: Manual OCR")
        print("  'a': Auto mode ON/OFF")
        print("  '+/-': Adjust confidence")
        print("  'q': Quit")
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
                
                # 모니터링 화면 생성
                final_display = self._create_monitoring_display(display_frame)
                
                # 자동 모드 OCR
                current_time = time.time()
                if (self.auto_mode and buttons and 
                    current_time - self.last_test_time > self.test_interval):
                    
                    self._process_buttons(frame, buttons)
                    self.last_test_time = current_time
                
                # 화면 출력
                cv2.imshow('Button OCR Tool', final_display)
                
                # 키보드 처리
                key = cv2.waitKey(1) & 0xFF
                if not self._handle_keyboard(key, frame, buttons):
                    break
                    
        except KeyboardInterrupt:
            print("\nUser interrupted")
        except Exception as e:
            print(f"Runtime error: {e}")
        finally:
            self.cleanup()
    
    def _draw_buttons(self, image: np.ndarray, buttons: List[dict]) -> np.ndarray:
        """버튼 시각화 + OCR 결과 오버레이"""
        display_image = image.copy()
        
        for i, button in enumerate(buttons):
            x, y, w, h = button['bbox']
            conf = button['confidence']
            
            # 바운딩박스 (마젠타색)
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (255, 0, 255), 2)
            
            # OCR 결과를 실시간으로 가져와서 오버레이
            try:
                ocr_result = self.ocr_button(image, button['bbox'])
                if ocr_result:
                    text = ocr_result['text']
                    ocr_conf = ocr_result['confidence']
                    button_id = self.map_text_to_button_id(text)
                    
                    # OCR 결과 표시 (버튼 중앙에 큰 글씨로)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # 배경 박스 (반투명)
                    overlay = display_image.copy()
                    cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 0), -1)
                    cv2.addWeighted(overlay, 0.3, display_image, 0.7, 0, display_image)
                    
                    # 인식된 글씨 (중앙, 큰 글씨)
                    cv2.putText(display_image, f"{text}", 
                               (center_x - 15, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                    
                    # Button ID (작은 글씨, 우상단)
                    if button_id:
                        cv2.putText(display_image, f"ID:{button_id}", 
                                   (x + w - 40, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    
                    # OCR 신뢰도 (작은 글씨, 좌상단)
                    cv2.putText(display_image, f"{ocr_conf:.2f}", 
                               (x + 5, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                else:
                    # OCR 실패시
                    center_y = y + h // 2
                    cv2.putText(display_image, "?", 
                               (x + w//2 - 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            except:
                pass
            
            # 버튼 번호 (좌하단)
            cv2.putText(display_image, f"#{i+1}", 
                       (x, y + h - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # 상태 정보 (간단하게)
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
                print("No button detected")
        
        elif key == ord('a'):
            self.auto_mode = not self.auto_mode
            print(f"Auto mode: {'ON' if self.auto_mode else 'OFF'}")
        

        
        elif key == ord('+') or key == ord('='):
            self.confidence_threshold = min(0.95, self.confidence_threshold + 0.05)
            print(f"Confidence: {self.confidence_threshold:.2f}")
        
        elif key == ord('-'):
            self.confidence_threshold = max(0.1, self.confidence_threshold - 0.05)
            print(f"Confidence: {self.confidence_threshold:.2f}")
        
        return True
    
    def _process_buttons(self, image: np.ndarray, buttons: List[dict]):
        """버튼 OCR 처리"""
        if not buttons:
            return
        
        print(f"\nProcessing OCR for {len(buttons)} buttons...")
        
        results = []
        for i, button in enumerate(buttons):
            bbox = button['bbox']
            conf = button['confidence']
            
            # OCR 실행
            ocr_result = self.ocr_button(image, bbox)
            
            if ocr_result:
                text = ocr_result['text']
                ocr_conf = ocr_result['confidence']
                method = ocr_result.get('method', 'default')
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
                    'method': 'failed'
                })
        
        # 결과 출력 (층수 버튼만)
        floor_results = [r for r in results if r['button_id'] is not None and 1 <= r['button_id'] <= 14]
        
        if floor_results:
            print("Floor button OCR results:")
            for result in floor_results:
                idx = result['index']
                text = result['text']
                btn_id = result['button_id']
                det_conf = result['detection_conf']
                ocr_conf = result['ocr_conf']
                method = result['method']
                
                print(f"  #{idx}: '{text}' -> button_id={btn_id} (detection:{det_conf:.2f}, OCR:{ocr_conf:.2f}, {method})")
        else:
            print("No floor button recognition results")
        
        print("-" * 40)
    
    def cleanup(self):
        """리소스 정리"""
        self.is_running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("Cleanup completed")

    def _detect_circle_in_roi(self, roi: np.ndarray) -> Optional[tuple]:
        """ROI에서 원형 버튼 감지"""
        try:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
            
            # 노이즈 제거
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            
            # HoughCircles로 원 감지
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=max(roi.shape[0], roi.shape[1]) // 4,
                param1=50,
                param2=30,
                minRadius=min(roi.shape[0], roi.shape[1]) // 6,
                maxRadius=min(roi.shape[0], roi.shape[1]) // 2
            )
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                # 가장 큰 원 선택 (대부분의 경우 버튼)
                if len(circles) > 0:
                    largest_circle = max(circles, key=lambda c: c[2])  # 반지름 기준
                    return tuple(largest_circle)  # (x, y, radius)
            
            return None
            
        except Exception as e:
            return None
    
    def _create_circular_mask(self, shape: tuple, center: tuple, radius: int) -> np.ndarray:
        """원형 마스크 생성"""
        h, w = shape[:2]
        y, x = np.ogrid[:h, :w]
        cx, cy = center
        
        # 원형 마스크 (테두리 많이 제거)
        mask_radius = max(1, int(radius * 0.75))  # 반지름의 65%만 사용
        mask = (x - cx) ** 2 + (y - cy) ** 2 <= mask_radius ** 2
        
        return mask.astype(np.uint8) * 255
    
    def _apply_circular_roi(self, roi: np.ndarray, circle_info: tuple) -> np.ndarray:
        """원형 ROI 적용"""
        try:
            cx, cy, radius = circle_info
            
            # 원형 마스크 생성
            mask = self._create_circular_mask(roi.shape, (cx, cy), radius)
            
            # 글씨 색상 감지 (밝은 부분과 어두운 부분 분석)
            if len(roi.shape) == 3:
                gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            else:
                gray_roi = roi.copy()
            
            # 마스크 영역 내부에서 글씨 색상 감지
            button_area = gray_roi[mask > 0]
            if len(button_area) > 0:
                # 밝은 픽셀(흰색 글씨 후보)과 어두운 픽셀 비율 계산
                bright_pixels = np.sum(button_area > 200)  # 밝은 픽셀
                dark_pixels = np.sum(button_area < 100)    # 어두운 픽셀
                total_pixels = len(button_area)
                
                bright_ratio = bright_pixels / total_pixels
                dark_ratio = dark_pixels / total_pixels
                
                # 글씨가 흰색인지 검은색인지 판단
                is_white_text = bright_ratio > 0.1 and bright_ratio > dark_ratio
            else:
                is_white_text = False  # 기본값
            
            # 글씨만 추출해서 배경 설정
            if len(roi.shape) == 3:
                # 컬러 이미지 처리
                gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                
                if is_white_text:
                    # 흰색 글씨 추출: 적응형 이진화 사용
                    # 가우시안 적응형 이진화 (흰색 글씨를 위해 THRESH_BINARY 사용)
                    text_mask = cv2.adaptiveThreshold(gray_roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, -10)
                    text_mask = cv2.bitwise_and(text_mask, mask)  # 원형 마스크와 결합
                    
                    # 모폴로지 연산으로 글씨 빈 공간 채우기
                    kernel = np.ones((3,3), np.uint8)
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_CLOSE, kernel)  # 구멍 메우기
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_DILATE, kernel, iterations=1)  # 글씨 두껍게
                    
                    # 결과 이미지 생성 (검은 배경)
                    masked = np.zeros_like(roi)
                    masked[text_mask > 0] = [255, 255, 255]  # 글씨는 흰색
                else:
                    # 검은색 글씨 추출: 적응형 이진화 사용
                    # 가우시안 적응형 이진화 (검은색 글씨를 위해 THRESH_BINARY_INV 사용)
                    text_mask = cv2.adaptiveThreshold(gray_roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 10)
                    text_mask = cv2.bitwise_and(text_mask, mask)  # 원형 마스크와 결합
                    
                    # 모폴로지 연산으로 글씨 빈 공간 채우기
                    kernel = np.ones((3,3), np.uint8)
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_CLOSE, kernel)  # 구멍 메우기
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_DILATE, kernel, iterations=1)  # 글씨 두껍게
                    
                    # 결과 이미지 생성 (흰 배경)
                    masked = np.full_like(roi, [255, 255, 255])
                    masked[text_mask > 0] = [0, 0, 0]  # 글씨는 검은색
            else:
                # 그레이스케일 이미지 처리
                if is_white_text:
                    # 흰색 글씨 추출: 적응형 이진화 사용
                    # 가우시안 적응형 이진화 (흰색 글씨를 위해 THRESH_BINARY 사용)
                    text_mask = cv2.adaptiveThreshold(gray_roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, -10)
                    text_mask = cv2.bitwise_and(text_mask, mask)  # 원형 마스크와 결합
                    
                    # 모폴로지 연산으로 글씨 빈 공간 채우기
                    kernel = np.ones((3,3), np.uint8)
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_CLOSE, kernel)  # 구멍 메우기
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_DILATE, kernel, iterations=1)  # 글씨 두껍게
                    
                    # 결과 이미지 생성 (검은 배경)
                    masked = np.zeros_like(gray_roi)
                    masked[text_mask > 0] = 255  # 글씨는 흰색
                else:
                    # 검은색 글씨 추출: 적응형 이진화 사용
                    # 가우시안 적응형 이진화 (검은색 글씨를 위해 THRESH_BINARY_INV 사용)
                    text_mask = cv2.adaptiveThreshold(gray_roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 10)
                    text_mask = cv2.bitwise_and(text_mask, mask)  # 원형 마스크와 결합
                    
                    # 모폴로지 연산으로 글씨 빈 공간 채우기
                    kernel = np.ones((3,3), np.uint8)
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_CLOSE, kernel)  # 구멍 메우기
                    text_mask = cv2.morphologyEx(text_mask, cv2.MORPH_DILATE, kernel, iterations=1)  # 글씨 두껍게
                    
                    # 결과 이미지 생성 (흰 배경)
                    masked = np.full_like(gray_roi, 255)
                    masked[text_mask > 0] = 0  # 글씨는 검은색
            
            return masked
            
        except Exception as e:
            return roi

    def _extract_number_region(self, gray: np.ndarray) -> np.ndarray:
        """숫자 영역만 추출 (상단 60% 영역)"""
        try:
            h, w = gray.shape
            # 버튼의 상단 60% 영역만 사용 (숫자는 보통 위쪽에 있음)
            top_region = gray[:int(h * 0.6), :]
            
            # 크기가 너무 작아지지 않도록 최소 크기 보장
            if top_region.shape[0] < 20 or top_region.shape[1] < 20:
                return gray
            
            return top_region
        except:
            return gray
    
    def _remove_braille_noise(self, gray: np.ndarray) -> np.ndarray:
        """점자 노이즈 제거"""
        try:
            # 1. 작은 점들 제거를 위한 모폴로지 연산
            kernel_small = np.ones((2, 2), np.uint8)
            
            # Opening 연산으로 작은 점들 제거
            opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel_small)
            
            # 2. 큰 구조물(숫자) 보존을 위한 Closing
            kernel_large = np.ones((3, 3), np.uint8)
            cleaned = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel_large)
            
            # 3. 가우시안 블러로 점자의 작은 특징 제거
            blurred = cv2.GaussianBlur(cleaned, (3, 3), 0)
            
            return blurred
        except:
            return gray


def main():
    if not EASYOCR_AVAILABLE or not YOLO_AVAILABLE:
        print("❌ Required libraries are not installed")
        return -1
    
    print("🚀 Button OCR Tool")
    print("   - EasyOCR + YOLO based button detection and OCR")
    print("   - Real-time OCR results overlay on original video")
    print("   - Automatic button_id mapping")
    
    tool = ButtonOCRTool()
    
    if not tool.initialize():
        print("❌ Initialization failed")
        return -1
    
    tool.run()
    print("✅ Program ended")
    return 0


if __name__ == "__main__":
    exit(main()) 