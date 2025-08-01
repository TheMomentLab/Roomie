#!/usr/bin/env python3
"""
OCR 모델 테스트 도구 (웹캠 전용)
vs_node와 동일한 구조로 웹캠에서 ROI를 추출하여 다양한 OCR 모델을 테스트하는 스크립트
"""

import cv2
import numpy as np
import time
import os
import sys
import logging
from typing import Optional, Tuple, List, Dict

# display_ocr.py 직접 import
script_dir = os.path.dirname(os.path.abspath(__file__))
roomie_vs_dir = os.path.join(script_dir, "..", "roomie_vs")
display_ocr_path = os.path.join(roomie_vs_dir, "display_ocr.py")

import importlib.util
spec = importlib.util.spec_from_file_location("display_ocr", display_ocr_path)
display_ocr_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(display_ocr_module)

MultiModelOCR = display_ocr_module.MultiModelOCR


class Logger:
    """간단한 로거 클래스"""
    def __init__(self, name="OCRTestTool"):
        self.name = name
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(name)
    
    def info(self, msg):
        self.logger.info(msg)
        print(f"[INFO] {msg}")
    
    def warning(self, msg):
        self.logger.warning(msg)
        print(f"[WARN] {msg}")
    
    def error(self, msg):
        self.logger.error(msg)
        print(f"[ERROR] {msg}")
    
    def debug(self, msg):
        self.logger.debug(msg)


class WebcamManager:
    """웹캠 관리 클래스 (vs_node의 WebCamCamera 기반)"""
    
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    def __init__(self, logger, camera_id=0, prefer_front_camera=True):
=======
    def __init__(self, logger, camera_id=0):
>>>>>>> Stashed changes
=======
    def __init__(self, logger, camera_id=0):
>>>>>>> Stashed changes
        self.logger = logger
        self.camera_id = camera_id
        self.cap = None
        self.is_running = False
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        self.prefer_front_camera = prefer_front_camera
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        
        # vs_node와 동일한 설정
        self.camera_ids_to_try = [0, 1, 2, 3]
        
    def initialize(self) -> bool:
        """웹캠 초기화 (vs_node의 WebCamCamera 방식)"""
        try:
            self.logger.info(f"웹캠 초기화 시작...")
            
            # 사용 가능한 카메라 스캔
            available_cameras = self._scan_available_cameras()
            
            if not available_cameras:
                self.logger.error("사용 가능한 웹캠을 찾을 수 없습니다")
                return False
            
            # 첫 번째 사용 가능한 카메라 선택
            selected_camera = available_cameras[0]
            camera_id = selected_camera['id']
            
            # 카메라 열기
            self.cap = cv2.VideoCapture(camera_id)
            if not self.cap.isOpened():
                self.logger.error(f"웹캠을 열 수 없습니다 (ID: {camera_id})")
                return False
            
            # vs_node와 동일한 해상도 설정
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            self.camera_id = camera_id
            self.is_running = True
            
            self.logger.info(f"✅ 웹캠 초기화 완료 (ID: {camera_id}, 이름: {selected_camera['name']})")
            return True
            
        except Exception as e:
            self.logger.error(f"웹캠 초기화 실패: {e}")
            return False
    
    def _scan_available_cameras(self) -> List[dict]:
        """사용 가능한 카메라 스캔 (vs_node 방식)"""
        available_cameras = []
        
        for camera_id in self.camera_ids_to_try:
            try:
                cap = cv2.VideoCapture(camera_id)
                if cap.isOpened():
                    # 테스트 프레임 읽기
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        device_name = self._get_camera_device_name(camera_id)
                        available_cameras.append({
                            'id': camera_id,
                            'name': device_name,
                            'backend': cap.getBackendName()
                        })
                        self.logger.info(f"  카메라 발견: ID={camera_id}, 이름={device_name}")
                    cap.release()
                
            except Exception as e:
                self.logger.debug(f"카메라 ID {camera_id} 테스트 실패: {e}")
        
        return available_cameras
    
    def _get_camera_device_name(self, camera_id: int) -> str:
        """카메라 디바이스 이름 획득"""
        try:
            # /dev/video* 장치 정보 확인
            video_device_path = f"/dev/video{camera_id}"
            if os.path.exists(video_device_path):
                return f"Video{camera_id}"
            else:
                return f"Camera{camera_id}"
        except:
            return f"Unknown Camera {camera_id}"
    
    def get_frame(self) -> Optional[np.ndarray]:
        """프레임 획득"""
        try:
            if not self.is_running or not self.cap:
                return None
            
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return None
            
            return frame
            
        except Exception as e:
            self.logger.error(f"프레임 획득 실패: {e}")
            return None
    
    def cleanup(self):
        """리소스 정리"""
        try:
            self.is_running = False
            if self.cap:
                self.cap.release()
            self.logger.info("웹캠 리소스 정리 완료")
        except Exception as e:
            self.logger.error(f"웹캠 정리 실패: {e}")


class SimpleYOLODetector:
    """간단한 YOLO 탐지기 (Elevator 모델 사용)"""
    
    def __init__(self, logger):
        self.logger = logger
        self.model = None
        self.model_classes = ['button', 'direction_light', 'display', 'door']
        self._initialize_model()
    
    def _initialize_model(self):
        """YOLO 모델 초기화"""
        try:
            from ultralytics import YOLO
            
            # elevator 모델 찾기
            elevator_model_path = self._find_elevator_model()
            
            if elevator_model_path:
                self.model = YOLO(elevator_model_path)
                # GPU 사용 가능하면 GPU로
                try:
                    self.model.to('cuda')
                    self.logger.info(f"✅ Elevator YOLO 모델 로딩 성공 (GPU): {elevator_model_path}")
                except:
                    self.logger.info(f"✅ Elevator YOLO 모델 로딩 성공 (CPU): {elevator_model_path}")
            else:
                self.logger.warning("⚠️ Elevator 모델을 찾을 수 없습니다")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"YOLO 모델 초기화 실패: {e}")
            return False
    
    def _find_elevator_model(self):
        """Elevator 모델 파일 찾기"""
        possible_paths = [
            "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_vs/training/elevator/best.pt",
            "training/elevator/best.pt",
            "../training/elevator/best.pt",
            "../../training/elevator/best.pt"
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                self.logger.info(f"모델 발견: {path}")
                return path
        
        return None
    
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    def detect_objects(self, image: np.ndarray, conf_threshold: float = 0.7, target_classes: List[str] = None) -> List[dict]:
        """객체 감지 (display, button 등)"""
=======
    def detect_display_objects(self, image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """Display 객체 감지"""
>>>>>>> Stashed changes
=======
    def detect_display_objects(self, image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """Display 객체 감지"""
>>>>>>> Stashed changes
        try:
            if self.model is None:
                return []
            
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            if target_classes is None:
                target_classes = ['display', 'button']  # 기본적으로 display와 button 모두 감지
            
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            results = self.model(image, conf=conf_threshold, verbose=False)
            
            objects = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # 클래스 정보
                        cls_id = int(box.cls.cpu().numpy())
                        if cls_id < len(self.model_classes):
                            class_name = self.model_classes[cls_id]
                        else:
                            class_name = f"class_{cls_id}"
                        
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                        # 타겟 클래스만 필터링
                        if class_name in target_classes:
=======
                        # display 객체만 필터링
                        if class_name == 'display':
>>>>>>> Stashed changes
=======
                        # display 객체만 필터링
                        if class_name == 'display':
>>>>>>> Stashed changes
                            # 바운딩박스 좌표
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            
                            objects.append({
                                'class_name': class_name,
                                'bbox': (int(x1), int(y1), int(x2-x1), int(y2-y1)),  # x, y, w, h 형식
                                'confidence': float(box.conf.cpu().numpy()),
                                'model_name': 'elevator'
                            })
            
            return objects
            
        except Exception as e:
            self.logger.error(f"객체 감지 실패: {e}")
            return []
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    
    def detect_display_objects(self, image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """Display 객체 감지 (하위 호환성)"""
        return self.detect_objects(image, conf_threshold, ['display'])
    
    def detect_button_objects(self, image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """Button 객체 감지"""
        return self.detect_objects(image, conf_threshold, ['button'])
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes


class OCRTestTool:
    """OCR 테스트 도구 메인 클래스 (웹캠 전용)"""
    
    def __init__(self):
        self.logger = Logger("OCRTestTool")
        self.webcam = None
        self.yolo_detector = None
        self.multi_ocr = None
        self.is_running = False
        
        # 설정
        self.confidence_threshold = 0.5
        self.test_mode = 'auto'  # 'auto' 또는 'manual'
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        self.target_objects = 'both'  # 'display', 'button', 'both'
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        
        # 상태
        self.last_test_time = 0
        self.test_interval = 2.0  # 2초마다 자동 테스트
        
    def initialize(self) -> bool:
        """시스템 초기화"""
        try:
            self.logger.info("🚀 OCR 테스트 도구 초기화 시작...")
            
            # 1. 웹캠 초기화
            self.webcam = WebcamManager(self.logger)
            if not self.webcam.initialize():
                self.logger.error("웹캠 초기화 실패")
                return False
            
            # 2. YOLO 탐지기 초기화
            self.yolo_detector = SimpleYOLODetector(self.logger)
            
            # 3. 멀티 OCR 초기화
            self.multi_ocr = MultiModelOCR(self.logger)
            
            # 사용 가능한 모델 확인
            available_models = self.multi_ocr.get_available_models()
            self.logger.info(f"🎯 사용 가능한 OCR 모델: {available_models}")
            
            model_status = self.multi_ocr.get_model_status()
            for name, status in model_status.items():
                self.logger.info(f"   - {name}: {status['type']} ({'GPU' if status['gpu_mode'] else 'CPU'})")
            
            self.is_running = True
            self.logger.info("✅ OCR 테스트 도구 초기화 완료!")
            
            return True
            
        except Exception as e:
            self.logger.error(f"시스템 초기화 실패: {e}")
            return False
    
    def run(self):
        """메인 실행 루프"""
        try:
            self.logger.info("🎯 OCR 테스트 도구 실행 시작")
            self.logger.info("=" * 60)
            self.logger.info("키보드 컨트롤:")
            self.logger.info("  SPACE: 수동 OCR 테스트")
            self.logger.info("  'a': 자동 모드 ON/OFF")
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            self.logger.info("  'd': Display 객체만 감지")
            self.logger.info("  'b': Button 객체만 감지")
            self.logger.info("  'x': Display + Button 모두 감지")
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            self.logger.info("  '+': 신뢰도 증가")
            self.logger.info("  '-': 신뢰도 감소")
            self.logger.info("  'q': 종료")
            self.logger.info("=" * 60)
            
            while self.is_running:
                # 웹캠 프레임 획득
                color_image = self.webcam.get_frame()
                
                if color_image is None:
                    continue
                
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                # 타겟 객체 감지
                target_classes = self._get_target_classes()
                detected_objects = self.yolo_detector.detect_objects(
                    color_image, self.confidence_threshold, target_classes
                )
                
                # 시각화
                display_image = self._draw_visualizations(color_image, detected_objects)
=======
=======
>>>>>>> Stashed changes
                # Display 객체 감지
                display_objects = self.yolo_detector.detect_display_objects(
                    color_image, self.confidence_threshold
                )
                
                # 시각화
                display_image = self._draw_visualizations(color_image, display_objects)
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
                
                # 자동 모드에서 OCR 테스트
                current_time = time.time()
                if (self.test_mode == 'auto' and 
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                    detected_objects and 
                    current_time - self.last_test_time > self.test_interval):
                    
                    self._run_ocr_tests(color_image, detected_objects)
                    self.last_test_time = current_time
                
                # 화면 출력
                cv2.imshow('OCR Test Tool (Display & Button)', display_image)
                
                # 키보드 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if not self._handle_keyboard(key, color_image, detected_objects):
=======
=======
>>>>>>> Stashed changes
                    display_objects and 
                    current_time - self.last_test_time > self.test_interval):
                    
                    self._run_ocr_tests(color_image, display_objects)
                    self.last_test_time = current_time
                
                # 화면 출력
                cv2.imshow('OCR Test Tool (Webcam)', display_image)
                
                # 키보드 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if not self._handle_keyboard(key, color_image, display_objects):
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
                    break
            
        except KeyboardInterrupt:
            self.logger.info("사용자에 의해 중단됨")
        except Exception as e:
            self.logger.error(f"실행 중 오류: {e}")
        finally:
            self.cleanup()
    
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    def _get_target_classes(self) -> List[str]:
        """타겟 클래스 목록 반환"""
        if self.target_objects == 'display':
            return ['display']
        elif self.target_objects == 'button':
            return ['button']
        else:  # 'both'
            return ['display', 'button']
    
    def _draw_visualizations(self, image: np.ndarray, detected_objects: List[dict]) -> np.ndarray:
        """시각화 그리기"""
        display_image = image.copy()
        
        # 감지된 객체 그리기
        display_count = 0
        button_count = 0
        
        for obj in detected_objects:
            x, y, w, h = obj['bbox']
            confidence = obj['confidence']
            class_name = obj['class_name']
            
            # 클래스별 색상 설정
            if class_name == 'display':
                color = (0, 255, 255)  # 노란색 (BGR)
                label = f"DISPLAY {confidence:.2f}"
                display_count += 1
            elif class_name == 'button':
                color = (255, 0, 255)  # 마젠타색 (BGR)
                label = f"BUTTON {confidence:.2f}"
                button_count += 1
            else:
                color = (128, 128, 128)  # 회색
                label = f"{class_name.upper()} {confidence:.2f}"
            
            # 바운딩박스
            cv2.rectangle(display_image, (x, y), (x + w, y + h), color, 2)
            
            # 라벨
            cv2.putText(display_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
=======
=======
>>>>>>> Stashed changes
    def _draw_visualizations(self, image: np.ndarray, display_objects: List[dict]) -> np.ndarray:
        """시각화 그리기"""
        display_image = image.copy()
        
        # Display 객체 그리기
        for obj in display_objects:
            x, y, w, h = obj['bbox']
            confidence = obj['confidence']
            
            # 바운딩박스
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            
            # 라벨
            label = f"DISPLAY {confidence:.2f}"
            cv2.putText(display_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        
        # 상태 정보 표시
        info_y = 30
        cv2.putText(display_image, f"Confidence: {self.confidence_threshold:.2f}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        mode_text = "AUTO" if self.test_mode == 'auto' else "MANUAL"
        cv2.putText(display_image, f"Mode: {mode_text}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        target_text = {"display": "DISPLAY", "button": "BUTTON", "both": "BOTH"}[self.target_objects]
        cv2.putText(display_image, f"Target: {target_text}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        cv2.putText(display_image, f"Objects: D={display_count}, B={button_count}", 
=======
        cv2.putText(display_image, f"Display Objects: {len(display_objects)}", 
>>>>>>> Stashed changes
=======
        cv2.putText(display_image, f"Display Objects: {len(display_objects)}", 
>>>>>>> Stashed changes
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 사용 가능한 OCR 모델 표시
        available_models = self.multi_ocr.get_available_models()
        info_y += 25
        cv2.putText(display_image, f"OCR Models: {', '.join(available_models)}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return display_image
    
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    def _handle_keyboard(self, key: int, color_image: np.ndarray, detected_objects: List[dict]) -> bool:
=======
    def _handle_keyboard(self, key: int, color_image: np.ndarray, display_objects: List[dict]) -> bool:
>>>>>>> Stashed changes
=======
    def _handle_keyboard(self, key: int, color_image: np.ndarray, display_objects: List[dict]) -> bool:
>>>>>>> Stashed changes
        """키보드 입력 처리"""
        if key == ord('q'):
            self.logger.info("종료 요청")
            return False
        
        elif key == ord(' '):  # SPACE
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            if detected_objects:
                self.logger.info("🎯 수동 OCR 테스트 시작...")
                self._run_ocr_tests(color_image, detected_objects)
            else:
                self.logger.warning("감지된 객체가 없습니다")
=======
=======
>>>>>>> Stashed changes
            if display_objects:
                self.logger.info("🎯 수동 OCR 테스트 시작...")
                self._run_ocr_tests(color_image, display_objects)
            else:
                self.logger.warning("Display 객체가 감지되지 않았습니다")
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        
        elif key == ord('a'):
            self.test_mode = 'manual' if self.test_mode == 'auto' else 'auto'
            self.logger.info(f"테스트 모드 변경: {self.test_mode}")
        
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        elif key == ord('d'):
            self.target_objects = 'display'
            self.logger.info("타겟 객체: Display만 감지")
        
        elif key == ord('b'):
            self.target_objects = 'button'
            self.logger.info("타겟 객체: Button만 감지")
        
        elif key == ord('x'):
            self.target_objects = 'both'
            self.logger.info("타겟 객체: Display + Button 모두 감지")
        
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        elif key == ord('+') or key == ord('='):
            self.confidence_threshold = min(0.95, self.confidence_threshold + 0.05)
            self.logger.info(f"신뢰도 임계값: {self.confidence_threshold:.2f}")
        
        elif key == ord('-'):
            self.confidence_threshold = max(0.1, self.confidence_threshold - 0.05)
            self.logger.info(f"신뢰도 임계값: {self.confidence_threshold:.2f}")
        
        return True
    
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    def _run_ocr_tests(self, color_image: np.ndarray, detected_objects: List[dict]):
=======
    def _run_ocr_tests(self, color_image: np.ndarray, display_objects: List[dict]):
>>>>>>> Stashed changes
=======
    def _run_ocr_tests(self, color_image: np.ndarray, display_objects: List[dict]):
>>>>>>> Stashed changes
        """OCR 테스트 실행"""
        try:
            self.logger.info("🔥 OCR 모델 테스트 시작...")
            
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            for idx, obj in enumerate(detected_objects):
                class_name = obj['class_name']
                confidence = obj['confidence']
                
                if class_name == 'display':
                    icon = "📱"
                    color_desc = "노란색"
                elif class_name == 'button':
                    icon = "🔘"
                    color_desc = "마젠타색"
                else:
                    icon = "📦"
                    color_desc = "회색"
                
                self.logger.info(f"{icon} {class_name.upper()} 객체 {idx + 1}/{len(detected_objects)} 테스트 중... "
                               f"(신뢰도: {confidence:.3f}, {color_desc})")
=======
            for idx, obj in enumerate(display_objects):
                self.logger.info(f"📱 Display 객체 {idx + 1}/{len(display_objects)} 테스트 중...")
>>>>>>> Stashed changes
=======
            for idx, obj in enumerate(display_objects):
                self.logger.info(f"📱 Display 객체 {idx + 1}/{len(display_objects)} 테스트 중...")
>>>>>>> Stashed changes
                
                bbox = obj['bbox']
                
                # 모든 모델로 테스트
                results = self.multi_ocr.test_all_models_on_roi(color_image, bbox)
                
                # 결과 출력
                self.logger.info("🏆 테스트 결과 (신뢰도 순):")
                sorted_results = sorted(results.items(), 
                                      key=lambda x: x[1].get('confidence', 0), 
                                      reverse=True)
                
                for model_name, result in sorted_results:
                    text = result.get('text', '?')
                    confidence = result.get('confidence', 0)
                    processing_time = result.get('processing_time', 0)
                    gpu_mode = result.get('gpu_mode', False)
                    
                    gpu_text = "🚀GPU" if gpu_mode else "💻CPU"
                    self.logger.info(f"  {model_name:12}: '{text:4}' "
                                   f"(신뢰도: {confidence:.3f}, "
                                   f"시간: {processing_time:.3f}s, {gpu_text})")
                
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                # 최고 성능 모델 및 button_id 매핑
                if sorted_results:
                    winner = sorted_results[0]
                    winner_text = winner[1].get('text', '?')
                    self.logger.info(f"🥇 최고 성능: {winner[0]} - '{winner_text}'")
                    
                    # 버튼인 경우 button_id 매핑 시도
                    if class_name == 'button':
                        button_id = self._map_button_text_to_id(winner_text)
                        if button_id is not None:
                            self.logger.info(f"🎯 Button ID 매핑: '{winner_text}' → button_id={button_id}")
                        else:
                            self.logger.info(f"❓ Button ID 매핑 실패: '{winner_text}' (알 수 없는 텍스트)")
=======
=======
>>>>>>> Stashed changes
                # 최고 성능 모델 강조
                if sorted_results:
                    winner = sorted_results[0]
                    self.logger.info(f"🥇 최고 성능: {winner[0]} - '{winner[1].get('text', '?')}'")
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
                
                self.logger.info("-" * 50)
        
        except Exception as e:
            self.logger.error(f"OCR 테스트 실행 실패: {e}")
    
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    def _map_button_text_to_id(self, text: str) -> Optional[int]:
        """버튼 텍스트를 button_id로 매핑"""
        if not text or text.strip() == '':
            return None
        
        text = text.strip().upper()
        
        # 층수 버튼 매핑
        floor_mapping = {
            '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6,
            '7': 7, '8': 8, '9': 9, '10': 10, '11': 11, '12': 12,
            'B1': 13, 'B2': 14, '1F': 1, '2F': 2, '3F': 3, '4F': 4,
            '5F': 5, '6F': 6, '7F': 7, '8F': 8, '9F': 9, '10F': 10,
            '11F': 11, '12F': 12
        }
        
        # 특수 버튼 매핑
        special_mapping = {
            '▼': 100, '↓': 100, 'DOWN': 100, '하행': 100,
            '▲': 101, '↑': 101, 'UP': 101, '상행': 101,
            '◀': 102, '열기': 102, 'OPEN': 102,
            '▶': 103, '닫기': 103, 'CLOSE': 103
        }
        
        # 먼저 특수 버튼 매핑 시도
        if text in special_mapping:
            return special_mapping[text]
        
        # 층수 버튼 매핑 시도
        if text in floor_mapping:
            return floor_mapping[text]
        
        # 숫자만 있는 경우 (1~12)
        try:
            floor_num = int(text)
            if 1 <= floor_num <= 12:
                return floor_num
        except ValueError:
            pass
        
        return None
    
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
    def cleanup(self):
        """리소스 정리"""
        try:
            self.is_running = False
            
            if self.webcam:
                self.webcam.cleanup()
            
            cv2.destroyAllWindows()
            
            self.logger.info("✅ 시스템 정리 완료")
            
        except Exception as e:
            self.logger.error(f"정리 중 오류: {e}")


def main():
    """메인 함수"""
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    print("🚀 OCR 테스트 도구 시작... (Display & Button 감지)")
    print("   - Display 객체: 엘리베이터 층수 표시기 OCR")
    print("   - Button 객체: 버튼 텍스트 OCR → button_id 자동 매핑")
=======
    print("🚀 OCR 테스트 도구 시작... (웹캠 전용)")
>>>>>>> Stashed changes
=======
    print("🚀 OCR 테스트 도구 시작... (웹캠 전용)")
>>>>>>> Stashed changes
    
    # 도구 초기화
    tool = OCRTestTool()
    
    if not tool.initialize():
        print("❌ 초기화 실패")
        return -1
    
    # 실행
    tool.run()
    
    print("👋 OCR 테스트 도구 종료")
    return 0


if __name__ == "__main__":
    exit(main()) 