import os
import cv2
import numpy as np
from typing import List, Dict

# PyTorch 및 YOLO 라이브러리 임포트 시도
try:
    import torch
    import torch.nn as nn
    import torchvision.transforms as transforms
    from PIL import Image
    import yaml
    from ultralytics import YOLO
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    # PyTorch가 없어도 YOLO는 단독으로 동작할 수 있으므로, 전체를 비활성화하지는 않습니다.
    # 각 클래스에서 TORCH_AVAILABLE 플래그를 확인하여 기능을 제어합니다.

class BalancedButtonCNN(nn.Module):
    """버튼 분류를 위해 성능과 메모리 사용량의 균형을 맞춘 CNN 모델 아키텍처입니다."""
    
    def __init__(self, num_classes=18):
        super(BalancedButtonCNN, self).__init__()
        
        # 특징 추출을 위한 합성곱 레이어
        self.features = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=3, padding=1),
            nn.BatchNorm2d(24),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.2),
            
            nn.Conv2d(24, 48, kernel_size=3, padding=1),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.3),
            
            nn.Conv2d(48, 96, kernel_size=3, padding=1),
            nn.BatchNorm2d(96),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4)),
            nn.Dropout(0.3),
        )
        
        # 분류를 위한 완전 연결 레이어
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(96 * 4 * 4, 192),
            nn.ReLU(inplace=True),
            nn.Dropout(0.5),
            nn.Linear(192, num_classes)
        )
    
    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

class ButtonPressedCNN:
    """버튼의 눌림 상태(pressed/unpressed)를 감지하는 CNN 모델을 관리하고 실행하는 클래스입니다."""
    
    def __init__(self, logger, model_path):
        self.logger = logger
        self.model_path = model_path
        self.model = None
        self.device = None
        self._initialize_model()
    
    def _initialize_model(self):
        """버튼 눌림 감지용 CNN 모델을 로드하고 초기화합니다."""
        if not TORCH_AVAILABLE:
            self.logger.warning("PyTorch가 설치되지 않아 ButtonPressedCNN 기능을 비활성화합니다.")
            return False
        try:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            
            if not os.path.exists(self.model_path):
                self.logger.warning(f"버튼 눌림 감지 모델 파일을 찾을 수 없습니다: {self.model_path}")
                return False
            
            # 버튼 눌림 감지 모델은 클래스 2개(눌림/안눌림)를 가진 BalancedButtonCNN 아키텍처를 사용합니다.
            self.model = BalancedButtonCNN(num_classes=2)
            self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
            self.model.to(self.device)
            self.model.eval()
            
            self.logger.info("버튼 눌림 감지 CNN 모델이 성공적으로 로드되었습니다.")
            return True
            
        except Exception as e:
            self.logger.error(f"버튼 눌림 감지 CNN 모델 초기화에 실패했습니다: {e}")
            return False
    
    def classify_pressed(self, color_image: np.ndarray, bbox: tuple) -> dict:
        """주어진 이미지와 경계 상자(ROI)를 기반으로 버튼의 눌림 상태를 분류합니다."""
        if self.model is None:
            return {'is_pressed': False, 'confidence': 0.0, 'method': 'no_model'}
            
        try:
            x1, y1, x2, y2 = bbox
            roi = color_image[y1:y2, x1:x2]
            
            if roi.size == 0:
                return {'is_pressed': False, 'confidence': 0.0, 'method': 'empty_roi'}
            
            roi_resized = cv2.resize(roi, (32, 32))
            roi_normalized = self._preprocess_image(roi_resized)
            
            with torch.no_grad():
                roi_tensor = torch.from_numpy(roi_normalized).float().unsqueeze(0).to(self.device)
                outputs = self.model(roi_tensor)
                probabilities = torch.softmax(outputs, dim=1)
                
                # 클래스 0: pressed, 클래스 1: unpressed
                pressed_prob = float(probabilities[0][0])
                unpressed_prob = float(probabilities[0][1])
                
                is_pressed = pressed_prob > unpressed_prob
                confidence = max(pressed_prob, unpressed_prob)
                
                return {
                    'is_pressed': is_pressed,
                    'confidence': confidence,
                    'pressed_prob': pressed_prob,
                    'unpressed_prob': unpressed_prob,
                    'method': 'cnn_only'
                }
                
        except Exception as e:
            self.logger.error(f"CNN 버튼 눌림 상태 분류 중 오류가 발생했습니다: {e}")
            return {'is_pressed': False, 'confidence': 0.0, 'method': 'error'}
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """추론을 위해 이미지를 ImageNet 표준에 맞게 전처리합니다."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_normalized = image_rgb.astype(np.float32) / 255.0
        
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        
        image_normalized = (image_normalized - mean) / std
        image_chw = np.transpose(image_normalized, (2, 0, 1))
        
        return image_chw

class CNNButtonClassifier:
    """CNN 모델을 사용하여 버튼의 종류(예: 숫자, 열림/닫힘)를 분류하는 클래스입니다."""
    
    def __init__(self, logger, model_path, config_path):
        self.logger = logger
        self.model = None
        self.transform = None
        self.torch_device = torch.device('cuda' if torch.cuda.is_available() else 'cpu') if TORCH_AVAILABLE else None
        self.class_names = []
        self.button_id_mapping = {}
        
        if TORCH_AVAILABLE:
            self._load_cnn_model(model_path, config_path)
        else:
            self.logger.warning("PyTorch가 설치되지 않아 CNN 버튼 분류 기능을 비활성화합니다.")
    
    def _load_cnn_model(self, model_path, config_path):
        """분류용 CNN 모델과 관련 설정 파일을 로드합니다."""
        try:
            if not os.path.exists(model_path) or not os.path.exists(config_path):
                self.logger.warning(f"CNN 모델 또는 설정 파일이 존재하지 않습니다. 경로를 확인해주세요. 모델: {model_path}, 설정: {config_path}")
                return False
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.class_names = config['dataset_info']['class_names']
            self._create_button_mapping()
            
            self.model = BalancedButtonCNN(num_classes=len(self.class_names))
            self.model.load_state_dict(torch.load(model_path, map_location=self.torch_device))
            self.model.to(self.torch_device)
            self.model.eval()
            
            self.transform = transforms.Compose([
                transforms.Resize((32, 32)),
                transforms.ToTensor(),
                transforms.Normalize(mean=config['preprocessing']['normalize_mean'], std=config['preprocessing']['normalize_std'])
            ])
            
            self.logger.info(f"CNN 버튼 분류 모델이 성공적으로 로드되었습니다 ({len(self.class_names)}개 클래스).")
            self.logger.info(f"지원되는 버튼 목록: {self.class_names}")
            return True
            
        except Exception as e:
            self.logger.error(f"CNN 모델 로드 중 오류가 발생했습니다: {e}")
            return False
    
    def _create_button_mapping(self):
        """클래스 이름을 시스템에서 사용하는 버튼 ID로 매핑합니다."""
        self.button_id_mapping = {
            'btn_1': 1, 'btn_2': 2, 'btn_3': 3, 'btn_4': 4,
            'btn_5': 5, 'btn_6': 6, 'btn_7': 7, 'btn_8': 8,
            'btn_9': 9, 'btn_10': 10, 'btn_11': 11, 'btn_12': 12,
            'btn_b1': 13, 'btn_b2': 14,
            'btn_open': 102, 'btn_close': 103,
            'btn_upward': 101, 'btn_downward': 100
        }
    
    def classify_button(self, color_image: np.ndarray, button_bbox: tuple) -> dict:
        """잘라낸 버튼 이미지를 CNN으로 분류하여 버튼의 종류와 ID를 반환합니다."""
        if self.model is None or not TORCH_AVAILABLE:
            return None
            
        try:
            x1, y1, x2, y2 = button_bbox
            button_crop = color_image[y1:y2, x1:x2]
            if button_crop.size == 0: return None
            
            button_crop_rgb = cv2.cvtColor(button_crop, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(button_crop_rgb)
            input_tensor = self.transform(pil_image).unsqueeze(0).to(self.torch_device)
            
            with torch.no_grad():
                outputs = self.model(input_tensor)
                probabilities = torch.softmax(outputs, dim=1)
                predicted_class = torch.argmax(probabilities, dim=1).item()
                confidence = probabilities[0][predicted_class].item()
            
            class_name = self.class_names[predicted_class]
            button_id = self.button_id_mapping.get(class_name, 'unknown')
            
            if button_id in [102, 103]: floor_type = 'control'
            elif button_id in [13, 14]: floor_type = 'basement'
            elif button_id in [100, 101]: floor_type = 'direction'
            else: floor_type = 'floor'
            
            return {
                'button_id': button_id,
                'confidence': confidence,
                'class_name': class_name,
                'floor_type': floor_type,
                'recognition_method': 'cnn_classification'
            }
            
        except Exception as e:
            self.logger.error(f"CNN 버튼 분류 중 오류가 발생했습니다: {e}")
            return None

class MultiModelDetector:
    """상황에 따라 여러 YOLO 모델을 전환하며 객체를 탐지하는 클래스입니다."""
    
    def __init__(self, logger, config):
        self.logger = logger
        self.config = config
        self.models = {}
        self.current_model_name = None
        self.current_model = None
        self.button_pressed_cnn = None
        
        # 객체 경계 상자(Bounding Box) 안정화를 위한 변수
        self.previous_objects = []
        self.object_tracking_threshold = 0.5
        self.stability_frames = 3
        self.object_history = {}
        
    def set_button_pressed_cnn(self, button_pressed_cnn):
        """외부에서 생성된 버튼 눌림 감지 CNN 모델을 설정합니다."""
        self.button_pressed_cnn = button_pressed_cnn
        
        self.model_classes = {
            'normal': ['chair', 'door', 'person'],
            'elevator': ['button', 'direction_light', 'display', 'door']
        }
        
        self.model_id_maps = {
            'normal': {'chair': 'CHAIR', 'door': 'DOOR', 'person': 'PERSON'},
            'elevator': {'button': 'BUTTON', 'direction_light': 'DIRECTION_LIGHT', 'display': 'DISPLAY', 'door': 'DOOR'}
        }
        
        self._initialize_models()
        
    def _initialize_models(self):
        """설정 파일에 명시된 모든 YOLO 모델을 로드하고 초기화합니다."""
        self.logger.info("다중 YOLO 모델 초기화를 시작합니다...")
        try:
            # 일반 주행용 모델 로드
            normal_model_path = self.config['yolo']['models']['normal']
            if os.path.exists(normal_model_path):
                try:
                    self.models['normal'] = YOLO(normal_model_path)
                    self.models['normal'].to('cuda')
                    self.logger.info(f"일반 주행용 모델을 성공적으로 로드했습니다 (GPU): {normal_model_path}")
                except Exception as e:
                    self.logger.error(f"일반 주행용 모델 로드에 실패했습니다: {e}")
            else:
                self.logger.error(f"일반 주행용 모델 파일을 찾을 수 없습니다: {normal_model_path}")
            
            # 엘리베이터용 모델 로드
            elevator_model_path = self.config['yolo']['models']['elevator']
            if os.path.exists(elevator_model_path):
                try:
                    self.models['elevator'] = YOLO(elevator_model_path)
                    self.models['elevator'].to('cuda')
                    self.logger.info(f"엘리베이터용 모델을 성공적으로 로드했습니다 (GPU): {elevator_model_path}")
                except Exception as e:
                    self.logger.warning(f"엘리베이터용 모델 로드에 실패했습니다: {e}")
            else:
                self.logger.warning(f"엘리베이터용 모델 파일을 찾을 수 없습니다: {elevator_model_path}")
            
            # 기본 모델 설정
            if 'normal' in self.models:
                self.current_model_name = 'normal'
                self.current_model = self.models['normal']
                self.logger.info("기본 모델을 'normal'(일반 주행용)으로 설정합니다.")
            elif 'elevator' in self.models:
                self.current_model_name = 'elevator'
                self.current_model = self.models['elevator']
                self.logger.info("기본 모델을 'elevator'(엘리베이터용)으로 설정합니다.")
            else:
                self.logger.error("사용 가능한 YOLO 모델이 없어 초기화에 실패했습니다.")
            
            return len(self.models) > 0
                
        except ImportError:
            self.logger.error("YOLO 모델을 사용하려면 'ultralytics' 패키지가 필요합니다.")
            raise
        except Exception as e:
            self.logger.error(f"다중 모델 초기화 중 예외가 발생했습니다: {e}")
            return False
    
    def set_model_for_mode(self, mode_id):
        """서비스 모드에 따라 사용할 YOLO 모델을 전환합니다."""
        try:
            target_model = None
            if mode_id == 5:  # 일반 주행 모드
                target_model = 'normal'
            elif mode_id in [3, 4]:  # 엘리베이터 모드
                target_model = 'elevator'
            
            if target_model:
                if target_model in self.models:
                    self.current_model_name = target_model
                    self.current_model = self.models[target_model]
                    return True
                else:
                    self.logger.warning(f"'{target_model}' 모델이 로드되지 않아 전환할 수 없습니다.")
                    return False
            else:
                self.current_model_name = None
                self.current_model = None
                return True
                
        except Exception as e:
            self.logger.error(f"모델 전환 중 오류가 발생했습니다: {e}")
            return False
    
    def detect_objects(self, color_image: np.ndarray, depth_image: np.ndarray, conf_threshold: float = 0.7, mode_id: int = 0) -> List[dict]:
        """현재 선택된 모델로 객체를 탐지하고, 결과를 안정화하여 반환합니다."""
        if color_image is None or self.current_model is None:
            return []
        try:
            return self._detect_with_current_model(color_image, depth_image, conf_threshold)
        except Exception as e:
            self.logger.error(f"객체 탐지 중 오류가 발생했습니다: {e}")
            return []
    
    def _detect_with_current_model(self, color_image: np.ndarray, depth_image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """실제 YOLO 모델 추론을 수행하고, 탐지된 객체 정보를 정제합니다."""
        try:
            results = self.current_model.predict(color_image, conf=conf_threshold, device='cuda', verbose=False)
            
            objects = []
            if not results or len(results) == 0 or results[0].boxes is None:
                return []

            result = results[0]
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            
            current_class_names = self.model_classes.get(self.current_model_name, [])
            current_id_map = self.model_id_maps.get(self.current_model_name, {})
            
            for box, conf, cls in zip(boxes, confs, classes):
                x1, y1, x2, y2 = map(int, box)
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                
                class_id = int(cls)
                class_name = current_class_names[class_id] if class_id < len(current_class_names) else f"unknown_{class_id}"
                
                depth_value = depth_image[center_y, center_x] if depth_image is not None and 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1] else 1000
                
                obj_data = {
                    'center': (center_x, center_y),
                    'radius': max(x2 - x1, y2 - y1) // 2,
                    'depth_mm': int(depth_value),
                    'class_name': class_name,
                    'object_id': current_id_map.get(class_name, class_name.upper()),
                    'confidence': float(conf),
                    'bbox': (x1, y1, x2, y2),
                    'is_button': class_name == 'button',
                    'model_name': self.current_model_name
                }

                if class_name == 'button':
                    pressed_result = self._check_button_pressed_cnn(color_image, (x1, y1, x2, y2))
                    obj_data.update(pressed_result)
                
                objects.append(obj_data)
            
            return self._apply_box_stabilization(objects)
            
        except Exception as e:
            self.logger.error(f"'{self.current_model_name}' 모델로 탐지 중 오류가 발생했습니다: {e}")
            return []
    
    def _check_button_pressed_cnn(self, color_image: np.ndarray, bbox: tuple) -> dict:
        """버튼 눌림 상태를 확인하기 위해 CNN 모델을 호출합니다."""
        if self.button_pressed_cnn is None:
            return {'is_pressed': False, 'confidence': 0.0, 'method': 'no_cnn'}
        return self.button_pressed_cnn.classify_pressed(color_image, bbox)
    
    def _apply_box_stabilization(self, objects: List[dict]) -> List[dict]:
        """탐지된 객체의 경계 상자가 떨리는 현상을 줄이기 위한 후처리 기법들을 적용합니다."""
        if not objects: return []
        
        nms_objects = self._apply_nms(objects)
        tracked_objects = self._apply_object_tracking(nms_objects)
        return self._apply_confidence_filtering(tracked_objects)
    
    def _apply_nms(self, objects: List[dict], iou_threshold: float = 0.5) -> List[dict]:
        """동일 클래스에 대해 겹치는 경계 상자를 제거하는 Non-Maximum Suppression을 적용합니다."""
        if len(objects) <= 1: return objects
        
        class_groups = {}
        for obj in objects:
            class_groups.setdefault(obj['class_name'], []).append(obj)
        
        nms_objects = []
        for _, class_objects in class_groups.items():
            if len(class_objects) <= 1:
                nms_objects.extend(class_objects)
                continue
            
            class_objects.sort(key=lambda x: x['confidence'], reverse=True)
            
            while class_objects:
                current = class_objects.pop(0)
                nms_objects.append(current)
                class_objects = [obj for obj in class_objects if self._calculate_iou(current['bbox'], obj['bbox']) < iou_threshold]
        
        return nms_objects
    
    def _calculate_iou(self, box1: tuple, box2: tuple) -> float:
        """두 경계 상자 간의 IoU(Intersection over Union)를 계산합니다."""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        
        x1_inter = max(x1_1, x1_2)
        y1_inter = max(y1_1, y1_2)
        x2_inter = min(x2_1, x2_2)
        y2_inter = min(y2_1, y2_2)
        
        inter_area = max(0, x2_inter - x1_inter) * max(0, y2_inter - y1_inter)
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = area1 + area2 - inter_area
        
        return inter_area / union_area if union_area > 0 else 0.0
    
    def _apply_object_tracking(self, objects: List[dict]) -> List[dict]:
        """프레임 간 객체 추적을 통해 탐지 결과를 안정화합니다."""
        if not hasattr(self, 'previous_objects'):
            self.previous_objects = objects
            return objects
        
        tracked_objects = []
        import time
        current_time = time.time()
        
        for obj in objects:
            best_match, best_iou = None, 0.0
            for prev_obj in self.previous_objects:
                if prev_obj['class_name'] == obj['class_name']:
                    iou = self._calculate_iou(obj['bbox'], prev_obj['bbox'])
                    if iou > best_iou and iou > self.object_tracking_threshold:
                        best_iou, best_match = iou, prev_obj
            
            if best_match:
                obj['center'] = self._smooth_position(obj['center'], best_match['center'])
                obj['bbox'] = self._smooth_bbox(obj['bbox'], best_match['bbox'])
                obj['tracking_id'] = best_match.get('tracking_id', f"obj_{len(tracked_objects)}")
                obj['stable_frames'] = best_match.get('stable_frames', 0) + 1
            else:
                obj['tracking_id'] = f"obj_{current_time}_{len(tracked_objects)}"
                obj['stable_frames'] = 1
            
            tracked_objects.append(obj)
        
        self.previous_objects = tracked_objects.copy()
        return tracked_objects
    
    def _smooth_position(self, current_pos: tuple, prev_pos: tuple, alpha: float = 0.7) -> tuple:
        """지수 이동 평균을 사용하여 좌표를 부드럽게 보정합니다."""
        return tuple(int(alpha * c + (1 - alpha) * p) for c, p in zip(current_pos, prev_pos))
    
    def _smooth_bbox(self, current_bbox: tuple, prev_bbox: tuple, alpha: float = 0.7) -> tuple:
        """지수 이동 평균을 사용하여 경계 상자 좌표를 부드럽게 보정합니다."""
        return tuple(int(alpha * c + (1 - alpha) * p) for c, p in zip(current_bbox, prev_bbox))
    
    def _apply_confidence_filtering(self, objects: List[dict]) -> List[dict]:
        """객체의 신뢰도와 안정화된 프레임 수를 기반으로 최종 결과를 필터링합니다."""
        filtered_objects = []
        for obj in objects:
            if obj['confidence'] < 0.3: continue
            
            stable_frames = obj.get('stable_frames', 1)
            min_confidence = 0.7 if stable_frames < self.stability_frames else 0.5
            
            if obj['confidence'] >= min_confidence:
                filtered_objects.append(obj)
        
        return filtered_objects

    def get_current_model_info(self):
        """현재 활성화된 모델의 정보를 반환합니다."""
        return {
            'model_name': self.current_model_name,
            'available_models': list(self.models.keys()),
            'class_names': self.model_classes.get(self.current_model_name, []),
            'is_active': self.current_model is not None
        }