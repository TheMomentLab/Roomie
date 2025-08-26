"""
PersonTracker: YOLOv8n으로 사람을 감지하고 DeepSORT 알고리즘을 이용해 추적하는 모듈입니다.
DeepSORT가 추적을 놓쳤을 경우, 색상 및 형태 특징을 이용한 대체 추적(Fallback) 로직을 포함합니다.
"""

import numpy as np
import cv2
import time
import threading
from collections import deque
from typing import Optional, Dict, List, Any, Tuple

try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
    DEEPSORT_AVAILABLE = True
except ImportError:
    print("경고: DeepSort 라이브러리를 찾을 수 없습니다. `pip install deep-sort-realtime` 명령어로 설치하세요.")
    DEEPSORT_AVAILABLE = False

class PersonTracker:
    """
    YOLO와 DeepSORT를 사용하여 사람을 감지하고 추적하는 기능을 제공하는 클래스입니다.
    - 모드 관리: 대기, 등록, 추적 모드를 지원합니다.
    - 타겟 등록: 지정된 시간 동안 가장 안정적인 사람을 타겟으로 등록합니다.
    - 추적 및 이벤트 생성: 타겟의 상태(정상, 멀어짐, 잃어버림, 재획득)에 따라 이벤트를 생성합니다.
    - 대체 추적: DeepSORT가 타겟을 놓쳤을 때, 저장된 특징을 기반으로 타겟을 다시 찾으려고 시도합니다.
    """
    
    def __init__(self, logger, config, yolo_model=None):
        self.logger = logger
        self.config = config['person_tracker']
        self.yolo_model = yolo_model
        
        self.person_class_index = 0
        self._resolve_person_class_index()
        
        # --- 상태 변수 ---
        self.current_mode = 0  # 0: 대기, 1: 등록, 2: 추적
        self.target_registered = False
        self.target_id = None
        self.tracking_active = False
        
        if DEEPSORT_AVAILABLE:
            self.tracker = DeepSort(
                max_age=self.config['max_age'],
                n_init=self.config['n_init'],
                nms_max_overlap=self.config['nms_max_overlap'],
                max_cosine_distance=self.config['max_cosine_distance']
            )
        else:
            self.tracker = None
            self.logger.warning("DeepSORT를 사용할 수 없어 추적 기능이 비활성화됩니다.")
        
        # --- 프레임 버퍼 ---
        self.frame_buffer = deque(maxlen=5)
        self.frame_lock = threading.Lock()
        
        # --- 등록 관련 변수 ---
        self.registration_candidates = []
        self.registration_start_time = None
        self.registration_duration = self.config['registration_duration']
        self.track_appearances = {}
        
        # --- 추적 상태 관련 변수 ---
        self.last_seen_time = None
        self.lost_count = 0
        self.reacquired_count = 0
        self.previous_tracking_state = None
        self.is_target_distant = False
        self.distant_frame_count = 0
        self.normal_frame_count = 0
        self.required_frames = self.config['required_frames_for_state_change']
        
        # --- 대체 추적(Fallback) 관련 변수 ---
        self.target_color_features = None
        self.target_bbox_features = None
        self.fallback_tracking_enabled = self.config['fallback_tracking_enabled']
        
        # --- 디버깅 및 시각화용 변수 ---
        self.deepsort_bbox = None
        self.fallback_bbox = None
        self.last_detections = []
        self.last_tracks = []
        self.last_target_bbox = None

        self.logger.info("PersonTracker가 성공적으로 초기화되었습니다.")

    def _resolve_person_class_index(self):
        """YOLO 모델의 클래스 이름 목록에서 'person'에 해당하는 인덱스를 찾습니다."""
        if not self.yolo_model or not hasattr(self.yolo_model, 'names'):
            self.person_class_index = 0
            self.logger.warning("YOLO 모델 정보가 없어 person 클래스 인덱스를 0으로 가정합니다.")
            return
        try:
            name_map = self.yolo_model.names
            candidates = [k for k, v in name_map.items() if str(v).lower() == "person"]
            if candidates:
                self.person_class_index = int(candidates[0])
                self.logger.info(f"YOLO 모델에서 'person' 클래스 인덱스를 {self.person_class_index}로 설정했습니다.")
            else:
                self.person_class_index = 0
                self.logger.warning("YOLO 모델에서 'person' 클래스를 찾지 못해 기본 인덱스 0을 사용합니다.")
        except Exception as e:
            self.person_class_index = 0
            self.logger.error(f"person 클래스 인덱스 확인 중 오류 발생: {e}")
    
    def set_mode(self, mode_id: int):
        """추적기의 동작 모드를 설정합니다 (0: 대기, 1: 등록, 2: 추적)."""
        prev_mode = self.current_mode
        self.current_mode = mode_id
        
        if mode_id == 1: self._prepare_registration()
        elif mode_id == 2: self._start_tracking()
        else: self._stop_tracking()
        
        self.logger.info(f"PersonTracker 모드가 {prev_mode}에서 {mode_id}로 변경되었습니다.")
    
    def process_frame(self, frame: np.ndarray, timestamp: float = None) -> List[Dict]:
        """새로운 프레임을 받아 현재 모드에 맞게 처리하고, 추적 이벤트를 반환합니다."""
        timestamp = timestamp or time.time()
        with self.frame_lock: self.frame_buffer.append((frame.copy(), timestamp))
        
        if self.current_mode == 1: # 등록 모드
            self._process_registration_frame(frame, timestamp)
            return []
        elif self.current_mode == 2: # 추적 모드
            return self._process_tracking_frame(frame, timestamp)
        else: # 대기 모드
            self._process_idle_detection_frame(frame, timestamp)
            return []

    def _process_idle_detection_frame(self, frame: np.ndarray, timestamp: float):
        """대기 모드: 사람을 감지만 하고 추적은 하지 않습니다."""
        detections = self._detect_persons(frame)
        self.last_detections = [tuple(det[:5]) for det in detections] if detections.size > 0 else []
        self.last_tracks = []
        self.last_target_bbox = None
    
    def register_target(self, duration_sec: float = 3.0) -> Dict[str, Any]:
        """타겟 등록 절차를 시작합니다."""
        if self.current_mode != 1:
            return {"success": False, "message": "등록 모드가 아니므로 타겟을 등록할 수 없습니다."}
        
        self.registration_duration = duration_sec
        self.registration_start_time = time.time()
        self.logger.info(f"타겟 등록을 시작합니다 (지속 시간: {duration_sec}초)." )
        return {"success": True, "message": "등록 시작"}
    
    def stop_tracking(self) -> Dict[str, Any]:
        """현재 진행 중인 모든 추적 및 등록 활동을 중지합니다."""
        self._stop_tracking()
        return {"success": True, "message": "추적이 중지되었습니다."}
    
    def get_registration_progress(self) -> float:
        """현재 등록 진행률을 0.0에서 1.0 사이의 값으로 반환합니다."""
        if not self.registration_start_time or not self.registration_duration:
            return 0.0
        try:
            elapsed = time.time() - self.registration_start_time
            return min(elapsed / self.registration_duration, 1.0)
        except (TypeError, ValueError): return 0.0
    
    def _prepare_registration(self):
        """새로운 타겟 등록을 위해 모든 관련 상태를 초기화합니다."""
        self._stop_tracking() # 추적 상태 초기화
        self.registration_candidates = []
        self.track_appearances = {}
        if DEEPSORT_AVAILABLE:
            self.tracker = DeepSort(max_age=self.config['max_age'], n_init=self.config['n_init'])
    
    def _start_tracking(self):
        """등록된 타겟에 대한 추적을 시작합니다."""
        if not self.target_registered:
            self.logger.warning("추적을 시작할 타겟이 등록되지 않았습니다.")
            return
        self.tracking_active = True
        self.last_seen_time = time.time()
        self.lost_count = 0
        self.previous_tracking_state = "tracking"
        self.logger.info(f"ID '{self.target_id}' 타겟에 대한 추적을 시작합니다.")
    
    def _stop_tracking(self):
        """추적을 중지하고 모든 타겟 및 추적 관련 정보를 초기화합니다."""
        self.tracking_active = False
        self.target_registered = False
        self.target_id = None
        self.target_color_features = None
        self.target_bbox_features = None
        self.deepsort_bbox = None
        self.fallback_bbox = None
        self.previous_tracking_state = None
        self.is_target_distant = False
        self.distant_frame_count = 0
        self.normal_frame_count = 0
        self.lost_count = 0
        self.last_detections = []
        self.last_tracks = []
        self.last_target_bbox = None
    
    def _process_registration_frame(self, frame: np.ndarray, timestamp: float):
        """등록 모드에서 프레임을 처리하여 가장 적합한 타겟 후보를 찾습니다."""
        detections = self._detect_persons(frame)
        self.last_detections = [tuple(det[:5]) for det in detections] if detections.size > 0 else []

        if self.registration_start_time and (time.time() - self.registration_start_time > self.registration_duration):
            self._finalize_registration()
            return

        if detections.size == 0 or not self.tracker: return

        ds_dets = [([float(x1), float(y1), float(x2-x1), float(y2-y1)], float(conf), int(cls)) for x1, y1, x2, y2, conf, cls in detections]
        tracks = self.tracker.update_tracks(ds_dets, frame=frame)
        
        self.last_tracks = [track.to_tlbr() for track in tracks if track.is_confirmed()]
        
        for track in tracks:
            if not track.is_confirmed(): continue
            self.track_appearances[track.track_id] = self.track_appearances.get(track.track_id, 0) + 1
            self.registration_candidates.append({
                'track_id': track.track_id,
                'bbox': track.to_tlbr(),
                'stability': self._calculate_stability(track.track_id)
            })

    def _process_tracking_frame(self, frame: np.ndarray, timestamp: float) -> List[Dict]:
        """추적 모드에서 프레임을 처리하고, 타겟의 상태 변화에 따라 이벤트를 생성합니다."""
        if not self.tracking_active or not self.target_registered: return []
        
        detections = self._detect_persons(frame)
        self.last_detections = [tuple(det[:5]) for det in detections] if detections.size > 0 else []
        
        if not self.tracker: return []

        ds_dets = [([float(x1), float(y1), float(x2-x1), float(y2-y1)], float(conf), int(cls)) for x1, y1, x2, y2, conf, cls in detections]
        tracks = self.tracker.update_tracks(ds_dets, frame=frame)
        self.last_tracks = [t.to_tlbr() for t in tracks if t.is_confirmed()]
        
        target_track = next((t for t in tracks if t.track_id == self.target_id), None)
        
        if target_track:
            self.deepsort_bbox = target_track.to_tlbr()
            self.last_target_bbox = self.deepsort_bbox
            self._update_target_features_during_tracking(frame, self.deepsort_bbox)
        elif self.fallback_tracking_enabled:
            target_track = self._attempt_fallback_tracking(frame, detections, timestamp)
        
        return self._generate_tracking_events(target_track, frame.shape, timestamp)

    def _detect_persons(self, frame: np.ndarray) -> np.ndarray:
        """YOLO 모델을 사용하여 프레임에서 사람을 감지합니다."""
        if not self.yolo_model: return np.empty((0, 6))
        try:
            results = self.yolo_model(frame, classes=[self.person_class_index], conf=0.7, verbose=False)
            return results[0].boxes.data.cpu().numpy() if results and results[0].boxes is not None else np.empty((0, 6))
        except Exception as e:
            self.logger.warning(f"YOLO 감지 중 오류 발생: {e}")
            return np.empty((0, 6))
    
    def _finalize_registration(self):
        """등록 시간이 끝나면, 가장 안정적인 후보를 최종 타겟으로 선정합니다."""
        self.registration_start_time = None
        if not self.registration_candidates:
            self.logger.warning("등록 시간 동안 감지된 후보가 없어 등록에 실패했습니다.")
            self.target_registered = False
            return

        best_candidate = max(self.registration_candidates, key=lambda x: x['stability'])
        
        if best_candidate['stability'] < 0.1:
            self.logger.warning(f"가장 유력한 후보의 안정성 점수({best_candidate['stability']:.2f})가 임계값 미만이라 등록에 실패했습니다.")
            self.target_registered = False
            return

        self.target_id = best_candidate['track_id']
        self.target_registered = True
        self._save_target_features(best_candidate)
        self.logger.info(f"타겟 등록 완료: 최종 타겟 ID는 '{self.target_id}'입니다 (안정성: {best_candidate['stability']:.2f}).")
    
    def _calculate_stability(self, track_id: int) -> float:
        """등록 과정에서 특정 트랙 ID가 얼마나 꾸준히 나타났는지를 기반으로 안정성 점수를 계산합니다."""
        if not self.registration_start_time: return 0.0
        try:
            appearance_count = self.track_appearances.get(track_id, 0)
            duration = time.time() - self.registration_start_time
            expected_frames = duration * 10  # 추정 FPS 기반 기대 프레임 수
            return min(appearance_count / max(expected_frames, 1), 1.0)
        except (TypeError, ValueError): return 0.0
    
    def _generate_tracking_events(self, target_track, frame_shape, timestamp) -> List[Dict]:
        """추적 상태에 따라 이벤트를 생성합니다. (0: 정상, 1: 멀어짐, 2: 놓침, 3: 재획득)"""
        events = []
        current_state = "lost"
        
        if target_track and target_track.is_confirmed():
            current_state = "tracking"
            if self.previous_tracking_state == "lost":
                events.append({'id': self.target_id, 'event': 3}) # REACQUIRED
                self.logger.info(f"타겟 '{self.target_id}'을(를) 다시 찾았습니다.")
            
            bbox = target_track.to_tlbr()
            width_ratio = (bbox[2] - bbox[0]) / frame_shape[1]
            
            if width_ratio < 0.3:
                self.distant_frame_count += 1
                self.normal_frame_count = 0
                if self.distant_frame_count >= self.required_frames and not self.is_target_distant:
                    events.append({'id': self.target_id, 'event': 1}) # DISTANT
                    self.is_target_distant = True
                    self.logger.info(f"타겟이 멀어졌습니다 (너비 비율: {width_ratio:.2f}).")
            else:
                self.normal_frame_count += 1
                self.distant_frame_count = 0
                if self.normal_frame_count >= self.required_frames and self.is_target_distant:
                    events.append({'id': self.target_id, 'event': 0}) # NORMAL
                    self.is_target_distant = False
                    self.logger.info("타겟이 정상 거리로 돌아왔습니다.")
            
            self.lost_count = 0
            self.last_seen_time = timestamp
        
        else:
            self.lost_count += 1
            if self.lost_count == 1 and self.previous_tracking_state != "lost":
                events.append({'id': self.target_id, 'event': 2}) # LOST
                self.logger.warning(f"타겟 '{self.target_id}'을(를) 놓쳤습니다.")

        self.previous_tracking_state = current_state
        return events
    
    def get_overlay_frame(self, base_frame: np.ndarray) -> np.ndarray:
        """추적 상태를 시각화하여 원본 프레임 위에 그립니다."""
        overlay = base_frame.copy()
        # 여기에 시각화 로직 추가 (필요 시)
        return overlay 
    
    def _extract_color_features(self, image: np.ndarray, bbox: Tuple[float, float, float, float]) -> Dict[str, np.ndarray]:
        """경계 상자 내에서 상체, 하체, 전체에 대한 색상(RGB, HSV) 히스토그램 특징을 추출합니다."""
        try:
            x1, y1, x2, y2 = map(int, bbox)
            person_crop = image[max(0, y1):min(image.shape[0], y2), max(0, x1):min(image.shape[1], x2)]
            if person_crop.size == 0: return {}

            crop_h, crop_w = person_crop.shape[:2]
            upper_body = person_crop[:int(crop_h * 0.4), :] if crop_h > 0 else person_crop
            lower_body = person_crop[int(crop_h * 0.6):, :] if crop_h > 0 else person_crop
            
            features = {}
            for name, region in [("upper", upper_body), ("lower", lower_body), ("full", person_crop)]:
                if region.size == 0: continue
                hsv_region = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
                # RGB 히스토그램
                hist_r = cv2.normalize(cv2.calcHist([region], [2], None, [16], [0, 256]), None, 0, 1, cv2.NORM_MINMAX).flatten()
                hist_g = cv2.normalize(cv2.calcHist([region], [1], None, [16], [0, 256]), None, 0, 1, cv2.NORM_MINMAX).flatten()
                hist_b = cv2.normalize(cv2.calcHist([region], [0], None, [16], [0, 256]), None, 0, 1, cv2.NORM_MINMAX).flatten()
                features[f"{name}_rgb"] = np.concatenate([hist_r, hist_g, hist_b])
                # HSV 히스토그램
                hist_h = cv2.normalize(cv2.calcHist([hsv_region], [0], None, [16], [0, 180]), None, 0, 1, cv2.NORM_MINMAX).flatten()
                hist_s = cv2.normalize(cv2.calcHist([hsv_region], [1], None, [16], [0, 256]), None, 0, 1, cv2.NORM_MINMAX).flatten()
                features[f"{name}_hsv"] = np.concatenate([hist_h, hist_s])
            return features
        except Exception as e:
            self.logger.warning(f"색상 특징 추출 중 오류 발생: {e}")
            return {}
    
    def _calculate_color_similarity(self, features1: Dict[str, np.ndarray], features2: Dict[str, np.ndarray]) -> float:
        """두 특징 세트 간의 코사인 유사도를 계산하여 평균을 반환합니다."""
        if not features1 or not features2: return 0.0
        try:
            similarities = [cv2.compareHist(features1[key], features2[key], cv2.HISTCMP_CORREL) for key in features1 if key in features2]
            return np.mean(similarities) if similarities else 0.0
        except Exception as e:
            self.logger.warning(f"색상 유사도 계산 중 오류 발생: {e}")
            return 0.0
    
    def _calculate_bbox_similarity(self, bbox1: Tuple, bbox2: Tuple) -> float:
        """두 경계 상자의 크기 및 종횡비 유사도를 계산합니다."""
        try:
            w1, h1 = bbox1[2] - bbox1[0], bbox1[3] - bbox1[1]
            w2, h2 = bbox2[2] - bbox2[0], bbox2[3] - bbox2[1]
            if w1 <= 0 or h1 <= 0 or w2 <= 0 or h2 <= 0: return 0.0
            
            size_ratio = min(w1*h1, w2*h2) / max(w1*h1, w2*h2)
            aspect_ratio1 = w1 / h1
            aspect_ratio2 = w2 / h2
            aspect_ratio_sim = min(aspect_ratio1, aspect_ratio2) / max(aspect_ratio1, aspect_ratio2)
            return 0.7 * size_ratio + 0.3 * aspect_ratio_sim
        except Exception as e:
            self.logger.warning(f"경계 상자 유사도 계산 중 오류 발생: {e}")
            return 0.0
    
    def _calculate_position_proximity(self, bbox1: Tuple, bbox2: Tuple, frame_shape: Tuple) -> float:
        """두 경계 상자 중심점 간의 정규화된 거리를 기반으로 근접도를 계산합니다."""
        try:
            cx1, cy1 = (bbox1[0] + bbox1[2]) / 2, (bbox1[1] + bbox1[3]) / 2
            cx2, cy2 = (bbox2[0] + bbox2[2]) / 2, (bbox2[1] + bbox2[3]) / 2
            norm_dist = np.sqrt(((cx1-cx2)/frame_shape[1])**2 + ((cy1-cy2)/frame_shape[0])**2)
            return max(0, 1 - norm_dist / 0.5) # 0.5는 최대 허용 정규화 거리
        except Exception as e:
            self.logger.warning(f"위치 근접도 계산 중 오류 발생: {e}")
            return 0.0
    
    def _save_target_features(self, candidate: Dict):
        """등록된 타겟의 색상 및 형태 특징을 저장합니다."""
        try:
            with self.frame_lock:
                if not self.frame_buffer: return
                frame, _ = self.frame_buffer[-1]
                bbox = candidate['bbox']
                self.target_color_features = self._extract_color_features(frame, bbox)
                self.target_bbox_features = {'width': bbox[2] - bbox[0], 'height': bbox[3] - bbox[1]}
                self.logger.info(f"ID '{candidate['track_id']}' 타겟의 특징을 저장했습니다.")
        except Exception as e:
            self.logger.warning(f"타겟 특징 저장 중 오류 발생: {e}")

    def _update_target_features_during_tracking(self, frame: np.ndarray, bbox: Tuple):
        """추적 중인 타겟의 특징을 점진적으로 업데이트하여 변화에 대응합니다."""
        if not hasattr(self, '_feature_update_counter'): self._feature_update_counter = 0
        self._feature_update_counter += 1
        if self._feature_update_counter % 30 != 0: return # 30프레임마다 업데이트

        try:
            new_features = self._extract_color_features(frame, bbox)
            if new_features and self.target_color_features:
                for key in new_features:
                    if key in self.target_color_features:
                        # 지수 이동 평균(EMA)을 사용하여 부드럽게 업데이트
                        self.target_color_features[key] = (0.9 * self.target_color_features[key] + 0.1 * new_features[key])
                self.logger.debug("추적 중인 타겟의 색상 특징을 업데이트했습니다.")
        except Exception as e:
            self.logger.warning(f"타겟 특징 업데이트 중 오류 발생: {e}")
    
    def _attempt_fallback_tracking(self, frame: np.ndarray, detections: np.ndarray, timestamp: float):
        """DeepSORT가 타겟을 놓쳤을 때, 저장된 특징과 현재 감지된 객체들을 비교하여 타겟을 다시 찾습니다."""
        if detections.size == 0 or not self.target_color_features: return None
        
        try:
            best_match, best_score = None, 0.0
            for x1, y1, x2, y2, conf, cls in detections:
                bbox = (float(x1), float(y1), float(x2), float(y2))
                features = self._extract_color_features(frame, bbox)
                
                color_sim = self._calculate_color_similarity(self.target_color_features, features)
                bbox_sim = self._calculate_bbox_similarity(self.last_target_bbox, bbox)
                pos_prox = self._calculate_position_proximity(self.last_target_bbox, bbox, frame.shape)
                
                # 종합 점수: 색상 유사도에 가장 큰 가중치 부여
                composite_score = (0.5 * color_sim + 0.2 * bbox_sim + 0.2 * pos_prox + 0.1 * float(conf))
                
                if composite_score > best_score and composite_score > 0.4:
                    best_score, best_match = composite_score, {'bbox': bbox}
            
            if best_match:
                self.logger.info(f"대체 추적 성공: ID '{self.target_id}'를 점수 {best_score:.2f}로 재식별했습니다.")
                self.fallback_bbox = best_match['bbox']
                self.last_target_bbox = self.fallback_bbox
                return self._create_virtual_track(best_match)
            return None
        except Exception as e:
            self.logger.warning(f"대체 추적 중 오류 발생: {e}")
            return None
    
    def _create_virtual_track(self, match_data: Dict):
        """대체 추적 성공 시, DeepSORT의 Track 객체와 유사한 가상 트랙 객체를 생성합니다."""
        class VirtualTrack:
            def __init__(self, bbox, track_id):
                self._bbox = bbox
                self.track_id = track_id
            def to_tlbr(self): return self._bbox
            def is_confirmed(self): return True
        return VirtualTrack(match_data['bbox'], self.target_id)