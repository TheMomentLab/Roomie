"""
PersonTracker: DeepSORT + YOLOv8n 기반 사람 추적 모듈
기존 roomie_vs 인터페이스 (/vs/tracking, /vs/action/enroll 등)와 완전 호환
"""

import numpy as np
import cv2
import time
import threading
from collections import deque
from typing import Optional, Dict, List, Any, Tuple
import rclpy
from rclpy.node import Node
from roomie_msgs.msg import Tracking

try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
    DEEPSORT_AVAILABLE = True
except ImportError:
    print("Warning: DeepSort not available. Install with: pip install deep-sort-realtime")
    DEEPSORT_AVAILABLE = False


class PersonTracker:
    """
    사람 추적 모듈 - vs_node와 독립적으로 동작
    
    기존 인터페이스 호환:
    - 모드 0: 대기 (비활성)
    - 모드 1: 등록 준비
    - 모드 2: 추적 활성
    """
    
    def __init__(self, vs_node: Node, yolo_model=None):
        self.vs_node = vs_node
        self.yolo_model = yolo_model
        self.logger = vs_node.get_logger()
        
        # 모델별 person 클래스 인덱스 동적 해석 (기본 0)
        self.person_class_index = 0
        self._resolve_person_class_index()
        
        # 추적 상태
        self.current_mode = 0
        self.target_registered = False
        self.target_id = None
        self.tracking_active = False
        
        # DeepSORT 초기화
        if DEEPSORT_AVAILABLE:
            self.tracker = DeepSort(
                max_age=30,
                n_init=3,
                nms_max_overlap=0.5,
                max_cosine_distance=0.5
            )
        else:
            self.tracker = None
            self.logger.warning("DeepSORT not available, tracking disabled")
        
        # 프레임 버퍼 (스레드 안전)
        self.frame_buffer = deque(maxlen=5)
        self.frame_lock = threading.Lock()
        
        # 등록 관련
        self.registration_candidates = []
        self.registration_start_time = None
        self.registration_duration = 3.0  # 기본 3초
        self.track_appearances = {}  # track_id별 출현 횟수 기록
        
        # 추적 통계
        self.last_seen_time = None
        self.lost_count = 0
        self.reacquired_count = 0
        
        # 색상 기반 추적을 위한 타겟 특성 저장
        self.target_color_features = None  # Dict with color histograms
        self.target_bbox_features = None   # Last known bbox dimensions
        self.fallback_tracking_enabled = True  # YOLO 폴백 추적 활성화
        
        # bbox 관리 (DeepSORT vs 폴백)
        self.deepsort_bbox = None  # DeepSORT의 정확한 bbox
        self.fallback_bbox = None  # 폴백 추적의 YOLO bbox
        
        # 상태 전환 감지를 위한 이전 상태 저장
        self.previous_tracking_state = None  # "tracking", "lost", None
        self.is_target_distant = False  # 타겟이 멀어진 상태인지 추적
        
        # 프레임 기반 보수적 판단을 위한 카운터
        self.distant_frame_count = 0  # 멀어짐 조건을 만족한 연속 프레임 수
        self.normal_frame_count = 0   # 정상 조건을 만족한 연속 프레임 수
        self.required_frames = 5      # 상태 변경에 필요한 연속 프레임 수
        
        # ROS 퍼블리셔 (vs_node 통해 퍼블리시)
        self.tracking_pub = vs_node.create_publisher(
            Tracking, 
            '/vs/tracking', 
            10
        )
        
        self.logger.info("PersonTracker 초기화 완료 (DeepSORT)")
        
        # 오버레이 표시용 최근 검출/추적 상태
        self.last_detections = []  # List[Tuple[x1,y1,x2,y2,conf]]
        self.last_tracks = []      # List[Tuple[x1,y1,x2,y2]]
        self.last_target_bbox = None  # Tuple[x1,y1,x2,y2] | None

    def _resolve_person_class_index(self):
        """YOLO 모델의 names에서 'person' 계열 클래스 인덱스를 찾아 설정"""
        if not self.yolo_model:
            self.person_class_index = 0
            return
        try:
            names = getattr(self.yolo_model, 'names', None)
            if names is None:
                self.person_class_index = 0
                self.logger.warning("YOLO 모델 names가 없어 person 인덱스를 0으로 설정")
                return
            if isinstance(names, list):
                name_map = {i: n for i, n in enumerate(names)}
            elif isinstance(names, dict):
                name_map = names
            else:
                name_map = {}
            candidates = [k for k, v in name_map.items() if str(v).lower() in ("person", "people", "human")]
            if candidates:
                self.person_class_index = int(candidates[0])
                self.logger.info(f"person 클래스 인덱스: {self.person_class_index} ({name_map[self.person_class_index]})")
            else:
                # 기본값 유지하되 경고
                self.person_class_index = 0
                self.logger.warning("모델 names에 'person'이 없어 기본 인덱스 0 사용")
        except Exception as e:
            self.person_class_index = 0
            self.logger.warning(f"person 인덱스 해석 실패, 기본 0 사용: {e}")
    
    def set_mode(self, mode_id: int):
        """모드 변경 - vs_node.set_vs_mode_callback에서 호출"""
        prev_mode = self.current_mode
        self.current_mode = mode_id
        
        if mode_id == 1:  # 등록모드
            self._prepare_registration()
        elif mode_id == 2:  # 추적모드
            self._start_tracking()
        else:  # 기타 모드 (0, 3, 4, 5, 6)
            self._stop_tracking()
        
        self.logger.info(f"PersonTracker 모드 변경: {prev_mode} → {mode_id}")
    
    def push_frame(self, frame: np.ndarray, timestamp: float = None):
        """
        후방 카메라 프레임 전달
        vs_node._rear_camera_streaming_loop에서 호출
        """
        if timestamp is None:
            timestamp = time.time()
        
        with self.frame_lock:
            self.frame_buffer.append((frame.copy(), timestamp))
        
        # 현재 모드에 따른 처리
        if self.current_mode == 1:  # 등록모드
            self._process_registration_frame(frame, timestamp)
        elif self.current_mode == 2:  # 추적모드
            self._process_tracking_frame(frame, timestamp)
        else:
            # 대기 등 기타 모드에서도 YOLO로 상시 감지 (디버깅/확인용)
            self._process_idle_detection_frame(frame, timestamp)

    def _process_idle_detection_frame(self, frame: np.ndarray, timestamp: float):
        """대기 등 기타 모드에서의 상시 감지 처리 (YOLO만)"""
        detections = self._detect_persons(frame)
        
        # 디버깅: 대기모드 감지 결과 (주기적으로만)
        if not hasattr(self, '_idle_frame_count'):
            self._idle_frame_count = 0
        self._idle_frame_count += 1
        
        # 30프레임마다 한 번씩 로그 (약 2초마다) - 간소화
        if self._idle_frame_count % 30 == 0:
            if detections.size > 0:
                self.logger.debug(f"🔵 대기모드: {len(detections)}명 감지")
            else:
                self.logger.debug("🔵 대기모드: 감지 없음")
        
        self.last_detections = [
            (float(x1), float(y1), float(x2), float(y2), float(conf))
            for x1, y1, x2, y2, conf, cls in detections
        ] if detections.size > 0 else []
        # 트랙/타겟은 표시하지 않음
        self.last_tracks = []
        self.last_target_bbox = None
    
    def register_target(self, duration_sec: float = 3.0) -> Dict[str, Any]:
        """
        타겟 등록 시작 - enroll 액션에서 호출
        duration_sec 동안 가장 적합한 person 선택
        """
        if self.current_mode != 1:
            return {"success": False, "message": "등록모드가 아닙니다"}
        
        self.registration_duration = duration_sec
        self.registration_start_time = time.time()
        self.registration_candidates = []
        self.track_appearances = {}  # 등록 시작 시 초기화
        
        self.logger.info(f"타겟 등록 시작 (duration: {duration_sec}초)")
        return {"success": True, "message": "등록 시작"}
    
    def stop_tracking(self) -> Dict[str, Any]:
        """추적 중지 - stop_tracking 서비스에서 호출"""
        self._stop_tracking()
        return {"success": True, "message": "추적이 중지되었습니다"}
    
    def get_registration_progress(self) -> float:
        """등록 진행률 반환 - enroll 액션 피드백용"""
        if not self.registration_start_time or not self.registration_duration:
            return 0.0
        
        # 안전한 시간 계산
        try:
            if self.registration_start_time is None:
                return 0.0
            elapsed = time.time() - self.registration_start_time
            progress = min(elapsed / self.registration_duration, 1.0)
            return progress
        except (TypeError, ValueError):
            return 0.0
    
    def _prepare_registration(self):
        """등록모드 준비"""
        self.target_registered = False
        self.target_id = None
        self.registration_candidates = []
        
        # 이전 타겟 특성 초기화
        self.target_color_features = None
        self.target_bbox_features = None
        self.deepsort_bbox = None
        self.fallback_bbox = None
        
        # 상태 전환 추적 초기화
        self.previous_tracking_state = None
        self.is_target_distant = False
        self.distant_frame_count = 0
        self.normal_frame_count = 0
        self.lost_count = 0
        
        if DEEPSORT_AVAILABLE:
            self.tracker = DeepSort(
                max_age=30,
                n_init=3,
                nms_max_overlap=0.5,
                max_cosine_distance=0.5
            )
        # 트랙과 타겟만 리셋 (감지는 유지)
        self.last_tracks = []
        self.last_target_bbox = None
    
    def _start_tracking(self):
        """추적모드 시작"""
        if not self.target_registered:
            self.logger.warning("등록된 타겟이 없습니다")
            return
        
        self.tracking_active = True
        self.last_seen_time = time.time()
        self.lost_count = 0
        
        # 추적 시작 시 상태 초기화 (등록 직후 정상 상태로 가정)
        self.previous_tracking_state = "tracking"
        
        self.logger.info("추적 시작 (초기 상태: tracking)")
    
    def _stop_tracking(self):
        """추적 중지"""
        self.tracking_active = False
        self.target_registered = False
        self.target_id = None
        
        # 타겟 특성 초기화
        self.target_color_features = None
        self.target_bbox_features = None
        self.deepsort_bbox = None
        self.fallback_bbox = None
        
        # 상태 전환 추적 초기화
        self.previous_tracking_state = None
        self.is_target_distant = False
        self.distant_frame_count = 0
        self.normal_frame_count = 0
        self.lost_count = 0
        
        self.logger.info("추적 중지 (타겟 특성 및 상태 초기화 완료)")
        # 추적 중지 시 최근 상태 리셋
        self.last_detections = []
        self.last_tracks = []
        self.last_target_bbox = None
    
    def _process_registration_frame(self, frame: np.ndarray, timestamp: float):
        """등록 프레임 처리 - 대기모드와 동일하게 처리"""
        # YOLO로 사람 검출 (대기모드와 동일)
        detections = self._detect_persons(frame)
        
        # 디버깅: 프레임마다 인식 결과 로그 (간소화)
        if detections.size > 0:
            self.logger.debug(f"🟡 등록프레임: {len(detections)}명 감지")
        else:
            self.logger.debug("🟡 등록프레임: 감지 없음")
        
        # 오버레이용 최근 검출 저장 (대기모드와 동일)
        self.last_detections = [
            (float(x1), float(y1), float(x2), float(y2), float(conf))
            for x1, y1, x2, y2, conf, cls in detections
        ] if detections.size > 0 else []
        
        # 등록 프로세스는 따로 처리 (바운딩박스 표시와 무관)
        if self.registration_start_time:
            try:
                if self.registration_start_time is None:
                    return
                elapsed = time.time() - self.registration_start_time
                progress = elapsed / self.registration_duration * 100
                self.logger.info(f"🟡 등록진행: {progress:.1f}% ({elapsed:.1f}/{self.registration_duration:.1f}초)")
                
                if elapsed > self.registration_duration:
                    self.logger.info("🟡 등록시간 완료 - 최종 처리 시작")
                    self._finalize_registration()
                    return
            except (TypeError, ValueError) as e:
                self.logger.warning(f"등록 진행률 계산 오류: {e}")
                return
        if detections.size == 0:
            return
        
        # DeepSORT로 추적 (등록 기간 동안 안정성 확인)
        if self.tracker:
            ds_dets: List = []
            for x1, y1, x2, y2, conf, cls in detections:
                ds_dets.append(([float(x1), float(y1), float(x2), float(y2)], float(conf), int(cls)))
            
            # DeepSORT 처리 (디버깅 로그 간소화)
            self.logger.debug(f"🔥 DeepSORT 입력: {len(ds_dets)}개 감지")
            tracks = self.tracker.update_tracks(ds_dets, frame=frame)
            
            confirmed_count = sum(1 for tr in tracks if getattr(tr, 'is_confirmed', lambda: True)())
            self.logger.debug(f"🔥 DeepSORT 출력: {len(tracks)}개 트랙 ({confirmed_count}개 확정)")
            # 오버레이용 최근 트랙 저장 (등록 단계에서는 타겟 미지정)
            self.last_tracks = []
            for tr in tracks:
                if not getattr(tr, 'is_confirmed', lambda: True)():
                    continue
                bbox_tlbr = tr.to_tlbr() if hasattr(tr, 'to_tlbr') else None
                if bbox_tlbr is not None:
                    x1, y1, x2, y2 = bbox_tlbr
                    self.last_tracks.append((float(x1), float(y1), float(x2), float(y2)))
            
            # 후보 추가 (track_id, bbox) - 로그 간소화
            for track in tracks:
                if not getattr(track, 'is_confirmed', lambda: True)():
                    continue
                    
                track_id = getattr(track, 'track_id', None)
                bbox = track.to_tlbr() if hasattr(track, 'to_tlbr') else None
                if track_id is None or bbox is None:
                    continue
                
                stability = self._calculate_stability(track_id)
                
                candidate = {
                    'track_id': track_id,
                    'bbox': bbox,
                    'score': float(getattr(track, 'det_confidence', 1.0)),
                    'timestamp': timestamp,
                    'stability': stability
                }
                self.registration_candidates.append(candidate)
                
                # track_appearances 기록
                if track_id not in self.track_appearances:
                    self.track_appearances[track_id] = 0
                self.track_appearances[track_id] += 1
                
                self.logger.debug(f"🔥 후보추가: track_id={track_id}, stability={stability:.3f}")
    
    def _process_tracking_frame(self, frame: np.ndarray, timestamp: float):
        """추적 프레임 처리"""
        if not self.tracking_active or not self.target_registered:
            return
        
        # YOLO 검출
        detections = self._detect_persons(frame)
        # 오버레이용 최근 검출 저장
        self.last_detections = [
            (float(x1), float(y1), float(x2), float(y2), float(conf))
            for x1, y1, x2, y2, conf, cls in detections
        ] if detections.size > 0 else []
        
        # DeepSORT 업데이트
        if self.tracker:
            ds_dets: List = []
            for x1, y1, x2, y2, conf, cls in detections:
                ds_dets.append(([float(x1), float(y1), float(x2), float(y2)], float(conf), int(cls)))
            tracks = self.tracker.update_tracks(ds_dets, frame=frame)
            # 오버레이용 최근 트랙 저장
            self.last_tracks = []
            self.last_target_bbox = None
            for tr in tracks:
                if not getattr(tr, 'is_confirmed', lambda: True)():
                    continue
                bbox_tlbr = tr.to_tlbr() if hasattr(tr, 'to_tlbr') else None
                if bbox_tlbr is not None:
                    x1, y1, x2, y2 = bbox_tlbr
                    self.last_tracks.append((float(x1), float(y1), float(x2), float(y2)))
            
            # 등록된 타겟 찾기
            target_track = None
            for track in tracks:
                if getattr(track, 'track_id', None) == self.target_id:
                    target_track = track
                    # 타겟의 YOLO 검출 결과 찾기 (DeepSORT bbox 대신 사용)
                    if hasattr(track, 'to_tlbr'):
                        ds_bbox = track.to_tlbr()
                        if ds_bbox is not None:
                            # DeepSORT bbox 중심과 가장 가까운 YOLO 검출 찾기
                            ds_center_x = (ds_bbox[0] + ds_bbox[2]) / 2
                            ds_center_y = (ds_bbox[1] + ds_bbox[3]) / 2
                            
                            best_yolo_bbox = None
                            min_distance = float('inf')
                            
                            for x1, y1, x2, y2, conf in self.last_detections:
                                yolo_center_x = (x1 + x2) / 2
                                yolo_center_y = (y1 + y2) / 2
                                distance = ((ds_center_x - yolo_center_x)**2 + (ds_center_y - yolo_center_y)**2)**0.5
                                
                                if distance < min_distance:
                                    min_distance = distance
                                    best_yolo_bbox = (x1, y1, x2, y2)
                            
                            if best_yolo_bbox:
                                # YOLO bbox를 실제 타겟 bbox로 사용
                                self.deepsort_bbox = best_yolo_bbox
                                self.last_target_bbox = best_yolo_bbox
                                # 추적 중 타겟 특성 업데이트 (YOLO bbox 기준)
                                self._update_target_features_during_tracking(frame, best_yolo_bbox)
                    break
            
            # DeepSORT 추적 실패 시 폴백 추적 시도
            if target_track is None and self.fallback_tracking_enabled:
                target_track = self._attempt_fallback_tracking(frame, detections, timestamp)
            
            # 추적 상태 퍼블리시
            self._publish_tracking_result(target_track, frame.shape, timestamp)
    
    def _detect_persons(self, frame: np.ndarray) -> np.ndarray:
        """YOLO로 person 검출"""
        if not self.yolo_model:
            if not hasattr(self, '_no_model_logged') or not self._no_model_logged:
                self.logger.warning("❌ PersonTracker: YOLO 모델이 없습니다!")
                self._no_model_logged = True
            return np.empty((0, 6))
        
        try:
            # YOLOv8n으로 person만 검출 (confidence threshold 0.5)
            results = self.yolo_model(frame, classes=[self.person_class_index], conf=0.7, verbose=False)
            
            detections = []
            for r in results:
                if hasattr(r, 'boxes') and r.boxes is not None:
                    for box in r.boxes:
                        # 좌표/신뢰도를 안전하게 float로 변환
                        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                        try:
                            conf = float(box.conf[0].item())
                        except Exception:
                            conf = float(box.conf[0]) if hasattr(box.conf[0], '__float__') else 0.0
                        detections.append([x1, y1, x2, y2, conf, 0])
            
            return np.array(detections) if detections else np.empty((0, 6))
            
        except Exception as e:
            self.logger.warning(f"YOLO 검출 오류: {e}")
            return np.empty((0, 6))
    
    def _finalize_registration(self):
        """등록 완료 처리"""
        self.logger.info(f"🎯 등록 완료 처리 시작: 후보수={len(self.registration_candidates)}")
        if len(self.registration_candidates) > 0:
            best_candidate = max(self.registration_candidates, key=lambda x: x['stability'])
            self.logger.debug(f"🎯 최고 후보: track_id={best_candidate['track_id']}, stability={best_candidate['stability']:.3f}")
        
        if not self.registration_candidates:
            # DeepSORT 후보가 없어도 최근 감지된 사람으로 등록 시도
            if self.last_detections:
                # 가장 높은 confidence의 감지 결과로 등록
                best_detection = max(self.last_detections, key=lambda x: x[4])  # confidence 기준
                self.target_id = 1  # 기본 ID
                self.target_registered = True
                
                # YOLO 기반 등록 시에도 색상 특성 저장
                self._save_target_features_from_detection(best_detection)
                
                self.logger.info(f"YOLO 감지 기반 등록 완료: target_id={self.target_id}, conf={best_detection[4]:.2f}")
            else:
                self.logger.warning("등록할 후보가 없습니다")
                self.target_registered = False
                self.target_id = None
            
            # 등록 상태 리셋
            self.registration_start_time = None
            self.registration_candidates = []
            return
        
        # DeepSORT 후보 중 최소 안정성 기준을 만족하는 후보 선택
        stable_candidates = [
            c for c in self.registration_candidates 
            if c['stability'] >= 0.1  # 최소 10% 안정성 요구
        ]
        
        if not stable_candidates:
            self.logger.warning("👤 안정성 기준을 만족하는 후보가 없습니다")
            # YOLO 기반 등록으로 폴백
            if self.last_detections:
                best_detection = max(self.last_detections, key=lambda x: x[4])
                self.target_id = 1
                self.target_registered = True
                
                # YOLO 기반 등록 시에도 색상 특성 저장
                self._save_target_features_from_detection(best_detection)
                
                self.logger.info(f"YOLO 감지 기반 등록 완료: target_id={self.target_id}, conf={best_detection[4]:.2f}")
            else:
                self.logger.warning("👤 YOLO 감지 데이터도 없어 등록 실패")
                self.target_registered = False
                self.target_id = None
            
            # 등록 상태 리셋
            self.registration_start_time = None
            self.registration_candidates = []
            return
        
        # 가장 안정적인 후보 선택
        best_candidate = max(stable_candidates, key=lambda x: x['stability'])
        
        self.target_id = best_candidate['track_id']
        self.target_registered = True
        
        # 타겟의 색상 특성 저장 (최근 프레임 기준)
        self._save_target_features(best_candidate)
        
        # 등록 완료 후 등록 상태 리셋
        self.registration_start_time = None
        self.registration_candidates = []
        
        self.logger.info(f"DeepSORT 기반 등록 완료: track_id={self.target_id}")
    
    def _calculate_stability(self, track_id: int) -> float:
        """track_id의 안정성 점수 계산"""
        if not self.registration_start_time:
            return 0.0
        
        # 현재 track_id의 출현 횟수
        appearance_count = self.track_appearances.get(track_id, 0)
        
        # 지속성 점수 (출현 횟수 / 전체 프레임 수)
        try:
            duration = time.time() - self.registration_start_time
            expected_frames = duration * 10  # 10fps 가정 (프레임 처리 속도 고려)
            stability = appearance_count / max(expected_frames, 1)
            return min(stability, 1.0)  # 최대 1.0으로 제한
        except (TypeError, ValueError):
            return 0.0
    
    def _publish_tracking_result(self, target_track, frame_shape, timestamp):
        """추적 결과 퍼블리시 (/vs/tracking) - 상태 전환 시에만 발행"""
        current_state = None
        event_to_publish = None
        
        if target_track and hasattr(target_track, 'to_tlbr'):
            # 추적 성공
            current_state = "tracking"
            
            # LOST/REACQUIRED 이벤트 처리 (2,3)
            reacquired_event = None
            if self.lost_count > 0:
                reacquired_event = 3  # REACQUIRED
                self.reacquired_count += 1
                
                # 재획득 로그는 lost_count가 클 때만 상세히 출력
                if self.lost_count > 5:
                    self.logger.info(f"🎯 타겟 재획득! (lost_count: {self.lost_count})")
                else:
                    self.logger.debug(f"🎯 타겟 재획득! (lost_count: {self.lost_count})")
            
            # 멀어짐/정상상태 이벤트 처리 (0,1) - 독립적으로 체크
            distance_event = None
            if target_track.to_tlbr() is not None and len(frame_shape) >= 2:
                bbox = target_track.to_tlbr()
                bbox_width = bbox[2] - bbox[0]  # x2 - x1
                frame_width = frame_shape[1]    # width (cols)
                width_ratio = bbox_width / frame_width
                
                # 프레임 기반 보수적 판단: 연속 5프레임 조건 만족 시에만 상태 변경 (히스테리시스 제거, 0.2 기준)
                if width_ratio < 0.2:
                    # 멀어짐 조건 만족
                    self.distant_frame_count += 1
                    self.normal_frame_count = 0  # 리셋
                    
                    if self.distant_frame_count >= self.required_frames and not self.is_target_distant:
                        # 연속 5프레임 멀어짐 → 상태 변경
                        distance_event = 1  # 멀어짐
                        self.is_target_distant = True
                        self.logger.info(f"📏 타겟 멀어짐 감지! 너비 비율: {width_ratio:.3f} < 0.2 ({self.distant_frame_count}프레임 연속)")
                        
                elif width_ratio >= 0.2:
                    # 정상 조건 만족
                    self.normal_frame_count += 1
                    self.distant_frame_count = 0  # 리셋
                    
                    if self.normal_frame_count >= self.required_frames and self.is_target_distant:
                        # 연속 5프레임 정상 → 상태 변경
                        distance_event = 0  # 정상상태 복귀
                        self.is_target_distant = False
                        self.logger.info(f"📏 타겟 정상상태 복귀! 너비 비율: {width_ratio:.3f} >= 0.2 ({self.normal_frame_count}프레임 연속)")
            
            # 각 이벤트를 독립적으로 처리 (우선순위 없음)
            # REACQUIRED 이벤트 발행 (2,3)
            if reacquired_event is not None:
                self._publish_single_event(reacquired_event, target_track, current_state)
            
            # 거리 이벤트 발행 (0,1) - 완전 독립적
            if distance_event is not None:
                self._publish_single_event(distance_event, target_track, current_state)
            
            self.lost_count = 0
            self.last_seen_time = timestamp
        
        else:
            # 추적 실패
            self.lost_count += 1
            
            if self.lost_count == 1:
                # Tracking → Lost 상태 전환 (처음 잃음)
                current_state = "lost"
                event_to_publish = 2  # LOST
                self.logger.warning("❌ 타겟 추적 실패")
            else:
                # 계속 Lost 상태 (상태 변화 없음)
                current_state = "lost"
        
        # LOST 이벤트만 상태 전환에 따라 발행 (기존 로직 유지)
        if (self.previous_tracking_state != current_state and 
            current_state == "lost"):
            self._publish_single_event(2, None, current_state)  # LOST
        
        # 이전 상태 업데이트
        self.previous_tracking_state = current_state
    
    def _publish_single_event(self, event_id, target_track, current_state):
        """단일 이벤트 발행"""
        msg = Tracking()
        msg.id = int(self.target_id) if self.target_id is not None else -1
        msg.event = int(event_id)
        
        # 참고: Tracking.msg에는 위치 정보 필드가 없음 (id, event만 있음)
        
        self.tracking_pub.publish(msg)
        
        # 로그 출력
        event_names = {0: "정상상태복귀", 1: "멀어짐", 2: "LOST", 3: "REACQUIRED"}
        event_name = event_names.get(event_id, f"UNKNOWN({event_id})")
        
        if event_id in [0, 1]:  # 거리 이벤트
            self.logger.info(f"📡 거리 상태 변화: (이벤트: {event_name})")
        else:  # 추적 이벤트
            state_change = f"{self.previous_tracking_state} → {current_state}"
            self.logger.info(f"📡 추적 상태 변화: {state_change} (이벤트: {event_name})")
    
    def get_overlay_frame(self, base_frame: np.ndarray) -> np.ndarray:
        """
        UDP 스트리밍용 오버레이 프레임 생성
        vs_node에서 선택적으로 호출
        """
        overlay = base_frame.copy()
        

        
        # 모드별 오버레이 정보 표시
        if self.current_mode == 1:  # 등록모드
            # 등록용 바운딩박스 표시 (대기모드와 동일한 초록색)
            for i, det in enumerate(self.last_detections):
                x1, y1, x2, y2, conf = det
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                
                # 초록색 바운딩박스 제거
                
                # 중앙에 작은 원
                cv2.circle(overlay, center, 5, (0, 255, 0), -1)
                cv2.circle(overlay, center, 8, (255, 255, 255), 2)
                
                # confidence 표시
                cv2.putText(overlay, f"P{i+1}: {conf:.2f}", (p1[0], max(0, p1[1]-10)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 등록 상태 표시 (화면 하단)
            if self.target_registered:
                # 등록 완료 상태 표시
                text = f"REGISTERED: ID {self.target_id}"
                cv2.rectangle(overlay, (10, 450), (300, 480), (0, 100, 0), -1)  # 녹색 배경
                cv2.putText(overlay, text, (20, 470), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (255, 255, 255), 2)
            elif self.registration_start_time and self.registration_duration:
                # 등록 진행률 표시 (안전한 조건에서만)
                try:
                    progress = self.get_registration_progress()
                    text = f"REGISTRATION: {progress:.1%}"
                    cv2.rectangle(overlay, (10, 450), (300, 480), (0, 0, 0), -1)  # 검은 배경
                    cv2.putText(overlay, text, (20, 470), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.6, (0, 255, 255), 2)
                except Exception:
                    # 에러 발생 시 무시
                    pass
            else:
                # 등록 대기 상태 - 파란 배경 제거
                text = "REGISTRATION MODE"
                cv2.putText(overlay, text, (20, 470), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (255, 255, 255), 2)
        
        elif self.current_mode == 2:  # 추적모드
            status = "TRACKING" if self.target_registered else "NO TARGET"
            color = (0, 255, 0) if self.target_registered else (0, 0, 255)
            cv2.putText(overlay, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, color, 2)
            
            # 추적 상태 정보 표시
            if self.lost_count > 0:
                cv2.putText(overlay, f"LOST: {self.lost_count}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # 폴백 추적 활성화 표시
                if self.fallback_tracking_enabled:
                    cv2.putText(overlay, "FALLBACK MODE", (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # YOLO 검출 결과 표시 (박스 제거)
            for i, det in enumerate(self.last_detections):
                x1, y1, x2, y2, conf = det
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                # 초록색 박스 제거
                cv2.putText(overlay, f"YOLO:{conf:.2f}", (p1[0], p1[1]-50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        else:
            # 기타 모드(대기 등): 최근 YOLO 검출만 표시 (적절한 크기로)
            for i, det in enumerate(self.last_detections):
                x1, y1, x2, y2, conf = det
                p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
                center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                
                # 초록색 바운딩박스 제거
                
                # 중앙에 작은 원
                cv2.circle(overlay, center, 5, (0, 255, 0), -1)
                cv2.circle(overlay, center, 8, (255, 255, 255), 2)
                
                # confidence 표시
                cv2.putText(overlay, f"P{i+1}: {conf:.2f}", (p1[0], max(0, p1[1]-10)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return overlay 
    
    def _extract_color_features(self, image: np.ndarray, bbox: Tuple[float, float, float, float]) -> Dict[str, np.ndarray]:
        """
        바운딩박스 영역에서 색상 특성 추출
        상체/하체를 분리하여 각각의 색상 히스토그램 계산
        """
        try:
            x1, y1, x2, y2 = [int(coord) for coord in bbox]
            h, w = image.shape[:2]
            
            # 바운딩박스 유효성 검사
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            
            if x2 <= x1 or y2 <= y1:
                return {}
            
            # 사람 영역 크롭
            person_crop = image[y1:y2, x1:x2]
            crop_h, crop_w = person_crop.shape[:2]
            
            if crop_h < 20 or crop_w < 10:  # 너무 작은 영역은 제외
                return {}
            
            # 상체/하체 분리 (대략 2:3 비율)
            upper_end = int(crop_h * 0.4)  # 상체 40%
            lower_start = int(crop_h * 0.6)  # 하체는 60%부터
            
            upper_body = person_crop[:upper_end, :] if upper_end > 0 else person_crop
            lower_body = person_crop[lower_start:, :] if lower_start < crop_h else person_crop
            
            features = {}
            
            # RGB 히스토그램 계산
            for region_name, region_img in [("upper", upper_body), ("lower", lower_body), ("full", person_crop)]:
                if region_img.size > 0:
                    # RGB 각 채널별 히스토그램 (16 bins each)
                    hist_r = cv2.calcHist([region_img], [0], None, [16], [0, 256])
                    hist_g = cv2.calcHist([region_img], [1], None, [16], [0, 256])
                    hist_b = cv2.calcHist([region_img], [2], None, [16], [0, 256])
                    
                    # 정규화
                    hist_r = cv2.normalize(hist_r, hist_r, 0, 1, cv2.NORM_MINMAX).flatten()
                    hist_g = cv2.normalize(hist_g, hist_g, 0, 1, cv2.NORM_MINMAX).flatten()
                    hist_b = cv2.normalize(hist_b, hist_b, 0, 1, cv2.NORM_MINMAX).flatten()
                    
                    # HSV도 추가 (색상 정보 강화)
                    hsv_img = cv2.cvtColor(region_img, cv2.COLOR_BGR2HSV)
                    hist_h = cv2.calcHist([hsv_img], [0], None, [16], [0, 180])
                    hist_s = cv2.calcHist([hsv_img], [1], None, [16], [0, 256])
                    hist_v = cv2.calcHist([hsv_img], [2], None, [16], [0, 256])
                    
                    hist_h = cv2.normalize(hist_h, hist_h, 0, 1, cv2.NORM_MINMAX).flatten()
                    hist_s = cv2.normalize(hist_s, hist_s, 0, 1, cv2.NORM_MINMAX).flatten()
                    hist_v = cv2.normalize(hist_v, hist_v, 0, 1, cv2.NORM_MINMAX).flatten()
                    
                    # 결합된 특성 벡터
                    features[f"{region_name}_rgb"] = np.concatenate([hist_r, hist_g, hist_b])
                    features[f"{region_name}_hsv"] = np.concatenate([hist_h, hist_s, hist_v])
            
            return features
            
        except Exception as e:
            self.logger.warning(f"색상 특성 추출 오류: {e}")
            return {}
    
    def _calculate_color_similarity(self, features1: Dict[str, np.ndarray], features2: Dict[str, np.ndarray]) -> float:
        """두 색상 특성간의 유사도 계산 (0~1, 높을수록 유사)"""
        if not features1 or not features2:
            return 0.0
        
        try:
            similarities = []
            
            # 각 영역별 유사도 계산
            for key in features1:
                if key in features2:
                    # 코사인 유사도 계산
                    feat1, feat2 = features1[key], features2[key]
                    
                    norm1 = np.linalg.norm(feat1)
                    norm2 = np.linalg.norm(feat2)
                    
                    if norm1 > 0 and norm2 > 0:
                        cosine_sim = np.dot(feat1, feat2) / (norm1 * norm2)
                        similarities.append(max(0, cosine_sim))  # 음수 제거
            
            # 평균 유사도 반환
            return np.mean(similarities) if similarities else 0.0
            
        except Exception as e:
            self.logger.warning(f"색상 유사도 계산 오류: {e}")
            return 0.0
    
    def _calculate_bbox_similarity(self, bbox1: Tuple[float, float, float, float], 
                                  bbox2: Tuple[float, float, float, float]) -> float:
        """바운딩박스 크기/비율 유사도 계산 (0~1)"""
        try:
            x1, y1, x2, y2 = bbox1
            w1, h1 = x2 - x1, y2 - y1
            
            x1_, y1_, x2_, y2_ = bbox2  
            w2, h2 = x2_ - x1_, y2_ - y1_
            
            if w1 <= 0 or h1 <= 0 or w2 <= 0 or h2 <= 0:
                return 0.0
            
            # 크기 비율 유사도
            size_ratio = min(w1/w2, w2/w1) * min(h1/h2, h2/h1)
            
            # 종횡비 유사도  
            aspect1, aspect2 = w1/h1, w2/h2
            aspect_ratio = min(aspect1/aspect2, aspect2/aspect1)
            
            # 종합 유사도 (크기 70%, 종횡비 30%)
            return 0.7 * size_ratio + 0.3 * aspect_ratio
            
        except Exception as e:
            self.logger.warning(f"바운딩박스 유사도 계산 오류: {e}")
            return 0.0
    
    def _calculate_position_proximity(self, bbox1: Tuple[float, float, float, float],
                                    bbox2: Tuple[float, float, float, float],
                                    frame_shape: Tuple[int, int]) -> float:
        """바운딩박스 위치 근접도 계산 (0~1, 가까울수록 높음)"""
        try:
            # 중심점 계산
            cx1, cy1 = (bbox1[0] + bbox1[2]) / 2, (bbox1[1] + bbox1[3]) / 2
            cx2, cy2 = (bbox2[0] + bbox2[2]) / 2, (bbox2[1] + bbox2[3]) / 2
            
            # 정규화된 거리 계산
            h, w = frame_shape[:2]
            norm_dist = np.sqrt(((cx1-cx2)/w)**2 + ((cy1-cy2)/h)**2)
            
            # 근접도로 변환 (최대 0.5 화면 거리까지 고려)
            proximity = max(0, 1 - norm_dist / 0.5)
            return proximity
            
        except Exception as e:
            self.logger.warning(f"위치 근접도 계산 오류: {e}")
            return 0.0
    
    def _save_target_features(self, candidate: Dict):
        """DeepSORT 후보로부터 타겟 특성 저장"""
        try:
            # 최근 프레임에서 색상 특성 추출
            with self.frame_lock:
                if self.frame_buffer:
                    recent_frame, _ = self.frame_buffer[-1]
                    bbox = candidate['bbox']
                    
                    # 색상 특성 추출 및 저장
                    color_features = self._extract_color_features(recent_frame, bbox)
                    if color_features:
                        self.target_color_features = color_features
                        self.target_bbox_features = {
                            'width': bbox[2] - bbox[0],
                            'height': bbox[3] - bbox[1],
                            'aspect_ratio': (bbox[2] - bbox[0]) / (bbox[3] - bbox[1]) if bbox[3] > bbox[1] else 1.0
                        }
                        self.logger.info(f"🎨 타겟 색상 특성 저장 완료 (DeepSORT 기반)")
                    else:
                        self.logger.warning("🎨 타겟 색상 특성 추출 실패")
        except Exception as e:
            self.logger.warning(f"타겟 특성 저장 오류: {e}")
    
    def _save_target_features_from_detection(self, detection: Tuple[float, float, float, float, float]):
        """YOLO 검출 결과로부터 타겟 특성 저장"""
        try:
            # 최근 프레임에서 색상 특성 추출
            with self.frame_lock:
                if self.frame_buffer:
                    recent_frame, _ = self.frame_buffer[-1]
                    x1, y1, x2, y2, conf = detection
                    bbox = (x1, y1, x2, y2)
                    
                    # 색상 특성 추출 및 저장
                    color_features = self._extract_color_features(recent_frame, bbox)
                    if color_features:
                        self.target_color_features = color_features
                        self.target_bbox_features = {
                            'width': x2 - x1,
                            'height': y2 - y1,
                            'aspect_ratio': (x2 - x1) / (y2 - y1) if y2 > y1 else 1.0
                        }
                        self.logger.info(f"🎨 타겟 색상 특성 저장 완료 (YOLO 기반)")
                    else:
                        self.logger.warning("🎨 타겟 색상 특성 추출 실패")
        except Exception as e:
            self.logger.warning(f"타겟 특성 저장 오류 (YOLO): {e}")
    
    def _update_target_features_during_tracking(self, frame: np.ndarray, bbox: Tuple[float, float, float, float]):
        """추적 중 타겟 특성 주기적 업데이트 (프레임 30개마다)"""
        try:
            if not hasattr(self, '_feature_update_counter'):
                self._feature_update_counter = 0
            
            self._feature_update_counter += 1
            
            # 30프레임(약 2초)마다 한번씩 특성 업데이트
            if self._feature_update_counter % 30 == 0:
                new_color_features = self._extract_color_features(frame, bbox)
                if new_color_features and self.target_color_features:
                    # 기존 특성과 새 특성을 블렌딩 (90% 기존, 10% 새로운)
                    for key in new_color_features:
                        if key in self.target_color_features:
                            self.target_color_features[key] = (
                                0.9 * self.target_color_features[key] + 
                                0.1 * new_color_features[key]
                            )
                    self.logger.debug("🎨 타겟 색상 특성 업데이트 완료")
                
        except Exception as e:
            self.logger.warning(f"타겟 특성 업데이트 오류: {e}")
    
    def _attempt_fallback_tracking(self, frame: np.ndarray, detections: np.ndarray, timestamp: float):
        """DeepSORT 실패 시 YOLO + 색상/위치 기반 폴백 추적"""
        if detections.size == 0 or not self.target_color_features or not self.target_bbox_features:
            return None
        
        try:
            self.logger.debug("🔄 폴백 추적 시도: 색상/위치/크기 기반 매칭")
            
            best_match = None
            best_score = 0.0
            
            # 각 YOLO 검출에 대해 유사도 계산
            for x1, y1, x2, y2, conf, cls in detections:
                detection_bbox = (float(x1), float(y1), float(x2), float(y2))
                
                # 1. 색상 유사도 계산
                detection_color_features = self._extract_color_features(frame, detection_bbox)
                color_similarity = 0.0
                if detection_color_features:
                    color_similarity = self._calculate_color_similarity(
                        self.target_color_features, detection_color_features
                    )
                
                # 2. 크기/비율 유사도 계산
                target_bbox = (0, 0, self.target_bbox_features['width'], self.target_bbox_features['height'])
                bbox_similarity = self._calculate_bbox_similarity(target_bbox, 
                    (0, 0, x2-x1, y2-y1))  # 상대적 크기 비교
                
                # 3. 위치 근접도 계산 (마지막 알려진 위치 기준)
                position_proximity = 0.0
                if self.last_target_bbox:
                    position_proximity = self._calculate_position_proximity(
                        self.last_target_bbox, detection_bbox, frame.shape
                    )
                
                # 4. YOLO confidence 점수
                conf_score = float(conf)
                
                # 5. 종합 점수 계산 (가중 평균)
                # 색상 50%, 크기 20%, 위치 20%, confidence 10%
                composite_score = (
                    0.5 * color_similarity +
                    0.2 * bbox_similarity + 
                    0.2 * position_proximity +
                    0.1 * conf_score
                )
                
                self.logger.debug(f"🔄 후보 분석: conf={conf:.2f}, "
                               f"색상={color_similarity:.2f}, 크기={bbox_similarity:.2f}, "
                               f"위치={position_proximity:.2f} → 종합={composite_score:.2f}")
                
                # 최고 점수 후보 선택 (최소 임계값 0.4 이상)
                if composite_score > best_score and composite_score > 0.4:
                    best_score = composite_score
                    best_match = {
                        'bbox': detection_bbox,
                        'confidence': conf_score,
                        'composite_score': composite_score,
                        'color_sim': color_similarity,
                        'bbox_sim': bbox_similarity,
                        'pos_prox': position_proximity
                    }
            
            # 매칭 성공 시 가상 트랙 객체 생성
            if best_match:
                # 성공 로그는 주기적으로만 출력 (30프레임마다)
                if not hasattr(self, '_fallback_success_count'):
                    self._fallback_success_count = 0
                self._fallback_success_count += 1
                
                if self._fallback_success_count % 60 == 1:  # 첫 번째와 60의 배수마다 (더 적게)
                    self.logger.info(f"🎯 폴백 추적 성공! 종합점수: {best_score:.2f} "
                                   f"(색상:{best_match['color_sim']:.2f}, "
                                   f"크기:{best_match['bbox_sim']:.2f}, "
                                   f"위치:{best_match['pos_prox']:.2f})")
                else:
                    self.logger.debug(f"🎯 폴백 추적 성공! 종합점수: {best_score:.2f}")
                
                # 타겟 바운딩박스 업데이트 (YOLO bbox를 약간 축소하여 DeepSORT와 유사하게)
                x1, y1, x2, y2 = best_match['bbox']
                width = x2 - x1
                height = y2 - y1
                
                # 너비/높이를 40% 축소 (중앙 기준) - 등록모드 크기에 맞추기 위해
                shrink_ratio = 0.6
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                new_width = width * shrink_ratio
                new_height = height * shrink_ratio
                
                adjusted_bbox = (
                    center_x - new_width/2,
                    center_y - new_height/2,
                    center_x + new_width/2,
                    center_y + new_height/2
                )
                # 폴백 bbox 저장 (축소된 크기)
                self.fallback_bbox = adjusted_bbox
                self.last_target_bbox = adjusted_bbox
                
                # 가상 트랙 객체 생성 (DeepSORT 트랙과 호환)
                return self._create_virtual_track(best_match)
            else:
                self.logger.debug("🔄 폴백 추적 실패: 유사도 임계값 미달")
                return None
                
        except Exception as e:
            self.logger.warning(f"폴백 추적 오류: {e}")
            return None
    
    def _create_virtual_track(self, match_data: Dict):
        """폴백 매칭을 위한 가상 트랙 객체 생성"""
        class VirtualTrack:
            def __init__(self, bbox, track_id):
                self._bbox = bbox
                self.track_id = track_id
                
            def to_tlbr(self):
                return self._bbox
                
            def is_confirmed(self):
                return True
        
        return VirtualTrack(match_data['bbox'], self.target_id)