"""
PersonTracker: DeepSORT + YOLOv8n 기반 사람 추적 모듈
기존 roomie_vs 인터페이스 (/vs/tracking, /vs/action/enroll 등)와 완전 호환
"""

import numpy as np
import cv2
import time
import threading
from collections import deque
from typing import Optional, Dict, List, Any
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
        
        # 추적 통계
        self.last_seen_time = None
        self.lost_count = 0
        self.reacquired_count = 0
        
        # ROS 퍼블리셔 (vs_node 통해 퍼블리시)
        self.tracking_pub = vs_node.create_publisher(
            Tracking, 
            '/vs/tracking', 
            10
        )
        
        self.logger.info("PersonTracker 초기화 완료 (DeepSORT)")
    
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
        
        self.logger.info(f"타겟 등록 시작 (duration: {duration_sec}초)")
        return {"success": True, "message": "등록 시작"}
    
    def stop_tracking(self) -> Dict[str, Any]:
        """추적 중지 - stop_tracking 서비스에서 호출"""
        self._stop_tracking()
        return {"success": True, "message": "추적이 중지되었습니다"}
    
    def get_registration_progress(self) -> float:
        """등록 진행률 반환 - enroll 액션 피드백용"""
        if not self.registration_start_time:
            return 0.0
        
        elapsed = time.time() - self.registration_start_time
        progress = min(elapsed / self.registration_duration, 1.0)
        return progress
    
    def _prepare_registration(self):
        """등록모드 준비"""
        self.target_registered = False
        self.target_id = None
        self.registration_candidates = []
        if DEEPSORT_AVAILABLE:
            self.tracker = DeepSort(
                max_age=30,
                n_init=3,
                nms_max_overlap=0.5,
                max_cosine_distance=0.5
            )
    
    def _start_tracking(self):
        """추적모드 시작"""
        if not self.target_registered:
            self.logger.warning("등록된 타겟이 없습니다")
            return
        
        self.tracking_active = True
        self.last_seen_time = time.time()
        self.lost_count = 0
        self.logger.info("추적 시작")
    
    def _stop_tracking(self):
        """추적 중지"""
        self.tracking_active = False
        self.target_registered = False
        self.target_id = None
        self.logger.info("추적 중지")
    
    def _process_registration_frame(self, frame: np.ndarray, timestamp: float):
        """등록 프레임 처리"""
        if not self.registration_start_time:
            return
        
        elapsed = time.time() - self.registration_start_time
        if elapsed > self.registration_duration:
            # 등록 완료 - 가장 적합한 후보 선택
            self._finalize_registration()
            return
        
        # YOLO로 사람 검출
        detections = self._detect_persons(frame)
        if detections.size == 0:
            return
        
        # DeepSORT로 추적 (등록 기간 동안 안정성 확인)
        if self.tracker:
            ds_dets: List = []
            for x1, y1, x2, y2, conf, cls in detections:
                ds_dets.append(([float(x1), float(y1), float(x2), float(y2)], float(conf), int(cls)))
            tracks = self.tracker.update_tracks(ds_dets, frame=frame)
            
            # 후보 추가 (track_id, bbox)
            for track in tracks:
                if not getattr(track, 'is_confirmed', lambda: True)():
                    continue
                track_id = getattr(track, 'track_id', None)
                bbox = track.to_tlbr() if hasattr(track, 'to_tlbr') else None
                if track_id is None or bbox is None:
                    continue
                candidate = {
                    'track_id': track_id,
                    'bbox': bbox,
                    'score': float(getattr(track, 'det_confidence', 1.0)),
                    'timestamp': timestamp,
                    'stability': self._calculate_stability(track_id)
                }
                self.registration_candidates.append(candidate)
    
    def _process_tracking_frame(self, frame: np.ndarray, timestamp: float):
        """추적 프레임 처리"""
        if not self.tracking_active or not self.target_registered:
            return
        
        # YOLO 검출
        detections = self._detect_persons(frame)
        
        # DeepSORT 업데이트
        if self.tracker:
            ds_dets: List = []
            for x1, y1, x2, y2, conf, cls in detections:
                ds_dets.append(([float(x1), float(y1), float(x2), float(y2)], float(conf), int(cls)))
            tracks = self.tracker.update_tracks(ds_dets, frame=frame)
            
            # 등록된 타겟 찾기
            target_track = None
            for track in tracks:
                if getattr(track, 'track_id', None) == self.target_id:
                    target_track = track
                    break
            
            # 추적 상태 퍼블리시
            self._publish_tracking_result(target_track, frame.shape, timestamp)
    
    def _detect_persons(self, frame: np.ndarray) -> np.ndarray:
        """YOLO로 person 검출"""
        if not self.yolo_model:
            return np.empty((0, 6))
        
        try:
            # YOLOv8n으로 person만 검출
            results = self.yolo_model(frame, classes=[0], verbose=False)
            
            detections = []
            for r in results:
                if hasattr(r, 'boxes') and r.boxes is not None:
                    for box in r.boxes:
                        if box.cls == 0:  # person
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            conf = box.conf[0].cpu().numpy()
                            detections.append([x1, y1, x2, y2, conf, 0])
            
            return np.array(detections) if detections else np.empty((0, 6))
            
        except Exception as e:
            self.logger.warning(f"YOLO 검출 오류: {e}")
            return np.empty((0, 6))
    
    def _finalize_registration(self):
        """등록 완료 처리"""
        if not self.registration_candidates:
            self.logger.warning("등록할 후보가 없습니다")
            return
        
        # 가장 안정적인 후보 선택 (안정성 점수 기준)
        best_candidate = max(
            self.registration_candidates, 
            key=lambda x: x['stability']
        )
        
        self.target_id = best_candidate['track_id']
        self.target_registered = True
        
        self.logger.info(f"타겟 등록 완료: track_id={self.target_id}")
    
    def _calculate_stability(self, track_id: int) -> float:
        """track_id의 안정성 점수 계산"""
        # 등록 기간 동안 해당 track_id가 얼마나 지속적으로 나타났는지
        track_appearances = [
            c for c in self.registration_candidates 
            if c['track_id'] == track_id
        ]
        
        if not track_appearances:
            return 0.0
        
        # 지속성 점수 (출현 횟수 / 전체 프레임 수)
        duration = time.time() - self.registration_start_time
        expected_frames = duration * 15  # 15fps 가정
        stability = len(track_appearances) / max(expected_frames, 1)
        
        return min(stability, 1.0)
    
    def _publish_tracking_result(self, target_track, frame_shape, timestamp):
        """추적 결과 퍼블리시 (/vs/tracking)"""
        msg = Tracking()
        msg.id = self.target_id if self.target_id else -1
        
        if target_track and hasattr(target_track, 'to_tlbr'):
            # 추적 성공 → 좌표/스케일 제거, 이벤트만 유지
            if self.lost_count > 0:
                msg.event = 2  # REACQUIRED
                self.reacquired_count += 1
                self.logger.info(f"타겟 재획득 (lost_count: {self.lost_count})")
            else:
                msg.event = 0  # NONE
            
            self.lost_count = 0
            self.last_seen_time = timestamp
        
        else:
            # 추적 실패 → 이벤트만 발행
            self.lost_count += 1
            
            # 처음 잃었을 때만 LOST 이벤트
            if self.lost_count == 1:
                msg.event = 1  # LOST
                self.logger.warning("타겟 추적 실패")
            else:
                msg.event = 0  # NONE
        
        self.tracking_pub.publish(msg)
    
    def get_overlay_frame(self, base_frame: np.ndarray) -> np.ndarray:
        """
        UDP 스트리밍용 오버레이 프레임 생성
        vs_node에서 선택적으로 호출
        """
        if self.current_mode not in [1, 2]:
            return base_frame
        
        overlay = base_frame.copy()
        
        # 모드별 오버레이 정보 표시
        if self.current_mode == 1:  # 등록모드
            progress = self.get_registration_progress()
            text = f"REGISTRATION: {progress:.1%}"
            cv2.putText(overlay, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (0, 255, 255), 2)
        
        elif self.current_mode == 2 and self.tracking_active:  # 추적모드
            status = "TRACKING" if self.target_registered else "NO TARGET"
            color = (0, 255, 0) if self.target_registered else (0, 0, 255)
            cv2.putText(overlay, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, color, 2)
            
            if self.lost_count > 0:
                cv2.putText(overlay, f"LOST: {self.lost_count}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return overlay 