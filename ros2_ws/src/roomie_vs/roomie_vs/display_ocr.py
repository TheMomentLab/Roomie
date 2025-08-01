#!/usr/bin/env python3

import cv2
import numpy as np
import time
import os
import easyocr
from typing import List, Tuple, Optional, Dict
from collections import Counter

class DisplayOCR:
    """디지털 디스플레이 OCR 클래스 (EasyOCR 기반)"""
    
    def __init__(self, logger):
        self.logger = logger
        
        # 캐싱 시스템
        self.cache = {}  # 캐시 저장소
        self.cache_timeout = 3.0  # 3초 캐시
        self.last_stable_result = None  # 마지막 안정된 결과
        self.recent_results = []  # 최근 결과들 (다수결용)
        
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        # 🔥 강화된 안정성 시스템
        self.consecutive_failures = 0  # 연속 실패 횟수
        self.max_consecutive_failures = 5  # 최대 연속 실패 허용치
        self.stable_result_timeout = 10.0  # 안정된 결과 유지 시간 (10초)
        self.last_stable_time = None  # 마지막 안정된 결과 시간
        
        # 유효한 층수 목록 (B2, 1~12층만)
        self.valid_floors = set(str(i) for i in range(1, 13))  # 1~12층만
        self.valid_floors.update(['B2'])  # B2만
=======
        # 유효한 층수 목록
        self.valid_floors = set(str(i) for i in range(1, 51))  # 1~50층
        self.valid_floors.update(['B1', 'B2', 'B3', 'B4', 'B5'])  # 지하층
>>>>>>> Stashed changes
=======
        # 유효한 층수 목록
        self.valid_floors = set(str(i) for i in range(1, 51))  # 1~50층
        self.valid_floors.update(['B1', 'B2', 'B3', 'B4', 'B5'])  # 지하층
>>>>>>> Stashed changes
        
        # 디버그 이미지 저장 경로 설정
        self.debug_dir = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_vs/debug"
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)
            logger.info(f"📁 디버그 폴더 생성: {self.debug_dir}")
        
        # 기본 설정
        self.config = {
            'debug_mode': True,            # 🔥 디버그 모드 ON (문제 확인용)
            'crop_margin': 5,              # 크롭 여백
            'use_simple_crop': False,      # 🎯 단순 크롭 사용 여부 (MultiModelOCR 방식)
        }
        
        # 🚀 EasyOCR 초기화
        self.gpu_mode = True  # GPU 모드 상태 추적
        self.ocr_enabled = True  # OCR 활성화 상태 추적
        
        try:
            self.logger.info("🔥 EasyOCR 초기화 중 (GPU 활성화)...")
            self.reader = easyocr.Reader(['en'], gpu=True, verbose=False)  # 🚀 GPU 활성화!
            self.logger.info("✅ EasyOCR 초기화 완료 (GPU 모드)!")
        except Exception as e:
            self.logger.error(f"❌ EasyOCR GPU 초기화 실패: {e}")
            self.logger.info("🔄 CPU 모드로 폴백 시도...")
            try:
                self.reader = easyocr.Reader(['en'], gpu=False, verbose=False)
                self.gpu_mode = False
                self.logger.info("✅ EasyOCR 초기화 완료 (CPU 모드)!")
            except Exception as e2:
                self.logger.error(f"❌ EasyOCR CPU 초기화도 실패: {e2}")
                self.reader = None
                self.ocr_enabled = False
    
    def recognize_from_display_bbox(self, full_image: np.ndarray, bbox: tuple) -> dict:
        """YOLO로 감지된 디스플레이 바운딩 박스에서 OCR 수행"""
        try:
            if not self.ocr_enabled:
                return {"text": "OCR_DISABLED", "digit_bbox": None}
                
            if self.reader is None:
                return {"text": "?", "digit_bbox": None}
                
            # 1단계: YOLO 바운딩 박스로 디스플레이 크롭
            x, y, w, h = bbox
            crop_margin = self.config['crop_margin']
            x1 = max(0, x - crop_margin)
            y1 = max(0, y - crop_margin)  
            x2 = min(full_image.shape[1], x + w + crop_margin)
            y2 = min(full_image.shape[0], y + h + crop_margin)
            
            display_image = full_image[y1:y2, x1:x2]
            
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
=======
>>>>>>> Stashed changes
            # 간단한 로그만
            if self.config.get('debug_mode', False):
                self.logger.info(f"디스플레이 크롭 영역: ({x1}, {y1}) -> ({x2}, {y2}), 크기: {display_image.shape}")
            
            # 디버깅용 이미지 저장 (옵션)
            if self.config.get('debug_mode', False):
                try:
                    cv2.imwrite(f'{self.debug_dir}/display_crop_debug_{time.strftime("%Y%m%d_%H%M%S")}.jpg', display_image)
                    self.logger.debug(f"디스플레이 크롭 이미지 저장: {self.debug_dir}/display_crop_debug_*.jpg")
                except:
                    pass
            
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            # 🎯 디스플레이 내에서 숫자 영역 ROI 추출 후 OCR 수행
            result_dict = self.recognize_display_with_smart_roi(display_image)
            
            # 전체 이미지 좌표계로 변환 (digit_bbox가 있는 경우)
            if result_dict.get('digit_bbox'):
                dx1, dy1, dx2, dy2 = result_dict['digit_bbox']
                # 디스플레이 크롭 좌표를 전체 이미지 좌표로 변환
                result_dict['digit_bbox'] = (
                    x1 + dx1,  # 절대 x1
                    y1 + dy1,  # 절대 y1
                    x1 + dx2,  # 절대 x2
                    y1 + dy2   # 절대 y2
                )
            
            return result_dict
            
        except Exception as e:
            self.logger.error(f"EasyOCR 디스플레이 인식 에러: {e}")
            return {"text": "?", "digit_bbox": None}
    
    def recognize_from_display_bbox_stable(self, full_image: np.ndarray, bbox: tuple) -> dict:
        """🎯 단순화된 안정적인 OCR - 직접적인 EasyOCR 사용"""
        try:
            if not self.ocr_enabled:
                return {"text": "OCR_DISABLED", "digit_bbox": None}
            
            if self.reader is None:
                return {"text": "?", "digit_bbox": None}
                
            # 🔥 단순 크롭 모드 (MultiModelOCR와 동일)
            if self.config.get('use_simple_crop', False):
                return self._simple_crop_ocr(full_image, bbox)
            
            # 기존 복잡한 크롭 모드 (호환성 유지)
            return self._complex_crop_ocr(full_image, bbox)
                
        except Exception as e:
            self.logger.error(f"안정적 OCR 에러: {e}")
            return {"text": "?", "digit_bbox": None}
    
    def _simple_crop_ocr(self, full_image: np.ndarray, bbox: tuple) -> dict:
        """🚀 단순 크롭 OCR (MultiModelOCR 방식 - 최적화)"""
        try:
            # 1단계: 단순 크롭 (MultiModelOCR와 동일)
            x, y, w, h = bbox
            roi_image = full_image[y:y+h, x:x+w]
            
            if roi_image.size == 0:
                return {"text": "?", "digit_bbox": None}
            
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
            if self.config.get('debug_mode', False):
                self.logger.info(f"🎯 단순 크롭: bbox({x},{y},{w},{h}) -> 크기:{roi_image.shape}")
            
>>>>>>> Stashed changes
=======
            if self.config.get('debug_mode', False):
                self.logger.info(f"🎯 단순 크롭: bbox({x},{y},{w},{h}) -> 크기:{roi_image.shape}")
            
>>>>>>> Stashed changes
            # 2단계: 직접 EasyOCR 수행 (MultiModelOCR와 동일 파라미터)
            results = self.reader.readtext(
                roi_image,
                allowlist='0123456789BF',
                width_ths=0.05,
                height_ths=0.05,
                paragraph=False,
                min_size=1,
                text_threshold=0.2,
                low_text=0.1,
                link_threshold=0.1,
                canvas_size=4000,
                mag_ratio=3.0
            )
            
            if not results:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
                if self.config.get('debug_mode', False):
                    self.logger.warn("❌ EasyOCR: 텍스트를 찾을 수 없음")
>>>>>>> Stashed changes
=======
                if self.config.get('debug_mode', False):
                    self.logger.warn("❌ EasyOCR: 텍스트를 찾을 수 없음")
>>>>>>> Stashed changes
                return {"text": "?", "digit_bbox": None}
            
            # 3단계: 가장 신뢰도 높은 결과 선택
            best_result = max(results, key=lambda x: x[2])
            bbox_points, text, confidence = best_result
            
            # 4단계: 텍스트 정리
            cleaned_text = self._clean_elevator_text(text)
            
            # 5단계: 바운딩박스 좌표 변환 (ROI -> 전체 이미지)
            digit_bbox = None
            if bbox_points:
                x_coords = [point[0] for point in bbox_points]
                y_coords = [point[1] for point in bbox_points]
                
                # 전체 이미지 좌표로 변환
                digit_bbox = (
                    x + int(min(x_coords)),
                    y + int(min(y_coords)),
                    x + int(max(x_coords)),
                    y + int(max(y_coords))
                )
            
            # 6단계: 결과 검증
            min_confidence = 0.3
            if confidence >= min_confidence and cleaned_text and cleaned_text != "?":
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
                if self.config.get('debug_mode', False):
                    self.logger.info(f"✅ 단순크롭 EasyOCR 성공: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f})")
                
>>>>>>> Stashed changes
=======
                if self.config.get('debug_mode', False):
                    self.logger.info(f"✅ 단순크롭 EasyOCR 성공: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f})")
                
>>>>>>> Stashed changes
                self.last_stable_result = cleaned_text
                
                return {
                    "text": cleaned_text, 
                    "digit_bbox": digit_bbox,
                    "confidence": confidence,
                    "raw_text": text
                }
            else:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
                if self.config.get('debug_mode', False):
                    self.logger.warn(f"❌ 단순크롭 인식 실패: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f})")
                
>>>>>>> Stashed changes
=======
                if self.config.get('debug_mode', False):
                    self.logger.warn(f"❌ 단순크롭 인식 실패: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f})")
                
>>>>>>> Stashed changes
                # 실패시 마지막 안정된 결과 유지
                if self.last_stable_result:
                    return {"text": self.last_stable_result, "digit_bbox": None, "from_cache": True}
                
                return {"text": "?", "digit_bbox": None}
                
        except Exception as e:
            self.logger.error(f"단순 크롭 OCR 에러: {e}")
            return {"text": "?", "digit_bbox": None}
    
    def _complex_crop_ocr(self, full_image: np.ndarray, bbox: tuple) -> dict:
        """🎛️ 복잡한 크롭 OCR (기존 방식 - 호환성 유지)"""
        try:
            # 1단계: YOLO 바운딩 박스로 디스플레이 크롭
            x, y, w, h = bbox
            crop_margin = self.config['crop_margin']
            x1 = max(0, x - crop_margin)
            y1 = max(0, y - crop_margin)  
            x2 = min(full_image.shape[1], x + w + crop_margin)
            y2 = min(full_image.shape[0], y + h + crop_margin)
            
            display_image = full_image[y1:y2, x1:x2]
            
            if self.config.get('debug_mode', False):
                self.logger.info(f"디스플레이 크롭 영역: ({x1}, {y1}) -> ({x2}, {y2}), 크기: {display_image.shape}")
            
            # 2단계: 스마트 ROI (30% 중앙 영역) 적용
            h_roi, w_roi = display_image.shape[:2]
            center_x = w_roi // 2
            roi_width = int(w_roi * 0.3)  # 전체 너비의 30%
            
            roi_x1 = max(0, center_x - roi_width // 2)
            roi_x2 = min(w_roi, center_x + roi_width // 2)
            roi_y1 = 0
            roi_y2 = h_roi
            
            roi_image = display_image[roi_y1:roi_y2, roi_x1:roi_x2]
            
            if self.config.get('debug_mode', False):
                self.logger.info(f"🎯 ROI 크롭: 원본({w_roi}x{h_roi}) -> ROI({roi_x2-roi_x1}x{roi_y2-roi_y1})")
            
            # 3단계: 직접적인 EasyOCR 수행
            results = self.reader.readtext(
                roi_image,
                allowlist='0123456789BF',
                width_ths=0.05,
                height_ths=0.05,
                paragraph=False,
                min_size=1,
                text_threshold=0.2,
                low_text=0.1,
                link_threshold=0.1,
                canvas_size=4000,
                mag_ratio=3.0
            )
            
            if not results:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
                if self.config.get('debug_mode', False):
                    self.logger.warn("❌ EasyOCR: 텍스트를 찾을 수 없음")
>>>>>>> Stashed changes
=======
                if self.config.get('debug_mode', False):
                    self.logger.warn("❌ EasyOCR: 텍스트를 찾을 수 없음")
>>>>>>> Stashed changes
                return {"text": "?", "digit_bbox": None}
            
            # 4단계: 가장 신뢰도 높은 결과 선택
            best_result = max(results, key=lambda x: x[2])
            bbox_points, text, confidence = best_result
            
            # 5단계: 텍스트 정리
            cleaned_text = self._clean_elevator_text(text)
            
            # 6단계: 바운딩박스 좌표 변환 (ROI -> 전체 이미지)
            if bbox_points:
                x_coords = [point[0] for point in bbox_points]
                y_coords = [point[1] for point in bbox_points]
                
                # ROI 내 좌표
                roi_digit_bbox = (
                    int(min(x_coords)),
                    int(min(y_coords)),
                    int(max(x_coords)),
                    int(max(y_coords))
                )
                
                # 전체 이미지 좌표로 변환
                digit_bbox = (
                    x1 + roi_x1 + roi_digit_bbox[0],
                    y1 + roi_y1 + roi_digit_bbox[1],
                    x1 + roi_x1 + roi_digit_bbox[2],
                    y1 + roi_y1 + roi_digit_bbox[3]
                )
            else:
                digit_bbox = None
            
            # 7단계: 결과 검증
            min_confidence = 0.3
            if confidence >= min_confidence and cleaned_text and cleaned_text != "?":
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
                if self.config.get('debug_mode', False):
                    self.logger.info(f"✅ EasyOCR 성공: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f})")
                
>>>>>>> Stashed changes
=======
                if self.config.get('debug_mode', False):
                    self.logger.info(f"✅ EasyOCR 성공: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f})")
                
>>>>>>> Stashed changes
                # 성공한 결과 캐싱
                self.last_stable_result = cleaned_text
                
                return {
                    "text": cleaned_text, 
                    "digit_bbox": digit_bbox,
                    "confidence": confidence,
                    "raw_text": text
                }
            else:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
=======
>>>>>>> Stashed changes
                if self.config.get('debug_mode', False):
                    self.logger.warn(f"❌ 인식 실패: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f})")
                    # 모든 후보 출력
                    for i, (_, candidate_text, candidate_confidence) in enumerate(results):
                        candidate_cleaned = self._clean_elevator_text(candidate_text)
                        self.logger.warn(f"   후보 {i+1}: '{candidate_text}' -> '{candidate_cleaned}' (신뢰도: {candidate_confidence:.3f})")
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
                
                # 실패시 마지막 안정된 결과 유지
                if self.last_stable_result:
                    if self.config.get('debug_mode', False):
                        self.logger.info(f"🔄 이전 안정 결과 유지: '{self.last_stable_result}'")
                    return {"text": self.last_stable_result, "digit_bbox": None, "from_cache": True}
                
                return {"text": "?", "digit_bbox": None}
                
        except Exception as e:
            self.logger.error(f"복잡한 크롭 OCR 에러: {e}")
            return {"text": "?", "digit_bbox": None}
    
    def _get_majority_vote_result(self, full_image: np.ndarray, bbox: tuple, attempts: int = 3) -> dict:
        """다수결 투표로 가장 신뢰할 수 있는 결과 선택"""
        results = []
        confidence_sum = {}
        
        for i in range(attempts):
            result = self.recognize_from_display_bbox(full_image, bbox)
            text = result.get('text', '?')
            confidence = result.get('confidence', 0.0)
            
            if text != '?' and len(text) <= 3 and text in self.valid_floors:
                results.append(text)
                confidence_sum[text] = confidence_sum.get(text, 0) + confidence
        
        if not results:
            return {"text": "?", "digit_bbox": None}
        
        # 다수결 투표
        vote_count = Counter(results)
        
        if self.config.get('debug_mode', False):
            self.logger.info(f"🗳️ 투표 결과: {dict(vote_count)}")
        
        # 가장 많이 나온 결과들 중 신뢰도가 높은 것 선택
        most_common = vote_count.most_common()
        winner_text = most_common[0][0]
        winner_count = most_common[0][1]
        
        # 최소 2번 이상 같은 결과가 나와야 신뢰
        if winner_count >= 2:
            avg_confidence = confidence_sum[winner_text] / winner_count
            return {"text": winner_text, "digit_bbox": None, "confidence": avg_confidence}
        elif winner_count == 1 and len(most_common) == 1:
            # 한 번만 나왔지만 유일한 결과면 사용
            avg_confidence = confidence_sum[winner_text]
            return {"text": winner_text, "digit_bbox": None, "confidence": avg_confidence}
        else:
            return {"text": "?", "digit_bbox": None}
    
    def get_current_floor_display(self) -> str:
        """현재 층수를 GUI 표시용으로 반환"""
        if self.last_stable_result:
            return f"FLOOR: {self.last_stable_result}"
        else:
            return "FLOOR: --"
    
    def clear_cache(self):
        """캐시 초기화"""
        self.cache.clear()
        self.recent_results.clear()
        if self.config.get('debug_mode', False):
            self.logger.info("💾 캐시 초기화 완료")
    
    def recognize_display_with_easyocr(self, display_image: np.ndarray) -> dict:
        """EasyOCR을 사용한 디스플레이 텍스트 인식 + 바운딩박스 반환"""
        try:
            if display_image is None or display_image.size == 0:
                return {"text": "?", "digit_bbox": None}
            
            if self.reader is None:
                return {"text": "?", "digit_bbox": None}
            
            # 🚀 EasyOCR로 텍스트 인식 (엘리베이터 디스플레이 특화 파라미터)
            results = self.reader.readtext(
                display_image,
                allowlist='0123456789BF',     # 🎯 숫자 + B(지하) + F(층) 문자만 허용
                width_ths=0.05,               # 🔥 매우 좁은 글자도 허용 (0.1 -> 0.05)
                height_ths=0.05,              # 🔥 매우 작은 글자도 허용 (0.1 -> 0.05)
                paragraph=False,              # 단일 라인 텍스트
                min_size=1,                   # 🔥 최소 크기를 1로 (거의 모든 크기 허용)
                text_threshold=0.2,           # 🔥 텍스트 인식 임계값 대폭 완화 (0.4 -> 0.2)
                low_text=0.1,                 # 🔥 매우 낮은 품질 텍스트도 허용 (0.2 -> 0.1)
                link_threshold=0.1,           # 🔥 문자 연결 임계값 완화 (0.2 -> 0.1)
                canvas_size=4000,             # 🔥 처리 해상도 더 증가 (3000 -> 4000)
                mag_ratio=3.0                 # 🔥 확대 비율 증가 (2.0 -> 3.0)
            )
            
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            # 결과 처리
            if not results:
                return {"text": "?", "digit_bbox": None}
            
            # 🔥 모든 결과 출력 (디버깅용) - 제거됨
=======
=======
>>>>>>> Stashed changes
            # 디버깅용 전처리 이미지 저장
            if self.config.get('debug_mode', False):
                try:
                    cv2.imwrite(f'{self.debug_dir}/easyocr_input_debug_{time.strftime("%Y%m%d_%H%M%S")}.jpg', display_image)
                    self.logger.debug(f"EasyOCR 입력 이미지 저장: {self.debug_dir}/easyocr_input_debug_*.jpg")
                except:
                    pass
            
            # 결과 처리
            if not results:
                if self.config.get('debug_mode', False):
                    self.logger.warn("❌ EasyOCR: 텍스트를 찾을 수 없음")
                return {"text": "?", "digit_bbox": None}
            
            # 🔥 모든 결과 출력 (디버깅용)
            if self.config.get('debug_mode', False):
                self.logger.info(f"🔍 EasyOCR 전체 결과 ({len(results)}개):")
                for i, (bbox_points, text, confidence) in enumerate(results):
                    self.logger.info(f"   결과 {i+1}: '{text}' (신뢰도: {confidence:.3f})")
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            
            # 🎯 가장 신뢰도가 높은 결과 선택
            best_result = max(results, key=lambda x: x[2])  # confidence 기준
            bbox_points, text, confidence = best_result
            
            # 바운딩박스 좌표 변환 (4개 점 → x1,y1,x2,y2)
            x_coords = [point[0] for point in bbox_points]
            y_coords = [point[1] for point in bbox_points]
            digit_bbox = (
                int(min(x_coords)),  # x1
                int(min(y_coords)),  # y1
                int(max(x_coords)),  # x2
                int(max(y_coords))   # y2
            )
            
            # 🎯 텍스트 후처리 (엘리베이터 특화)
            cleaned_text = self._clean_elevator_text(text)
            
            # 🔥 엘리베이터 디스플레이용 낮은 신뢰도 기준 (0.3으로 완화)
            min_confidence = 0.3  # 0.5 -> 0.3으로 낮춤
            
            # 결과 로깅
            if confidence >= min_confidence and cleaned_text and cleaned_text != "?":
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                return {"text": cleaned_text, "digit_bbox": digit_bbox}
            else:
=======
=======
>>>>>>> Stashed changes
                if self.config.get('debug_mode', False):
                    self.logger.info(f"✅ EasyOCR 성공: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f}, 기준: {min_confidence})")
                return {"text": cleaned_text, "digit_bbox": digit_bbox}
            else:
                if self.config.get('debug_mode', False):
                    self.logger.warn(f"❌ 인식 실패: '{text}' -> '{cleaned_text}' (신뢰도: {confidence:.3f}, 기준: {min_confidence})")
                    # 🔥 실패한 경우에도 모든 후보 표시
                    self.logger.warn(f"💡 다른 후보들:")
                    for i, (_, candidate_text, candidate_confidence) in enumerate(results):
                        candidate_cleaned = self._clean_elevator_text(candidate_text)
                        self.logger.warn(f"   후보 {i+1}: '{candidate_text}' -> '{candidate_cleaned}' (신뢰도: {candidate_confidence:.3f})")
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
                return {"text": "?", "digit_bbox": None}
                
        except Exception as e:
            if self.config.get('debug_mode', False):
                self.logger.error(f"EasyOCR 인식 에러: {e}")
            return {"text": "?", "digit_bbox": None}
    
    def recognize_display_with_smart_roi(self, display_image: np.ndarray) -> dict:
        """🎯 스마트 ROI: 디스플레이에서 좌우 중앙 30% 영역만 크롭 후 OCR"""
        try:
            if display_image is None or display_image.size == 0:
                return {"text": "?", "digit_bbox": None}
            
            if self.reader is None:
                return {"text": "?", "digit_bbox": None}
            
            h, w = display_image.shape[:2]
            
            # 🎯 좌우 중앙 30% 영역 계산
            center_x = w // 2
            roi_width = int(w * 0.3)  # 전체 너비의 30%
            
            # 좌우 크롭 (상하는 자르지 않음)
            x1 = center_x - roi_width // 2
            x2 = center_x + roi_width // 2
            y1 = 0      # 상단 그대로
            y2 = h      # 하단 그대로
            
            # 경계 확인
            x1 = max(0, x1)
            x2 = min(w, x2)
            
            # ROI 크롭
            roi_image = display_image[y1:y2, x1:x2]
            
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
=======
>>>>>>> Stashed changes
            if self.config.get('debug_mode', False):
                self.logger.info(f"🎯 ROI 크롭: 원본({w}x{h}) -> ROI({x2-x1}x{y2-y1}), 중앙 30% 영역")
                
            # 디버깅용 ROI 이미지 저장
            if self.config.get('debug_mode', False):
                try:
                    cv2.imwrite(f'{self.debug_dir}/roi_crop_debug_{time.strftime("%Y%m%d_%H%M%S")}.jpg', roi_image)
                except:
                    pass
            
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
            # 🚀 ROI 영역에서 EasyOCR 수행
            result_dict = self.recognize_display_with_easyocr(roi_image)
            
            # 🔄 digit_bbox를 전체 디스플레이 좌표계로 변환
            if result_dict.get('digit_bbox'):
                dx1, dy1, dx2, dy2 = result_dict['digit_bbox']
                # ROI 좌표를 전체 디스플레이 좌표로 변환
                result_dict['digit_bbox'] = (
                    x1 + dx1,  # ROI 시작점 + 상대 좌표
                    y1 + dy1,  
                    x1 + dx2,  
                    y1 + dy2   
                )
                
                if self.config.get('debug_mode', False):
                    self.logger.info(f"📍 digit_bbox 좌표 변환: ROI({dx1},{dy1},{dx2},{dy2}) -> 전체({x1+dx1},{y1+dy1},{x1+dx2},{y1+dy2})")
            
            return result_dict
            
        except Exception as e:
            self.logger.error(f"스마트 ROI OCR 에러: {e}")
            # 폴백: 전체 디스플레이에서 OCR 시도
            if self.config.get('debug_mode', False):
                self.logger.info("🔄 ROI 실패, 전체 디스플레이로 폴백")
            return self.recognize_display_with_easyocr(display_image)
    
    def _clean_elevator_text(self, text: str) -> str:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        """🔥 엘리베이터 층수 텍스트 정리 (강화된 필터링)"""
=======
        """엘리베이터 층수 텍스트 정리 (숫자+B,F 중심)"""
>>>>>>> Stashed changes
=======
        """엘리베이터 층수 텍스트 정리 (숫자+B,F 중심)"""
>>>>>>> Stashed changes
        if not text:
            return "?"
        
        # 공백 제거 및 대문자 변환
        cleaned = text.strip().upper()
        
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
        if self.config.get('debug_mode', False):
            self.logger.info(f"🧹 텍스트 정리: '{text}' -> '{cleaned}'")
        
>>>>>>> Stashed changes
=======
        if self.config.get('debug_mode', False):
            self.logger.info(f"🧹 텍스트 정리: '{text}' -> '{cleaned}'")
        
>>>>>>> Stashed changes
        # 빈 문자열 체크
        if not cleaned:
            return "?"
        
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        # 🚫 즉시 거부할 패턴들 (비정상적인 OCR 결과)
        reject_patterns = [
            r'^B$',           # "B"만 있는 경우
            r'^\d+B$',        # "1B", "2B" 등 (순서 잘못)
            r'^F\d*$',        # "F", "F1" 등
            r'.*[A-Z]{2,}.*', # 연속된 알파벳 2개 이상
            r'.*[^B0-9F].*',  # B, 숫자, F 외의 문자 포함
        ]
        
        import re
        for pattern in reject_patterns:
            if re.match(pattern, cleaned):
                return "?"
        
        # 🎯 유효한 패턴만 허용 (엄격한 필터링)
        
        # 지하층 패턴: B2만 허용
        basement_pattern = re.match(r'^B(\d+)$', cleaned)
        if basement_pattern:
            basement_num = int(basement_pattern.group(1))
            if basement_num == 2:  # B2만 허용
                return cleaned
            else:
                return "?"  # B2가 아니면 거부
        
        # 층수+F 패턴: 1F~12F만 허용
        floor_pattern = re.match(r'^(\d+)F$', cleaned)
        if floor_pattern:
            floor_num = int(floor_pattern.group(1))
            if 1 <= floor_num <= 12:
                return floor_pattern.group(1)  # F 제거하고 숫자만 반환
            else:
                return "?"  # 범위 벗어나면 거부
        
        # 순수 숫자만 (1~12층 범위)
        if cleaned.isdigit():
            floor_num = int(cleaned)
            if 1 <= floor_num <= 12:
                return cleaned
            elif floor_num == 0:
                return "1"  # 0층은 1층으로 변환
            else:
                return "?"  # 13층 이상은 거부
        
        # 🔥 모든 패턴에 맞지 않으면 거부
=======
=======
>>>>>>> Stashed changes
        # 🎯 층수 관련 패턴만 허용 (엄격한 필터링)
        import re
        
        # 지하층 패턴: B1, B2, B10 등
        basement_pattern = re.match(r'^B(\d+)$', cleaned)
        if basement_pattern:
            return cleaned  # B1, B2 등 그대로 반환
        
        # 층수+F 패턴: 1F, 2F, 12F 등
        floor_pattern = re.match(r'^(\d+)F?$', cleaned)
        if floor_pattern:
            floor_num = floor_pattern.group(1)
            return floor_num  # F 제거하고 숫자만 반환
        
        # 순수 숫자만 (1~50층 범위)
        if cleaned.isdigit():
            floor_num = int(cleaned)
            if 1 <= floor_num <= 50:
                return cleaned
            elif floor_num == 0:
                return "1"  # 0층은 1층으로 변환
        
        # 🔥 층수 관련이 아닌 텍스트는 거부
        if self.config.get('debug_mode', False):
            self.logger.warning(f"❌ 층수 패턴 불일치로 거부: '{cleaned}'")
        
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        return "?"
    
    # 🔧 기존 인터페이스 호환성을 위한 함수들
    def recognize_display(self, display_image: np.ndarray) -> str:
        """호환성을 위한 단순 텍스트 반환 함수"""
        result = self.recognize_display_with_easyocr(display_image)
        return result.get('text', '?')
    
    def switch_to_cpu_mode(self):
        """🔄 EasyOCR을 CPU 모드로 전환"""
        if not self.gpu_mode:
            self.logger.info("이미 CPU 모드입니다")
            return True
        
        try:
            self.logger.info("🔄 EasyOCR을 CPU 모드로 전환 중...")
            
            # 기존 GPU 리더 해제
            if self.reader:
                del self.reader
                self.reader = None
            
            # CPU 모드로 새 리더 생성
            self.reader = easyocr.Reader(['en'], gpu=False, verbose=False)
            self.gpu_mode = False
            self.ocr_enabled = True
            
            # 캐시 초기화
            self.clear_cache()
            
            self.logger.info("✅ EasyOCR CPU 모드 전환 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ CPU 모드 전환 실패: {e}")
            self.reader = None
            self.ocr_enabled = False
            return False
    
    def switch_to_gpu_mode(self):
        """🔄 EasyOCR을 GPU 모드로 전환 (주의: GPU 메모리 사용)"""
        if self.gpu_mode:
            self.logger.info("이미 GPU 모드입니다")
            return True
        
        try:
            self.logger.info("🔄 EasyOCR을 GPU 모드로 전환 중...")
            
            # 기존 CPU 리더 해제
            if self.reader:
                del self.reader
                self.reader = None
            
            # GPU 모드로 새 리더 생성
            self.reader = easyocr.Reader(['en'], gpu=True, verbose=False)
            self.gpu_mode = True
            self.ocr_enabled = True
            
            # 캐시 초기화
            self.clear_cache()
            
            self.logger.info("✅ EasyOCR GPU 모드 전환 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ GPU 모드 전환 실패: {e}")
            # GPU 실패 시 CPU 모드로 폴백
            self.logger.info("🔄 GPU 실패, CPU 모드로 폴백...")
            return self.switch_to_cpu_mode()
    
    def disable_ocr(self):
        """🚫 OCR 기능 완전 비활성화"""
        try:
            self.logger.info("🚫 OCR 기능을 비활성화합니다...")
            
            # 리더 해제
            if self.reader:
                del self.reader
                self.reader = None
            
            self.ocr_enabled = False
            self.clear_cache()
            
            self.logger.info("✅ OCR 기능 비활성화 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ OCR 비활성화 실패: {e}")
            return False
    
    def enable_ocr(self, use_gpu: bool = False):
        """✅ OCR 기능 다시 활성화"""
        if self.ocr_enabled and self.reader:
            self.logger.info("OCR이 이미 활성화되어 있습니다")
            return True
        
        try:
            self.logger.info(f"✅ OCR 기능을 활성화합니다 ({'GPU' if use_gpu else 'CPU'} 모드)...")
            
            if use_gpu:
                return self.switch_to_gpu_mode()
            else:
                return self.switch_to_cpu_mode()
                
        except Exception as e:
            self.logger.error(f"❌ OCR 활성화 실패: {e}")
            return False
    
    def get_status(self) -> dict:
        """📊 현재 OCR 상태 반환"""
        return {
            "ocr_enabled": self.ocr_enabled,
            "gpu_mode": self.gpu_mode,
            "reader_available": self.reader is not None,
            "last_stable_result": self.last_stable_result,
            "cache_size": len(self.cache)
        }

    def update_config(self, **kwargs):
        """설정 업데이트"""
        self.config.update(kwargs)
        if 'debug_mode' in kwargs:
            self.logger.info(f"디버그 모드: {kwargs['debug_mode']}")


class MultiModelOCR:
    """🔥 다양한 OCR 모델을 테스트할 수 있는 멀티 모델 OCR 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.models = {}
        self.valid_floors = set(str(i) for i in range(1, 51))  # 1~50층
        self.valid_floors.update(['B1', 'B2', 'B3', 'B4', 'B5'])  # 지하층
        
        # 디버그 폴더 설정
        self.debug_dir = "/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_vs/debug/multi_ocr"
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)
            logger.info(f"📁 멀티 OCR 디버그 폴더 생성: {self.debug_dir}")
        
        # 사용 가능한 모델들 초기화
        self._initialize_models()
    
    def _initialize_models(self):
        """사용 가능한 OCR 모델들을 초기화"""
        
        # 1. EasyOCR 초기화
        try:
            self.logger.info("🔥 EasyOCR 초기화 중...")
            reader = easyocr.Reader(['en'], gpu=True, verbose=False)
            self.models['easyocr'] = {
                'reader': reader,
                'type': 'easyocr',
                'status': 'active',
                'gpu_mode': True
            }
            self.logger.info("✅ EasyOCR 초기화 완료 (GPU)")
        except Exception as e:
            try:
                reader = easyocr.Reader(['en'], gpu=False, verbose=False)
                self.models['easyocr'] = {
                    'reader': reader,
                    'type': 'easyocr',
                    'status': 'active',
                    'gpu_mode': False
                }
                self.logger.info("✅ EasyOCR 초기화 완료 (CPU)")
            except Exception as e2:
                self.logger.error(f"❌ EasyOCR 초기화 실패: {e2}")
        
        # 2. PaddleOCR 초기화 시도
        try:
            from paddleocr import PaddleOCR
            self.logger.info("🔥 PaddleOCR 초기화 중...")
            paddle_ocr = PaddleOCR(use_angle_cls=True, lang='en', use_gpu=True, show_log=False)
            self.models['paddleocr'] = {
                'reader': paddle_ocr,
                'type': 'paddleocr',
                'status': 'active',
                'gpu_mode': True
            }
            self.logger.info("✅ PaddleOCR 초기화 완료")
        except Exception as e:
            self.logger.warning(f"⚠️ PaddleOCR 초기화 실패: {e}")
        
        # 3. Tesseract 초기화 시도
        try:
            import pytesseract
            # Tesseract 경로 확인
            tesseract_cmd = pytesseract.pytesseract.tesseract_cmd
            self.models['tesseract'] = {
                'reader': pytesseract,
                'type': 'tesseract',
                'status': 'active',
                'gpu_mode': False
            }
            self.logger.info("✅ Tesseract 초기화 완료")
        except Exception as e:
            self.logger.warning(f"⚠️ Tesseract 초기화 실패: {e}")
        
        # 4. TrOCR 초기화 시도 (Transformers 기반)
        try:
            from transformers import TrOCRProcessor, VisionEncoderDecoderModel
            import torch
            self.logger.info("🔥 TrOCR 초기화 중...")
            processor = TrOCRProcessor.from_pretrained("microsoft/trocr-base-printed")
            model = VisionEncoderDecoderModel.from_pretrained("microsoft/trocr-base-printed")
            
            # GPU 사용 가능하면 GPU로 이동
            if torch.cuda.is_available():
                model = model.cuda()
                gpu_mode = True
            else:
                gpu_mode = False
            
            self.models['trocr'] = {
                'processor': processor,
                'model': model,
                'type': 'trocr',
                'status': 'active',
                'gpu_mode': gpu_mode
            }
            self.logger.info(f"✅ TrOCR 초기화 완료 ({'GPU' if gpu_mode else 'CPU'})")
        except Exception as e:
            self.logger.warning(f"⚠️ TrOCR 초기화 실패: {e}")
        
        self.logger.info(f"🎯 초기화된 OCR 모델: {list(self.models.keys())}")
    
    def test_all_models_on_roi(self, image: np.ndarray, bbox: tuple) -> Dict[str, dict]:
        """모든 모델로 ROI에서 OCR 테스트 수행"""
        results = {}
        
        # ROI 추출
        x, y, w, h = bbox
        roi_image = image[y:y+h, x:x+w]
        
        if roi_image.size == 0:
            self.logger.error("❌ ROI 이미지가 비어있음")
            return results
        
        # 디버그용 ROI 이미지 저장
        timestamp = time.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        roi_path = f"{self.debug_dir}/roi_test_{timestamp}.jpg"
        cv2.imwrite(roi_path, roi_image)
        
        self.logger.info(f"🎯 {len(self.models)}개 모델로 ROI 테스트 시작 (크기: {roi_image.shape})")
        
        for model_name, model_info in self.models.items():
            try:
                start_time = time.time()
                
                if model_info['type'] == 'easyocr':
                    result = self._test_easyocr(roi_image, model_info)
                elif model_info['type'] == 'paddleocr':
                    result = self._test_paddleocr(roi_image, model_info)
                elif model_info['type'] == 'tesseract':
                    result = self._test_tesseract(roi_image, model_info)
                elif model_info['type'] == 'trocr':
                    result = self._test_trocr(roi_image, model_info)
                else:
                    continue
                
                processing_time = time.time() - start_time
                result['processing_time'] = processing_time
                result['model_name'] = model_name
                result['gpu_mode'] = model_info.get('gpu_mode', False)
                
                results[model_name] = result
                
                self.logger.info(f"✅ {model_name}: '{result.get('text', '?')}' "
                               f"(신뢰도: {result.get('confidence', 0):.3f}, "
                               f"시간: {processing_time:.3f}s)")
                
            except Exception as e:
                self.logger.error(f"❌ {model_name} 테스트 실패: {e}")
                results[model_name] = {
                    'text': '?',
                    'confidence': 0.0,
                    'error': str(e),
                    'model_name': model_name
                }
        
        # 결과 요약 저장
        self._save_test_results(results, roi_path, bbox)
        
        return results
    
    def _test_easyocr(self, image: np.ndarray, model_info: dict) -> dict:
        """EasyOCR 테스트"""
        reader = model_info['reader']
        
        results = reader.readtext(
            image,
            allowlist='0123456789BF',
            width_ths=0.05,
            height_ths=0.05,
            paragraph=False,
            min_size=1,
            text_threshold=0.2,
            low_text=0.1,
            link_threshold=0.1,
            canvas_size=4000,
            mag_ratio=3.0
        )
        
        if not results:
            return {'text': '?', 'confidence': 0.0}
        
        # 가장 신뢰도 높은 결과 선택
        best_result = max(results, key=lambda x: x[2])
        bbox_points, text, confidence = best_result
        
        cleaned_text = self._clean_elevator_text(text)
        
        return {
            'text': cleaned_text,
            'confidence': confidence,
            'raw_text': text,
            'bbox': bbox_points
        }
    
    def _test_paddleocr(self, image: np.ndarray, model_info: dict) -> dict:
        """PaddleOCR 테스트"""
        paddle_ocr = model_info['reader']
        
        results = paddle_ocr.ocr(image, cls=True)
        
        if not results or not results[0]:
            return {'text': '?', 'confidence': 0.0}
        
        # 가장 신뢰도 높은 결과 선택
        best_confidence = 0
        best_text = '?'
        best_bbox = None
        
        for line in results[0]:
            bbox, (text, confidence) = line
            if confidence > best_confidence:
                best_confidence = confidence
                best_text = text
                best_bbox = bbox
        
        cleaned_text = self._clean_elevator_text(best_text)
        
        return {
            'text': cleaned_text,
            'confidence': best_confidence,
            'raw_text': best_text,
            'bbox': best_bbox
        }
    
    def _test_tesseract(self, image: np.ndarray, model_info: dict) -> dict:
        """Tesseract 테스트"""
        import pytesseract
        
        # 이미지 전처리 (Tesseract는 전처리가 중요함)
        processed_image = self._preprocess_for_tesseract(image)
        
        # 설정: 숫자와 몇 개 문자만 허용
        config = '--oem 3 --psm 8 -c tessedit_char_whitelist=0123456789BF'
        
        try:
            # 텍스트 추출
            text = pytesseract.image_to_string(processed_image, config=config).strip()
            
            # 신뢰도 정보 추출
            data = pytesseract.image_to_data(processed_image, config=config, output_type=pytesseract.Output.DICT)
            confidences = [int(conf) for conf in data['conf'] if int(conf) > 0]
            avg_confidence = sum(confidences) / len(confidences) if confidences else 0
            
            cleaned_text = self._clean_elevator_text(text)
            
            return {
                'text': cleaned_text,
                'confidence': avg_confidence / 100.0,  # 0-1 범위로 정규화
                'raw_text': text,
                'bbox': None
            }
            
        except Exception as e:
            return {'text': '?', 'confidence': 0.0, 'error': str(e)}
    
    def _test_trocr(self, image: np.ndarray, model_info: dict) -> dict:
        """TrOCR 테스트"""
        from PIL import Image
        import torch
        
        processor = model_info['processor']
        model = model_info['model']
        
        # OpenCV 이미지를 PIL로 변환
        pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        
        # 전처리
        pixel_values = processor(images=pil_image, return_tensors="pt").pixel_values
        
        if model_info['gpu_mode']:
            pixel_values = pixel_values.cuda()
        
        # 예측
        generated_ids = model.generate(pixel_values)
        generated_text = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
        
        cleaned_text = self._clean_elevator_text(generated_text)
        
        # TrOCR는 신뢰도를 직접 제공하지 않으므로 텍스트 품질로 추정
        confidence = self._estimate_confidence(cleaned_text)
        
        return {
            'text': cleaned_text,
            'confidence': confidence,
            'raw_text': generated_text,
            'bbox': None
        }
    
    def _preprocess_for_tesseract(self, image: np.ndarray) -> np.ndarray:
        """Tesseract를 위한 이미지 전처리"""
        # 그레이스케일 변환
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 크기 확대 (Tesseract는 큰 이미지에서 더 잘 작동)
        scale_factor = 4
        height, width = gray.shape
        enlarged = cv2.resize(gray, (width * scale_factor, height * scale_factor), 
                            interpolation=cv2.INTER_CUBIC)
        
        # 이진화
        _, binary = cv2.threshold(enlarged, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # 노이즈 제거
        kernel = np.ones((2, 2), np.uint8)
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        return cleaned
    
    def _estimate_confidence(self, text: str) -> float:
        """텍스트 품질로 신뢰도 추정"""
        if text == '?':
            return 0.0
        
        if text in self.valid_floors:
            return 0.9
        
        # 숫자만 있으면 중간 신뢰도
        if text.isdigit():
            return 0.7
        
        return 0.3
    
    def _clean_elevator_text(self, text: str) -> str:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        """🔥 엘리베이터 층수 텍스트 정리 (B2, 1~12층만)"""
=======
        """엘리베이터 층수 텍스트 정리"""
>>>>>>> Stashed changes
=======
        """엘리베이터 층수 텍스트 정리"""
>>>>>>> Stashed changes
        if not text:
            return "?"
        
        cleaned = text.strip().upper()
        
        if not cleaned:
            return "?"
        
        import re
        
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        # 지하층 패턴: B2만 허용
        basement_pattern = re.match(r'^B(\d+)$', cleaned)
        if basement_pattern:
            basement_num = int(basement_pattern.group(1))
            if basement_num == 2:  # B2만 허용
                return cleaned
            else:
                return "?"  # B2가 아니면 거부
        
        # 층수+F 패턴: 1F~12F만 허용
        floor_pattern = re.match(r'^(\d+)F?$', cleaned)
        if floor_pattern:
            floor_num = int(floor_pattern.group(1))
            if 1 <= floor_num <= 12:
                return floor_pattern.group(1)
            else:
                return "?"
        
        # 순수 숫자만 (1~12층 범위)
        if cleaned.isdigit():
            floor_num = int(cleaned)
            if 1 <= floor_num <= 12:
                return cleaned
            elif floor_num == 0:
                return "1"
            else:
                return "?"
=======
=======
>>>>>>> Stashed changes
        # 지하층 패턴: B1, B2, B10 등
        basement_pattern = re.match(r'^B(\d+)$', cleaned)
        if basement_pattern:
            return cleaned
        
        # 층수+F 패턴: 1F, 2F, 12F 등
        floor_pattern = re.match(r'^(\d+)F?$', cleaned)
        if floor_pattern:
            floor_num = floor_pattern.group(1)
            return floor_num
        
        # 순수 숫자만 (1~50층 범위)
        if cleaned.isdigit():
            floor_num = int(cleaned)
            if 1 <= floor_num <= 50:
                return cleaned
            elif floor_num == 0:
                return "1"
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        
        return "?"
    
    def _save_test_results(self, results: Dict[str, dict], roi_path: str, bbox: tuple):
        """테스트 결과를 파일로 저장"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        result_file = f"{self.debug_dir}/test_results_{timestamp}.txt"
        
        with open(result_file, 'w', encoding='utf-8') as f:
            f.write(f"OCR 모델 테스트 결과\n")
            f.write(f"시간: {timestamp}\n")
            f.write(f"ROI 바운딩박스: {bbox}\n")
            f.write(f"ROI 이미지: {roi_path}\n")
            f.write("=" * 50 + "\n\n")
            
            # 결과를 신뢰도 순으로 정렬
            sorted_results = sorted(results.items(), 
                                  key=lambda x: x[1].get('confidence', 0), 
                                  reverse=True)
            
            for model_name, result in sorted_results:
                f.write(f"모델: {model_name}\n")
                f.write(f"  텍스트: '{result.get('text', '?')}'\n")
                f.write(f"  원본 텍스트: '{result.get('raw_text', '?')}'\n")
                f.write(f"  신뢰도: {result.get('confidence', 0):.3f}\n")
                f.write(f"  처리시간: {result.get('processing_time', 0):.3f}초\n")
                f.write(f"  GPU 모드: {result.get('gpu_mode', False)}\n")
                if 'error' in result:
                    f.write(f"  에러: {result['error']}\n")
                f.write("\n")
        
        self.logger.info(f"📊 테스트 결과 저장: {result_file}")
    
    def get_available_models(self) -> List[str]:
        """사용 가능한 모델 목록 반환"""
        return list(self.models.keys())
    
    def get_model_status(self) -> Dict[str, dict]:
        """모델 상태 정보 반환"""
        status = {}
        for name, info in self.models.items():
            status[name] = {
                'type': info['type'],
                'status': info['status'],
                'gpu_mode': info.get('gpu_mode', False)
            }
        return status 