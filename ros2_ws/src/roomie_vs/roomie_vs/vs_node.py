#!/usr/bin/env python3

# 🔧 ROS2 동적 라이브러리 로딩 순서 문제 해결 (시스템 레벨 해결로 더 이상 불필요)
# import ctypes
# import os
# try:
#     # roomie_msgs 라이브러리들을 순서대로 강제 로드
#     roomie_lib_path = '/home/jinhyuk2me/project_ws/Roomie/ros2_ws/install/roomie_msgs/lib'
#     
#     # 필요한 라이브러리들만 명시적으로 로드
#     essential_libs = [
#         'libroomie_msgs__rosidl_generator_c.so',
#         'libroomie_msgs__rosidl_typesupport_c.so',
#         'libroomie_msgs__rosidl_typesupport_fastrtps_c.so', 
#         'libroomie_msgs__rosidl_typesupport_introspection_c.so',
#         'libroomie_msgs__rosidl_generator_py.so',
#     ]
#     
#     loaded_count = 0
#     for lib_name in essential_libs:
#         lib_path = f'{roomie_lib_path}/{lib_name}'
#         try:
#             if os.path.exists(lib_path):
#                 ctypes.CDLL(lib_path)
#                 loaded_count += 1
#         except Exception:
#             pass  # 개별 라이브러리 로딩 실패는 무시
#     
#     print(f"✅ roomie_msgs 라이브러리 pre-loading 완료 ({loaded_count}/{len(essential_libs)})")
# except Exception as e:
#     print(f"⚠️ roomie_msgs 라이브러리 pre-loading 실패: {e}")

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
import threading
import time
import os
import numpy as np
import cv2
import cv2
from typing import Optional, Tuple, List

# CNN 버튼 분류를 위한 추가 import
try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    import torchvision.transforms as transforms
    from PIL import Image
    import yaml
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

# 장애물 감지 import
from .obstacle_detector import ObstacleDetector

# UDP 비디오 스트리밍 import
from .udp_streamer import UDPVideoStreamer

# 사람 추적 모듈 import
from .person_tracking import PersonTracker

# CNN 모델 아키텍처 정의 (실제 훈련된 모델과 일치)
class BalancedButtonCNN(nn.Module):
    """성능과 메모리 균형을 맞춘 CNN 모델"""
    
    def __init__(self, num_classes=18):
        super(BalancedButtonCNN, self).__init__()
        
        # 균형잡힌 특징 추출
        self.features = nn.Sequential(
            # Block 1: 적당한 시작
            nn.Conv2d(3, 24, kernel_size=3, padding=1),
            nn.BatchNorm2d(24),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.2),
            
            # Block 2: 중간 확장
            nn.Conv2d(24, 48, kernel_size=3, padding=1),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Dropout(0.3),
            
            # Block 3: 충분한 특징
            nn.Conv2d(48, 96, kernel_size=3, padding=1),
            nn.BatchNorm2d(96),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4)),  # 적당한 출력
            nn.Dropout(0.3),
        )
        
        # 균형잡힌 분류기
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

# 디스플레이 OCR 모듈
from .display_ocr import DisplayOCR, MultiModelOCR

# ROS2 메시지 타입들
from geometry_msgs.msg import Point

# 커스텀 서비스
from roomie_msgs.srv import (
    ButtonStatus, 
    SetVSMode,
    ElevatorStatus, 
    DoorStatus,
    Location
)
from roomie_msgs.msg import Obstacle, GlassDoorStatus
from roomie_msgs.action import Enroll
from std_srvs.srv import Trigger

# OpenNI2 환경변수 설정
import os

def setup_openni2_environment():
    """OpenNI2 실행을 위한 환경변수 설정"""
    # 우선 실제 Downloads 디렉토리의 OpenNI2 경로 확인
    downloads_openni_path = os.path.expanduser("~/Downloads/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux/ros2_astra_camera/astra_camera/openni2_redist/x64")
    project_openni_path = os.path.expanduser("~/project_ws/Roomie/ros2_ws/src/roomie_vs/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux/ros2_astra_camera/astra_camera/openni2_redist/x64")
    
    # Downloads에 있는 것을 우선 확인
    if os.path.exists(downloads_openni_path):
        openni_path = downloads_openni_path
        print(f"✅ OpenNI2 경로 발견 (Downloads): {openni_path}")
    elif os.path.exists(project_openni_path):
        openni_path = project_openni_path
        print(f"✅ OpenNI2 경로 발견 (Project): {openni_path}")
    else:
        print(f"❌ OpenNI2 경로를 찾을 수 없습니다.")
        print(f"   확인한 경로들:")
        print(f"   - {downloads_openni_path}")
        print(f"   - {project_openni_path}")
        return False
    
    # 환경변수 설정
    os.environ['OPENNI2_REDIST'] = openni_path
    if 'LD_LIBRARY_PATH' in os.environ:
        os.environ['LD_LIBRARY_PATH'] += f":{openni_path}"
    else:
        os.environ['LD_LIBRARY_PATH'] = openni_path
    
    # PYTHONPATH에 사용자 라이브러리 경로 추가
    user_lib_path = "/home/jinhyuk2me/.local/lib/python3.12/site-packages"
    if 'PYTHONPATH' in os.environ:
        os.environ['PYTHONPATH'] += f":{user_lib_path}"
    else:
        os.environ['PYTHONPATH'] = user_lib_path
    
    print(f"✅ OpenNI2 환경변수 설정 완료: {openni_path}")
    print(f"   OPENNI2_REDIST: {os.environ.get('OPENNI2_REDIST')}")
    print(f"   LD_LIBRARY_PATH에 추가됨: {openni_path}")
    return True

# 환경설정 먼저 실행
if not setup_openni2_environment():
    import sys
    sys.exit(1)

# 환경설정 후 OpenNI2 import
try:
    from primesense import openni2
    from primesense import _openni2 as c_api
    print("✅ primesense 모듈 import 성공")
except ImportError as e:
    print(f"❌ primesense 모듈 import 실패: {e}")
    print("pip install primesense --break-system-packages 명령으로 설치하세요")
    import sys
    sys.exit(1)

class OpenNI2Camera:
    """OpenNI2를 직접 사용한 안정적인 Astra 카메라 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.is_running = False
        self.device = None
        self.rgb_stream = None
        self.depth_stream = None
        
        # 카메라 내부 파라미터 (Astra 실제값 추정)
        self.depth_fx = 1140.6  # 2배 증가 (스케일 보정)
        self.depth_fy = 1140.6  # 2배 증가 (스케일 보정)
        self.depth_cx = 320.0
        self.depth_cy = 240.0
        
        # 현재 프레임들
        self.current_depth = None
        self.current_color = None
        self.frame_lock = threading.Lock()
        
    def initialize(self) -> bool:
        """OpenNI2 카메라 초기화"""
        try:
            self.logger.info("OpenNI2 카메라 초기화 시작...")
            
            # OpenNI2 초기화
            openni2.initialize()
            self.logger.info("OpenNI2 초기화 완료")
            
            # 장치 열기
            self.device = openni2.Device.open_any()
            self.logger.info("장치 열기 완료")
            
            # 장치 정보 출력
            device_info = self.device.get_device_info()
            self.logger.info(f"장치: {device_info.name.decode()} ({device_info.vendor.decode()})")
            
            # RGB 스트림 생성
            try:
                self.rgb_stream = self.device.create_color_stream()
                self.rgb_stream.start()
                video_mode = self.rgb_stream.get_video_mode()
                self.logger.info(f"RGB 스트림: {video_mode.resolutionX}x{video_mode.resolutionY}@{video_mode.fps}fps")
            except Exception as e:
                self.logger.warning(f"RGB 스트림 생성 실패: {e}")
                self.rgb_stream = None
            
            # Depth 스트림 생성
            try:
                self.depth_stream = self.device.create_depth_stream()
                self.depth_stream.start()
                video_mode = self.depth_stream.get_video_mode()
                self.logger.info(f"Depth 스트림: {video_mode.resolutionX}x{video_mode.resolutionY}@{video_mode.fps}fps")
            except Exception as e:
                self.logger.warning(f"Depth 스트림 생성 실패: {e}")
                self.depth_stream = None
            
            if not self.rgb_stream and not self.depth_stream:
                self.logger.error("RGB와 Depth 스트림 모두 생성 실패")
                return False
            
            self.is_running = True
            self.logger.info("OpenNI2 카메라 초기화 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"OpenNI2 카메라 초기화 실패: {e}")
            return False
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """OpenNI2에서 RGB와 Depth 프레임 획득"""
        if not self.is_running:
            raise RuntimeError("카메라가 초기화되지 않았습니다")
        
        try:
            depth_image = None
            color_image = None
            
            # RGB 프레임 획득
            if self.rgb_stream:
                try:
                    rgb_frame = self.rgb_stream.read_frame()
                    rgb_data = rgb_frame.get_buffer_as_uint8()
                    rgb_array = np.frombuffer(rgb_data, dtype=np.uint8)
                    
                    h = rgb_frame.height
                    w = rgb_frame.width
                    rgb_image = rgb_array.reshape((h, w, 3))
                    
                    # BGR로 변환
                    color_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                    
                except Exception as e:
                    self.logger.warning(f"RGB 프레임 읽기 실패: {e}")
            
            # Depth 프레임 획득
            if self.depth_stream:
                try:
                    depth_frame = self.depth_stream.read_frame()
                    depth_data = depth_frame.get_buffer_as_uint16()
                    depth_array = np.frombuffer(depth_data, dtype=np.uint16)
                    
                    h = depth_frame.height
                    w = depth_frame.width  
                    depth_image = depth_array.reshape((h, w))
                    
                except Exception as e:
                    self.logger.warning(f"Depth 프레임 읽기 실패: {e}")
            
            # 현재 프레임 저장
            with self.frame_lock:
                if depth_image is not None:
                    self.current_depth = depth_image.copy()
                if color_image is not None:
                    self.current_color = color_image.copy()
            
            return depth_image, color_image
            
        except Exception as e:
            self.logger.error(f"프레임 획득 실패: {e}")
            raise RuntimeError(f"카메라 프레임 획득 실패: {e}")
    
    def pixel_to_3d(self, u: int, v: int, depth_mm: int, is_flipped: bool = False) -> Tuple[float, float, float]:
        """2D 픽셀 좌표를 3D 월드 좌표로 변환 (카메라 내부 파라미터 기반)"""
        if depth_mm <= 0:
            return 0.0, 0.0, 0.0
        
        # 좌우반전된 경우 원본 좌표로 변환
        if is_flipped:
            u = int(self.depth_cx * 2) - u  # 640 - u (해상도가 640x480인 경우)
            
        # Z축 계산: 역산으로 수정
        z = 1000.0 / depth_mm if depth_mm > 0 else 0.0  # 역산
        
        # 카메라 내부 파라미터를 사용한 정확한 3D 좌표 계산
        # X축 계산: 픽셀 오프셋을 실제 거리로 변환 (스케일링 조정)
        pixel_offset_x = u - self.depth_cx  # 중심에서 픽셀 차이
        x = (pixel_offset_x * z) / self.depth_fx  # 원래 크기로 조정
        
        # Y축 계산: 픽셀 오프셋을 실제 거리로 변환 (스케일링 조정)
        pixel_offset_y = v - self.depth_cy  # 중심에서 픽셀 차이
        y = (pixel_offset_y * z) / self.depth_fy  # 원래 크기로 조정
        
        return x, y, z
    
    def cleanup(self):
        """카메라 정리"""
        self.is_running = False
        
        try:
            if self.rgb_stream:
                self.rgb_stream.stop()
                self.rgb_stream = None
                
            if self.depth_stream:
                self.depth_stream.stop()
                self.depth_stream = None
                
            if self.device:
                self.device.close()
                self.device = None
                
            openni2.unload()
            self.logger.info("OpenNI2 카메라 정리 완료")
            
        except Exception as e:
            self.logger.warning(f"카메라 정리 중 에러: {e}")

class WebCamCamera:
    """일반 웹캠을 위한 카메라 클래스 (자동 탐지 지원)"""
    
    def __init__(self, logger, camera_id=None, camera_ids_to_try=None, camera_name="Webcam"):
        self.logger = logger
        self.preferred_camera_id = camera_id  # 우선 시도할 ID
        self.camera_ids_to_try = camera_ids_to_try or [0, 1, 2, 3]  # 시도할 ID 목록
        self.camera_name = camera_name
        self.actual_camera_id = None  # 실제 작동하는 ID
        self.is_running = False
        self.cap = None
        
        # 현재 프레임
        self.current_color = None
        self.current_depth = None  # 웹캠은 depth가 없지만 일관성을 위해 None으로 초기화
        self.frame_lock = threading.Lock()
        
    def initialize(self) -> bool:
        """웹캠 카메라 자동 탐지 초기화 (백엔드 정보 고려)"""
        try:
            # 우선 지정된 camera_id 시도 (있는 경우)
            if self.preferred_camera_id is not None:
                if self._try_camera_id(self.preferred_camera_id):
                    return True
            
            # 모든 카메라 스캔해서 백엔드 정보 고려하여 선택
            self.logger.info(f"{self.camera_name} 자동 탐지 시작... (시도할 ID: {self.camera_ids_to_try})")
            
            # 사용 가능한 카메라들을 모두 스캔
            available_cameras = self._scan_available_cameras()
            
            if not available_cameras:
                self.logger.error(f"{self.camera_name} 자동 탐지 실패: 사용 가능한 카메라 없음")
                return False
            
            # 카메라 타입에 따라 적절한 카메라 선택
            selected_camera = self._select_appropriate_camera(available_cameras)
            
            if selected_camera is not None:
                return self._try_camera_id(selected_camera['id'])
            
            self.logger.error(f"{self.camera_name} 자동 탐지 실패: 적절한 카메라를 찾을 수 없음")
            return False
            
        except Exception as e:
            self.logger.error(f"{self.camera_name} 초기화 실패: {e}")
            return False
    
    def _scan_available_cameras(self) -> list:
        """사용 가능한 모든 카메라 스캔하여 정보 수집"""
        available_cameras = []
        
        for camera_id in self.camera_ids_to_try:
            if self.preferred_camera_id is not None and camera_id == self.preferred_camera_id:
                continue  # 이미 시도했으므로 스킵
                
            try:
                cap = cv2.VideoCapture(camera_id)
                if not cap.isOpened():
                    cap.release()
                    continue
                
                # 테스트 프레임 읽기
                ret, frame = cap.read()
                if not ret or frame is None:
                    cap.release()
                    continue
                
                # 카메라 정보 수집
                backend = cap.getBackendName()
                height, width = frame.shape[:2]
                
                # 카메라 디바이스 이름 가져오기 (v4l2-ctl 사용)
                device_name = self._get_camera_device_name(camera_id)
                
                camera_info = {
                    'id': camera_id,
                    'backend': backend,
                    'width': width,
                    'height': height,
                    'device_name': device_name
                }
                
                available_cameras.append(camera_info)
                self.logger.info(f"발견된 카메라: ID={camera_id}, {width}x{height}, backend={backend}, device='{device_name}'")
                
                cap.release()
                
            except Exception as e:
                self.logger.debug(f"camera_id={camera_id} 스캔 중 에러: {e}")
                continue
        
        return available_cameras
    
    def _get_camera_device_name(self, camera_id: int) -> str:
        """v4l2-ctl을 사용하여 카메라 디바이스 이름 가져오기"""
        try:
            import subprocess
            device_path = f"/dev/video{camera_id}"
            
            # v4l2-ctl로 디바이스 정보 가져오기
            result = subprocess.run(
                ['v4l2-ctl', '--device', device_path, '--info'],
                capture_output=True, text=True, timeout=3
            )
            
            if result.returncode == 0:
                # Card 이름 추출 (실제 카메라 이름)
                for line in result.stdout.split('\n'):
                    if 'Card type' in line:
                        card_name = line.split(':', 1)[1].strip()
                        return card_name
                    elif 'Device name' in line:
                        device_name = line.split(':', 1)[1].strip()
                        return device_name
            
            return f"Unknown (ID={camera_id})"
            
        except Exception as e:
            self.logger.debug(f"카메라 디바이스 이름 가져오기 실패 (ID={camera_id}): {e}")
            return f"Unknown (ID={camera_id})"
    
    def _select_appropriate_camera(self, available_cameras: list) -> dict:
        """디바이스 이름을 기반으로 적절한 카메라 선택"""
        if not available_cameras:
            return None
        
        # 전방 USB 웹캠인 경우 (오직 HCA 카메라와 ABKO 카메라만)
        if "USB" in self.camera_name:
            self.logger.info("🎯 전방 USB 웹캠 선택 로직 시작 (HCA/ABKO만)")
            
            # 1순위: HCAM01N 찾기
            for camera in available_cameras:
                device_name = camera['device_name'].lower()
                if 'hcam01n' in device_name:
                    self.logger.info(f"✅ HCAM01N 전방카메라 선택: ID={camera['id']}, device='{camera['device_name']}'")
                    return camera
            
            # 2순위: ABKO 등 허용된 외부 USB 웹캠만 찾기
            for camera in available_cameras:
                device_name = camera['device_name'].lower()
                # 허용된 전방 카메라만 (HD Webcam 완전 제외)
                allowed_keywords = ['abko apc930', 'abko ap', 'apc930', 'abko', 'c920', 'c922', 'c930', 'logitech']
                
                # 디버그: 각 카메라 확인
                self.logger.info(f"🔍 전방 카메라 검사: {device_name}")
                
                # HD Webcam 완전 제외 (정확한 매칭)
                if device_name.startswith('hd webcam') or device_name == 'hd webcam: hd webcam':
                    self.logger.info(f"❌ HD Webcam 제외됨: {device_name}")
                    continue
                
                if any(keyword in device_name for keyword in allowed_keywords):
                    self.logger.info(f"📹 허용된 외부 USB 웹캠 선택: ID={camera['id']}, device='{camera['device_name']}'")
                    return camera
            
            # 전방용 카메라가 없으면 에러
            self.logger.error("❌ 전방용 카메라(HCA/ABKO)를 찾을 수 없습니다!")
            raise RuntimeError("전방용 카메라(HCA 또는 ABKO)를 찾을 수 없습니다.")
        
        # 후방 내장 카메라인 경우 (HD Webcam 무조건 선택)
        elif "Built-in" in self.camera_name:
            self.logger.info("🎯 후방 내장 카메라 선택 로직 시작 (HD Webcam 무조건)")
            
            # 1순위: 정확한 HD Webcam 무조건 찾기
            for camera in available_cameras:
                device_name = camera['device_name'].lower()
                if 'hd webcam: hd webcam' in device_name:
                    self.logger.info(f"✅ 정확한 HD Webcam 후방카메라 선택: ID={camera['id']}, device='{camera['device_name']}'")
                    return camera
            
            # HD Webcam이 없으면 에러 발생
            self.logger.error("❌ HD Webcam을 찾을 수 없습니다! 후방 카메라는 반드시 HD Webcam이어야 합니다.")
            raise RuntimeError("후방 카메라용 HD Webcam을 찾을 수 없습니다.")
        
        # 기본적으로 첫 번째 사용 가능한 카메라 선택
        return available_cameras[0]
    
    def _get_optimal_resolution(self, camera_id: int) -> tuple:
        """카메라별 최적 해상도 반환"""
        try:
            # HCAM01N (ID 0)은 800x600 지원
            if camera_id == 0:
                # 디바이스 이름으로 한번 더 확인
                device_name = self._get_camera_device_name(camera_id).lower()
                if 'hcam01n' in device_name:
                    self.logger.info(f"📐 HCAM01N 고해상도 설정: 800x600")
                    return (800, 600)
            
            # 기본 해상도 640x480
            self.logger.info(f"📐 기본 해상도 설정: 640x480 (camera_id={camera_id})")
            return (640, 480)
            
        except Exception as e:
            self.logger.warning(f"해상도 설정 오류: {e}, 기본값 사용")
            return (640, 480)
    
    def _try_camera_id(self, camera_id: int) -> bool:
        """특정 camera_id로 웹캠 초기화 시도"""
        try:
            self.logger.info(f"{self.camera_name} camera_id={camera_id} 시도 중...")
            
            cap = cv2.VideoCapture(camera_id)
            if not cap.isOpened():
                self.logger.debug(f"camera_id={camera_id} 열기 실패")
                cap.release()
                return False
            
            # 카메라별 최적 해상도 설정
            width, height = self._get_optimal_resolution(camera_id)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # 테스트 프레임 읽기
            ret, frame = cap.read()
            if not ret or frame is None:
                self.logger.debug(f"camera_id={camera_id} 프레임 읽기 실패")
                cap.release()
                return False
            
            # 성공!
            self.cap = cap
            self.actual_camera_id = camera_id
            self.is_running = True
            actual_height, actual_width = frame.shape[:2]
            
            # 카메라 백엔드 정보 확인
            backend = cap.getBackendName()
            
            # 설정된 해상도와 실제 해상도 비교
            if actual_width == width and actual_height == height:
                self.logger.info(f"✅ {self.camera_name} 초기화 성공: camera_id={camera_id}, {actual_width}x{actual_height}, backend={backend}")
            else:
                self.logger.warning(f"⚠️ {self.camera_name} 해상도 불일치: 요청({width}x{height}) → 실제({actual_width}x{actual_height}), camera_id={camera_id}, backend={backend}")
            
            return True
            
        except Exception as e:
            self.logger.debug(f"camera_id={camera_id} 시도 중 에러: {e}")
            return False
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """웹캠에서 프레임 획득 (depth는 None 반환)"""
        if not self.is_running or self.cap is None:
            raise RuntimeError("웹캠이 초기화되지 않았습니다")
        
        try:
            ret, color_image = self.cap.read()
            if not ret:
                self.logger.warning("웹캠 프레임 읽기 실패")
                return None, None
            
            # 현재 프레임 저장
            with self.frame_lock:
                self.current_color = color_image.copy()
            
            # depth는 없으므로 None 반환
            return None, color_image
            
        except Exception as e:
            self.logger.error(f"웹캠 프레임 획득 실패: {e}")
            raise RuntimeError(f"웹캠 프레임 획득 실패: {e}")
    
    def cleanup(self):
        """웹캠 정리"""
        self.is_running = False
        
        try:
            if self.cap:
                self.cap.release()
                self.cap = None
                
            self.logger.info(f"{self.camera_name} 정리 완료 (camera_id={self.actual_camera_id})")
            
        except Exception as e:
            self.logger.warning(f"웹캠 정리 중 에러: {e}")

class MultiCameraManager:
    """멀티 카메라 시스템 관리 클래스 (자동 웹캠 탐지 지원)"""
    
    def __init__(self, logger):
        self.logger = logger
        
        # 환경변수에서 camera_id 읽기 (선택사항)
        import os
        front_cam_id_env = os.getenv('FRONT_CAMERA_ID')
        rear_cam_id_env = os.getenv('REAR_CAMERA_ID')
        
        front_preferred_id = int(front_cam_id_env) if front_cam_id_env else None
        rear_preferred_id = int(rear_cam_id_env) if rear_cam_id_env else None
        
        # 전방 카메라들 - 고정 설정: 뎁스 + USB 웹캠
        self.front_webcam = WebCamCamera(
            logger, 
            camera_id=front_preferred_id,  # 환경변수 우선 또는 None
            camera_ids_to_try=[0, 1, 2, 3],  # 디바이스 이름으로 구분
            camera_name="Front USB Webcam"
        )
        self.front_depth = OpenNI2Camera(logger)  # 뎁스 카메라
        
        # 후방 카메라 - 고정 설정: 노트북 내장캠
        self.rear_webcam = WebCamCamera(
            logger, 
            camera_id=rear_preferred_id,  # 환경변수 우선 또는 None
            camera_ids_to_try=[0, 1, 2, 3],  # 모든 ID 시도하되 백엔드로 내장캠 선택
            camera_name="Rear Built-in Camera"
        )
        
        # 초기화 상태
        self.front_webcam_initialized = False
        self.front_depth_initialized = False
        self.rear_webcam_initialized = False
        
    def initialize_all_cameras(self):
        """모든 카메라 독립적 초기화 (레거시 메서드 - 모드 기반 초기화로 대체됨)"""
        self.logger.info("📌 레거시 전체 카메라 초기화 - 현재는 모드 기반 동적 초기화 사용")
        return True  # 대기모드는 항상 지원하므로 True
    
    def _initialize_front_cameras(self):
        """전방 카메라 시스템 초기화"""
        self.logger.info("🔧 전방 카메라 시스템 초기화...")
        front_success = False
        
        # 전방 웹캠 초기화
        try:
            if self.front_webcam.initialize():
                self.front_webcam_initialized = True
                self.logger.info("✅ 전방 웹캠 초기화 성공")
                front_success = True
            else:
                self.logger.warning("⚠️ 전방 웹캠 초기화 실패")
        except Exception as e:
            self.logger.warning(f"전방 웹캠 초기화 중 에러: {e}")
        
        # 전방 뎁스 카메라 초기화
        try:
            if self.front_depth.initialize():
                self.front_depth_initialized = True
                self.logger.info("✅ 전방 뎁스 카메라 초기화 성공")
                front_success = True
            else:
                self.logger.warning("⚠️ 전방 뎁스 카메라 초기화 실패")
        except Exception as e:
            self.logger.warning(f"전방 뎁스 카메라 초기화 중 에러: {e}")
            
        return front_success
    
    def _initialize_rear_camera(self):
        """후방 카메라 시스템 초기화"""
        self.logger.info("🔧 후방 카메라 시스템 초기화...")
        
        # 후방 웹캠 초기화
        try:
            if self.rear_webcam.initialize():
                self.rear_webcam_initialized = True
                self.logger.info("✅ 후방 웹캠 초기화 성공")
                return True
            else:
                self.logger.warning("⚠️ 후방 웹캠 초기화 실패")
                return False
        except Exception as e:
            self.logger.warning(f"후방 웹캠 초기화 중 에러: {e}")
            return False
    
    def get_camera_for_mode(self, mode_id):
        """모드에 따른 카메라 선택 (모든 모드에서 카메라 활성화)"""
        if mode_id in [0, 1, 2]:  # 후방 관련 모드들
            if self.rear_webcam_initialized:
                if mode_id == 0:
                    return self.rear_webcam, None, "Rear Webcam (Standby)"
                else:  # 등록모드(1), 추적모드(2)
                    return self.rear_webcam, None, "Rear Webcam"
            else:
                self.logger.warning(f"후방 웹캠이 초기화되지 않아 모드 {mode_id} 사용 불가")
                return None, None, "None"
                
        elif mode_id in [3, 4, 5, 6]:  # 전방 관련 모드들
            # 모든 전방 모드에서 웹캠 + 뎁스 카메라 제공 (모델 적용은 별도)
            if self.front_webcam_initialized and self.front_depth_initialized:
                if mode_id == 3:
                    return self.front_webcam, self.front_depth, "Front Webcam + Depth (Elevator Out)"
                elif mode_id == 4:
                    return self.front_webcam, self.front_depth, "Front Webcam + Depth (Elevator In)"
                elif mode_id == 5:
                    return self.front_webcam, self.front_depth, "Front Webcam + Depth"
                else:  # mode_id == 6
                    return self.front_webcam, self.front_depth, "Front Webcam + Depth (Standby)"
            elif self.front_webcam_initialized:
                self.logger.warning("뎁스 카메라 없이 웹캠만 사용")
                if mode_id == 3:
                    return self.front_webcam, None, "Front Webcam Only (Elevator Out)"
                elif mode_id == 4:
                    return self.front_webcam, None, "Front Webcam Only (Elevator In)"
                elif mode_id == 5:
                    return self.front_webcam, None, "Front Webcam Only"
                else:  # mode_id == 6
                    return self.front_webcam, None, "Front Webcam Only (Standby)"
            elif self.front_depth_initialized:
                # 뎁스 카메라만 있는 경우 - 단독으로도 사용 가능
                self.logger.info("웹캠 없이 뎁스 카메라만 사용")
                if mode_id == 3:
                    return self.front_depth, None, "Front Depth Only (Elevator Out)"
                elif mode_id == 4:
                    return self.front_depth, None, "Front Depth Only (Elevator In)"
                elif mode_id == 5:
                    return self.front_depth, None, "Front Depth Only"
                else:  # mode_id == 6
                    return self.front_depth, None, "Front Depth Only (Standby)"
            else:
                self.logger.warning("전방 카메라들이 초기화되지 않았습니다")
                return None, None, "None"
        else:
            # 시뮬레이션 모드 등
            return None, None, "Simulation Mode"
    
    def cleanup_all_cameras(self):
        """모든 카메라 정리"""
        self.logger.info("모든 카메라 정리 시작...")
        
        try:
            self.front_webcam.cleanup()
        except Exception as e:
            self.logger.warning(f"전방 웹캠 정리 중 에러: {e}")
            
        try:
            self.front_depth.cleanup()
        except Exception as e:
            self.logger.warning(f"전방 뎁스 정리 중 에러: {e}")
            
        try:
            self.rear_webcam.cleanup()
        except Exception as e:
            self.logger.warning(f"후방 웹캠 정리 중 에러: {e}")
        
        self.logger.info("모든 카메라 정리 완료!")
    
    def get_required_cameras_for_mode(self, mode_id):
        """모드별 필요한 카메라 목록 반환"""
        camera_requirements = {
            # 후방 관련 모드 - 모두 후방 웹캠 사용
            0: ['rear_webcam'],           # 후방 대기: 후방 웹캠 (항상 켜놓기)
            1: ['rear_webcam'],           # 등록 모드: 후방 웹캠만
            2: ['rear_webcam'],           # 추적 모드: 후방 웹캠만
            
            # 전방 관련 모드 - 모두 전방 웹캠 + 뎁스 사용 (카메라는 항상 켜두고 모델만 선택적 적용)
            3: ['front_webcam', 'front_depth'],  # 엘리베이터 외부: 전방 웹캠 + 뎁스
            4: ['front_webcam', 'front_depth'],  # 엘리베이터 내부: 전방 웹캠 + 뎁스
            5: ['front_webcam', 'front_depth'],  # 일반 주행: 전방 웹캠 + 뎁스
            6: ['front_webcam', 'front_depth'],  # 전방 대기: 전방 웹캠 + 뎁스 (항상 켜놓기)
            
            # 시뮬레이션 모드들 (카메라 불필요)
            100: [], 101: [], 102: [], 103: [], 104: []
        }
        
        return camera_requirements.get(mode_id, [])
    
    def initialize_cameras_for_mode(self, mode_id):
        """모드에 필요한 카메라만 초기화 (GPU 리소스 절약)"""
        required_cameras = self.get_required_cameras_for_mode(mode_id)
        
        self.logger.info(f"🎯 모드 {mode_id}에 필요한 카메라: {required_cameras}")
        
        # 전방/후방 카메라는 독립적으로 유지 - 기존 카메라 정리하지 않음
        
        # 필요한 카메라만 초기화
        success = True
        initialized_cameras = []
        
        if 'rear_webcam' in required_cameras:
            if not self.rear_webcam_initialized:
                if self._initialize_rear_camera():
                    initialized_cameras.append('후방 웹캠')
                else:
                    success = False
                    
        if 'front_webcam' in required_cameras:
            if not self.front_webcam_initialized:
                if self._initialize_front_webcam():
                    initialized_cameras.append('전방 웹캠')
                else:
                    success = False
                    
        if 'front_depth' in required_cameras:
            if not self.front_depth_initialized:
                if self._initialize_front_depth():
                    initialized_cameras.append('전방 뎁스')
                else:
                    success = False
        
        # 결과 로그
        if required_cameras:
            if initialized_cameras:
                self.logger.info(f"✅ 모드 {mode_id} 카메라 초기화 완료: {', '.join(initialized_cameras)}")
            else:
                self.logger.info(f"🔄 모드 {mode_id}: 카메라 이미 초기화됨")
        else:
            self.logger.info(f"🟡 모드 {mode_id}: 시뮬레이션 모드 - 카메라 불필요")
            
        return success or len(required_cameras) == 0
    
    def _initialize_front_webcam(self):
        """전방 웹캠만 초기화"""
        try:
            if self.front_webcam.initialize():
                self.front_webcam_initialized = True
                self.logger.info("✅ 전방 웹캠 초기화 성공")
                return True
            else:
                self.logger.warning("⚠️ 전방 웹캠 초기화 실패")
                return False
        except Exception as e:
            self.logger.warning(f"전방 웹캠 초기화 중 에러: {e}")
            return False
    
    def _initialize_front_depth(self):
        """전방 뎁스 카메라만 초기화"""
        try:
            if self.front_depth.initialize():
                self.front_depth_initialized = True
                self.logger.info("✅ 전방 뎁스 카메라 초기화 성공")
                return True
            else:
                self.logger.warning("⚠️ 전방 뎁스 카메라 초기화 실패")
                return False
        except Exception as e:
            self.logger.warning(f"전방 뎁스 카메라 초기화 중 에러: {e}")
            return False

class ButtonPressedCNN:
    """버튼 눌림 상태 감지 CNN 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.model_path = None
        self.model = None
        self.device = None
        self._initialize_model()
    
    def _initialize_model(self):
        """버튼 눌림 감지 CNN 모델 초기화"""
        try:
            import torch
            import torch.nn as nn
            import torch.nn.functional as F
            
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            
            # 모델 파일 경로 찾기
            model_path = self._find_pressed_model()
            if not model_path:
                self.logger.warning("⚠️ 버튼 눌림 감지 모델을 찾을 수 없습니다")
                return False
            
            # BalancedButtonCNN 아키텍처 정의 (실제 저장된 모델과 일치)
            class PressedButtonCNN(nn.Module):
                def __init__(self, num_classes=2):
                    super(PressedButtonCNN, self).__init__()
                    
                    # 균형잡힌 특징 추출 - 실제 모델 구조와 일치
                    self.features = nn.Sequential(
                        # Block 1: 적당한 시작
                        nn.Conv2d(3, 24, kernel_size=3, padding=1),
                        nn.BatchNorm2d(24),
                        nn.ReLU(inplace=True),
                        nn.MaxPool2d(kernel_size=2, stride=2),
                        nn.Dropout(0.2),
                        
                        # Block 2: 중간 확장
                        nn.Conv2d(24, 48, kernel_size=3, padding=1),
                        nn.BatchNorm2d(48),
                        nn.ReLU(inplace=True),
                        nn.MaxPool2d(kernel_size=2, stride=2),
                        nn.Dropout(0.3),
                        
                        # Block 3: 충분한 특징
                        nn.Conv2d(48, 96, kernel_size=3, padding=1),
                        nn.BatchNorm2d(96),
                        nn.ReLU(inplace=True),
                        nn.AdaptiveAvgPool2d((4, 4)),  # 적당한 출력
                        nn.Dropout(0.3),
                    )
                    
                    # 균형잡힌 분류기
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
            
            # 모델 로드
            self.model = PressedButtonCNN(num_classes=2)
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.to(self.device)
            self.model.eval()
            
            self.logger.info("✅ 버튼 눌림 감지 CNN 모델 로드 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ 버튼 눌림 CNN 모델 초기화 실패: {e}")
            return False
    
    def _find_pressed_model(self):
        """버튼 눌림 감지 모델 파일 찾기"""
        import os
        
        # 현재 스크립트 파일의 위치를 기준으로 모델 찾기
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        possible_paths = [
            # 스크립트 기준 상대 경로
            os.path.join(current_dir, "..", "training", "button_pressed_cnn", "best_roomie_button_model_32px_with_metadata.pth"),
            # 절대 경로들
            os.path.join(os.path.expanduser("~"), "project_ws", "Roomie", "ros2_ws", "src", "roomie_vs", "training", "button_pressed_cnn", "best_roomie_button_model_32px_with_metadata.pth"),
            os.path.join(os.getcwd(), "src", "roomie_vs", "training", "button_pressed_cnn", "best_roomie_button_model_32px_with_metadata.pth"),
            "src/roomie_vs/training/button_pressed_cnn/best_roomie_button_model_32px_with_metadata.pth",
            "training/button_pressed_cnn/best_roomie_button_model_32px_with_metadata.pth",
            "roomie_vs/training/button_pressed_cnn/best_roomie_button_model_32px_with_metadata.pth",
            "../training/button_pressed_cnn/best_roomie_button_model_32px_with_metadata.pth"
        ]
        
        for path in possible_paths:
            abs_path = os.path.abspath(path)
            if os.path.exists(abs_path):
                self.logger.info(f"📂 버튼 눌림 모델 발견: {abs_path}")
                return abs_path
        
        # 디버깅을 위해 모든 경로 출력
        self.logger.debug("🔍 버튼 눌림 모델 검색 경로들:")
        for path in possible_paths:
            abs_path = os.path.abspath(path)
            self.logger.debug(f"   - {abs_path} (존재: {os.path.exists(abs_path)})")
        
        return None
    
    def classify_pressed(self, color_image: np.ndarray, bbox: tuple) -> dict:
        """버튼 ROI에서 눌림 상태 분류"""
        if self.model is None:
            return {'is_pressed': False, 'confidence': 0.0, 'method': 'no_model'}
            
        try:
            import torch
            
            # 1. ROI 추출
            x1, y1, x2, y2 = bbox
            roi = color_image[y1:y2, x1:x2]
            
            if roi.size == 0:
                return {'is_pressed': False, 'confidence': 0.0, 'method': 'empty_roi'}
            
            # 2. 32x32로 리사이즈
            roi_resized = cv2.resize(roi, (32, 32))
            
            # 3. ImageNet 정규화
            roi_normalized = self._preprocess_image(roi_resized)
            
            # 4. CNN 추론
            with torch.no_grad():
                roi_tensor = torch.from_numpy(roi_normalized).float().unsqueeze(0).to(self.device)
                outputs = self.model(roi_tensor)
                probabilities = torch.softmax(outputs, dim=1)
                
                # 0: pressed, 1: unpressed
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
            self.logger.error(f"CNN 분류 실패: {e}")
            return {'is_pressed': False, 'confidence': 0.0, 'method': 'error'}
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """ImageNet 표준 전처리"""
        # BGR → RGB 변환
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 0-1 정규화
        image_normalized = image_rgb.astype(np.float32) / 255.0
        
        # ImageNet 정규화 (float32로 명시적 변환)
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        
        image_normalized = (image_normalized - mean) / std
        
        # CHW 순서로 변경
        image_chw = np.transpose(image_normalized, (2, 0, 1))
        
        return image_chw


class CNNButtonClassifier:
    """CNN 기반 버튼 분류 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.model = None
        self.transform = None
        self.torch_device = torch.device('cuda' if torch.cuda.is_available() else 'cpu') if TORCH_AVAILABLE else None
        self.class_names = []
        self.button_id_mapping = {}
        
        if TORCH_AVAILABLE:
            # 모델 로드
            self._load_cnn_model()
        else:
            self.logger.warning("⚠️ PyTorch가 설치되지 않아 CNN 버튼 분류를 사용할 수 없습니다")
    
    def _load_cnn_model(self):
        """CNN 모델과 설정 로드"""
        try:
            # 모델 경로 찾기 (설치된 패키지 경로와 소스 경로 모두 시도)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            
            # 1. 설치된 패키지 경로 시도
            from ament_index_python.packages import get_package_share_directory
            try:
                share_dir = get_package_share_directory('roomie_vs')
                model_dir = os.path.join(share_dir, 'training', 'button_cnn')
            except Exception:
                # 2. 소스 경로 시도 (개발 중)
                model_dir = os.path.join(current_dir, '..', 'training', 'button_cnn')
            
            model_path = os.path.join(model_dir, 'best_smart_balanced_model_32px_with_metadata.pth')
            config_path = os.path.join(model_dir, 'best_smart_balanced_model_32px_with_metadata_config.yaml')
            
            if not os.path.exists(model_path):
                # 소스 경로도 시도
                source_model_dir = os.path.join(current_dir, '..', 'training', 'button_cnn')
                model_path = os.path.join(source_model_dir, 'best_smart_balanced_model_32px_with_metadata.pth')
                config_path = os.path.join(source_model_dir, 'best_smart_balanced_model_32px_with_metadata_config.yaml')
                
                if not os.path.exists(model_path):
                    self.logger.warning(f"⚠️ CNN 모델 파일을 찾을 수 없습니다: {model_path}")
                    return False
                
            if not os.path.exists(config_path):
                self.logger.warning(f"⚠️ CNN 설정 파일을 찾을 수 없습니다: {config_path}")
                return False
            
            # 설정 파일 로드
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.class_names = config['dataset_info']['class_names']
            
            # 클래스명을 버튼 ID로 매핑
            self._create_button_mapping()
            
            # 모델 아키텍처 생성 및 state_dict 로드
            self.model = BalancedButtonCNN(num_classes=len(self.class_names))
            
            # state_dict 직접 로드 (참고 코드 방식)
            self.model.load_state_dict(torch.load(model_path, map_location=self.torch_device))
            
            self.model.to(self.torch_device)
            self.model.eval()
            
            # 전처리 파이프라인 (설정 파일 기반)
            self.transform = transforms.Compose([
                transforms.Resize((32, 32)),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=config['preprocessing']['normalize_mean'],
                    std=config['preprocessing']['normalize_std']
                )
            ])
            
            self.logger.info(f"✅ CNN 버튼 분류 모델 로드 완료: {len(self.class_names)}개 클래스")
            self.logger.info(f"📋 지원 버튼: {self.class_names}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ CNN 모델 로드 실패: {e}")
            return False
    
    def _create_button_mapping(self):
        """클래스명을 버튼 ID로 매핑"""
        self.button_id_mapping = {
            'btn_1': 1, 'btn_2': 2, 'btn_3': 3, 'btn_4': 4,
            'btn_5': 5, 'btn_6': 6, 'btn_7': 7, 'btn_8': 8,
            'btn_9': 9, 'btn_10': 10, 'btn_11': 11, 'btn_12': 12,
            'btn_b1': 13, 'btn_b2': 14,
            'btn_open': 102, 'btn_close': 103,
            'btn_upward': 101, 'btn_downward': 100
        }
    
    def classify_button(self, color_image: np.ndarray, button_bbox: tuple) -> dict:
        """개별 버튼 이미지를 CNN으로 분류"""
        if self.model is None or not TORCH_AVAILABLE:
            return None
            
        try:
            # 버튼 영역 크롭
            x1, y1, x2, y2 = button_bbox
            button_crop = color_image[y1:y2, x1:x2]
            
            if button_crop.size == 0:
                return None
            
            # OpenCV → PIL 변환
            button_crop_rgb = cv2.cvtColor(button_crop, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(button_crop_rgb)
            
            # 전처리
            input_tensor = self.transform(pil_image).unsqueeze(0).to(self.torch_device)
            
            # 추론
            with torch.no_grad():
                outputs = self.model(input_tensor)
                probabilities = torch.softmax(outputs, dim=1)
                predicted_class = torch.argmax(probabilities, dim=1).item()
                confidence = probabilities[0][predicted_class].item()
            
            # 결과 매핑
            class_name = self.class_names[predicted_class]
            button_id = self.button_id_mapping.get(class_name, 'unknown')
            
            # 버튼 타입 분류
            if button_id in [102, 103]:  # 열기/닫기
                floor_type = 'control'
            elif button_id in [13, 14]:  # B1/B2
                floor_type = 'basement'
            elif button_id in [100, 101]:  # 상행/하행
                floor_type = 'direction'
            else:  # 층수 버튼
                floor_type = 'floor'
            
            return {
                'button_id': button_id,
                'confidence': confidence,
                'class_name': class_name,
                'floor_type': floor_type,
                'recognition_method': 'cnn_classification'
            }
            
        except Exception as e:
            self.logger.error(f"❌ CNN 버튼 분류 실패: {e}")
            return None


class MultiModelDetector:
    """다중 YOLO 모델을 지원하는 탐지 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.models = {}
        self.current_model_name = None
        self.current_model = None
        self.button_pressed_cnn = None  # 나중에 설정됨
        
        # 📦 박스 안정화를 위한 변수들
        self.previous_objects = []  # 이전 프레임 객체들
        self.object_tracking_threshold = 0.5  # IoU 임계값 (겹침 판정)
        self.stability_frames = 3  # 안정화를 위한 최소 프레임 수
        self.object_history = {}  # 객체별 히스토리 {id: [frame_data, ...]}
        
    def set_button_pressed_cnn(self, button_pressed_cnn):
        """버튼 눌림 감지 CNN 설정"""
        self.button_pressed_cnn = button_pressed_cnn
        
        # 모델별 클래스 정의 (실제 모델 순서에 맞게 수정)
        self.model_classes = {
            'normal': ['chair', 'door', 'person'],  # 실제 순서: 0=chair, 1=door, 2=person
            'elevator': ['button', 'direction_light', 'display', 'door']  # 엘리베이터용: 버튼, 방향등, 디스플레이, 문
        }
        
        # 모델별 ID 매핑 (실제 순서에 맞게 수정)
        self.model_id_maps = {
            'normal': {
                'chair': 'CHAIR',  # 0: chair
                'door': 'DOOR',    # 1: door
                'person': 'PERSON' # 2: person
            },
            'elevator': {
                'button': 'BUTTON',
                'direction_light': 'DIRECTION_LIGHT',
                'display': 'DISPLAY',
                'door': 'DOOR'
            }
        }
        

        
        # 모델 초기화
        self._initialize_models()
        
    def _initialize_models(self):
        """모든 YOLO 모델 초기화"""
        self.logger.info("다중 YOLO 모델 초기화 시작...")
        
        try:
            from ultralytics import YOLO
            
            # 1. 일반 주행용 모델 (training/normal/best.pt)
            normal_model_path = self._find_model_in_subdir('normal', 'best.pt')
            if normal_model_path:
                try:
                    self.models['normal'] = YOLO(normal_model_path)
                    # 🚀 GPU 설정 추가
                    self.models['normal'].to('cuda')
                    # 실제 모델 클래스 확인
                    if hasattr(self.models['normal'], 'names'):
                        actual_classes = list(self.models['normal'].names.values())
                        self.logger.info(f"✅ 일반 주행 모델 로딩 성공 (GPU): {normal_model_path}")
                        self.logger.info(f"📋 실제 클래스: {actual_classes}")
                    else:
                        self.logger.info(f"✅ 일반 주행 모델 로딩 성공 (GPU): {normal_model_path}")
                except Exception as e:
                    self.logger.warning(f"⚠️ 일반 주행 모델 로딩 실패: {e}")
            else:
                # 일반 주행용 모델이 없으면 COCO 사전훈련 모델 사용
                try:
                    self.models['normal'] = YOLO('yolov8n.pt')
                    # 🚀 GPU 설정 추가
                    self.models['normal'].to('cuda')
                    # COCO 모델 클래스 확인
                    if hasattr(self.models['normal'], 'names'):
                        actual_classes = list(self.models['normal'].names.values())
                        self.logger.info("✅ 일반 주행용으로 COCO 사전훈련 모델(yolov8n.pt) 사용 (GPU)")
                        self.logger.info(f"📋 COCO 클래스 (전체 {len(actual_classes)}개): person, chair 등만 필터링 사용")
                    else:
                        self.logger.info("✅ 일반 주행용으로 COCO 사전훈련 모델(yolov8n.pt) 사용 (GPU)")
                except Exception as e:
                    self.logger.warning(f"⚠️ COCO 모델도 로딩 실패: {e}")
            
            # 2. 엘리베이터용 모델 (best_v2.pt 우선, best_v1.pt, best.pt 순서)
            elevator_model_path = self._find_elevator_model()
            if elevator_model_path:
                try:
                    self.models['elevator'] = YOLO(elevator_model_path)
                    # 🚀 GPU 설정 추가
                    self.models['elevator'].to('cuda')
                    # 실제 모델 클래스 확인
                    if hasattr(self.models['elevator'], 'names'):
                        actual_classes = list(self.models['elevator'].names.values())
                        self.logger.info(f"✅ 엘리베이터 모델 로딩 성공 (GPU): {elevator_model_path}")
                        self.logger.info(f"📋 실제 클래스: {actual_classes}")
                    else:
                        self.logger.info(f"✅ 엘리베이터 모델 로딩 성공 (GPU): {elevator_model_path}")
                except Exception as e:
                    self.logger.warning(f"⚠️ 엘리베이터 모델 로딩 실패: {e}")
            else:
                self.logger.warning("⚠️ 엘리베이터용 모델을 찾을 수 없습니다")
            
            # 초기화 결과
            loaded_models = list(self.models.keys())
            self.logger.info(f"모델 초기화 완료: {loaded_models} ({len(loaded_models)}/2개)")
            
            # 기본 모델 설정
            if 'elevator' in self.models:
                self.current_model_name = 'elevator'
                self.current_model = self.models['elevator']
            elif 'normal' in self.models:
                self.current_model_name = 'normal'
                self.current_model = self.models['normal']
            
            return len(self.models) > 0
                
        except ImportError:
            self.logger.error("ultralytics 패키지가 필요합니다: pip install ultralytics")
            raise ImportError("ultralytics 패키지를 설치하세요")
        except Exception as e:
            self.logger.error(f"다중 모델 초기화 실패: {e}")
            return False
    
    def _find_model(self, model_filename):
        """모델 파일 찾기"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        possible_dirs = [
            os.path.join(script_dir, "..", "training"),
            os.path.join(script_dir, "..", "models"),
            os.path.join(os.path.expanduser("~"), "project_ws", "Roomie", "ros2_ws", "src", "roomie_vs", "training"),
            os.path.join(os.path.expanduser("~"), "project_ws", "Roomie", "ros2_ws", "src", "roomie_vs", "models"),
            os.path.join(os.getcwd(), "ros2_ws", "src", "roomie_vs", "training"),
            os.path.join(os.getcwd(), "ros2_ws", "src", "roomie_vs", "models"),
            "ros2_ws/src/roomie_vs/training",
            "ros2_ws/src/roomie_vs/models"
        ]
        
        for search_dir in possible_dirs:
            if os.path.exists(search_dir):
                model_path = os.path.join(search_dir, model_filename)
                if os.path.exists(model_path):
                    self.logger.debug(f"모델 발견: {model_path}")
                    return model_path
        
        self.logger.debug(f"모델을 찾을 수 없음: {model_filename}")
        return None
    
    def _find_model_in_subdir(self, subdir, model_filename):
        """training 폴더의 서브디렉토리에서 모델 파일 찾기"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        possible_dirs = [
            os.path.join(script_dir, "..", "training"),
            os.path.join(os.path.expanduser("~"), "project_ws", "Roomie", "ros2_ws", "src", "roomie_vs", "training"),
            os.path.join(os.getcwd(), "ros2_ws", "src", "roomie_vs", "training"),
            "ros2_ws/src/roomie_vs/training"
        ]
        
        for search_dir in possible_dirs:
            if os.path.exists(search_dir):
                model_path = os.path.join(search_dir, subdir, model_filename)
                if os.path.exists(model_path):
                    self.logger.debug(f"모델 발견: {model_path}")
                    return model_path
        
        self.logger.debug(f"모델을 찾을 수 없음: {subdir}/{model_filename}")
        return None
    
    def _find_elevator_model(self):
        """엘리베이터 모델 찾기 (best_v2.pt 우선)"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        possible_dirs = [
            os.path.join(script_dir, "..", "training"),
            os.path.join(os.path.expanduser("~"), "project_ws", "Roomie", "ros2_ws", "src", "roomie_vs", "training"),
            os.path.join(os.getcwd(), "ros2_ws", "src", "roomie_vs", "training"),
            "ros2_ws/src/roomie_vs/training"
        ]
        
        for search_dir in possible_dirs:
            if os.path.exists(search_dir):
                elevator_dir = os.path.join(search_dir, "elevator")
                if os.path.exists(elevator_dir):
                    # 1순위: best_v2.pt (최신 버전)
                    best_v2_path = os.path.join(elevator_dir, "best_v2.pt")
                    if os.path.exists(best_v2_path):
                        self.logger.info(f"✅ 엘리베이터 모델 발견 (v2): {best_v2_path}")
                        return best_v2_path
                    
                    # 2순위: best_v1.pt (이전 버전)
                    best_v1_path = os.path.join(elevator_dir, "best_v1.pt")
                    if os.path.exists(best_v1_path):
                        self.logger.info(f"✅ 엘리베이터 모델 발견 (v1): {best_v1_path}")
                        return best_v1_path
                    
                    # 3순위: best.pt (기본)
                    best_path = os.path.join(elevator_dir, "best.pt")
                    if os.path.exists(best_path):
                        self.logger.info(f"✅ 엘리베이터 모델 발견 (기본): {best_path}")
                        return best_path
        
        self.logger.warning("⚠️ 엘리베이터용 모델을 찾을 수 없습니다 (best_v2.pt, best_v1.pt, best.pt)")
        return None
    
    def set_model_for_mode(self, mode_id):
        """모드에 따른 모델 선택"""
        try:
            if mode_id == 5:  # 일반 주행 모드
                if 'normal' in self.models:
                    old_model = self.current_model_name
                    self.current_model_name = 'normal'
                    self.current_model = self.models['normal']
                    if old_model != 'normal':
                        self.logger.info(f"🤖 모델 변경: {old_model} → normal (일반 주행용)")
                    return True
                else:
                    self.logger.warning("일반 주행용 모델이 없습니다")
                    return False
                    
            elif mode_id in [3, 4]:  # 엘리베이터 모드
                if 'elevator' in self.models:
                    old_model = self.current_model_name
                    self.current_model_name = 'elevator'
                    self.current_model = self.models['elevator']
                    if old_model != 'elevator':
                        self.logger.info(f"🤖 모델 변경: {old_model} → elevator (엘리베이터용)")
                    return True
                else:
                    self.logger.warning("엘리베이터용 모델이 없습니다")
                    return False
            else:
                # 다른 모드는 모델 사용 안함
                if self.current_model_name:
                    self.logger.info(f"🤖 모델 비활성화 (모드 {mode_id})")
                    self.current_model_name = None
                    self.current_model = None
                return True
                
        except Exception as e:
            self.logger.error(f"모델 선택 중 에러: {e}")
            return False
    
    def detect_objects(self, color_image: np.ndarray, depth_image: np.ndarray, conf_threshold: float = 0.7, mode_id: int = 0) -> List[dict]:
        """현재 선택된 모델로 객체 탐지 (모드별 버튼 인식 포함)"""
        if color_image is None or self.current_model is None:
            return []
            
        try:
            objects = self._detect_with_current_model(color_image, depth_image, conf_threshold)
            return objects
        except Exception as e:
            self.logger.error(f"객체 탐지 중 에러: {e}")
            return []
    
    def _detect_with_current_model(self, color_image: np.ndarray, depth_image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """현재 모델을 사용한 객체 탐지"""
        try:
            results = self.current_model.predict(
                color_image, 
                conf=conf_threshold,
                device='cuda',  # 🚀 GPU 사용
                verbose=False
            )
            
            objects = []
            if results and len(results) > 0:
                result = results[0]
                
                if result.boxes is not None and len(result.boxes) > 0:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    confs = result.boxes.conf.cpu().numpy()
                    classes = result.boxes.cls.cpu().numpy()
                    
                    # 실제 모델의 클래스 이름 사용 (모델에서 직접 가져오기)
                    if hasattr(self.current_model, 'names'):
                        # YOLO 모델이 가진 실제 클래스 이름들
                        actual_class_names = list(self.current_model.names.values())
                        self.logger.debug(f"실제 모델 클래스: {actual_class_names}")
                        current_class_names = actual_class_names
                    else:
                        # 백업: 수동 정의된 클래스 이름
                        current_class_names = self.model_classes.get(self.current_model_name, [])
                        self.logger.warning(f"모델에서 클래스 이름을 가져올 수 없어 수동 정의 사용: {current_class_names}")
                    
                    # ID 매핑도 실제 클래스 이름에 맞게 동적 생성
                    if hasattr(self.current_model, 'names'):
                        current_id_map = {name: name.upper() for name in current_class_names}
                    else:
                        current_id_map = self.model_id_maps.get(self.current_model_name, {})
                    
                    for box, conf, cls in zip(boxes, confs, classes):
                        x1, y1, x2, y2 = box.astype(int)
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        width = x2 - x1
                        height = y2 - y1
                        radius = int(max(width, height) / 2)
                        
                        # 클래스 정보
                        class_id = int(cls)
                        
                        # COCO 모델의 경우 클래스 매핑
                        if self.current_model_name == 'normal' and 'normal' not in self.models:
                            # COCO 클래스 이름들
                            coco_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 
                                         'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 
                                         'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 
                                         'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 
                                         'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 
                                         'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 
                                         'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 
                                         'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 
                                         'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 
                                         'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 
                                         'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 
                                         'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 
                                         'scissors', 'teddy bear', 'hair drier', 'toothbrush']
                            
                            if class_id < len(coco_names):
                                class_name = coco_names[class_id]
                                # 관심 있는 객체만 필터링
                                if class_name not in ['person', 'chair']:
                                    continue  # 사람과 의자만 탐지
                            else:
                                class_name = f"unknown_{class_id}"
                        else:
                            # 커스텀 모델의 경우
                            if class_id < len(current_class_names):
                                class_name = current_class_names[class_id]
                            else:
                                class_name = f"unknown_{class_id}"
                        
                        # Depth 정보
                        depth_value = depth_image[center_y, center_x] if depth_image is not None else 1000
                        
                        # 객체별 특별 처리
                        is_pressed = False
                        pressed_confidence = 0.0
                        pressed_method = 'none'
                        object_id = current_id_map.get(class_name, class_name.upper())
                        
                        if class_name == 'button':
                            # CNN 기반 버튼 눌림 감지
                            pressed_result = self._check_button_pressed_cnn(color_image, (x1, y1, x2, y2))
                            is_pressed = pressed_result['is_pressed']
                            pressed_confidence = pressed_result['confidence']
                            pressed_method = pressed_result['method']
                            # pressed_prob과 unpressed_prob 값도 저장
                            pressed_prob = pressed_result.get('pressed_prob', 0.0)
                            unpressed_prob = pressed_result.get('unpressed_prob', 0.0)
                        
                        objects.append({
                            'center': (center_x, center_y),
                            'radius': radius,
                            'depth_mm': int(depth_value),
                            'is_pressed': is_pressed,
                            'pressed_confidence': pressed_confidence,
                            'pressed_method': pressed_method,
                            'pressed_prob': pressed_prob if class_name == 'button' else 0.0,
                            'unpressed_prob': unpressed_prob if class_name == 'button' else 0.0,
                            'class_name': class_name,
                            'class_id': class_id,
                            'object_id': object_id,
                            'confidence': float(conf),
                            'bbox': (x1, y1, x2, y2),
                            'is_button': class_name == 'button',
                            'model_name': self.current_model_name
                        })
            
            # 📦 박스 안정화 적용
            stabilized_objects = self._apply_box_stabilization(objects)
            
            self.logger.debug(f"{self.current_model_name} 모델로 {len(objects)}개 객체 탐지 → {len(stabilized_objects)}개 안정화")
            return stabilized_objects
            
        except Exception as e:
            self.logger.error(f"{self.current_model_name} 모델 탐지 에러: {e}")
            return []
    
    def _check_button_pressed_cnn(self, color_image: np.ndarray, bbox: tuple) -> dict:
        """CNN 기반 버튼 눌림 상태 확인"""
        if self.button_pressed_cnn is None:
            return {'is_pressed': False, 'confidence': 0.0, 'method': 'no_cnn'}
        
        return self.button_pressed_cnn.classify_pressed(color_image, bbox)
    
    def _apply_box_stabilization(self, objects: List[dict]) -> List[dict]:
        """박스 겹침 안정화 적용"""
        if not objects:
            return objects
        
        # 1. NMS (Non-Maximum Suppression) 적용
        nms_objects = self._apply_nms(objects)
        
        # 2. 객체 추적 및 안정화
        tracked_objects = self._apply_object_tracking(nms_objects)
        
        # 3. 신뢰도 기반 필터링
        filtered_objects = self._apply_confidence_filtering(tracked_objects)
        
        return filtered_objects
    
    def _apply_nms(self, objects: List[dict], iou_threshold: float = 0.5) -> List[dict]:
        """Non-Maximum Suppression을 적용하여 겹치는 박스 제거"""
        if len(objects) <= 1:
            return objects
        
        # 클래스별로 NMS 적용
        class_groups = {}
        for obj in objects:
            class_name = obj['class_name']
            if class_name not in class_groups:
                class_groups[class_name] = []
            class_groups[class_name].append(obj)
        
        nms_objects = []
        for class_name, class_objects in class_groups.items():
            if len(class_objects) <= 1:
                nms_objects.extend(class_objects)
                continue
            
            # 신뢰도 기준으로 정렬
            class_objects.sort(key=lambda x: x['confidence'], reverse=True)
            
            keep_objects = []
            while class_objects:
                # 가장 높은 신뢰도 객체 선택
                current = class_objects.pop(0)
                keep_objects.append(current)
                
                # 나머지 객체들과 IoU 계산하여 겹치는 것들 제거
                remaining = []
                for obj in class_objects:
                    iou = self._calculate_iou(current['bbox'], obj['bbox'])
                    if iou < iou_threshold:
                        remaining.append(obj)
                class_objects = remaining
            
            nms_objects.extend(keep_objects)
        
        return nms_objects
    
    def _calculate_iou(self, box1: tuple, box2: tuple) -> float:
        """두 박스 간의 IoU (Intersection over Union) 계산"""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        
        # 교집합 영역 계산
        x1_inter = max(x1_1, x1_2)
        y1_inter = max(y1_1, y1_2)
        x2_inter = min(x2_1, x2_2)
        y2_inter = min(y2_1, y2_2)
        
        if x2_inter <= x1_inter or y2_inter <= y1_inter:
            return 0.0
        
        inter_area = (x2_inter - x1_inter) * (y2_inter - y1_inter)
        
        # 합집합 영역 계산
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = area1 + area2 - inter_area
        
        return inter_area / union_area if union_area > 0 else 0.0
    
    def _apply_object_tracking(self, objects: List[dict]) -> List[dict]:
        """객체 추적을 통한 안정화"""
        if not hasattr(self, 'previous_objects'):
            self.previous_objects = objects
            return objects
        
        tracked_objects = []
        import time
        current_time = time.time()
        
        for obj in objects:
            # 이전 프레임 객체들과 매칭
            best_match = None
            best_iou = 0.0
            
            for prev_obj in self.previous_objects:
                if prev_obj['class_name'] == obj['class_name']:
                    iou = self._calculate_iou(obj['bbox'], prev_obj['bbox'])
                    if iou > best_iou and iou > self.object_tracking_threshold:
                        best_iou = iou
                        best_match = prev_obj
            
            if best_match:
                # 기존 객체와 매칭됨 - 위치 스무딩 적용
                obj['center'] = self._smooth_position(obj['center'], best_match['center'], 0.7)
                obj['bbox'] = self._smooth_bbox(obj['bbox'], best_match['bbox'], 0.7)
                obj['tracking_id'] = best_match.get('tracking_id', f"obj_{len(tracked_objects)}")
                obj['stable_frames'] = best_match.get('stable_frames', 0) + 1
            else:
                # 새로운 객체
                obj['tracking_id'] = f"obj_{current_time}_{len(tracked_objects)}"
                obj['stable_frames'] = 1
            
            tracked_objects.append(obj)
        
        self.previous_objects = tracked_objects.copy()
        return tracked_objects
    
    def _smooth_position(self, current_pos: tuple, prev_pos: tuple, alpha: float = 0.7) -> tuple:
        """위치 스무딩 (지수 이동 평균)"""
        curr_x, curr_y = current_pos
        prev_x, prev_y = prev_pos
        
        smooth_x = int(alpha * curr_x + (1 - alpha) * prev_x)
        smooth_y = int(alpha * curr_y + (1 - alpha) * prev_y)
        
        return (smooth_x, smooth_y)
    
    def _smooth_bbox(self, current_bbox: tuple, prev_bbox: tuple, alpha: float = 0.7) -> tuple:
        """바운딩박스 스무딩"""
        curr_x1, curr_y1, curr_x2, curr_y2 = current_bbox
        prev_x1, prev_y1, prev_x2, prev_y2 = prev_bbox
        
        smooth_x1 = int(alpha * curr_x1 + (1 - alpha) * prev_x1)
        smooth_y1 = int(alpha * curr_y1 + (1 - alpha) * prev_y1)
        smooth_x2 = int(alpha * curr_x2 + (1 - alpha) * prev_x2)
        smooth_y2 = int(alpha * curr_y2 + (1 - alpha) * prev_y2)
        
        return (smooth_x1, smooth_y1, smooth_x2, smooth_y2)
    
    def _apply_confidence_filtering(self, objects: List[dict]) -> List[dict]:
        """신뢰도 기반 필터링 및 안정성 체크"""
        filtered_objects = []
        
        for obj in objects:
            # 기본 신뢰도 필터링
            if obj['confidence'] < 0.3:  # 매우 낮은 신뢰도 제거
                continue
            
            # 안정성 체크 (새로운 객체는 높은 신뢰도 요구)
            stable_frames = obj.get('stable_frames', 1)
            min_confidence = 0.7 if stable_frames < self.stability_frames else 0.5
            
            if obj['confidence'] >= min_confidence:
                filtered_objects.append(obj)
        
        return filtered_objects

    
    def get_current_model_info(self):
        """현재 모델 정보 반환"""
        return {
            'model_name': self.current_model_name,
            'available_models': list(self.models.keys()),
            'class_names': self.model_classes.get(self.current_model_name, []),
            'is_active': self.current_model is not None
        }

# YOLOButtonDetector 클래스 제거됨 - MultiModelDetector가 실제로 사용됨

class VSNode(Node):
    """OpenNI2 기반 Vision Service ROS2 노드"""
    
    def __init__(self):
        super().__init__('vs_node')
        
        # 멀티 카메라 매니저와 다중 모델 탐지기 초기화
        self.camera_manager = MultiCameraManager(self.get_logger())
        self.model_detector = MultiModelDetector(self.get_logger())
        
        # 버튼 눌림 감지 CNN 초기화
        self.button_pressed_cnn = ButtonPressedCNN(self.get_logger())
        
        # CNN을 MultiModelDetector에 연결
        self.model_detector.set_button_pressed_cnn(self.button_pressed_cnn)
        
        # CNN 버튼 분류기 초기화
        self.cnn_classifier = CNNButtonClassifier(self.get_logger())
        
        # 🚧 장애물 감지기 초기화
        self.obstacle_detector = ObstacleDetector(self.get_logger())
        
        # 📹 UDP 비디오 스트리머 초기화 (후방 카메라 → RGUI)
        self.udp_streamer = UDPVideoStreamer(
            target_ip=os.environ.get('VS_UDP_TARGET_IP', '127.0.0.1'),
            target_port=int(os.environ.get('VS_UDP_TARGET_PORT', os.environ.get('RGUI_UDP_PORT', 5005))),
            max_fps=int(os.environ.get('VS_UDP_MAX_FPS', 15)),
            quality=int(os.environ.get('VS_UDP_QUALITY', 70)),
            logger=self.get_logger()
        )
        self.get_logger().info(f"📹 UDP 비디오 스트리머 초기화 완료 ({self.udp_streamer.addr[0]}:{self.udp_streamer.addr[1]})")
        
        # 🔥 최적화된 DisplayOCR 초기화 (EasyOCR만 사용, GPU 리소스 절약)
        self.display_ocr = DisplayOCR(self.get_logger())
        # EasyOCR test_all_models_on_roi와 동일한 단순 크롭 방식 사용
        self.display_ocr.update_config(debug_mode=True, use_simple_crop=True)
        
        # OCR 주기 조절 설정 (리소스 절약)
        self.ocr_counter = 0
        self.ocr_skip_frames = 5  # 5프레임마다 한 번씩 OCR 수행 (기존보다 느리게)
        self.last_ocr_objects = []  # 마지막 OCR 결과 캐싱
        
        # 🚪 유리 문 상태 이벤트 기반 발행을 위한 변수
        self.last_glass_door_opened = None  # 이전 유리 문 상태 (None: 초기화되지 않음)
        
        # 🚧 장애물 감지 1초 종합 평가를 위한 변수들
        self.obstacle_detection_history = {}  # {class_name: [detection_count, total_frames]}
        self.last_obstacle_publish_time = None  # 마지막 장애물 발행 시간
        self.obstacle_publish_interval = 1.0  # 장애물 발행 간격 (1초)
        self.obstacle_detection_threshold = 0.6  # 60% 이상 감지되어야 장애물로 인정
        
        # 🎮 GPU 리소스 모니터링 초기화
        try:
            from .gpu_monitor import GPUResourceMonitor
            # GPU 메모리 제한: RTX 2060 6GB 중 4GB만 사용 허용
            self.gpu_monitor = GPUResourceMonitor(
                self.get_logger(), 
                max_memory_mb=4096,  # 4GB 제한
                check_interval=3.0   # 3초마다 체크
            )
            
            # GPU 메모리 초과 시 대응 방법 설정
            self.gpu_monitor.set_memory_exceeded_callback(self._on_gpu_memory_exceeded)
            self.gpu_monitor.set_gpu_error_callback(self._on_gpu_error)
            
            # GPU 모니터링 시작
            if self.gpu_monitor.start_monitoring():
                self.get_logger().info("🎮 GPU 리소스 모니터링 활성화됨")
            else:
                self.get_logger().warning("⚠️ GPU 모니터링 시작 실패")
                
        except Exception as e:
            self.get_logger().warning(f"⚠️ GPU 모니터링 초기화 실패: {e}")
            self.gpu_monitor = None
        
        # 현재 선택된 카메라들 (모드별로 변경됨)
        self.current_camera = None
        self.current_depth_camera = None
        self.current_camera_name = "None"
        
        # 이미지 처리 옵션
        self.flip_horizontal = False  # 기본 좌우반전 끄기
        self.confidence_threshold = 0.7
        
        # 헤드리스 모드 설정 (GUI 없이 동작)
        self.headless_mode = os.environ.get('ROOMIE_HEADLESS', 'false').lower() in ['true', '1', 'yes']
        if self.headless_mode:
            self.get_logger().info("🖥️ 헤드리스 모드 활성화: GUI 없이 동작합니다")
        
        # ArUco 마커 감지 설정
        try:
            # OpenCV 버전 확인
            opencv_version = cv2.__version__
            self.get_logger().info(f"OpenCV 버전: {opencv_version}")
            self.get_logger().info(f"OpenCV 파일 위치: {cv2.__file__}")
            self.get_logger().info("🔍 ArUco 초기화 시작...")
            
            # ArUco 기본 설정 (단계별 테스트)
            self.get_logger().info("🔍 ArUco 기본 사전 로딩 시도...")
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            self.aruco_dict_name = "DICT_ARUCO_ORIGINAL"
            self.get_logger().info("✅ ArUco 사전 로딩 성공")
            
            self.get_logger().info("🔍 ArUco 파라미터 설정...")
            # OpenCV 4.7+ 신 API 우선 사용, 가능 시 DetectorParameters 및 ArucoDetector 생성
            try:
                self.aruco_params = cv2.aruco.DetectorParameters()
                if hasattr(cv2.aruco, 'ArucoDetector'):
                    self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                    self.aruco_api_version = "new"
                    self.get_logger().info("✅ ArUcoDetector 초기화 완료 (신 API)")
                else:
                    # 폴백: 레거시 API만 있는 경우 플래그로 활성화
                    self.aruco_detector = True
                    self.aruco_api_version = "legacy"
                    self.get_logger().info("✅ ArUco 감지 시스템 활성화 (레거시 API)")
            except Exception:
                # 최종 폴백: 파라미터를 None으로 두고 레거시로만 시도
                self.aruco_params = None
                self.aruco_detector = True if self.aruco_dict is not None else False
                self.aruco_api_version = "legacy" if self.aruco_detector else "error"
                self.get_logger().info("✅ ArUco 파라미터 설정 완료 (기본값/레거시)")
            
        except Exception as e:
            self.get_logger().warning(f"초기화 중 오류: {e}")
            self.aruco_dict = None
            self.aruco_params = None
            self.aruco_detector = None
            self.aruco_api_version = "error"
        
        # 마지막으로 감지된 위치 저장
        self.last_detected_location_id = 0  # 기본값: LOB_WAITING
        self.last_detection_time = None
        self.unknown_aruco_id = None  # 알 수 없는 ArUco 마커 ID 저장
        
        # 🚦 엘리베이터 방향 캐시
        self.last_elevator_direction = 0  # 0: 상행, 1: 하행
        self.last_direction_detection_time = None
        
        # 🔥 개선된 Direction Light 추적 (개별 위치 기반)
        self.previous_direction_lights = []  # 이전 프레임의 direction light 객체들 (위치+밝기)
        self.direction_light_history = []    # 최근 5프레임의 개수 히스토리
        self.brightness_threshold = 180      # 밝기 임계값 (0-255, 조금 낮춤)
        
        # 🎯 방향등 위치 기억 시스템 (간헐적 감지 보강)
        self.remembered_direction_positions = {
            'upper': None,  # 기억된 위쪽 방향등 위치 
            'lower': None   # 기억된 아래쪽 방향등 위치
        }
        self.last_position_update = None  # 마지막 위치 업데이트 시간
        
        # 🎯 밝기 변화 기반 깜빡임 감지 시스템
        self.brightness_history = {
            'upper': [],  # 위쪽 방향등 밝기 히스토리 (최근 10프레임)
            'lower': []   # 아래쪽 방향등 밝기 히스토리
        }
        self.blink_detection_enabled = True
        self.brightness_change_threshold_for_blink = 20  # 깜빡임 감지용 밝기 변화 임계값 (더 민감하게)
        self.history_size = 10  # 히스토리 저장 프레임 수
        self.last_blink_detected = False  # 마지막 방향 감지에서 깜빡임이 감지되었는지
        self.brightness_change_threshold = 50  # 밝기 변화 감지 임계값 (안정성 우선)
        self.position_tolerance = 50         # 같은 방향등으로 인식할 위치 허용 오차 (픽셀)
        
        # 🎯 마지막 감지된 객체들 저장 (L키 강제 학습용)
        self.last_detected_objects = []
        
        # ArUco 마커 ID와 location_id 직접 매핑 (interface 문서 기준)
        self.aruco_to_location = {
            0: 0,     # LOB_WAITING
            1: 1,     # LOB_CALL  
            2: 2,     # RES_PICKUP
            3: 3,     # RES_CALL
            4: 4,     # SUP_PICKUP
            5: 5,     # ELE_1
            6: 6,     # ELE_2
            101: 101, # ROOM_101
            102: 102, # ROOM_102
            201: 201, # ROOM_201
            202: 202, # ROOM_202
        }
        
        # VS 모드 상태 관리 - 전방/후방 독립적으로 관리
        self.current_front_mode_id = 6  # 전방 대기모드로 시작
        self.current_rear_mode_id = 0   # 후방 대기모드로 시작
        self.mode_names = {
            0: "Standby Mode (Rear)",
            1: "Registration Mode (Rear)", 
            2: "Tracking Mode (Rear)",
            3: "Elevator External Mode (Front)",
            4: "Elevator Internal Mode (Front)",
            5: "Normal Mode (Front)",
            6: "Standby Mode (Front)",
            100: "Delivery Simulation Mode",
            101: "Call Simulation Mode",
            102: "Guide Simulation Mode",
            103: "Return Simulation Mode",
            104: "Elevator Simulation Mode"
        }
        
        # 시뮬레이션 모드별 시나리오 카운터
        self.simulation_counters = {
            100: 0,  # 배송 시뮬레이션
            101: 0,  # 호출 시뮬레이션
            102: 0,  # 길안내 시뮬레이션
            103: 0,  # 복귀 시뮬레이션
            104: 0   # 엘리베이터 시뮬레이션
        }
        
        # 모든 카메라 활성화 (대기모드에서도 GUI 제공)
        self.camera_initialized = True
        self.get_logger().info("🚀 모든 카메라 활성화 초기화 시작")
        self.get_logger().info("📌 대기모드에서도 카메라와 GUI가 항상 활성화됩니다")
        self.get_logger().info("💡 실시간 영상 확인 가능 - 리소스 소모 증가")
            
        # 전방/후방 카메라 모두 활성화
        self.update_front_camera()  # 전방 카메라 초기화 (대기모드 6번)
        self.update_rear_camera()   # 후방 카메라 초기화 (대기모드 0번)
        
        # ROS2 서비스들 (/vs/command/*)
        self.get_logger().info("VS 서비스 인터페이스 초기화 중...")
        
        self.set_mode_service = self.create_service(
            SetVSMode,
            '/vs/command/set_vs_mode',
            self.set_vs_mode_callback
        )
        

        
        self.button_status_service = self.create_service(
            ButtonStatus, 
            '/vs/command/button_status', 
            self.button_status_callback
        )
        
        self.elevator_status_service = self.create_service(
            ElevatorStatus,
            '/vs/command/elevator_status',
            self.elevator_status_callback
        )
        
        self.door_status_service = self.create_service(
            DoorStatus,
            '/vs/command/door_status',
            self.door_status_callback
        )
        

        
        self.location_service = self.create_service(
            Location,
            '/vs/command/location',
            self.location_callback
        )
        
        # 👤 추적 관련 액션 및 서비스 추가
        self.enroll_action_server = ActionServer(
            self,
            Enroll,
            '/vs/action/enroll',
            self.enroll_action_callback
        )
        
        self.stop_tracking_service = self.create_service(
            Trigger,
            '/vs/command/stop_tracking',
            self.stop_tracking_callback
        )
        
        # ROS2 토픽 퍼블리셔들 (QoS 프로파일 명시적 설정)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        # QoS 프로파일 설정 (히스토리 크기 증가)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,  # 히스토리 크기를 50으로 증가
            reliability=ReliabilityPolicy.RELIABLE
        )
        

        
        # 🚧 장애물 토픽 퍼블리셔 추가
        self.obstacle_pub = self.create_publisher(
            Obstacle,
            '/vs/obstacle',
            qos_profile
        )
        
        # 🚪 유리 문 상태 토픽 퍼블리셔 추가
        self.glass_door_pub = self.create_publisher(
            GlassDoorStatus,
            '/vs/glass_door_status',
            qos_profile
        )
        
        self.get_logger().info("모든 VS 인터페이스 초기화 완료!")
        self.get_logger().info("구현된 서비스 7개: set_vs_mode, button_status, elevator_status, door_status, location, stop_tracking, enroll(action)")
        self.get_logger().info("구현된 토픽 3개: obstacle, glass_door_status, tracking")
        self.get_logger().info("ArUco 마커 기반 위치 감지 시스템 활성화")
        self.get_logger().info("🎯 GPU 리소스 절약형 동적 카메라 VS Node 초기화 완료!")
        self.get_logger().info(f"🚀 시작 모드: 전방 {self.mode_names[self.current_front_mode_id]} (ID: {self.current_front_mode_id}), 후방 {self.mode_names[self.current_rear_mode_id]} (ID: {self.current_rear_mode_id})")
        
        # 모드별 카메라 요구사항 요약 출력
        self.get_logger().info("=" * 60)
        self.get_logger().info("📋 모드별 카메라 사용 계획 (항상 활성화)")
        self.get_logger().info("=" * 60)
        self.get_logger().info("후방 관련: 0(대기) → 후방웹캠, 1(등록) → 후방웹캠, 2(추적) → 후방웹캠")
        self.get_logger().info("전방 관련: 3(엘외부) → 전방웹캠+뎁스, 4(엘내부) → 전방웹캠+뎁스, 5(일반) → 전방웹캠+뎁스, 6(대기) → 전방웹캠+뎁스")
        self.get_logger().info("💡 모든 모드에서 카메라와 GUI가 활성화되어 실시간 영상 확인 가능합니다")
        self.get_logger().info("=" * 60)
        
        # 👤 사람 추적 모듈 초기화 (마지막에 추가)
        try:
            # MultiModelDetector에서 YOLOv8n 모델 참조
            yolo_model = None
            if hasattr(self.model_detector, 'model_normal') and self.model_detector.model_normal:
                yolo_model = self.model_detector.model_normal
            elif hasattr(self.model_detector, 'fallback_model') and self.model_detector.fallback_model:
                yolo_model = self.model_detector.fallback_model
            
            self.person_tracker = PersonTracker(self, yolo_model)
            self.get_logger().info("👤 PersonTracker 초기화 완료")
        except Exception as e:
            self.get_logger().error(f"👤 PersonTracker 초기화 실패: {e}")
            self.person_tracker = None
    
    def update_camera_for_current_mode(self):
        """전방/후방 카메라 독립적 업데이트 (호환성 유지)"""
        try:
            # 전방과 후방 카메라를 각각 초기화 (독립적 관리)
            self.update_front_camera()
            self.update_rear_camera()
                
        except Exception as e:
            self.get_logger().error(f"카메라 업데이트 에러: {e}")
            self.current_camera = None
            self.current_depth_camera = None
            self.current_camera_name = "Error"
    
    def detect_and_update_location(self, input_image: np.ndarray = None) -> int:
        """ArUco 마커를 감지하여 위치 업데이트 및 현재 위치 반환"""
        if not self.aruco_detector or self.aruco_dict is None:
            self.get_logger().debug("ArUco 시스템이 초기화되지 않음")
            return self.last_detected_location_id
        
        try:
            # 입력 이미지가 제공되면 사용, 없으면 현재 카메라 프레임 사용
            if input_image is not None:
                current_color = input_image
            else:
                # 현재 카메라 프레임 획득
                if self.current_camera is None:
                    self.get_logger().debug("현재 카메라가 설정되지 않음")
                    return self.last_detected_location_id
                
                with self.current_camera.frame_lock:
                    current_color = self.current_camera.current_color
            
            if current_color is None:
                self.get_logger().debug("카메라 프레임이 없음")
                return self.last_detected_location_id
            
            # 좌우반전은 이미 적용되었다고 가정 (main에서 처리)
            processed_image = current_color.copy()
            
            # 그레이스케일 변환
            gray = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)
            
            # 레거시/신 API 대응하여 ArUco 마커 감지
            corners = ids = rejected = None
            try:
                if hasattr(cv2.aruco, 'ArucoDetector') and isinstance(self.aruco_detector, cv2.aruco.ArucoDetector):
                    corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
                elif hasattr(cv2.aruco, 'detectMarkers'):
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        gray,
                        self.aruco_dict,
                        parameters=self.aruco_params if self.aruco_params is not None else None
                    )
                else:
                    self.get_logger().error("ArUco 감지 API를 찾을 수 없습니다 (ArucoDetector/legacy detectMarkers 모두 없음)")
                    return self.last_detected_location_id
            except Exception as e:
                self.get_logger().error(f"ArUco 감지 호출 실패: {e}")
                return self.last_detected_location_id
            
            # 조용한 자동 감지 (로그 최소화)
            
            if ids is not None and len(ids) > 0:
                # 첫 번째 감지된 마커 사용
                detected_id = int(ids[0][0])
                
                # 매핑된 location_id 확인
                if detected_id in self.aruco_to_location:
                    new_location_id = self.aruco_to_location[detected_id]
                    
                    # 새로운 위치가 이전과 다르면 업데이트
                    if new_location_id != self.last_detected_location_id:
                        old_location = self.last_detected_location_id
                        self.last_detected_location_id = new_location_id
                        self.last_detection_time = self.get_clock().now()
                        
                        location_names = {
                            0: "LOB_WAITING", 1: "LOB_CALL", 2: "RES_PICKUP", 3: "RES_CALL",
                            4: "SUP_PICKUP", 5: "ELE_1", 6: "ELE_2", 101: "ROOM_101",
                            102: "ROOM_102", 201: "ROOM_201", 202: "ROOM_202"
                        }
                        old_name = location_names.get(old_location, f"UNKNOWN({old_location})")
                        new_name = location_names.get(new_location_id, f"UNKNOWN({new_location_id})")
                        
                        self.get_logger().info(f"🎯 위치 변경: {old_name} → {new_name} (ArUco 마커 {detected_id})")
                    else:
                        # 같은 위치 재확인 (조용하게)
                        self.last_detection_time = self.get_clock().now()
                    
                    return self.last_detected_location_id
                else:
                    # 알 수 없는 마커는 조용히 처리 (로그 최소화)
                    # GUI에 마커 ID를 오버레이로 표시
                    self.unknown_aruco_id = detected_id
                    return self.last_detected_location_id
            else:
                # 마커가 감지되지 않음 - 마지막 위치 유지
                return self.last_detected_location_id
                
        except Exception as e:
            self.get_logger().error(f"❌ ArUco 마커 감지 에러: {e}")
            import traceback
            self.get_logger().error(f"스택 트레이스: {traceback.format_exc()}")
            return self.last_detected_location_id
    
    def test_aruco_detection(self):
        """ArUco 감지 테스트 함수 ('A' 키용) - 모든 ArUco 사전 시도"""
        try:
            # 현재 카메라 프레임 획득
            if self.current_camera is None:
                self.get_logger().warning("⚠️ 현재 카메라가 설정되지 않았습니다")
                return
            
            with self.current_camera.frame_lock:
                current_color = self.current_camera.current_color
            
            if current_color is None:
                self.get_logger().warning("⚠️ 카메라 프레임이 없습니다")
                return
            
            # 좌우반전 적용
            processed_image = current_color.copy()
            if self.flip_horizontal:
                processed_image = cv2.flip(processed_image, 1)
                self.get_logger().info("🔄 좌우반전 적용됨")
            
            # 그레이스케일 변환
            gray = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)
            self.get_logger().info(f"📊 이미지 크기: {gray.shape}, 타입: {gray.dtype}")
            
            # 모든 주요 ArUco 사전 시도
            aruco_dicts_to_test = [
                (cv2.aruco.DICT_4X4_50, "DICT_4X4_50"),
                (cv2.aruco.DICT_4X4_100, "DICT_4X4_100"),
                (cv2.aruco.DICT_4X4_250, "DICT_4X4_250"),
                (cv2.aruco.DICT_4X4_1000, "DICT_4X4_1000"),
                (cv2.aruco.DICT_5X5_50, "DICT_5X5_50"),
                (cv2.aruco.DICT_5X5_100, "DICT_5X5_100"),
                (cv2.aruco.DICT_5X5_250, "DICT_5X5_250"),
                (cv2.aruco.DICT_5X5_1000, "DICT_5X5_1000"),
                (cv2.aruco.DICT_6X6_50, "DICT_6X6_50"),
                (cv2.aruco.DICT_6X6_100, "DICT_6X6_100"),
                (cv2.aruco.DICT_6X6_250, "DICT_6X6_250"),
                (cv2.aruco.DICT_6X6_1000, "DICT_6X6_1000"),
                (cv2.aruco.DICT_7X7_50, "DICT_7X7_50"),
                (cv2.aruco.DICT_7X7_100, "DICT_7X7_100"),
                (cv2.aruco.DICT_7X7_250, "DICT_7X7_250"),
                (cv2.aruco.DICT_7X7_1000, "DICT_7X7_1000"),
                (cv2.aruco.DICT_ARUCO_ORIGINAL, "DICT_ARUCO_ORIGINAL"),
            ]
            
            self.get_logger().info(f"🔍 모든 ArUco 사전 테스트 시작 ({len(aruco_dicts_to_test)}개)")
            
            detected_in_dicts = []
            
            for dict_id, dict_name in aruco_dicts_to_test:
                try:
                    # 테스트용 ArUco 사전 생성 (파라미터는 기본값)
                    test_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                    
                    # ArUco 마커 감지 (신 API 우선, 레거시 폴백)
                    try:
                        if hasattr(cv2.aruco, 'ArucoDetector'):
                            test_detector = cv2.aruco.ArucoDetector(test_dict, cv2.aruco.DetectorParameters())
                            corners, ids, rejected = test_detector.detectMarkers(gray)
                        elif hasattr(cv2.aruco, 'detectMarkers'):
                            corners, ids, rejected = cv2.aruco.detectMarkers(
                                gray,
                                test_dict,
                                parameters=None
                            )
                        else:
                            corners, ids, rejected = None, None, None
                    except Exception as e:
                        self.get_logger().warning(f"{dict_name} 감지 실패: {e}")
                        corners, ids, rejected = None, None, None
                    
                    detected_count = len(ids) if ids is not None else 0
                    rejected_count = len(rejected) if rejected is not None else 0
                    
                    if detected_count > 0:
                        self.get_logger().info(f"🎯 {dict_name}: {detected_count}개 마커 감지!")
                        
                        # 감지된 마커 ID들 출력
                        marker_ids = [int(id[0]) for id in ids]
                        self.get_logger().info(f"   감지된 마커 ID: {marker_ids}")
                        
                        # 1번 마커가 있는지 확인
                        if 1 in marker_ids:
                            self.get_logger().info(f"   ✅ 1번 마커 발견! {dict_name}을 사용하세요!")
                            detected_in_dicts.append((dict_name, marker_ids))
                        else:
                            detected_in_dicts.append((dict_name, marker_ids))
                    else:
                        if rejected_count > 0:
                            self.get_logger().debug(f"   {dict_name}: 0개 감지, {rejected_count}개 거부됨")
                        
                except Exception as e:
                    self.get_logger().debug(f"   {dict_name}: 테스트 실패 - {e}")
            
            # 결과 요약
            self.get_logger().info("📋 테스트 결과 요약:")
            if detected_in_dicts:
                self.get_logger().info(f"✅ 마커가 감지된 사전들 ({len(detected_in_dicts)}개):")
                for dict_name, marker_ids in detected_in_dicts:
                    self.get_logger().info(f"   {dict_name}: 마커 ID {marker_ids}")
                    if 1 in marker_ids:
                        self.get_logger().info(f"   👆 {dict_name}에서 1번 마커 발견! 이 사전을 사용하세요!")
            else:
                self.get_logger().warning("❌ 어떤 ArUco 사전에서도 마커를 감지하지 못했습니다")
                self.get_logger().info("💡 확인사항:")
                self.get_logger().info("   1. 마커가 화면에 선명하게 보이는가?")
                self.get_logger().info("   2. 조명이 충분한가?")
                self.get_logger().info("   3. 마커가 평평하고 왜곡되지 않았는가?")
                self.get_logger().info("   4. 마커 크기가 너무 작거나 크지 않은가?")
                
        except Exception as e:
            self.get_logger().error(f"❌ ArUco 테스트 에러: {e}")
            import traceback
            self.get_logger().error(f"스택 트레이스: {traceback.format_exc()}")
    
    def _add_aruco_visualization(self, image: np.ndarray):
        """ArUco 마커 감지 결과를 이미지에 표시 (이미 감지된 정보 사용)"""
        if not self.aruco_detector or self.aruco_dict is None:
            return
        
        try:
            # 이미 감지된 위치 정보 표시 (별도 감지하지 않음)
            if self.last_detected_location_id is not None:
                location_names = {
                    0: "LOB_WAITING", 1: "LOB_CALL", 2: "RES_PICKUP", 3: "RES_CALL",
                    4: "SUP_PICKUP", 5: "ELE_1", 6: "ELE_2", 101: "ROOM_101",
                    102: "ROOM_102", 201: "ROOM_201", 202: "ROOM_202"
                }
                location_name = location_names.get(self.last_detected_location_id, f"ID_{self.last_detected_location_id}")
                
                # 감지된 위치 정보 표시 (더 아래쪽으로 이동)
                cv2.putText(image, f"ArUco Status: Active", (10, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(image, f"Location: {location_name}", (10, 265), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # 마지막 감지 시간 표시 (선택적)
                if self.last_detection_time is not None:
                    import time
                    current_time = self.get_clock().now()
                    time_diff = (current_time - self.last_detection_time).nanoseconds / 1e9
                    cv2.putText(image, f"Last detected: {time_diff:.1f}s ago", (10, 290), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 255), 2)
            else:
                # 아직 감지된 마커가 없음
                cv2.putText(image, f"ArUco Status: Waiting", (10, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
            
            # 알 수 없는 ArUco 마커 ID 표시 (GUI 오버레이)
            if self.unknown_aruco_id is not None:
                cv2.putText(image, f"Unknown ArUco: {self.unknown_aruco_id}", (10, 315), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(image, f"Supported: {list(self.aruco_to_location.keys())}", (10, 340), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 2)
                
        except Exception as e:
            pass
    
    def button_status_callback(self, request, response):
        """버튼 상태 요청 처리 - 단일 버튼 감지"""
        try:
            self.get_logger().info(f"버튼 상태 요청: robot_id={request.robot_id}, button_id={request.button_id}")
            
            response.robot_id = request.robot_id
            response.button_id = request.button_id
            
            # 현재 프레임 획득
            if self.current_camera is None:
                self.get_logger().warning("현재 카메라가 설정되지 않음")
                response.success = False
                response.x = 0.0
                response.y = 0.0
                response.size = 0.0
                response.is_pressed = False
                response.timestamp = self.get_clock().now().to_msg()
                return response
                
            try:
                with self.current_camera.frame_lock:
                    # WebCamCamera에는 current_depth가 없으므로 안전하게 접근
                    current_depth = getattr(self.current_camera, 'current_depth', None)
                    current_color = self.current_camera.current_color
                
                # 이미지 좌우반전
                if self.flip_horizontal:
                    if current_color is not None:
                        current_color = cv2.flip(current_color, 1)
                    if current_depth is not None:
                        current_depth = cv2.flip(current_depth, 1)
                
                if current_color is None:
                    self.get_logger().warning("카메라 프레임이 없음")
                    response.success = False
                    response.x = 0.0
                    response.y = 0.0
                    response.size = 0.0
                    response.is_pressed = False
                    response.timestamp = self.get_clock().now().to_msg()
                    return response
                
                # 이미지 크기 정보
                img_height, img_width = current_color.shape[:2]
                
                # 설정된 목표 해상도 정보 (정규화용)
                if hasattr(self.current_camera, 'actual_camera_id') and self.current_camera.actual_camera_id is not None:
                    target_width, target_height = self.current_camera._get_optimal_resolution(self.current_camera.actual_camera_id)
                    self.get_logger().info(f"🔍 해상도 정보: 실제={img_width}x{img_height}, 목표={target_width}x{target_height}")
                else:
                    target_width, target_height = img_width, img_height
                    self.get_logger().info(f"🔍 해상도 정보: 실제=목표={img_width}x{img_height}")
                
                # 다중 모델로 객체 탐지 (현재 모드 전달)
                self.get_logger().info(f"🎯 탐지 설정: mode_id={self.current_front_mode_id}, confidence={self.confidence_threshold}")
                detected_objects = self.model_detector.detect_objects(current_color, current_depth, self.confidence_threshold, self.current_front_mode_id)
                
                # 디버깅: 단계별 버튼 탐지 확인
                raw_buttons = [obj for obj in detected_objects if obj.get('class_name') == 'button']
                self.get_logger().info(f"🔍 1단계 YOLO 탐지: 버튼 {len(raw_buttons)}개")
                
                # 객체에 OCR 결과 추가 (display 객체만)
                enhanced_objects = self._enhance_objects_with_ocr(current_color, detected_objects)
                
                enhanced_buttons = [obj for obj in enhanced_objects if obj.get('class_name') == 'button']
                self.get_logger().info(f"🔍 2단계 OCR 처리 후: 버튼 {len(enhanced_buttons)}개")
                
                # 버튼은 CNN으로 직접 처리, display는 OCR 처리된 상태 유지
                processed_objects = self._apply_enhanced_button_recognition(enhanced_objects, current_color, self.current_front_mode_id)
                
                # 디버깅: 처리된 버튼 객체들 확인
                all_button_objects = [obj for obj in processed_objects if obj.get('class_name') == 'button']
                for i, btn_obj in enumerate(all_button_objects):
                    button_id = btn_obj.get('button_id', 'None')
                    method = btn_obj.get('recognition_method', 'None')
                    confidence = btn_obj.get('confidence', 0)
                    self.get_logger().info(f"🔍 처리된 버튼 {i+1}: ID={button_id}, method={method}, conf={confidence:.3f}")
                
                # 버튼 필터링 로직: button_id=0이면 모든 탐지된 버튼, 아니면 ID가 할당된 버튼만
                total_button_objects = [obj for obj in processed_objects if obj.get('class_name') == 'button']
                
                if request.button_id == 0:
                    # button_id=0: 현재 유일하게 감지되는 버튼 (ID 할당 여부 무관)
                    detected_buttons = total_button_objects
                else:
                    # 특정 button_id 요청: button_id가 할당된 버튼들만
                    detected_buttons = [
                        obj for obj in processed_objects 
                        if obj.get('class_name') == 'button' and 
                        obj.get('button_id') not in [None, 'unmapped', -1, 'None']
                    ]
                
                # 버튼 개수에 따른 처리
                if len(detected_buttons) == 0:
                    # 탐지된 버튼이 없음
                    if request.button_id == 0:
                        self.get_logger().info(f"탐지된 버튼 없음: 총 0개")
                    else:
                        recognized_count = len([obj for obj in total_button_objects if obj.get('button_id') not in [None, 'unmapped', -1, 'None']])
                        self.get_logger().info(f"인식된 버튼 없음: 탐지 {len(total_button_objects)}개, 인식 성공 {recognized_count}개")
                    response.success = False
                    response.x = 0.0
                    response.y = 0.0
                    response.size = 0.0
                    response.is_pressed = False
                    response.timestamp = self.get_clock().now().to_msg()
                    
                elif len(detected_buttons) == 1:
                    # 정확히 1개의 버튼이 감지됨
                    btn = detected_buttons[0]
                    center = btn['center']
                    bbox = btn['bbox']
                    
                    # 좌표를 0~1 범위로 정규화 (설정된 목표 해상도 기준)
                    x_norm = float(center[0] / target_width)
                    y_norm = float(center[1] / target_height)
                    
                    # 버튼 크기를 0~1 범위로 정규화 (설정된 목표 해상도 기준)
                    bbox_width = bbox[2] - bbox[0]
                    bbox_height = bbox[3] - bbox[1]
                    bbox_area = bbox_width * bbox_height
                    target_area = target_width * target_height
                    size_norm = float(bbox_area / target_area)
                    
                    # 디버그: 크기 계산 상세 정보
                    img_area = img_width * img_height
                    size_norm_actual = float(bbox_area / img_area)
                    self.get_logger().info(f"🔍 크기 계산: bbox={bbox_width}x{bbox_height}(면적:{bbox_area})")
                    self.get_logger().info(f"🔍 면적 비교: 실제이미지={img_area}, target={target_area}")
                    self.get_logger().info(f"🔍 정규화: target기준={size_norm:.4f}, 실제기준={size_norm_actual:.4f}")
                    
                    response.success = True
                    response.x = x_norm
                    response.y = y_norm
                    response.size = size_norm
                    response.is_pressed = bool(btn.get('is_pressed', False))
                    response.timestamp = self.get_clock().now().to_msg()
                    
                    confidence = btn.get('confidence', 1.0)
                    button_id = btn.get('button_id', 'unknown')
                    recognition_method = btn.get('recognition_method', 'unknown')
                    pressed_confidence = btn.get('pressed_confidence', 0.0)
                    pressed_method = btn.get('pressed_method', 'none')
                    self.get_logger().info(f"버튼 인식 성공: ID={button_id} ({recognition_method}), "
                                         f"x={x_norm:.3f}, y={y_norm:.3f}, size={size_norm:.3f}, "
                                         f"pressed={btn.get('is_pressed', False)} ({pressed_method}:{pressed_confidence:.3f}), conf={confidence:.2f}")
                    
                    # button_id 매칭 검증
                    if request.button_id != 0:
                        # 0이 아닌 경우: 특정 버튼 요청 → ID 매칭 필수
                        detected_button_id = btn.get('button_id', -1)
                        
                        if detected_button_id == -1:
                            # OCR로 button_id를 감지하지 못함
                            self.get_logger().warning(f"요청된 button_id({request.button_id})에 대한 OCR 매칭 실패")
                            response.success = False
                            response.x = 0.0
                            response.y = 0.0
                            response.size = 0.0
                            response.is_pressed = False
                            response.timestamp = self.get_clock().now().to_msg()
                            return response
                            
                        elif detected_button_id != request.button_id:
                            # 요청된 ID와 다른 버튼이 감지됨
                            self.get_logger().warning(f"❌ ID 불일치: 요청({request.button_id}) ≠ 감지({detected_button_id})")
                            response.success = False
                            response.x = 0.0
                            response.y = 0.0
                            response.size = 0.0
                            response.is_pressed = False
                            response.timestamp = self.get_clock().now().to_msg()
                            return response
                        else:
                            # ID 매칭 성공
                            self.get_logger().info(f"✅ ID 매칭 성공: button_id={detected_button_id}")
                    else:
                        # button_id=0인 경우: 현재 유일하게 감지되는 버튼 (ID 매칭 불필요)
                        self.get_logger().info("📍 유일 버튼 감지 모드 (ID 매칭 생략)")
                else:
                    # 2개 이상의 버튼이 감지됨
                    button_ids = [obj.get('button_id', 'unknown') for obj in detected_buttons]
                    if request.button_id == 0:
                        self.get_logger().info(f"다중 버튼 탐지: 총 {len(detected_buttons)}개 (IDs: {button_ids})")
                    else:
                        recognized_count = len([obj for obj in total_button_objects if obj.get('button_id') not in [None, 'unmapped', -1, 'None']])
                        self.get_logger().info(f"다중 버튼 인식: 탐지 {len(total_button_objects)}개, 인식 성공 {recognized_count}개 (IDs: {button_ids})")
                    
                    if request.button_id == 0:
                        # button_id=0: 유일한 버튼만 허용 → success=false
                        self.get_logger().warning("button_id=0 요청이지만 유일한 버튼이 아님 - success=false")
                        response.success = False
                        response.x = 0.0
                        response.y = 0.0
                        response.size = 0.0
                        response.is_pressed = False
                        response.timestamp = self.get_clock().now().to_msg()
                    else:
                        # 특정 button_id 요청 시 다중 감지된 버튼들 중 해당 ID 찾기
                        target_button = next((btn for btn in detected_buttons if btn.get('button_id') == request.button_id), None)
                        
                        if target_button is None:
                            # 요청한 버튼이 없음
                            self.get_logger().warning(f"요청한 button_id={request.button_id}가 감지되지 않음")
                            response.success = False
                            response.x = 0.0
                            response.y = 0.0
                            response.size = 0.0
                            response.is_pressed = False
                            response.timestamp = self.get_clock().now().to_msg()
                        else:
                            # 요청한 버튼을 찾음 → 성공
                            btn = target_button
                            center = btn['center']
                            bbox = btn['bbox']
                            
                            # 좌표를 0~1 범위로 정규화 (설정된 목표 해상도 기준)
                            x_norm = float(center[0] / target_width)
                            y_norm = float(center[1] / target_height)
                            
                            # 버튼 크기를 0~1 범위로 정규화 (설정된 목표 해상도 기준)
                            bbox_width = bbox[2] - bbox[0]
                            bbox_height = bbox[3] - bbox[1]
                            bbox_area = bbox_width * bbox_height
                            target_area = target_width * target_height
                            size_norm = float(bbox_area / target_area)
                            
                            # 디버그: 크기 계산 상세 정보 (다중 버튼)
                            img_area = img_width * img_height
                            size_norm_actual = float(bbox_area / img_area)
                            self.get_logger().info(f"🔍 크기 계산(다중): bbox={bbox_width}x{bbox_height}(면적:{bbox_area})")
                            self.get_logger().info(f"🔍 면적 비교(다중): 실제이미지={img_area}, target={target_area}")
                            self.get_logger().info(f"🔍 정규화(다중): target기준={size_norm:.4f}, 실제기준={size_norm_actual:.4f}")
                            
                            response.success = True
                            response.x = x_norm
                            response.y = y_norm
                            response.size = size_norm
                            response.is_pressed = bool(btn.get('is_pressed', False))
                            response.timestamp = self.get_clock().now().to_msg()
                            
                            confidence = btn.get('confidence', 1.0)
                            self.get_logger().info(f"특정 버튼 탐지 성공: button_id={request.button_id}, "
                                                 f"x={x_norm:.3f}, y={y_norm:.3f}, size={size_norm:.3f}, "
                                                 f"pressed={btn.get('is_pressed', False)}, conf={confidence:.2f}")
                    
            except Exception as detection_error:
                self.get_logger().error(f"버튼 탐지 중 에러: {detection_error}")
                response.success = False
                response.x = 0.0
                response.y = 0.0
                response.size = 0.0
                response.is_pressed = False
                response.timestamp = self.get_clock().now().to_msg()
                
        except Exception as e:
            self.get_logger().error(f"버튼 상태 서비스 에러: {e}")
            response.robot_id = request.robot_id
            response.button_id = request.button_id
            response.success = False
            response.x = 0.0
            response.y = 0.0
            response.size = 0.0
            response.is_pressed = False
            response.timestamp = self.get_clock().now().to_msg()
        
        return response
    
    # 토픽 퍼블리시 메소드들
    

    
    def detect_and_publish_obstacles(self, objects, depth_camera, mode_id):
        """뎁스 카메라 기반 장애물 감지 및 발행 (보수적 유지 + 간단 트래킹)"""
        if mode_id != 5:  # 일반 주행 모드가 아니면 스킵
            return
        
        # 뎁스 카메라가 없으면 스킵
        if depth_camera is None or not hasattr(depth_camera, 'pixel_to_3d'):
            return
        
        try:
            current_time = self.get_clock().now()
            
            # 현재 프레임에서 감지된 장애물들 수집
            current_obstacles = self.obstacle_detector.detect_obstacles_from_objects(
                objects, depth_camera
            )
            
            # 내부 트래킹 상태 초기화
            if not hasattr(self, 'obstacle_tracks'):
                self.obstacle_tracks = {}  # tracking_id(or pseudo id) -> track dict
            if not hasattr(self, 'next_obstacle_track_id'):
                self.next_obstacle_track_id = 0
            
            # 매칭을 위한 헬퍼 (좌표 근접 기반)
            def match_track(obs, tracks, pos_tol=0.08):
                tid = obs.get('tracking_id')
                if tid is not None and tid in tracks:
                    return tid
                best_id = None
                best_dist = 1e9
                for track_id, tr in tracks.items():
                    if tr.get('class_name') != obs.get('class_name'):
                        continue
                    dx = (tr['x'] - obs['x'])
                    dy = (tr['y'] - obs['y'])
                    dist = (dx*dx + dy*dy) ** 0.5
                    if dist < best_dist and dist <= pos_tol:
                        best_dist = dist
                        best_id = track_id
                return best_id
            
            # 기존 트랙 업데이트 플래그 초기화
            for tr in getattr(self, 'obstacle_tracks', {}).values():
                tr['updated'] = False
            
            # 현재 장애물들을 트랙에 매칭/업데이트 및 즉시 발행
            for obs in current_obstacles:
                obs.setdefault('x', obs.get('normalized_x', 0.0))
                obs.setdefault('y', obs.get('normalized_y', 0.0))
                
                track_id = match_track(obs, self.obstacle_tracks)
                if track_id is None:
                    track_id = f"trk_{self.next_obstacle_track_id}"
                    self.next_obstacle_track_id += 1
                    self.obstacle_tracks[track_id] = {
                        'class_name': obs['class_name'],
                        'dynamic': obs['dynamic'],
                        'x': obs['x'],
                        'y': obs['y'],
                        'depth': obs['depth'],
                        'last_seen': current_time,
                        'missing_frames': 0,
                        'started': True
                    }
                else:
                    tr = self.obstacle_tracks[track_id]
                    tr['x'] = obs['x']
                    tr['y'] = obs['y']
                    tr['depth'] = obs['depth']
                    tr['last_seen'] = current_time
                    tr['missing_frames'] = 0
                    tr['dynamic'] = obs['dynamic']
                    tr['class_name'] = obs['class_name']
                    tr['updated'] = True
                    tr.setdefault('started', True)
                
                obstacle_msg = Obstacle()
                obstacle_msg.robot_id = obs['robot_id']
                obstacle_msg.dynamic = obs['dynamic']
                obstacle_msg.x = obs['x']
                obstacle_msg.y = obs['y']
                obstacle_msg.depth = obs['depth']
                self.obstacle_pub.publish(obstacle_msg)
            
            # 누락된 트랙 보수적 유지/종료
            HOLD_FRAMES = 5  # 잠깐 끊길 때 유지할 프레임 수
            to_delete = []
            for track_id, tr in self.obstacle_tracks.items():
                if tr.get('updated'):
                    continue
                tr['missing_frames'] = tr.get('missing_frames', 0) + 1
                if tr['missing_frames'] <= HOLD_FRAMES:
                    obstacle_msg = Obstacle()
                    obstacle_msg.robot_id = 0
                    obstacle_msg.dynamic = tr['dynamic']
                    obstacle_msg.x = tr['x']
                    obstacle_msg.y = tr['y']
                    obstacle_msg.depth = tr['depth']
                    self.obstacle_pub.publish(obstacle_msg)
                else:
                    to_delete.append(track_id)
            for tid in to_delete:
                del self.obstacle_tracks[tid]
            
            self.last_obstacle_publish_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"❌ 장애물 감지 및 발행 실패: {e}")
    

    
    def set_vs_mode_callback(self, request, response):
        """VS 모드 설정 처리 - 전방/후방 독립적 관리"""
        try:
            self.get_logger().info(f"VS 모드 설정 요청: robot_id={request.robot_id}, mode_id={request.mode_id}")
            
            if request.mode_id not in self.mode_names:
                self.get_logger().error(f"잘못된 모드 ID: {request.mode_id}")
                response.robot_id = request.robot_id
                response.success = False
                return response
            
            # 전방/후방 모드 구분
            is_front_mode = request.mode_id in [3, 4, 5, 6]
            is_rear_mode = request.mode_id in [0, 1, 2]
            
            if is_front_mode:
                old_mode_id = self.current_front_mode_id
                old_mode = self.mode_names.get(old_mode_id, "Unknown")
                new_mode = self.mode_names[request.mode_id]
                
                self.current_front_mode_id = request.mode_id
                self.get_logger().info(f"전방 모드 변경: {old_mode} → {new_mode}")
                
                # 전방 카메라만 업데이트
                self.update_front_camera()
                
            elif is_rear_mode:
                old_mode_id = self.current_rear_mode_id
                old_mode = self.mode_names.get(old_mode_id, "Unknown")
                new_mode = self.mode_names[request.mode_id]
                
                self.current_rear_mode_id = request.mode_id
                self.get_logger().info(f"후방 모드 변경: {old_mode} → {new_mode}")
                
                # 후방 카메라만 업데이트
                self.update_rear_camera()
                
                # 👤 PersonTracker 모드 연동 (후방 모드 변경시)
                if hasattr(self, 'person_tracker') and self.person_tracker:
                    self.person_tracker.set_mode(request.mode_id)
            
            response.robot_id = request.robot_id
            response.success = True
            
            # 시뮬레이션 모드 초기화 (전방/후방 구분 없이 처리)
            if request.mode_id in self.simulation_counters:
                self.simulation_counters[request.mode_id] = 0
                
        except Exception as e:
            self.get_logger().error(f"VS 모드 설정 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
        
        return response
    
    def get_active_mode_id(self):
        """현재 활성 모드 ID 반환 (전방 우선, 대기모드가 아닌 경우)"""
        # 전방이 대기모드가 아니면 전방 모드 반환
        if self.current_front_mode_id != 6:
            return self.current_front_mode_id
        # 후방이 대기모드가 아니면 후방 모드 반환  
        if self.current_rear_mode_id != 0:
            return self.current_rear_mode_id
        # 둘 다 대기모드면 전방 모드 반환
        return self.current_front_mode_id
    
    def get_active_mode_name(self):
        """현재 활성 모드 이름 반환"""
        mode_id = self.get_active_mode_id()
        return self.mode_names.get(mode_id, f"ID_{mode_id}")
    
    def get_active_cameras(self):
        """현재 활성화된 카메라들을 반환 (전방/후방 모두 포함)"""
        active_cameras = []
        
        # 전방 카메라 체크 - 웹캠과 뎁스를 분리해서 처리
        if self.current_front_mode_id in [3, 4, 5, 6]:
            # 웹캠이 실제로 있고 뎁스와 다른 경우만 웹캠 창 추가
            front_webcam = getattr(self.camera_manager, 'front_webcam', None)
            front_depth = getattr(self.camera_manager, 'front_depth', None)
            
            # 웹캠이 초기화되어 있으면 웹캠 창 추가
            if front_webcam and getattr(self.camera_manager, 'front_webcam_initialized', False):
                if self.current_front_mode_id == 3:
                    webcam_name = 'Front USB Webcam (Elevator Out)'
                elif self.current_front_mode_id == 4:
                    webcam_name = 'Front USB Webcam (Elevator In)'
                elif self.current_front_mode_id == 5:
                    webcam_name = 'Front USB Webcam (ArUco)'
                else:  # mode_id == 6
                    webcam_name = 'Front USB Webcam (Standby)'
                    
                active_cameras.append({
                    'camera': front_webcam,
                    'depth_camera': None,
                    'name': webcam_name,
                    'mode_id': self.current_front_mode_id,
                    'type': 'front_webcam'
                })
            
            # 뎁스 카메라가 초기화되어 있으면 뎁스 창 추가
            if front_depth and getattr(self.camera_manager, 'front_depth_initialized', False):
                if self.current_front_mode_id == 3:
                    depth_name = 'Front Depth Camera (Elevator Out)'
                elif self.current_front_mode_id == 4:
                    depth_name = 'Front Depth Camera (Elevator In)'
                elif self.current_front_mode_id == 5:
                    depth_name = 'Front Depth Camera (YOLO)'
                else:  # mode_id == 6
                    depth_name = 'Front Depth Camera (Standby)'
                    
                active_cameras.append({
                    'camera': front_depth,
                    'depth_camera': front_depth,
                    'name': depth_name,
                    'mode_id': self.current_front_mode_id,
                    'type': 'front_depth'
                })
        elif hasattr(self, 'current_camera') and self.current_camera is not None:
            # 다른 전방 모드들은 기존 방식 (혹시 있다면)
            active_cameras.append({
                'camera': self.current_camera,
                'depth_camera': getattr(self, 'current_depth_camera', None),
                'name': getattr(self, 'current_camera_name', 'Front Camera'),
                'mode_id': self.current_front_mode_id,
                'type': 'front'
            })
        
        # 후방 카메라 체크
        if hasattr(self, 'current_rear_camera') and self.current_rear_camera is not None:
            active_cameras.append({
                'camera': self.current_rear_camera,
                'depth_camera': None,  # 후방은 뎁스 카메라 없음
                'name': getattr(self, 'current_rear_camera_name', 'Rear Camera'),
                'mode_id': self.current_rear_mode_id,
                'type': 'rear'
            })
        
        return active_cameras
    
    def update_front_camera(self):
        """전방 카메라와 모델 업데이트"""
        try:
            mode_name = self.mode_names.get(self.current_front_mode_id, f"ID_{self.current_front_mode_id}")
            old_camera_name = getattr(self, 'current_front_camera_name', "No Camera")
            
            # 전방 카메라 초기화
            self.camera_manager.initialize_cameras_for_mode(self.current_front_mode_id)
            
            # 전방 카메라 업데이트
            camera, depth_camera, camera_name = self.camera_manager.get_camera_for_mode(self.current_front_mode_id)
            
            self.current_camera = camera  # 메인 카메라 (호환성)
            self.current_depth_camera = depth_camera
            self.current_camera_name = camera_name
            
            # 전방 모델 업데이트
            self.model_detector.set_model_for_mode(self.current_front_mode_id)
            model_info = self.model_detector.get_current_model_info()
            
            # 결과 로그
            if camera:
                self.get_logger().info(f"📷 전방 카메라: {old_camera_name} → {camera_name} (모드: {mode_name})")
            else:
                self.get_logger().warning(f"⚠️ 전방 모드 {mode_name}용 카메라가 없습니다")
            
            if model_info['is_active']:
                current_classes = ', '.join(model_info['class_names'][:3])
                if len(model_info['class_names']) > 3:
                    current_classes += "..."
                self.get_logger().info(f"�� 전방 모델: {model_info['model_name']} (클래스: {current_classes})")
            else:
                self.get_logger().info(f"🤖 전방 모델 비활성화")
                
        except Exception as e:
            self.get_logger().error(f"전방 카메라 업데이트 에러: {e}")
    
    def update_rear_camera(self):
        """후방 카메라 업데이트 (후방은 모델 사용하지 않음)"""
        try:
            mode_name = self.mode_names.get(self.current_rear_mode_id, f"ID_{self.current_rear_mode_id}")
            old_camera_name = getattr(self, 'current_rear_camera_name', "No Camera")
            
            # 후방 카메라 초기화
            self.camera_manager.initialize_cameras_for_mode(self.current_rear_mode_id)
            
            # 후방 카메라 직접 확인 및 설정 (모든 모드에서 카메라 사용)
            if self.camera_manager.rear_webcam_initialized:
                self.current_rear_camera = self.camera_manager.rear_webcam
                if self.current_rear_mode_id == 0:  # 후방 대기모드
                    self.current_rear_camera_name = "Rear Webcam (Standby)"
                else:  # 등록 모드(1), 추적 모드(2)
                    self.current_rear_camera_name = "Rear Webcam"
                
                self.get_logger().info(f"📷 후방 카메라: {old_camera_name} → {self.current_rear_camera_name} (모드: {mode_name})")
                
                # 📹 후방 카메라가 활성화되면 UDP 스트리밍 시작
                self._start_rear_camera_streaming()
                
            else:
                self.current_rear_camera = None
                self.current_rear_camera_name = "None"
                self.get_logger().warning(f"⚠️ 후방 모드 {mode_name}용 카메라가 없습니다")
                
                # 📹 후방 카메라가 비활성화되면 UDP 스트리밍 중지
                self._stop_rear_camera_streaming()
                
        except Exception as e:
            self.get_logger().error(f"후방 카메라 업데이트 에러: {e}")
    

    
    def elevator_status_callback(self, request, response):
        """엘리베이터 위치 및 방향 감지 처리 - display 객체 OCR 기반"""
        try:
            self.get_logger().info(f"엘리베이터 상태 감지 요청: robot_id={request.robot_id}")
            
            # 현재 활성 카메라에서 이미지 가져오기
            current_color = None
            current_depth = None
            
            # 전면 카메라 사용 (엘리베이터 디스플레이 감지에 적합)
            if self.current_camera:
                depth_frame, color_frame = self.current_camera.get_frames()
                if color_frame is not None:
                    current_color = color_frame
                    current_depth = depth_frame
            
            # 기본값 설정
            detected_floor = 1  # 기본 1층
            detected_direction = 0  # 기본 상행
            success = True  # 무조건 성공 반환
            
            if current_color is not None:
                # 객체 감지 수행
                detected_objects = self.model_detector.detect_objects(
                    current_color, 
                    current_depth, 
                    self.confidence_threshold, 
                    self.current_front_mode_id
                )
                
                # 객체에 OCR 결과 추가 (display 객체만)
                enhanced_objects = self._enhance_objects_with_ocr(current_color, detected_objects)
                
                # 'display' 객체에서 층수 정보 추출
                display_objects = [obj for obj in enhanced_objects if obj.get('class_name') == 'display']
                
                if display_objects:
                    for display_obj in display_objects:
                        floor_number = display_obj.get('floor_number')
                        floor_text = display_obj.get('ocr_text', '')
                        ocr_success = display_obj.get('ocr_success', False)
                        
                        if floor_number is not None:
                            detected_floor = floor_number
                            success = True
                            self.get_logger().debug(f"🏢 엘리베이터 층수 인식: '{floor_text}' -> {detected_floor}층 (신뢰도: {display_obj.get('confidence', 0):.2f})")
                            break  # 첫 번째 성공한 결과 사용
                        elif ocr_success:
                            self.get_logger().warn(f"층수 파싱 실패: '{floor_text}'")
                        else:
                            self.get_logger().debug(f"디스플레이 감지됨 (OCR 실패)")
                else:
                    self.get_logger().debug("display 객체가 감지되지 않음")
            else:
                self.get_logger().warn("카메라에서 이미지를 가져올 수 없음")
            
            # direction_light 객체로 방향 감지 - 엘리베이터 외부에서만 수행
            if self.current_front_mode_id != 4:  # 엘리베이터 내부가 아닌 경우에만
                if current_color is not None and 'enhanced_objects' in locals():
                    direction_objects = [obj for obj in enhanced_objects if obj.get('class_name') == 'direction_light']
                    if direction_objects:
                        # 🚦 방향등 실시간 감지 시도
                        detected_direction = self._detect_direction_by_lights(current_color, direction_objects)
                        
                        # 🎯 방향 업데이트 (깜빡임 감지 시 항상 업데이트)
                        if detected_direction != -1:
                            if (detected_direction != self.last_elevator_direction or self.last_blink_detected):
                                # 방향 변경되었거나 깜빡임이 감지된 경우 업데이트
                                self.last_elevator_direction = detected_direction
                                self.last_direction_detection_time = self.get_clock().now()
                                blink_info = " (깜빡임 감지)" if self.last_blink_detected else ""
                                self.get_logger().info(f"🎯 방향등 기반 방향 업데이트: {len(direction_objects)}개 → {'상행' if detected_direction == 0 else '하행'}{blink_info}")
                                
                                # 🔄 깜빡임 처리 완료 후 플래그 초기화 (다음 깜빡임 감지를 위해)
                                if self.last_blink_detected:
                                    self.last_blink_detected = False
                                    self.get_logger().info("🔄 깜빡임 감지 플래그 초기화 완료")
                            else:
                                # 방향 변화 없음 (깜빡임도 없음)
                                detected_direction = self.last_elevator_direction
                    else:
                        # 방향등이 감지되지 않으면 캐시된 방향 사용
                        detected_direction = self.last_elevator_direction
                        self.get_logger().debug(f"방향등 미감지 → 캐시된 방향 사용: {'상행' if detected_direction == 0 else '하행'}")
                else:
                    # 이미지나 객체가 없으면 캐시된 방향 사용
                    detected_direction = self.last_elevator_direction
                    self.get_logger().debug(f"이미지/객체 없음 → 캐시된 방향 사용: {'상행' if detected_direction == 0 else '하행'}")
            else:
                # 엘리베이터 내부 모드에서는 방향등이 없으므로 캐시된 방향 사용
                detected_direction = self.last_elevator_direction
                self.get_logger().debug(f"엘리베이터 내부 모드 → 캐시된 방향 사용: {'상행' if detected_direction == 0 else '하행'}")
            
            # 응답 설정
            response.robot_id = request.robot_id
            response.success = success
            response.direction = detected_direction
            response.position = detected_floor
            
            direction_str = "상행" if detected_direction == 0 else "하행"
            status_str = "OCR 성공" if success else "OCR 실패 (기본값 사용)"
            self.get_logger().info(f"엘리베이터 상태: {direction_str}, {detected_floor}층 ({status_str})")
                
        except Exception as e:
            self.get_logger().error(f"엘리베이터 상태 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = True  # 무조건 성공 반환
            response.direction = 0
            response.position = 1
        
        return response
    
    def _enhance_objects_with_ocr(self, color_image: np.ndarray, objects: List[dict]) -> List[dict]:
        """객체 목록에서 display 객체들에 대해 OCR 수행하고 direction_light 객체들에 대해 색상 분석 수행"""
        enhanced_objects = []
        
        # 방향등 위치 분석을 위해 먼저 direction_light 객체들을 찾아서 정렬
        direction_lights = [obj for obj in objects if obj.get('class_name') == 'direction_light']
        sorted_direction_lights = sorted(direction_lights, key=lambda x: x['center'][1]) if len(direction_lights) >= 2 else []
        
        for obj in objects:
            enhanced_obj = obj.copy()
            
            # display 객체에 대해서만 OCR 수행
            if obj.get('class_name') == 'display':
                try:
                    bbox = obj.get('bbox')
                    if bbox:
                        # 🔥 최적화된 EasyOCR 사용 (단순 크롭 + EasyOCR만)
                        ocr_result = self.display_ocr.recognize_from_display_bbox_stable(color_image, bbox)
                        floor_text = ocr_result.get('text', '?')
                        confidence = ocr_result.get('confidence', 0.0)
                        digit_bbox = ocr_result.get('digit_bbox', None)
                        floor_number = None
                        
                        if floor_text and floor_text.strip() and floor_text != "?":
                            floor_number = self._parse_floor_number(floor_text.strip())
                        
                        # OCR 결과를 객체에 저장
                        enhanced_obj['ocr_text'] = floor_text if floor_text else ""
                        enhanced_obj['floor_number'] = floor_number
                        enhanced_obj['ocr_success'] = bool(floor_text and floor_text.strip() and floor_text != "?")
                        enhanced_obj['ocr_confidence'] = confidence  # 신뢰도 추가
                        enhanced_obj['digit_bbox'] = digit_bbox  # 🎯 숫자 영역 바운딩박스 추가
                        
                except Exception as e:
                    self.get_logger().error(f"DisplayOCR 에러: {e}")
                    enhanced_obj['ocr_text'] = ""
                    enhanced_obj['floor_number'] = None
                    enhanced_obj['ocr_success'] = False
                    enhanced_obj['ocr_confidence'] = 0.0
                    enhanced_obj['digit_bbox'] = None
            
            # 🚦 direction_light 객체에 대해 색상 분석 및 위치 정보 추가
            elif obj.get('class_name') == 'direction_light':
                try:
                    # 색상 분석
                    detected_color = self._analyze_light_color(color_image, obj)
                    brightness = self._get_light_brightness(color_image, obj)
                    
                    # 위치 분석 (위쪽/아래쪽 구분)
                    light_position = 'unknown'
                    if len(sorted_direction_lights) >= 2:
                        current_obj_y = obj['center'][1]
                        if current_obj_y == sorted_direction_lights[0]['center'][1]:
                            light_position = 'upper'
                        elif current_obj_y == sorted_direction_lights[-1]['center'][1]:
                            light_position = 'lower'
                        else:
                            light_position = 'middle'
                    
                    # 색상 및 위치 정보를 객체에 저장
                    enhanced_obj['light_color'] = detected_color
                    enhanced_obj['light_brightness'] = brightness
                    enhanced_obj['light_position'] = light_position
                    
                    # 디버그 로그
                    self.get_logger().debug(f"🚦 방향등 분석: 위치={light_position}, 색상={detected_color}, 밝기={brightness:.1f}")
                    
                except Exception as e:
                    self.get_logger().error(f"방향등 색상 분석 에러: {e}")
                    enhanced_obj['light_color'] = 'UNKNOWN'
                    enhanced_obj['light_brightness'] = 0.0
                    enhanced_obj['light_position'] = 'unknown'
            
            enhanced_objects.append(enhanced_obj)
        
        return enhanced_objects
    
    def _parse_floor_number(self, floor_text: str) -> Optional[int]:
        """OCR 텍스트에서 층수 추출"""
        try:
            # 공백 제거
            text = floor_text.strip().upper()
            
            if not text:
                return None
            
            # 지하층 처리 (B1, B2 등)
            if text.startswith('B'):
                basement_str = text[1:]
                if basement_str.isdigit():
                    return -int(basement_str)  # 지하는 음수로
            
            # 일반 층수 (숫자만 추출)
            # "12F", "3층", "05" 등에서 숫자만 추출
            numbers = ''.join(c for c in text if c.isdigit())
            
            if numbers:
                floor_num = int(numbers)
                # 합리적인 범위 체크 (지상 1~50층)
                if 1 <= floor_num <= 50:
                    return floor_num
                elif floor_num == 0:
                    return 1  # 0은 1층으로 간주
            
            # 특수 경우 처리
            if text in ['G', 'GF', 'L']:  # Ground Floor, Lobby
                return 1
            
            # 범위를 벗어나는 층수 처리
            if numbers:
                floor_num = int(numbers)
                if floor_num > 50:
                    return 50
            
            self.get_logger().debug(f"층수 파싱 실패: '{floor_text}' -> '{text}'")
            return None
            
        except Exception as e:
            self.get_logger().error(f"층수 파싱 에러: {e}")
            return None
    
    def door_status_callback(self, request, response):
        """문 열림 감지 처리 - 객체 감지 기반"""
        try:
            self.get_logger().info(f"문 상태 감지 요청: robot_id={request.robot_id}")
            
            # 현재 활성 카메라에서 이미지 가져오기
            current_color = None
            current_depth = None
            
            # 전면 카메라 사용 (문 감지에 적합)
            if self.current_camera:
                depth_frame, color_frame = self.current_camera.get_frames()
                if color_frame is not None:
                    current_color = color_frame
                    current_depth = depth_frame
            
            # 카메라에서 이미지를 가져올 수 없는 경우 임시로 성공 반환
            if current_color is None:
                self.get_logger().warn("카메라에서 이미지를 가져올 수 없음 - 문 상태 감지 실패")
                response.robot_id = request.robot_id
                response.success = True  # 무조건 성공 반환
                response.door_opened = False  # 임시 조치: 카메라 실패 시에도 True
                return response
            
            # 객체 감지 수행
            detected_objects = self.model_detector.detect_objects(
                current_color, 
                current_depth, 
                self.confidence_threshold, 
                self.current_front_mode_id
            )
            
            # door 객체가 감지되었는지 확인
            door_detected = False
            door_count = 0
            
            for obj in detected_objects:
                if obj['class_name'] == 'door':
                    door_detected = True
                    door_count += 1
                    self.get_logger().info(f"문 객체 감지됨: 신뢰도={obj['confidence']:.2f}, 위치=({obj['center'][0]}, {obj['center'][1]})")
            
            # 문이 감지되지 않으면 열림, 감지되면 닫힘으로 판단
            response.robot_id = request.robot_id
            response.success = True
            response.door_opened = not door_detected  # 🔥 로직 수정: 문이 안 보이면 열린 것
            
            door_str = "열림" if not door_detected else "닫힘"
            self.get_logger().info(f"문 상태: {door_str} (감지된 문 객체 수: {door_count})")
                
        except Exception as e:
            self.get_logger().error(f"문 상태 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = True  # 무조건 성공 반환
            response.door_opened = False
        
        return response
    
    def detect_and_publish_glass_door_status(self, objects, mode_id):
        """유리 문 상태 감지 및 발행 (이벤트 기반)"""
        if mode_id != 5:  # 전방 일반 주행 모드에서만 (뎁스 카메라 사용)
            return
            
        try:
            # door 객체가 감지되었는지 확인
            door_detected = False
            
            for obj in objects:
                if obj['class_name'] == 'door':
                    door_detected = True
                    break
            
            # 현재 상태 계산 (문이 안 보이면 열린 것)
            current_opened = not door_detected
            
            # 이전 상태와 비교하여 변경되었을 때만 발행
            if self.last_glass_door_opened is None:
                # 첫 번째 실행 시 초기화
                self.last_glass_door_opened = current_opened
                self.get_logger().info(f"🚪 유리 문 상태 초기화: {'열림' if current_opened else '닫힘'}")
                return
            
            # 상태가 변경되었을 때만 발행
            if self.last_glass_door_opened != current_opened:
                # 유리 문 상태 메시지 생성 및 발행
                glass_door_msg = GlassDoorStatus()
                glass_door_msg.robot_id = 0
                glass_door_msg.opened = current_opened
                
                self.glass_door_pub.publish(glass_door_msg)
                
                # 상태 업데이트
                old_state = "열림" if self.last_glass_door_opened else "닫힘"
                new_state = "열림" if current_opened else "닫힘"
                self.last_glass_door_opened = current_opened
                
                # 로그 출력
                self.get_logger().info(f"🚪 유리 문 상태 변경: {old_state} → {new_state} (이벤트 발행)")
                
        except Exception as e:
            self.get_logger().error(f"❌ 유리 문 상태 감지 및 발행 실패: {e}")
    

    
    def location_callback(self, request, response):
        """현재 위치 감지 처리"""
        try:
            self.get_logger().info(f"위치 감지 요청: robot_id={request.robot_id}")
            
            response.robot_id = request.robot_id
            response.success = True
            
            # 시뮬레이션 모드별 위치 시나리오 처리
            if self.get_active_mode_id() == 100:  # 배송 시뮬레이션
                counter = self.simulation_counters[100]
                if counter == 0:
                    location_id = 2  # RES_PICKUP
                    self.get_logger().info("배송 시뮬레이션: 픽업 장소 도착")
                elif counter == 1:
                    location_id = 101  # ROOM_101
                    self.get_logger().info("배송 시뮬레이션: 101호 도착")
                else:
                    location_id = 101  # ROOM_101 유지
                    self.get_logger().info("배송 시뮬레이션: 101호 대기 중")
                
                self.simulation_counters[100] += 1
                response.location_id = location_id
                
            elif self.get_active_mode_id() == 103:  # 복귀 시뮬레이션
                counter = self.simulation_counters[103]
                if counter == 0:
                    location_id = 0  # LOB_WAITING
                    self.get_logger().info("복귀 시뮬레이션: 로비 대기 위치 도착")
                else:
                    location_id = 0  # LOB_WAITING 유지
                    self.get_logger().info("복귀 시뮬레이션: 로비 대기 중")
                
                self.simulation_counters[103] += 1
                response.location_id = location_id
                
            elif self.current_front_mode_id == 5:  # 일반 주행 모드 - ArUco 마커 기반 위치
                current_location = self.detect_and_update_location()
                response.location_id = current_location
                
                location_names = {
                    0: "LOB_WAITING", 1: "LOB_CALL", 2: "RES_PICKUP", 3: "RES_CALL",
                    4: "SUP_PICKUP", 5: "ELE_1", 6: "ELE_2", 101: "ROOM_101",
                    102: "ROOM_102", 201: "ROOM_201", 202: "ROOM_202"
                }
                location_name = location_names.get(current_location, f"UNKNOWN({current_location})")
                
                # 마지막 감지 시간 정보 포함
                if self.last_detection_time:
                    time_diff = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                    self.get_logger().debug(f"현재 위치: {location_name} (ArUco 기반, 마지막 감지: {time_diff:.1f}초 전)")
                else:
                    self.get_logger().debug(f"현재 위치: {location_name} (ArUco 기반, 초기값)")
                    
            else:  # 기타 모드 - 기본 위치 반환
                response.location_id = self.last_detected_location_id  # 마지막 알려진 위치 유지
                mode_name = self.get_active_mode_name()
                self.get_logger().debug(f"위치 서비스: {mode_name}에서는 ArUco 사용 안함 (마지막 위치 유지)")
                
        except Exception as e:
            self.get_logger().error(f"위치 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
            response.location_id = self.last_detected_location_id
        
        return response

    def _draw_objects_on_image(self, image: np.ndarray, objects: List[dict], mode_id: int = None) -> np.ndarray:
        """YOLO로 탐지된 객체들을 이미지에 시각화"""
        import cv2
        
        # 현재 모드 ID 가져오기 (매개변수로 전달되지 않은 경우)
        if mode_id is None:
            mode_id = self.get_active_mode_id()
        
        # 화면 중심점 계산
        image_height, image_width = image.shape[:2]
        screen_center = (image_width // 2, image_height // 2)
        
        # 객체 타입별 색상 정의
        color_map = {
            'person': (255, 0, 0),        # 빨간색 (동적 장애물)
            'chair': (0, 255, 255),       # 노란색 (정적 장애물)
            'door': (255, 255, 0),        # 청록색
            'button': (0, 255, 0),        # 초록색
            'direction_light': (255, 165, 0),  # 주황색
            'display': (0, 165, 255),     # 오렌지색
        }
        
        for i, obj in enumerate(objects):
            center = obj['center']
            is_pressed = obj.get('is_pressed', False)
            depth_mm = obj['depth_mm']
            class_name = obj.get('class_name', f'obj_{i+1}')
            confidence = obj.get('confidence', 1.0)
            bbox = obj.get('bbox', None)
            model_name = obj.get('model_name', 'unknown')
            
            # YOLO 바운딩박스 그리기
            if bbox and len(bbox) == 4:
                x1, y1, x2, y2 = bbox
                
                # 장애물 여부 확인
                is_obstacle = obj.get('is_obstacle', False)
                
                # 색상과 두께 결정
                if class_name == 'button' and is_pressed:
                    color = (0, 0, 255)  # 빨간색 (눌린 버튼)
                    thickness = 2
                elif is_obstacle:
                    # 장애물인 경우 더 두껍게 표시
                    color = color_map.get(class_name, (128, 128, 128))
                    thickness = 3
                else:
                    # 일반 객체
                    color = color_map.get(class_name, (128, 128, 128))
                    thickness = 2
                
                cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
                
                # 클래스 이름과 신뢰도 표시
                if class_name == 'button':
                    # 매핑된 버튼 ID 표시
                    button_id = obj.get('button_id', 'unknown')
                    floor_type = obj.get('floor_type', 'unknown')
                    recognition_method = obj.get('recognition_method', 'unknown')
                    
                    # 엘리베이터 내부 모드에서는 CNN 결과만 표시
                    if self.get_active_mode_id() == 4:  # 엘리베이터 내부 모드
                        if button_id == 102:
                            label = "OPEN"
                        elif button_id == 103:
                            label = "CLOSE"
                        elif button_id == 101:
                            label = "UP"
                        elif button_id == 100:
                            label = "DOWN"
                        elif button_id == 13:
                            label = "B1"
                        elif button_id == 14:
                            label = "B2"
                        elif isinstance(button_id, int) and button_id > 0:
                            label = f"{button_id}F"
                        else:
                            label = f"{button_id}"
                        
                        # CNN 신뢰도 표시
                        cnn_confidence = obj.get('confidence', 0.0)
                        if recognition_method == 'cnn_primary':
                            label += f" ({cnn_confidence:.2f})"
                        elif recognition_method == 'cnn_failed':
                            label += " (FAIL)"
                        elif recognition_method == 'cnn_unavailable':
                            label += " (NO_CNN)"
                    else:
                        # 엘리베이터 외부 모드에서는 기존 방식 유지
                        group_info = obj.get('group_info', '')
                        
                        if button_id == 102:
                            label = "OPEN"
                        elif button_id == 103:
                            label = "CLOSE"
                        elif button_id == 101:
                            label = "UP"
                        elif button_id == 100:
                            label = "DOWN"
                        elif floor_type == 'basement':
                            if button_id == 13:
                                label = "B1"
                            elif button_id == 14:
                                label = "B2"
                            else:
                                label = f"B{button_id}"
                        elif floor_type == 'floor' and isinstance(button_id, int):
                            label = f"{button_id}F"
                        else:
                            label = f"{button_id}"
                        
                        # 그룹 정보가 있으면 추가로 표시
                        if group_info:
                            label += f" ({group_info})"
                elif class_name == 'display':
                    # OCR 결과 가져오기
                    floor_number = obj.get('floor_number', None)
                    floor_text = obj.get('ocr_text', '')
                    ocr_success = obj.get('ocr_success', False)
                    digit_bbox = obj.get('digit_bbox', None)
                    
                    # 🎯 디스플레이 바운딩박스를 두껍게 표시 (YOLO가 감지한 전체 디스플레이 영역)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 165, 255), 3)  # 주황색으로 더 두껍게
                    
                    # 🎯 OCR이 실제로 인식한 숫자 영역 표시 (digit_bbox)
                    if digit_bbox and len(digit_bbox) == 4:
                        dx1, dy1, dx2, dy2 = digit_bbox
                        
                        # 🔥 숫자 영역 표시 (단순화)
                        cv2.rectangle(image, (dx1, dy1), (dx2, dy2), (0, 0, 255), 2)  # 빨간 박스
                    
                    # 디스플레이 라벨 (간단하게)
                    if floor_text and floor_text != "?":
                        label = f"DISPLAY: {floor_text}"
                    elif floor_number is not None:
                        label = f"DISPLAY: F{floor_number}"
                    else:
                        label = f"DISPLAY: {confidence:.2f}"
                elif class_name == 'direction_light':
                    # 🚦 방향등 정보 표시 (미리 계산된 정보 사용)
                    light_position = obj.get('light_position', 'unknown')
                    light_color = obj.get('light_color', 'UNKNOWN')
                    light_brightness = obj.get('light_brightness', 0.0)
                    
                    # 위치에 따른 라벨과 색상 설정
                    if light_position == 'upper':
                        position_text = "UP"
                        position_color = (0, 255, 0)  # 녹색
                    elif light_position == 'lower':
                        position_text = "DOWN"
                        position_color = (0, 0, 255)  # 빨간색
                    elif light_position == 'middle':
                        position_text = "MID"
                        position_color = (255, 255, 0)  # 노란색
                    else:
                        position_text = "UNKNOWN"
                        position_color = (128, 128, 128)  # 회색
                    
                    # 색상에 따른 표시기
                    if light_color == 'GREEN':
                        color_indicator = "GREEN"
                    elif light_color == 'RED':
                        color_indicator = "RED"
                    else:
                        color_indicator = "OFF"
                    
                    # 최종 라벨 생성 (단순화)
                    label = f"{position_text}"  # "UP" 또는 "DOWN"만 표시
                    
                    # 방향등 오버레이 제거됨 (깔끔한 표시를 위해)
                    
                    # 위치에 따른 색상 설정
                    color = position_color
                else:
                    label = f"{class_name}: {confidence:.2f}"
                
                # 장애물인 경우 특별한 라벨 생성
                if is_obstacle:
                    obstacle_type = obj.get('obstacle_type', 'unknown')
                    distance_m = obj.get('distance_m', 0.0)
                    world_x = obj.get('world_x', 0.0)
                    world_y = obj.get('world_y', 0.0)
                    
                    # 장애물 타입과 거리 표시
                    if obstacle_type == 'dynamic':
                        label = f"DYNAMIC OBSTACLE: {distance_m:.1f}m"
                    else:
                        label = f"STATIC OBSTACLE: {distance_m:.1f}m"
                    
                    # 라벨 배경 (가독성 향상)
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    cv2.rectangle(image, (x1, y1-25), (x1+label_size[0], y1), color, -1)
                    cv2.putText(image, label, (x1, y1-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    # 월드 좌표 표시 (객체 아래쪽)
                    coord_text = f"({world_x:.2f}m, {world_y:.2f}m)"
                    coord_size = cv2.getTextSize(coord_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
                    cv2.rectangle(image, (x1, y2), (x1+coord_size[0], y2+20), (0, 0, 0), -1)
                    cv2.putText(image, coord_text, (x1, y2+15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    
                    # 장애물 아이콘 표시 제거 (이모지로 인한 ??? 문제 해결)
                else:
                    # 기존 객체 라벨
                    cv2.putText(image, label, (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                
                # 버튼의 중심점 좌표 표시
                if class_name == 'button':
                    # 픽셀 좌표 표시
                    coord_text = f"({center[0]},{center[1]})"
                    cv2.putText(image, coord_text, (center[0]-25, center[1]+15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                    
                    # 버튼 ID 표시 (이미 있는 label 아래에 추가 정보)
                    button_id = obj.get('button_id', 'unknown')
                    recognition_method = obj.get('recognition_method', 'unknown')
                    
                    # 엘리베이터 내부 모드에서는 CNN 관련 정보만 표시
                    if self.get_active_mode_id() == 4:  # 엘리베이터 내부 모드
                        if recognition_method == 'cnn_primary':
                            # CNN 성공 시 신뢰도 표시
                            cnn_confidence = obj.get('confidence', 0.0)
                            id_text = f"CNN:{cnn_confidence:.2f}"
                            cv2.putText(image, id_text, (center[0]-25, center[1]+30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)  # 초록색
                        elif recognition_method == 'cnn_failed':
                            # CNN 실패 시 표시
                            id_text = "CNN:FAIL"
                            cv2.putText(image, id_text, (center[0]-25, center[1]+30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)  # 빨간색
                        elif recognition_method == 'cnn_unavailable':
                            # CNN 사용 불가 시 표시
                            id_text = "CNN:UNAVAIL"
                            cv2.putText(image, id_text, (center[0]-30, center[1]+30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (128, 128, 128), 1)  # 회색
                    else:
                        # 엘리베이터 외부 모드에서는 기존 방식 유지
                        if button_id != 'unknown':
                            id_text = f"ID:{button_id}"
                            cv2.putText(image, id_text, (center[0]-20, center[1]+30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
                
                # 모델 이름 표시 제거됨 (오버레이 정리)
                
                # 거리 정보 표시 제거됨 (오버레이 정리)
                
                # 버튼 눌림 상태 표시 (pressed/unpressed + 신뢰도)
                if class_name == 'button':
                    pressed_confidence = obj.get('pressed_confidence', 0.0)
                    pressed_method = obj.get('pressed_method', 'none')
                    
                    if pressed_method not in ['no_cnn', 'no_model', 'none']:
                        # pressed_prob과 unpressed_prob 값 가져오기
                        pressed_prob = obj.get('pressed_prob', 0.0)
                        unpressed_prob = obj.get('unpressed_prob', 0.0)
                        
                        # pressed/unpressed 상태 표시 (두 신뢰도 모두 표시)
                        if is_pressed:
                            pressed_text = f"PRESSED (P:{pressed_prob:.2f} U:{unpressed_prob:.2f})"
                            pressed_color = (0, 0, 255)  # 빨간색
                        else:
                            pressed_text = f"UNPRESSED (P:{pressed_prob:.2f} U:{unpressed_prob:.2f})"
                            pressed_color = (0, 255, 0)  # 초록색
                        
                        cv2.putText(image, pressed_text, (center[0]-60, center[1]+45), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, pressed_color, 1)
        
        # 🚗 일반주행 모드에서 장애물 전용 시각화
        if mode_id == 5:  # 일반 주행 모드
            # 화면 중심에 수직선 항상 표시 (파란색)
            cv2.line(image, (screen_center[0], 0), (screen_center[0], image_height), (255, 0, 0), 2)
            
            # 장애물에 대해서만 처리
            obstacle_objects = [obj for obj in objects if obj.get('is_obstacle', False)]
            
            for obj in obstacle_objects:
                center = obj['center']
                object_center_x = int(center[0])
                world_x = obj.get('world_x', 0.0)
                
                # 장애물 중심점에 수직선 그리기 (빨간색)
                cv2.line(image, (object_center_x, 0), (object_center_x, image_height), (0, 0, 255), 2)
                
                # X 차이 계산 (월드 좌표 기준)
                x_diff = abs(world_x)  # 화면 중심에서의 X 거리 (미터)
                
                # 양방향 화살표 그리기 (화면 하단에)
                arrow_y = image_height - 50
                arrow_start = screen_center[0]
                arrow_end = object_center_x
                
                # 화살표 선
                cv2.line(image, (arrow_start, arrow_y), (arrow_end, arrow_y), (0, 255, 0), 3)
                
                # 화살표 머리 그리기
                arrow_size = 10
                if arrow_end > arrow_start:  # 오른쪽 화살표
                    cv2.arrowedLine(image, (arrow_end - 20, arrow_y), (arrow_end, arrow_y), (0, 255, 0), 3, tipLength=0.3)
                    cv2.arrowedLine(image, (arrow_start + 20, arrow_y), (arrow_start, arrow_y), (0, 255, 0), 3, tipLength=0.3)
                else:  # 왼쪽 화살표
                    cv2.arrowedLine(image, (arrow_end + 20, arrow_y), (arrow_end, arrow_y), (0, 255, 0), 3, tipLength=0.3)
                    cv2.arrowedLine(image, (arrow_start - 20, arrow_y), (arrow_start, arrow_y), (0, 255, 0), 3, tipLength=0.3)
                
                # X 차이 텍스트 표시 (화살표 위에)
                x_diff_text = f"{x_diff:.2f}m"
                text_x = (arrow_start + arrow_end) // 2
                text_size = cv2.getTextSize(x_diff_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                
                # 배경 사각형
                cv2.rectangle(image, (text_x - text_size[0]//2 - 5, arrow_y - 35), 
                             (text_x + text_size[0]//2 + 5, arrow_y - 10), (0, 0, 0), -1)
                
                # 텍스트
                cv2.putText(image, x_diff_text, (text_x - text_size[0]//2, arrow_y - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 🎯 기억된 방향등 위치에 라벨 표시
        if (self.remembered_direction_positions['upper'] and 
            self.remembered_direction_positions['lower']):
            
            # 위쪽 방향등 위치에 "UP" 라벨 표시
            upper_pos = self.remembered_direction_positions['upper']['center']
            cv2.circle(image, (upper_pos[0], upper_pos[1]), 30, (0, 255, 0), 2)
            cv2.putText(image, "UP", (upper_pos[0] - 15, upper_pos[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 아래쪽 방향등 위치에 "DOWN" 라벨 표시
            lower_pos = self.remembered_direction_positions['lower']['center']
            cv2.circle(image, (lower_pos[0], lower_pos[1]), 30, (0, 0, 255), 2)
            cv2.putText(image, "DOWN", (lower_pos[0] - 25, lower_pos[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return image

    def _add_info_text(self, image: np.ndarray, objects: List[dict], custom_title: str = None):
        """다중 모델 탐지 결과 및 시스템 정보를 영상에 표시"""
        import cv2
        
        # 현재 모드 정보
        mode_name = self.get_active_mode_name()
        
        # 상단에 제목 (custom_title이 있으면 사용)
        if custom_title:
            cv2.putText(image, f"Roomie VS - {custom_title}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(image, f"Roomie Vision System v3 - {mode_name}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 현재 모드 ID 추출 (custom_title에서 카메라 타입 유추)
        current_mode_id = self.get_active_mode_id()
        
        # custom_title에서 카메라 타입 판단
        is_webcam = "Webcam" in (custom_title or "")
        is_depth = "Depth" in (custom_title or "")
        is_rear = "Rear" in (custom_title or "")
        
        # custom_title에서 모드 추출
        if "Elevator Out" in (custom_title or ""):
            current_mode_id = 3
        elif "Elevator In" in (custom_title or ""):
            current_mode_id = 4
        elif "ArUco" in (custom_title or "") or "YOLO" in (custom_title or ""):
            current_mode_id = 5
        elif "Standby" in (custom_title or ""):
            current_mode_id = 6 if not is_rear else 0
        else:
            current_mode_id = self.get_active_mode_id()  # 폴백
        
        # 실제 모델 적용 여부 판단 (카메라 타입 + 모드 조합)
        model_applied = False
        if is_webcam and current_mode_id in [3, 4]:  # 엘리베이터 모드의 웹캠
            model_applied = True
        elif is_depth and current_mode_id == 5:  # 일반 모드의 뎁스
            model_applied = True
            
        # 현재 모델 상태 및 설정 표시
        if model_applied:  # 실제로 모델이 적용되는 경우
            model_info = self.model_detector.get_current_model_info()
            # 안전한 모델 이름 표시 (한글 문제 방지)
            raw_model_name = model_info['model_name']
            if raw_model_name == 'normal':
                current_model = "Normal"
            elif raw_model_name == 'elevator':
                current_model = "Elevator" 
            elif raw_model_name is None:
                current_model = "None"
            else:
                current_model = str(raw_model_name)
        else:  # 모델이 적용되지 않는 경우 (영상만 표시)
            current_model = "Off"
            
        flip_status = "ON" if self.flip_horizontal else "OFF"
        
        # 카메라 이름은 custom_title에서 추출하거나 기본값 사용
        camera_name_display = custom_title if custom_title else self.current_camera_name
        
        cv2.putText(image, f"Model:{current_model} | Camera:{camera_name_display} | Flip:{flip_status} | Conf:{self.confidence_threshold}", 
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # 탐지된 객체 수
        cv2.putText(image, f"Objects Detected: {len(objects)}", (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # 장애물 정보 추가 (위치 조정하여 겹침 방지)
        obstacle_objects = [obj for obj in objects if obj.get('is_obstacle', False)]
        if obstacle_objects:
            dynamic_count = len([obj for obj in obstacle_objects if obj.get('obstacle_type') == 'dynamic'])
            static_count = len([obj for obj in obstacle_objects if obj.get('obstacle_type') == 'static'])
            
            # 장애물 요약 정보 (더 아래로 이동)
            cv2.putText(image, f"OBSTACLES: Dynamic={dynamic_count} | Static={static_count}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            
            # 가장 가까운 장애물 정보
            closest_obstacle = min(obstacle_objects, key=lambda x: x.get('distance_m', float('inf')))
            if closest_obstacle:
                obstacle_type = closest_obstacle.get('obstacle_type', 'unknown')
                distance = closest_obstacle.get('distance_m', 0.0)
                world_x = closest_obstacle.get('world_x', 0.0)
                world_y = closest_obstacle.get('world_y', 0.0)
                
                cv2.putText(image, f"CLOSEST: {obstacle_type.upper()} at {distance:.1f}m ({world_x:.2f}, {world_y:.2f})", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        else:
            cv2.putText(image, "NO OBSTACLES DETECTED", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # 🔥 방향등 기억된 위치 정보 표시
        if (self.remembered_direction_positions['upper'] and self.remembered_direction_positions['lower']):
            upper_pos = self.remembered_direction_positions['upper']['center']
            lower_pos = self.remembered_direction_positions['lower']['center']
            cv2.putText(image, f"Remembered Positions - UP: ({upper_pos[0]},{upper_pos[1]}) | DOWN: ({lower_pos[0]},{lower_pos[1]}) - TRACKING MODE", 
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        else:
            cv2.putText(image, "No Direction Light Positions Remembered - Waiting for 2 Lights...", 
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
                # 🔥 기억된 위치에서 현재 밝기 정보 표시
        if (self.remembered_direction_positions['upper'] and 
            self.remembered_direction_positions['lower'] and 
            image is not None):
            
            # 기억된 위치에서 실시간 밝기 측정
            upper_brightness = self._get_brightness_at_remembered_position(image, 'upper')
            lower_brightness = self._get_brightness_at_remembered_position(image, 'lower')
            
            cv2.putText(image, f"Current Brightness: UP={upper_brightness:.1f} | DOWN={lower_brightness:.1f}", 
                       (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # 🔥 깜빡임 상태 표시
            if self.blink_detection_enabled and len(self.brightness_history['upper']) >= 5:
                upper_blink = self._detect_blink_at_position('upper')
                lower_blink = self._detect_blink_at_position('lower')
                
                blink_status = ""
                if upper_blink and lower_blink:
                    blink_status = "BOTH BLINKING"
                    color = (0, 255, 255)  # 노란색
                elif upper_blink:
                    blink_status = "UP BLINKING"
                    color = (0, 255, 0)  # 초록색
                elif lower_blink:
                    blink_status = "DOWN BLINKING" 
                    color = (0, 0, 255)  # 빨간색
                else:
                    blink_status = "NO BLINK"
                    color = (128, 128, 128)  # 회색
                    
                cv2.putText(image, f"Blink Status: {blink_status}", 
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            else:
                cv2.putText(image, "Blink Detection: Collecting History...", 
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # 엘리베이터 방향 정보 표시 (내부 모드에서는 제외)
        if self.current_front_mode_id != 4:  # 엘리베이터 내부가 아닌 경우에만
            direction_text = "UP" if self.last_elevator_direction == 0 else "DOWN"
            cv2.putText(image, f"Elevator Direction: {direction_text}", (10, 170), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # 탐지된 객체 분류 표시
        if objects:
            object_counts = {}
            model_counts = {}
            
            for obj in objects:
                class_name = obj.get('class_name', 'unknown') 
                model_name = obj.get('model_name', 'unknown')
                
                object_counts[class_name] = object_counts.get(class_name, 0) + 1
                model_counts[model_name] = model_counts.get(model_name, 0) + 1
            
            if object_counts:
                counts_text = ", ".join([f"{k}:{v}" for k, v in object_counts.items()])
                cv2.putText(image, f"Objects: {counts_text}", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 255, 128), 1)
        
        # 눌린 버튼 표시
        pressed_buttons = []
        for obj in objects:
            if obj.get('is_pressed', False) and obj.get('class_name') == 'button':
                pressed_buttons.append("BUTTON")
        
        if pressed_buttons:
            pressed_text = f"Pressed: {len(pressed_buttons)} button(s)"
            cv2.putText(image, pressed_text, (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        # ArUco 마커 시각화 추가 (모드 5에서만)
        if self.current_front_mode_id == 5:
            self._add_aruco_visualization(image)
        
        # 현재 위치 정보 표시 (모드 5에서만)
        if self.current_front_mode_id == 5:
            location_names = {
                0: "LOB_WAITING", 1: "LOB_CALL", 2: "RES_PICKUP", 3: "RES_CALL",
                4: "SUP_PICKUP", 5: "ELE_1", 6: "ELE_2", 101: "ROOM_101",
                102: "ROOM_102", 201: "ROOM_201", 202: "ROOM_202"
            }
            current_location_name = location_names.get(self.last_detected_location_id, f"ID_{self.last_detected_location_id}")
            cv2.putText(image, f"Current Location: {current_location_name}", (10, 130), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 128, 0), 1)
        
        # 🏢 현재 엘리베이터 층수 표시 (엘리베이터 모드에서만) - 우하단 표시
        if current_mode_id in [3, 4]:  # 엘리베이터 외부/내부 모드
            current_floor = self.display_ocr.get_current_floor_display()
            
            # 화면 오른쪽 하단에 크게 표시
            text_size = cv2.getTextSize(current_floor, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 2)[0]
            text_x = image.shape[1] - text_size[0] - 15  # 오른쪽 정렬
            text_y = image.shape[0] - 90  # 하단에서 90픽셀 위 (방향 표시 공간 확보)
            
            # 큰 배경 박스
            cv2.rectangle(image, (text_x-8, text_y-25), (text_x+text_size[0]+8, text_y+8), (0, 0, 0), -1)
            cv2.rectangle(image, (text_x-8, text_y-25), (text_x+text_size[0]+8, text_y+8), (0, 255, 255), 2)
            
            # 현재 층수 텍스트 (크게)
            cv2.putText(image, current_floor, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
            
            # 🚦 엘리베이터 방향 표시 (층수 아래에 크게) - 내부 모드에서는 제외
            if self.current_front_mode_id != 4:  # 엘리베이터 내부가 아닌 경우에만
                direction_text = "UP" if self.last_elevator_direction == 0 else "DOWN"
                direction_color = (0, 255, 0) if self.last_elevator_direction == 0 else (0, 0, 255)  # UP: 초록, DOWN: 빨강
                
                # 🔥 깜빡임 감지 표시 추가
                if self.last_blink_detected:
                    direction_text += " ✦"  # 깜빡임 감지 시 별표 추가
                    direction_color = (0, 255, 255)  # 노란색으로 변경
                
                # 방향 텍스트 크기 계산
                dir_text_size = cv2.getTextSize(direction_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
                dir_text_x = image.shape[1] - dir_text_size[0] - 15  # 오른쪽 정렬
                dir_text_y = text_y + 45  # 층수 아래
                
                # 방향 배경 박스 (깜빡임 감지 시 더 두껍게)
                box_thickness = 4 if self.last_blink_detected else 2
                cv2.rectangle(image, (dir_text_x-8, dir_text_y-20), (dir_text_x+dir_text_size[0]+8, dir_text_y+8), (0, 0, 0), -1)
                cv2.rectangle(image, (dir_text_x-8, dir_text_y-20), (dir_text_x+dir_text_size[0]+8, dir_text_y+8), direction_color, box_thickness)
                
                # 방향 텍스트
                cv2.putText(image, direction_text, (dir_text_x, dir_text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, direction_color, 2)
                
                # 마지막 감지 시간 표시 (작게)
                if self.last_direction_detection_time:
                    time_diff = (self.get_clock().now() - self.last_direction_detection_time).nanoseconds / 1e9
                    time_text = f"({time_diff:.1f}초 전)"
                    cv2.putText(image, time_text, (dir_text_x, dir_text_y + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # 종료 안내
        cv2.putText(image, "ESC:Exit, B:Info, M:Status, F:Flip, C:Conf, A:ArUco, D:Reset, L:Remember (Blink Detection ON)", (10, image.shape[0]-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)

    def _on_gpu_memory_exceeded(self, used_memory: int, limit_memory: int, violation_count: int):
        """🚨 GPU 메모리 제한 초과 시 호출되는 콜백"""
        self.get_logger().error(f"🚨 GPU 메모리 한계 초과: {used_memory}MB > {limit_memory}MB (위반 #{violation_count})")
        
        # 🎯 단계별 대응 방법
        if violation_count == 1:
            # 1차: 경고만
            self.get_logger().warning("⚠️ 1차 경고: GPU 메모리 사용량을 줄여주세요")
            
        elif violation_count == 2:
            # 2차: EasyOCR CPU 모드로 전환
            self.get_logger().warning("⚠️ 2차 대응: EasyOCR을 CPU 모드로 전환합니다")
            try:
                self.display_ocr.switch_to_cpu_mode()
                self.get_logger().info("✅ EasyOCR CPU 모드 전환 완료")
            except Exception as e:
                self.get_logger().error(f"❌ CPU 모드 전환 실패: {e}")
                
        elif violation_count == 3:
            # 3차: OCR 기능 완전 비활성화
            self.get_logger().warning("⚠️ 3차 대응: OCR 기능을 비활성화합니다")
            try:
                self.display_ocr.disable_ocr()
                self.get_logger().info("✅ OCR 기능 비활성화 완료")
            except Exception as e:
                self.get_logger().error(f"❌ OCR 비활성화 실패: {e}")
                
        elif violation_count >= 5:
            # 최종: vs_node 강제 종료
            self.get_logger().critical("🚨 최종 대응: GPU 메모리 한계 초과로 vs_node를 종료합니다!")
            self.get_logger().critical(f"🚨 종료 사유: {violation_count}회 연속 GPU 메모리 제한 초과")
            
            try:
                # GPU 모니터링 중지
                if hasattr(self, 'gpu_monitor') and self.gpu_monitor:
                    self.gpu_monitor.stop_monitoring()
                
                # 카메라 정리
                if hasattr(self, 'camera_manager'):
                    self.camera_manager.cleanup_all_cameras()
                
                # ROS2 노드 종료
                self.destroy_node()
                
                # 프로세스 강제 종료
                import os
                import signal
                os.kill(os.getpid(), signal.SIGTERM)
                
            except Exception as e:
                self.get_logger().error(f"❌ 종료 처리 오류: {e}")
                import sys
                sys.exit(1)
    
    def _on_gpu_error(self, error: Exception):
        """🚨 GPU 오류 시 호출되는 콜백"""
        self.get_logger().error(f"🚨 GPU 오류 발생: {error}")
        
        # GPU 오류 시 자동으로 CPU 모드로 전환
        try:
            self.get_logger().warning("⚠️ GPU 오류로 인해 EasyOCR을 CPU 모드로 전환합니다")
            self.display_ocr.switch_to_cpu_mode()
            self.get_logger().info("✅ GPU 오류 대응: CPU 모드 전환 완료")
        except Exception as e:
            self.get_logger().error(f"❌ GPU 오류 대응 실패: {e}")

    # 👤 추적 관련 액션 및 서비스 콜백 메소드들
    def enroll_action_callback(self, goal_handle):
        """등록 액션 서버 콜백 - PersonTracker를 통해 처리"""
        self.get_logger().info(f"👤 등록 액션 요청: duration={goal_handle.request.duration_sec}초")
        
        try:
            # PersonTracker가 없거나 등록모드가 아니면 실패
            if not hasattr(self, 'person_tracker') or not self.person_tracker:
                goal_handle.abort()
                result = Enroll.Result()
                result.success = False
                self.get_logger().error("👤 PersonTracker가 초기화되지 않았습니다")
                return result
            
            if self.person_tracker.current_mode != 1:
                goal_handle.abort()
                result = Enroll.Result()
                result.success = False
                self.get_logger().error("👤 등록모드가 아닙니다 (현재 모드: {})".format(self.person_tracker.current_mode))
                return result
            
            # 등록 시작
            register_result = self.person_tracker.register_target(goal_handle.request.duration_sec)
            if not register_result["success"]:
                goal_handle.abort()
                result = Enroll.Result()
                result.success = False
                self.get_logger().error(f"👤 등록 시작 실패: {register_result['message']}")
                return result
            
            goal_handle.succeed()
            
            # 주기적으로 피드백 전송
            duration = goal_handle.request.duration_sec
            start_time = time.time()
            
            while time.time() - start_time < duration:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = Enroll.Result()
                    result.success = False
                    self.get_logger().info("👤 등록 액션이 취소되었습니다")
                    return result
                
                # 진행률 피드백
                progress = self.person_tracker.get_registration_progress()
                feedback = Enroll.Feedback()
                feedback.progress = progress
                goal_handle.publish_feedback(feedback)
                
                time.sleep(0.1)  # 10Hz 피드백
            
            # 등록 완료
            result = Enroll.Result()
            result.success = self.person_tracker.target_registered
            
            if result.success:
                self.get_logger().info(f"👤 등록 완료: target_id={self.person_tracker.target_id}")
            else:
                self.get_logger().warning("👤 등록 실패: 적합한 후보를 찾지 못했습니다")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"👤 등록 액션 처리 중 오류: {e}")
            goal_handle.abort()
            result = Enroll.Result()
            result.success = False
            return result
    
    def stop_tracking_callback(self, request, response):
        """추적 중지 서비스 콜백"""
        self.get_logger().info("👤 추적 중지 요청")
        
        try:
            if hasattr(self, 'person_tracker') and self.person_tracker:
                stop_result = self.person_tracker.stop_tracking()
                response.success = stop_result["success"]
                response.message = stop_result["message"]
                self.get_logger().info(f"👤 추적 중지 완료: {response.message}")
            else:
                response.success = False
                response.message = "PersonTracker가 초기화되지 않았습니다"
                self.get_logger().error("👤 PersonTracker가 없어 추적 중지 실패")
        
        except Exception as e:
            response.success = False
            response.message = f"추적 중지 중 오류: {e}"
            self.get_logger().error(f"👤 추적 중지 처리 중 오류: {e}")
        
        return response

    def __del__(self):
        """소멸자 - 멀티 카메라 시스템 정리"""
        # GPU 모니터링 정리
        if hasattr(self, 'gpu_monitor') and self.gpu_monitor:
            self.gpu_monitor.stop_monitoring()
        
        # 카메라 시스템 정리
        if hasattr(self, 'camera_manager'):
            self.camera_manager.cleanup_all_cameras()

    def _update_remembered_positions(self, direction_objects: List[dict]) -> bool:
        """방향등 2개가 감지되면 위치를 기억해둠 (간헐적 감지 대비)"""
        try:
            if len(direction_objects) != 2:
                return False
            
            # Y 좌표 기준으로 정렬 (위쪽이 먼저)
            sorted_lights = sorted(direction_objects, key=lambda obj: obj['center'][1])
            upper_light = sorted_lights[0]  # Y 좌표가 작은 것 (위쪽)
            lower_light = sorted_lights[1]  # Y 좌표가 큰 것 (아래쪽)
            
            # 위치 정보 저장
            self.remembered_direction_positions['upper'] = {
                'center': upper_light['center'],
                'bbox': upper_light['bbox']
            }
            self.remembered_direction_positions['lower'] = {
                'center': lower_light['center'],
                'bbox': lower_light['bbox']
            }
            
            self.last_position_update = self.get_clock().now()
            
            self.get_logger().info(f"🎯 방향등 위치 기억: 위쪽=({upper_light['center'][0]},{upper_light['center'][1]}), 아래쪽=({lower_light['center'][0]},{lower_light['center'][1]})")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"방향등 위치 기억 에러: {e}")
            return False

    def _get_brightness_at_remembered_position(self, image: np.ndarray, position_type: str) -> float:
        """기억된 위치에서 밝기 측정"""
        try:
            if not self.remembered_direction_positions[position_type] or image is None:
                return 0.0
                
            pos_info = self.remembered_direction_positions[position_type]
            
            # 가상의 light_obj 생성해서 기존 함수 활용
            virtual_light_obj = {
                'center': pos_info['center'],
                'bbox': pos_info['bbox']
            }
            
            # 기존 함수 활용
            return self._get_light_brightness_advanced(image, virtual_light_obj)
            
        except Exception as e:
            self.get_logger().error(f"기억된 위치 밝기 측정 에러: {e}")
            return 0.0

    def _update_brightness_history(self, image: np.ndarray):
        """기억된 위치에서 밝기 히스토리 업데이트"""
        try:
            if not (self.remembered_direction_positions['upper'] and 
                   self.remembered_direction_positions['lower'] and 
                   image is not None):
                return
                
            # 각 위치에서 현재 밝기 측정
            upper_brightness = self._get_brightness_at_remembered_position(image, 'upper')
            lower_brightness = self._get_brightness_at_remembered_position(image, 'lower')
            
            # 히스토리에 추가
            self.brightness_history['upper'].append(upper_brightness)
            self.brightness_history['lower'].append(lower_brightness)
            
            # 히스토리 크기 제한
            if len(self.brightness_history['upper']) > self.history_size:
                self.brightness_history['upper'].pop(0)
            if len(self.brightness_history['lower']) > self.history_size:
                self.brightness_history['lower'].pop(0)
                
        except Exception as e:
            self.get_logger().error(f"밝기 히스토리 업데이트 에러: {e}")

    def _detect_blink_at_position(self, position_type: str) -> bool:
        """특정 위치에서 깜빡임 감지"""
        try:
            history = self.brightness_history[position_type]
            
            # 최소 5프레임의 히스토리가 필요
            if len(history) < 5:
                return False
                
            # 최근 3프레임과 그 이전 프레임들 비교
            recent_frames = history[-3:]  # 최근 3프레임
            previous_frames = history[-8:-3]  # 그 이전 5프레임
            
            if len(previous_frames) == 0:
                return False
                
            # 평균 밝기 계산
            recent_avg = sum(recent_frames) / len(recent_frames)
            previous_avg = sum(previous_frames) / len(previous_frames)
            
            # 밝기 변화량 계산
            brightness_change = recent_avg - previous_avg
            
            # 깜빡임 감지: 임계값 이상 밝아짐
            is_blink = brightness_change > self.brightness_change_threshold_for_blink
            
            if is_blink:
                self.get_logger().info(f"🔥 {position_type.upper()} 방향등 깜빡임 감지! 변화량: {brightness_change:.1f}")
                # 🎯 깜빡임 감지 후 히스토리 일부 초기화 (연속 깜빡임 감지를 위해)
                self._reset_brightness_history_for_continuous_detection(position_type)
                
            return is_blink
            
        except Exception as e:
            self.get_logger().error(f"{position_type} 깜빡임 감지 에러: {e}")
            return False

    def _reset_brightness_history_for_continuous_detection(self, position_type: str):
        """깜빡임 감지 후 히스토리 일부 초기화 (연속 감지를 위해)"""
        try:
            if position_type in self.brightness_history:
                # 현재 히스토리의 마지막 3개 값만 유지 (나머지 제거)
                if len(self.brightness_history[position_type]) > 3:
                    self.brightness_history[position_type] = self.brightness_history[position_type][-3:]
                    self.get_logger().info(f"🔄 {position_type.upper()} 방향등 히스토리 초기화 (연속 깜빡임 감지용)")
        except Exception as e:
            self.get_logger().error(f"히스토리 초기화 에러: {e}")

    def _classify_light_by_learned_position(self, light_obj: dict) -> str:
        """학습된 위치 정보를 기반으로 방향등을 위/아래로 분류"""
        try:
            if (not self.direction_light_positions['upper'] or 
                not self.direction_light_positions['lower']):
                return 'unknown'
            
            light_y = light_obj['center'][1]
            upper_y = self.direction_light_positions['upper']['center'][1]
            lower_y = self.direction_light_positions['lower']['center'][1]
            
            # 학습된 위치와의 거리 계산
            dist_to_upper = abs(light_y - upper_y)
            dist_to_lower = abs(light_y - lower_y)
            
            # 더 가까운 위치로 분류 (허용 오차 50픽셀)
            if dist_to_upper < dist_to_lower and dist_to_upper < 50:
                return 'upper'
            elif dist_to_lower < dist_to_upper and dist_to_lower < 50:
                return 'lower'
            else:
                return 'unknown'
                
        except Exception as e:
            self.get_logger().error(f"방향등 위치 분류 에러: {e}")
            return 'unknown'

    def _detect_with_learned_positions(self, image: np.ndarray, current_lights: List[dict]) -> int:
        """학습된 위치 정보를 활용한 방향 감지"""
        try:
            # 위치별로 분류
            upper_lights = [light for light in current_lights if light.get('position_type') == 'upper']
            lower_lights = [light for light in current_lights if light.get('position_type') == 'lower']
            
            # 각 영역의 평균 밝기 계산
            upper_avg_brightness = sum([light['brightness'] for light in upper_lights]) / len(upper_lights) if upper_lights else 0
            lower_avg_brightness = sum([light['brightness'] for light in lower_lights]) / len(lower_lights) if lower_lights else 0
            
            self.get_logger().info(f"🔍 학습된 위치 기반 - 위쪽 평균 밝기: {upper_avg_brightness:.1f}, 아래쪽 평균 밝기: {lower_avg_brightness:.1f}")
            
            # 밝기 차이로 방향 판단
            brightness_diff = upper_avg_brightness - lower_avg_brightness
            threshold = 40.0  # 밝기 차이 임계값 (안정성 우선)
            
            self.get_logger().info(f"💡 밝기 차이: {brightness_diff:.1f} (임계값: ±{threshold})")
            
            if brightness_diff > threshold:
                self.get_logger().debug("🔥 위쪽 방향등이 더 밝음 → 상행")
                return 0  # 상행
            elif brightness_diff < -threshold:
                self.get_logger().debug("🔥 아래쪽 방향등이 더 밝음 → 하행") 
                return 1  # 하행
            else:
                # 차이가 미미하면 기존 방향 유지
                self.get_logger().debug(f"📊 밝기 차이가 임계값 이하 → 기존 방향 유지")
                return self.last_elevator_direction
                
        except Exception as e:
            self.get_logger().error(f"학습된 위치 기반 감지 에러: {e}")
            return self.last_elevator_direction

    def _detect_direction_by_lights(self, image: np.ndarray, direction_objects: List[dict]) -> int:
        """🔥 깜빡임 감지 기반 방향 판단 (위치 기억 + 밝기 변화 추적)"""
        try:
            current_count = len(direction_objects)
            
            # 1단계: 방향등 2개가 감지되면 위치 업데이트
            if current_count == 2:
                self._update_remembered_positions(direction_objects)
            
            # 2단계: 기억된 위치가 있으면 밝기 히스토리 업데이트
            if (self.remembered_direction_positions['upper'] and 
                self.remembered_direction_positions['lower'] and 
                image is not None):
                
                # 밝기 히스토리 업데이트
                self._update_brightness_history(image)
                
                # 3단계: 깜빡임 감지 우선 (방향 변화 감지)
                if self.blink_detection_enabled:
                    upper_blink = self._detect_blink_at_position('upper')
                    lower_blink = self._detect_blink_at_position('lower')
                    
                    if upper_blink and not lower_blink:
                        self.get_logger().info("🔥🔥 위쪽 방향등 깜빡임 감지 → 상행!")
                        self.last_blink_detected = True
                        return 0  # 상행
                    elif lower_blink and not upper_blink:
                        self.get_logger().info("🔥🔥 아래쪽 방향등 깜빡임 감지 → 하행!")
                        self.last_blink_detected = True
                        return 1  # 하행
                    elif upper_blink and lower_blink:
                        self.get_logger().info("⚠️ 양쪽 모두 깜빡임 감지됨, 밝기 차이로 판단")
                        self.last_blink_detected = True
                        # 양쪽 모두 깜빡이면 밝기 차이로 판단
                    else:
                        # 깜빡임이 없으면 밝기 차이로 판단
                        self.last_blink_detected = False
                
                # 4단계: 깜빡임이 없거나 양쪽 모두 깜빡이면 밝기 차이로 판단
                upper_brightness = self._get_brightness_at_remembered_position(image, 'upper')
                lower_brightness = self._get_brightness_at_remembered_position(image, 'lower')
                
                brightness_diff = upper_brightness - lower_brightness
                threshold = 40.0
                
                self.get_logger().debug(f"💡 기억된 위치 밝기: 위쪽={upper_brightness:.1f}, 아래쪽={lower_brightness:.1f}, 차이={brightness_diff:.1f}")
                
                if brightness_diff > threshold:
                    self.get_logger().debug("🔥 위쪽 방향등이 더 밝음 → 상행")
                    return 0  # 상행
                elif brightness_diff < -threshold:
                    self.get_logger().debug("🔥 아래쪽 방향등이 더 밝음 → 하행")
                    return 1  # 하행
                else:
                    self.get_logger().debug("📊 밝기 차이가 임계값 이하 → 기존 방향 유지")
                    return self.last_elevator_direction
            else:
                # 기억된 위치가 없으면 기존 방향 유지
                if current_count == 0:
                    self.get_logger().debug("방향등 미감지, 기억된 위치 없음 → 기존 방향 유지")
                else:
                    self.get_logger().info(f"방향등 {current_count}개 감지됨 (2개 필요), 기억된 위치 없음")
                return self.last_elevator_direction
            
        except Exception as e:
            self.get_logger().error(f"Direction light 감지 에러: {e}")
            return self.last_elevator_direction

    def _analyze_light_color(self, image: np.ndarray, light_obj: dict) -> str:
        """방향등 영역의 색상 분석"""
        try:
            bbox = light_obj.get('bbox')
            if not bbox:
                return 'UNKNOWN'
            
            x1, y1, x2, y2 = bbox
            
            # 방향등 영역 크롭
            light_region = image[y1:y2, x1:x2]
            
            if light_region.size == 0:
                return 'UNKNOWN'
            
            # BGR → HSV 변환
            hsv = cv2.cvtColor(light_region, cv2.COLOR_BGR2HSV)
            
            # 녹색 범위 검출
            green_mask = cv2.inRange(hsv, 
                                   np.array([40, 50, 50]),    # 녹색 하한
                                   np.array([80, 255, 255]))  # 녹색 상한
            
            # 빨간색 범위 검출
            red_mask1 = cv2.inRange(hsv,
                                   np.array([0, 50, 50]),     # 빨간색 하한1
                                   np.array([10, 255, 255]))  # 빨간색 상한1
            
            red_mask2 = cv2.inRange(hsv,
                                   np.array([170, 50, 50]),   # 빨간색 하한2  
                                   np.array([180, 255, 255])) # 빨간색 상한2
            
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            
            # 색상별 픽셀 수 계산
            green_pixels = cv2.countNonZero(green_mask)
            red_pixels = cv2.countNonZero(red_mask)
            total_pixels = light_region.shape[0] * light_region.shape[1]
            
            # 비율 계산
            green_ratio = green_pixels / total_pixels
            red_ratio = red_pixels / total_pixels
            
            # 임계값 (전체 영역의 10% 이상이면 해당 색상으로 판단)
            threshold = 0.1
            
            if green_ratio > threshold and green_ratio > red_ratio:
                return 'GREEN'
            elif red_ratio > threshold and red_ratio > green_ratio:
                return 'RED'
            else:
                return 'UNKNOWN'
                
        except Exception as e:
            self.get_logger().error(f"방향등 색상 분석 에러: {e}")
            return 'UNKNOWN'

    def _fallback_direction_by_brightness(self, image: np.ndarray, upper_light: dict, lower_light: dict) -> int:
        """색상이 불분명할 때 밝기로 방향 판단"""
        try:
            upper_brightness = self._get_light_brightness(image, upper_light)
            lower_brightness = self._get_light_brightness(image, lower_light)
            
            self.get_logger().info(f"방향등 밝기: 위쪽={upper_brightness:.2f}, 아래쪽={lower_brightness:.2f}")
            
            # 더 밝은 쪽이 켜진 것으로 가정
            if upper_brightness > lower_brightness * 1.2:  # 20% 이상 차이
                return 0  # 상행 (위쪽이 밝음)
            elif lower_brightness > upper_brightness * 1.2:
                return 1  # 하행 (아래쪽이 밝음)
            else:
                return 0  # 기본 상행
                
        except Exception as e:
            self.get_logger().error(f"밝기 기반 방향 판단 에러: {e}")
            return 0

    def _get_light_brightness(self, image: np.ndarray, light_obj: dict) -> float:
        """방향등의 평균 밝기 계산"""
        try:
            bbox = light_obj.get('bbox')
            if not bbox:
                return 0.0
            
            x1, y1, x2, y2 = bbox
            light_region = image[y1:y2, x1:x2]
            
            if light_region.size == 0:
                return 0.0
            
            # 그레이스케일 변환 후 평균 밝기
            gray = cv2.cvtColor(light_region, cv2.COLOR_BGR2GRAY)
            return np.mean(gray)
            
        except Exception as e:
            return 0.0

    def _detect_by_disappearance(self, image: np.ndarray, current_lights: List[dict]) -> int:
        """소실 감지: 사라진 방향등의 위치로 방향 판단"""
        try:
            if not self.previous_direction_lights:
                return -1
            
            # 이전 프레임과 비교하여 사라진 객체 찾기
            prev_positions = [(obj['center'][1], obj) for obj in self.previous_direction_lights]
            curr_positions = [obj['center'][1] for obj in current_lights]
            
            prev_positions.sort(key=lambda x: x[0])  # Y 좌표로 정렬
            
            disappeared_lights = []
            for y_pos, prev_obj in prev_positions:
                # 현재 프레임에서 비슷한 위치의 객체가 있는지 확인
                found = False
                for curr_y in curr_positions:
                    if abs(y_pos - curr_y) < 50:  # 50픽셀 이내면 같은 객체로 간주
                        found = True
                        break
                
                if not found:
                    disappeared_lights.append(prev_obj)
            
            if disappeared_lights:
                # 사라진 방향등이 위쪽인지 아래쪽인지 판단
                disappeared_y = [obj['center'][1] for obj in disappeared_lights]
                avg_disappeared_y = sum(disappeared_y) / len(disappeared_y)
                
                # 전체 이미지 중앙과 비교
                image_center_y = image.shape[0] // 2
                
                if avg_disappeared_y < image_center_y:
                    self.get_logger().info("🔥 위쪽 방향등 소실 감지 → 상행 (위쪽이 켜짐)")
                    return 0  # 상행
                else:
                    self.get_logger().info("🔥 아래쪽 방향등 소실 감지 → 하행 (아래쪽이 켜짐)")
                    return 1  # 하행
            
            return -1
            
        except Exception as e:
            self.get_logger().error(f"소실 감지 에러: {e}")
            return -1

    def _match_lights_by_position(self, prev_lights: List[dict], curr_lights: List[dict]) -> List[tuple]:
        """🔍 이전 프레임과 현재 프레임의 방향등을 위치 기반으로 매칭"""
        try:
            matched_pairs = []
            
            for prev_light in prev_lights:
                prev_center = prev_light['center']
                best_match = None
                best_distance = float('inf')
                
                for curr_light in curr_lights:
                    curr_center = curr_light['center']
                    
                    # 유클리드 거리 계산
                    distance = ((prev_center[0] - curr_center[0]) ** 2 + 
                               (prev_center[1] - curr_center[1]) ** 2) ** 0.5
                    
                    if distance < self.position_tolerance and distance < best_distance:
                        best_distance = distance
                        best_match = curr_light
                
                if best_match:
                    matched_pairs.append((prev_light, best_match))
            
            self.get_logger().info(f"🔗 매칭된 방향등: {len(matched_pairs)}쌍")
            return matched_pairs
            
        except Exception as e:
            self.get_logger().error(f"방향등 매칭 에러: {e}")
            return []

    def _detect_by_absolute_brightness(self, current_lights: List[dict]) -> int:
        """절대 밝기 기반 감지: 너무 밝은 것 = 켜진 것"""
        try:
            if len(current_lights) < 2:
                return -1
            
            # 위쪽/아래쪽 분류 (학습된 위치 정보가 있을 때만)
            upper_lights = []
            lower_lights = []
            has_position_info = False
            
            for light in current_lights:
                if light.get('position_type') == 'upper':
                    upper_lights.append(light)
                    has_position_info = True
                elif light.get('position_type') == 'lower':
                    lower_lights.append(light)
                    has_position_info = True
                # 위치 정보가 없으면 분류하지 않음 (안전)
            
            # 위치 정보가 없으면 판단 보류
            if not has_position_info:
                self.get_logger().warn("⚠️ 학습된 위치 정보가 없어 절대 밝기 감지를 건너뜁니다")
                return -1
            
            # 각 영역에서 가장 밝은 방향등 찾기
            upper_max_brightness = max([light['brightness'] for light in upper_lights]) if upper_lights else 0
            lower_max_brightness = max([light['brightness'] for light in lower_lights]) if lower_lights else 0
            
            self.get_logger().info(f"💡 절대 밝기: 위쪽 최대={upper_max_brightness:.1f}, 아래쪽 최대={lower_max_brightness:.1f} (임계값: {self.brightness_threshold})")
            
            # 매우 밝은 영역은 켜진 것으로 판단
            upper_too_bright = upper_max_brightness > self.brightness_threshold
            lower_too_bright = lower_max_brightness > self.brightness_threshold
            
            if upper_too_bright and not lower_too_bright:
                self.get_logger().info("🔥 위쪽 방향등이 매우 밝음 → 상행")
                return 0  # 상행
            elif lower_too_bright and not upper_too_bright:
                self.get_logger().info("🔥 아래쪽 방향등이 매우 밝음 → 하행")
                return 1  # 하행
            
            return -1
            
        except Exception as e:
            self.get_logger().error(f"절대 밝기 감지 에러: {e}")
            return -1

    def _detect_by_count_pattern(self) -> int:
        """개수 패턴 분석: 갑작스런 감소는 켜진 것"""
        try:
            if len(self.direction_light_history) < 3:
                return -1
            
            recent_counts = self.direction_light_history[-3:]  # 최근 3프레임
            
            # 2개 → 1개 또는 2개 → 0개 패턴 감지
            if recent_counts[-2] >= 2 and recent_counts[-1] < recent_counts[-2]:
                # 갑작스럽게 감소한 경우
                self.get_logger().info(f"🔍 개수 패턴 분석: {recent_counts} → 방향등이 켜져서 감지 불가")
                
                # 히스토리를 기반으로 이전 방향 유지하되, 변화 가능성 고려
                return self.last_elevator_direction
            
            return -1
            
        except Exception as e:
            self.get_logger().error(f"패턴 분석 에러: {e}")
            return -1

    def _get_light_brightness_advanced(self, image: np.ndarray, light_obj: dict) -> float:
        """방향등 영역의 평균 밝기 계산 (개선된 버전)"""
        try:
            bbox = light_obj.get('bbox')
            if not bbox:
                return 0.0
            
            x1, y1, x2, y2 = bbox
            light_region = image[y1:y2, x1:x2]
            
            if light_region.size == 0:
                return 0.0
            
            # BGR → Grayscale 변환
            gray = cv2.cvtColor(light_region, cv2.COLOR_BGR2GRAY)
            
            # 상위 20% 픽셀의 평균 밝기 (가장 밝은 부분)
            flat_pixels = gray.flatten()
            flat_pixels.sort()
            top_20_percent = flat_pixels[int(len(flat_pixels) * 0.8):]
            
            return float(np.mean(top_20_percent))
            
        except Exception as e:
            self.get_logger().error(f"밝기 계산 에러: {e}")
            return 0.0

    def _get_lights_with_brightness(self, image: np.ndarray, direction_objects: List[dict]) -> List[dict]:
        """🔥 각 방향등의 위치와 밝기 정보 추출"""
        try:
            lights_with_brightness = []
            
            for obj in direction_objects:
                center = obj.get('center', [0, 0])
                brightness = self._get_light_brightness_advanced(image, obj)
                
                light_info = {
                    'center': center,
                    'brightness': brightness,
                    'bbox': obj.get('bbox'),
                    'original_obj': obj
                }
                lights_with_brightness.append(light_info)
            
            return lights_with_brightness
            
        except Exception as e:
            self.get_logger().error(f"방향등 밝기 정보 추출 에러: {e}")
            return []

    def _detect_by_position_brightness_change(self, current_lights: List[dict]) -> int:
        """🔥 개별 위치별 밝기 변화 감지 (핵심 로직)"""
        try:
            if not self.previous_direction_lights or not current_lights:
                return -1
            
            # 이전 프레임과 현재 프레임의 방향등 매칭
            matched_lights = self._match_lights_by_position(self.previous_direction_lights, current_lights)
            
            if not matched_lights:
                return -1
            
            # 각 매칭된 방향등의 밝기 변화 계산
            brightness_changes = []
            for prev_light, curr_light in matched_lights:
                prev_brightness = prev_light['brightness']
                curr_brightness = curr_light['brightness']
                change = curr_brightness - prev_brightness
                
                # 학습된 위치 정보가 있을 때만 판단
                position_type = curr_light.get('position_type')
                if position_type == 'upper':
                    is_upper = True
                elif position_type == 'lower':
                    is_upper = False
                else:
                    # 위치 정보가 없으면 이 방향등은 건너뜀
                    continue
                
                light_info = {
                    'center': curr_light['center'],
                    'prev_brightness': prev_brightness,
                    'curr_brightness': curr_brightness,
                    'change': change,
                    'is_upper': is_upper
                }
                brightness_changes.append(light_info)
                
                self.get_logger().info(f"🔄 위치({curr_light['center'][0]},{curr_light['center'][1]}): {prev_brightness:.1f} → {curr_brightness:.1f} (변화: {change:+.1f})")
            
            # 🔥 밝기 급증한 방향등 찾기 (켜진 것)
            significant_increases = [light for light in brightness_changes if light['change'] > self.brightness_change_threshold]
            
            if significant_increases:
                # 위쪽/아래쪽 분류
                upper_increases = [light for light in significant_increases if light['is_upper']]
                lower_increases = [light for light in significant_increases if not light['is_upper']]
                
                if upper_increases and not lower_increases:
                    self.get_logger().info(f"🔥 위쪽 방향등 {len(upper_increases)}개 밝기 급증 → 상행")
                    return 0  # 상행
                elif lower_increases and not upper_increases:
                    self.get_logger().info(f"🔥 아래쪽 방향등 {len(lower_increases)}개 밝기 급증 → 하행")  
                    return 1  # 하행
            
            # 변화가 미미한 경우
            return -1
            
        except Exception as e:
            self.get_logger().error(f"위치별 밝기 변화 감지 에러: {e}")
            return -1
    
    def _apply_button_recog_1(self, objects: List[dict]) -> List[dict]:
        """button_recog_1: 엘리베이터 외부 - 상하 위치 기반 분류"""
        button_objects = [obj for obj in objects if obj.get('class_name') == 'button']
        
        if len(button_objects) < 2:
            return objects  # 버튼이 2개 미만이면 원본 반환
            
        # 버튼들을 Y 좌표 기준으로 정렬 (위에서 아래로)
        button_objects.sort(key=lambda x: x['center'][1])
        
        updated_objects = []
        
        for obj in objects:
            if obj.get('class_name') == 'button':
                # 정렬된 버튼 리스트에서 현재 버튼의 인덱스 찾기
                center_y = obj['center'][1]
                button_index = None
                for i, btn in enumerate(button_objects):
                    if btn['center'][1] == center_y and btn['center'][0] == obj['center'][0]:
                        button_index = i
                        break
                
                # 상위 50% 인덱스는 UP, 하위 50% 인덱스는 DOWN
                if button_index is not None:
                    mid_index = len(button_objects) // 2
                    if button_index < mid_index:
                        obj['button_id'] = 101  # 상행버튼 (UP)
                        obj['floor_type'] = 'up'
                    else:
                        obj['button_id'] = 100  # 하행버튼 (DOWN)
                        obj['floor_type'] = 'down'
                else:
                    # 매칭 실패시 기본값
                    obj['button_id'] = 100  # 하행버튼
                    obj['floor_type'] = 'down'
                    
                obj['recognition_method'] = 'button_recog_1'
                
            updated_objects.append(obj)
            
        return updated_objects
    

    
    def _apply_enhanced_button_recognition(self, objects: List[dict], color_image: np.ndarray, mode_id: int = 0) -> List[dict]:
        """엘리베이터 내부: CNN 전용, 외부: 배열 우선 + CNN 폴백"""
        button_objects = [obj for obj in objects if obj.get('class_name') == 'button']
        
        if not button_objects:
            return objects
        
        # 엘리베이터 내부(mode_id=4)일 때 CNN 우선, 외부(mode_id=3)일 때 배열 우선
        if mode_id == 4:  # 엘리베이터 내부 - CNN 우선 (배열 폴백 제거)
            # CNN 모델 인식만 사용
            for obj in button_objects:
                if 'bbox' in obj and self.cnn_classifier.model is not None:
                    cnn_result = self.cnn_classifier.classify_button(color_image, obj['bbox'])
                    if cnn_result and cnn_result['confidence'] > 0.6:  # 신뢰도 임계값
                        # CNN 결과로 업데이트
                        original_class_name = obj.get('class_name')
                        obj.update(cnn_result)
                        obj['class_name'] = original_class_name
                        obj['recognition_method'] = 'cnn_primary'
                    else:
                        # CNN 실패 시 unmapped로 설정 (배열 폴백 없음)
                        obj['button_id'] = 'unmapped'
                        obj['recognition_method'] = 'cnn_failed'
                else:
                    # CNN 모델이 없거나 bbox가 없는 경우
                    obj['button_id'] = 'unmapped'
                    obj['recognition_method'] = 'cnn_unavailable'
            
            return objects
            
        else:  # 엘리베이터 외부(mode_id=3) 또는 기타 모드 - 기존 배열 우선 방식 유지
            
            # 1순위: 기존 배열 기반 인식
            processed_objects = objects
            
            if mode_id == 3:  # 엘리베이터 외부
                processed_objects = self._apply_button_recog_1(objects)
            
            # 2순위: 배열 인식 실패한 버튼들에 CNN 적용
            successful_buttons = []
            failed_buttons = []
            
            for obj in processed_objects:
                if obj.get('class_name') == 'button':
                    if (obj.get('button_id') not in ['unmapped', None] and 
                        obj.get('recognition_method') == 'button_recog_1'):
                        successful_buttons.append(obj)
                    else:
                        failed_buttons.append(obj)
            
            # CNN 폴백 적용
            cnn_success_count = 0
            if failed_buttons and self.cnn_classifier.model is not None:
                for obj in failed_buttons:
                    if 'bbox' in obj:
                        cnn_result = self.cnn_classifier.classify_button(color_image, obj['bbox'])
                        if cnn_result and cnn_result['confidence'] > 0.6:
                            original_class_name = obj.get('class_name')
                            obj.update(cnn_result)
                            obj['class_name'] = original_class_name
                            obj['recognition_method'] = 'cnn_fallback'
                            successful_buttons.append(obj)
                            cnn_success_count += 1
            
            return processed_objects


    def _start_rear_camera_streaming(self):
        """후방 카메라 UDP 스트리밍 시작"""
        try:
            if not hasattr(self, 'streaming_active'):
                self.streaming_active = False
                self.streaming_thread = None
            
            # 카메라 유무와 무관하게 스트리밍 스레드를 시작하고,
            # 루프 내에서 카메라가 준비되었을 때만 프레임을 전송한다.
            if not self.streaming_active:
                self.streaming_active = True
                self.streaming_thread = threading.Thread(target=self._rear_camera_streaming_loop, daemon=True)
                self._rear_first_send_logged = False
                self._rear_wait_log_emitted = False
                self.streaming_thread.start()
                self.get_logger().info("📹 후방 카메라 UDP 스트리밍 스레드 시작 (카메라 준비와 무관하게 시작)")
                
        except Exception as e:
            self.get_logger().error(f"UDP 스트리밍 시작 실패: {e}")
    
    def _stop_rear_camera_streaming(self):
        """후방 카메라 UDP 스트리밍 중지"""
        try:
            if hasattr(self, 'streaming_active') and self.streaming_active:
                self.streaming_active = False
                if hasattr(self, 'streaming_thread') and self.streaming_thread:
                    self.streaming_thread.join(timeout=1.0)
                self.get_logger().info("📹 후방 카메라 UDP 스트리밍 중지")
                
        except Exception as e:
            self.get_logger().error(f"UDP 스트리밍 중지 실패: {e}")
    
    def _rear_camera_streaming_loop(self):
        """후방 카메라 프레임을 주기적으로 UDP로 전송하는 루프"""
        try:
            while self.streaming_active and rclpy.ok():
                try:
                    if self.current_rear_camera is not None:
                        # 후방 카메라에서 프레임 가져오기 (WebCamCamera: (depth=None, color) 반환)
                        _, color_frame = self.current_rear_camera.get_frames()
                        
                        if color_frame is not None:
                            # 👤 PersonTracker에 프레임 전달
                            if hasattr(self, 'person_tracker') and self.person_tracker:
                                self.person_tracker.push_frame(color_frame)
                            
                            # PersonTracker 오버레이 적용 (선택적)
                            display_frame = color_frame
                            if hasattr(self, 'person_tracker') and self.person_tracker:
                                display_frame = self.person_tracker.get_overlay_frame(color_frame)
                            
                            # UDP로 프레임 전송 (BGR 형식)
                            sent = self.udp_streamer.send_frame_bgr(display_frame)
                            if sent and not getattr(self, '_rear_first_send_logged', False):
                                ip, prt = self.udp_streamer.addr
                                h, w = color_frame.shape[:2]
                                self.get_logger().info(f"📤 후방 UDP 첫 프레임 전송: {w}x{h} → {ip}:{prt}")
                                self._rear_first_send_logged = True
                            self._rear_wait_log_emitted = True
                        else:
                            # 카메라 프레임이 없으면 1초 간격으로 테스트 프레임 송출
                            import time as _t
                            last_ts = getattr(self, '_rear_last_test_ts', 0.0)
                            if _t.time() - last_ts > 1.0:
                                import numpy as _np, cv2 as _cv2
                                test = _np.full((360, 640, 3), 255, dtype=_np.uint8)
                                ts = _t.strftime('%H:%M:%S')
                                _cv2.putText(test, ts, (50, 190), _cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
                                # 움직이는 바 추가로 갱신 가시화
                                bar_x = int((_t.time() * 50) % 600)
                                _cv2.rectangle(test, (bar_x, 320), (bar_x + 40, 350), (0, 128, 255), -1)
                                self.udp_streamer.send_frame_bgr(test, quality=85)
                                if not getattr(self, '_rear_test_send_logged', False):
                                    self.get_logger().info("🧪 후방 UDP 테스트 프레임 전송 (카메라 프레임 없음)")
                                    self._rear_test_send_logged = True
                                self._rear_last_test_ts = _t.time()
                    else:
                        if not getattr(self, '_rear_wait_log_emitted', False):
                            ip, prt = self.udp_streamer.addr
                            self.get_logger().info(f"⏳ 후방 카메라 대기 중... (UDP 대상: {ip}:{prt})")
                            self._rear_wait_log_emitted = True
                        # 카메라가 없을 때도 1초 간격 테스트 프레임 송출
                        import time as _t
                        last_ts = getattr(self, '_rear_last_test_ts', 0.0)
                        if _t.time() - last_ts > 1.0:
                            import numpy as _np, cv2 as _cv2
                            test = _np.full((360, 640, 3), 255, dtype=_np.uint8)
                            _cv2.putText(test, 'NO CAMERA', (120, 190), _cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 4)
                            bar_x = int((_t.time() * 50) % 600)
                            _cv2.rectangle(test, (bar_x, 320), (bar_x + 40, 350), (0, 128, 255), -1)
                            self.udp_streamer.send_frame_bgr(test, quality=85)
                            if not getattr(self, '_rear_test_send_logged', False):
                                self.get_logger().info("🧪 후방 UDP 테스트 프레임 전송 (카메라 없음)")
                                self._rear_test_send_logged = True
                            self._rear_last_test_ts = _t.time()
                    
                    # FPS 제한 (15fps ~= 66.7ms) → 약간 여유를 둠
                    time.sleep(0.070)
                    
                except Exception as e:
                    self.get_logger().warning(f"프레임 전송 오류: {e}")
                    time.sleep(0.1)  # 에러 시 잠시 대기
                    
        except Exception as e:
            self.get_logger().error(f"UDP 스트리밍 루프 오류: {e}")
        finally:
            self.streaming_active = False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VSNode()
        
        # 메인 쓰레드에서 GUI와 ROS2를 함께 처리
        node.get_logger().info("메인 쓰레드에서 GUI 시작!")
        
        import cv2
        frame_count = 0
        
        try:
            while rclpy.ok():
                frame_count += 1
                
                # GUI 처리를 우선순위로
                try:
                    active_cameras = node.get_active_cameras()
                    
                    for camera_info in active_cameras:
                        camera = camera_info['camera']
                        depth_camera = camera_info['depth_camera']
                        camera_name = camera_info['name']
                        camera_type = camera_info['type']
                        mode_id = camera_info['mode_id']
                        
                        # 디버그: 카메라 정보 출력
                        if frame_count % 100 == 1:
                            node.get_logger().info(f"🔍 GUI 카메라: name={camera_name}, type={camera_type}")
                        
                        depth_image, color_image = None, None
                        
                        # 메인 카메라에서 프레임 획득
                        if camera:
                            try:
                                depth_image, color_image = camera.get_frames()
                            except Exception as e:
                                if frame_count % 100 == 1:
                                    node.get_logger().warning(f"{camera_name} 프레임 획득 실패: {e}")
                        
                        # 추가 뎁스 카메라가 있으면 뎁스만 다시 획득
                        if depth_camera and depth_camera != camera:
                            try:
                                additional_depth, _ = depth_camera.get_frames()
                                if additional_depth is not None:
                                    depth_image = additional_depth
                            except Exception as e:
                                if frame_count % 100 == 1:
                                    node.get_logger().warning(f"{camera_name} 뎁스 카메라 프레임 획득 실패: {e}")
                        
                        # 이미지가 없으면 다음 카메라로
                        if color_image is None:
                            if frame_count % 100 == 1:
                                node.get_logger().warning(f"❌ {camera_name}: color_image가 None입니다")
                            continue
                        
                        # 이미지 좌우반전 (뎁스 카메라만)
                        if camera_type == 'front_depth':
                            if color_image is not None:
                                color_image = cv2.flip(color_image, 1)
                            if depth_image is not None:
                                depth_image = cv2.flip(depth_image, 1)
                        
                        # ArUco 마커 자동 감지 (일반 모드의 전방 웹캠에서만)
                        if color_image is not None and camera_type == 'front_webcam' and mode_id == 5:
                            node.detect_and_update_location()
                        
                        # 객체 탐지 및 시각화
                        objects = []
                        if color_image is not None:
                            # 모드별 + 카메라 타입별 세분화
                            if camera_type == 'front_webcam':
                                if mode_id in [3, 4]:  # 엘리베이터 모드: 웹캠에 엘리베이터 YOLO
                                    detected_objects = node.model_detector.detect_objects(color_image, depth_image, node.confidence_threshold, mode_id)
                                    
                                    # 🎯 OCR 리소스 절약: 지정된 프레임 간격마다만 OCR 수행
                                    node.ocr_counter += 1
                                    if node.ocr_counter >= node.ocr_skip_frames:
                                        enhanced_objects = node._enhance_objects_with_ocr(color_image, detected_objects)
                                        # button_status와 동일한 고급 버튼 인식 로직 추가
                                        objects = node._apply_enhanced_button_recognition(enhanced_objects, color_image, mode_id)
                                        node.last_ocr_objects = objects  # 결과 캐싱
                                        node.ocr_counter = 0  # 카운터 리셋
                                        if frame_count % 100 == 1:
                                            node.get_logger().debug(f"🔄 OCR 수행됨 (매 {node.ocr_skip_frames}프레임마다)")
                                    else:
                                        # OCR 건너뛰고 이전 결과 재사용 (객체 감지는 계속)
                                        objects = detected_objects.copy()
                                        # 이전 OCR 결과가 있으면 병합
                                        if hasattr(node, 'last_ocr_objects') and node.last_ocr_objects:
                                            for old_obj in node.last_ocr_objects:
                                                if old_obj.get('class_name') == 'display' and old_obj.get('ocr_text'):
                                                    # 이전 OCR 결과를 현재 display 객체에 적용
                                                    for new_obj in objects:
                                                        if (new_obj.get('class_name') == 'display' and 
                                                            not new_obj.get('ocr_text')):
                                                            new_obj['ocr_text'] = old_obj.get('ocr_text', '')
                                                            new_obj['floor_number'] = old_obj.get('floor_number')
                                                            new_obj['ocr_success'] = old_obj.get('ocr_success', False)
                                                            new_obj['digit_bbox'] = old_obj.get('digit_bbox')
                                                            break
                                        # button_status와 동일한 고급 버튼 인식 로직 추가
                                        objects = node._apply_enhanced_button_recognition(objects, color_image, mode_id)
                                elif mode_id == 5:  # 일반 모드: ArUco만 (이미 위에서 처리)
                                    pass
                                elif mode_id == 6:  # 대기 모드: 영상만
                                    pass
                            elif camera_type == 'front_depth':
                                if mode_id == 5:  # 일반 모드: 뎁스에 일반 YOLO (OCR 불필요 - ArUco만)
                                    detected_objects = node.model_detector.detect_objects(color_image, depth_image, node.confidence_threshold, mode_id)
                                    objects = detected_objects  # OCR 없이 그대로 사용
                                    
                                    # 🚧 장애물 감지 및 발행 추가
                                    node.detect_and_publish_obstacles(
                                        objects, depth_camera, mode_id
                                    )
                                    
                                    # 🚪 유리 문 상태 감지 및 발행 추가
                                    node.detect_and_publish_glass_door_status(
                                        objects, mode_id
                                    )
                                elif mode_id in [3, 4, 6]:  # 엘리베이터/대기 모드: 뎁스는 영상만
                                    pass
                            elif camera_type in ['rear', 'front']:
                                # 후방 카메라나 기타 전방 카메라: 영상만
                                pass
                            
                            # 🎯 마지막 감지된 객체들 저장 (L키용)
                            if objects and mode_id in [3, 4] and camera_type == 'front_webcam':
                                node.last_detected_objects = objects.copy()
                            
                            # 🎯 메인 루프에서 방향등 위치 기억 + 실시간 방향 감지 (엘리베이터 모드에서만)
                            if mode_id in [3, 4] and camera_type == 'front_webcam':
                                if objects:
                                    direction_objects = [obj for obj in objects if obj.get('class_name') == 'direction_light']
                                    if len(direction_objects) == 2:
                                        node._update_remembered_positions(direction_objects)
                                
                                # 기억된 위치가 있으면 항상 밝기 히스토리 업데이트 + 방향 감지
                                if (node.remembered_direction_positions['upper'] and 
                                    node.remembered_direction_positions['lower']):
                                    node._update_brightness_history(color_image)
                                    
                                    # 🚦 메인 루프에서도 실시간 방향 감지 (GUI 업데이트용)
                                    direction_objects_for_detection = []
                                    if objects:
                                        direction_objects_for_detection = [obj for obj in objects if obj.get('class_name') == 'direction_light']
                                    
                                    detected_direction = node._detect_direction_by_lights(color_image, direction_objects_for_detection)
                                    
                                    # 방향 변경되었거나 깜빡임이 감지된 경우 업데이트
                                    if detected_direction != -1:
                                        if (detected_direction != node.last_elevator_direction or node.last_blink_detected):
                                            node.last_elevator_direction = detected_direction
                                            node.last_direction_detection_time = node.get_clock().now()
                                            blink_info = " (깜빡임 감지)" if node.last_blink_detected else ""
                                            node.get_logger().info(f"🎯 [메인루프] 방향 업데이트: {'상행' if detected_direction == 0 else '하행'}{blink_info}")
                                            
                                            # 깜빡임 처리 완료 후 플래그 초기화
                                            if node.last_blink_detected:
                                                node.last_blink_detected = False
                            
                            display_image = color_image.copy()
                            if objects:
                                display_image = node._draw_objects_on_image(display_image, objects, mode_id)
                            node._add_info_text(display_image, objects, camera_name)
                            
                            # GUI 표시 (헤드리스 모드가 아닐 때만)
                            if not node.headless_mode:
                                # 창 이름을 카메라 타입 기준으로 고정 (모드 변경 시 창 재사용)
                                if camera_type == 'front_webcam':
                                    window_name = 'Roomie VS - Front Webcam'
                                elif camera_type == 'front_depth':
                                    window_name = 'Roomie VS - Front Depth'
                                elif camera_type in ['rear', 'front']:
                                    if 'Rear' in camera_name:
                                        window_name = 'Roomie VS - Rear Webcam'
                                    else:
                                        window_name = 'Roomie VS - Front Webcam'
                                else:
                                    window_name = f'Roomie VS - {camera_type}'
                                
                                cv2.imshow(window_name, display_image)
                                cv2.waitKey(1)
                    
                    # 키 처리 (헤드리스 모드가 아닐 때만)
                    if not node.headless_mode:
                        key = cv2.waitKey(30) & 0xFF
                        
                        if key == 27:  # ESC
                            node.get_logger().info("ESC 키 눌림 - GUI 종료")
                            break
                        elif key == ord('r') or key == ord('R'):  # R키: 추적 시뮬레이션 (삭제됨)
                            node.get_logger().info("'R' 키 눌림 - 추적 시뮬레이션 기능이 삭제되었습니다")
                        elif key == ord('t') or key == ord('T'):  # T키: 단일 추적 이벤트 (삭제됨)
                            node.get_logger().info("'T' 키 눌림 - 추적 이벤트 발행 기능이 삭제되었습니다")
                        elif key == ord('g') or key == ord('G'):  # G키: 등록 완료 이벤트 (삭제됨)
                            node.get_logger().info("'G' 키 눌림 - 등록 완료 이벤트 발행 기능이 삭제되었습니다")
                        elif key == ord('b') or key == ord('B'):  # B키: 객체 탐지 결과 출력
                            model_info = node.model_detector.get_current_model_info()
                            # 안전한 모델 이름 표시
                            raw_model_name = model_info['model_name']
                            if raw_model_name == 'normal':
                                current_model = "Normal"
                            elif raw_model_name == 'elevator':
                                current_model = "Elevator"
                            else:
                                current_model = raw_model_name or "None"
                            
                            if objects:
                                button_objects = [obj for obj in objects if obj.get('class_name') == 'button']
                                other_objects = [obj for obj in objects if obj.get('class_name') != 'button']
                                
                                node.get_logger().info(f"'B' 키 눌림 - 객체 탐지 결과 (모델: {current_model}):")
                                node.get_logger().info(f"  전체 객체: {len(objects)}개")
                                node.get_logger().info(f"  버튼: {len(button_objects)}개")
                                node.get_logger().info(f"  기타 객체: {len(other_objects)}개")
                                
                                if button_objects:
                                    node.get_logger().info("  탐지된 버튼들:")
                                    for i, obj in enumerate(button_objects):
                                        confidence = obj.get('confidence', 1.0)
                                        pressed = "눌림" if obj.get('is_pressed', False) else "안눌림"
                                        model_name = obj.get('model_name', 'unknown')
                                        button_id = obj.get('button_id', 'unknown')
                                        recognition_method = obj.get('recognition_method', 'none')
                                        floor_type = obj.get('floor_type', 'unknown')
                                        
                                        # 버튼 이름 변환
                                        if isinstance(button_id, int):
                                            if button_id == 100:
                                                button_name = "DOWN"
                                            elif button_id == 101:
                                                button_name = "UP"
                                            elif button_id == 102:
                                                button_name = "OPEN"
                                            elif button_id == 103:
                                                button_name = "CLOSE"
                                            elif button_id == 13:
                                                button_name = "B1F"
                                            elif button_id == 14:
                                                button_name = "B2F"
                                            else:
                                                button_name = f"{button_id}F"
                                        else:
                                            button_name = str(button_id)
                                        
                                        node.get_logger().info(f"    {i+1}. {button_name} ({model_name}/{recognition_method}) - 신뢰도:{confidence:.2f}, {pressed}, {obj['depth_mm']}mm")
                                
                                if other_objects:
                                    node.get_logger().info("  기타 객체들:")
                                    for i, obj in enumerate(other_objects):
                                        class_name = obj.get('class_name', 'unknown')
                                        confidence = obj.get('confidence', 1.0)
                                        model_name = obj.get('model_name', 'unknown')
                                        node.get_logger().info(f"    {i+1}. {class_name} ({model_name}) - 신뢰도:{confidence:.2f}, {obj['depth_mm']}mm")
                            else:
                                node.get_logger().info(f"'B' 키 눌림 - 탐지된 객체가 없습니다 (모델: {current_model})")
                        elif key == ord('f') or key == ord('F'):  # F키: 좌우반전 토글
                            node.flip_horizontal = not node.flip_horizontal
                            status = "켜짐" if node.flip_horizontal else "꺼짐"
                            node.get_logger().info(f"'F' 키 눌림 - 좌우반전: {status}")
                        elif key == ord('c') or key == ord('C'):  # C키: 신뢰도 임계값 조정
                            current_conf = node.confidence_threshold
                            if current_conf == 0.7:
                                node.confidence_threshold = 0.5
                            elif current_conf == 0.5:
                                node.confidence_threshold = 0.9
                            else:
                                node.confidence_threshold = 0.7
                            
                            node.get_logger().info(f"'C' 키 눌림 - 신뢰도 임계값: {current_conf:.2f} → {node.confidence_threshold:.2f}")

                        elif key == ord('m') or key == ord('M'):  # M키: 현재 모드 확인
                            current_mode = node.get_active_mode_name()
                            model_info = node.model_detector.get_current_model_info()
                            model_status = "✅" if model_info['is_active'] else "❌"
                            # 안전한 모델 이름 표시
                            raw_model_name = model_info['model_name']
                            if raw_model_name == 'normal':
                                current_model = "Normal"
                            elif raw_model_name == 'elevator':
                                current_model = "Elevator"
                            else:
                                current_model = raw_model_name or "None"
                            aruco_status = "✅" if node.aruco_dict else "❌"
                            
                            node.get_logger().info(f"'M' 키 눌림 - 현재 상태:")
                            node.get_logger().info(f"  VS 모드 - 전방: {node.mode_names[node.current_front_mode_id]} (ID:{node.current_front_mode_id}), 후방: {node.mode_names[node.current_rear_mode_id]} (ID:{node.current_rear_mode_id})")
                            node.get_logger().info(f"  현재 모델: {current_model} {model_status}")
                            node.get_logger().info(f"  사용 가능한 모델: {model_info['available_models']}")
                            node.get_logger().info(f"  현재 카메라: {node.current_camera_name}")
                            node.get_logger().info(f"  ArUco 시스템: {aruco_status}")
                            node.get_logger().info(f"  좌우반전: {'ON' if node.flip_horizontal else 'OFF'}")
                            node.get_logger().info(f"  신뢰도 임계값: {node.confidence_threshold}")
                            
                            # 현재 위치 정보
                            location_names = {
                                0: "LOB_WAITING", 1: "LOB_CALL", 2: "RES_PICKUP", 3: "RES_CALL",
                                4: "SUP_PICKUP", 5: "ELE_1", 6: "ELE_2", 101: "ROOM_101",
                                102: "ROOM_102", 201: "ROOM_201", 202: "ROOM_202"
                            }
                            current_location_name = location_names.get(node.last_detected_location_id, f"ID_{node.last_detected_location_id}")
                            node.get_logger().info(f"  현재 위치: {current_location_name}")
                            
                            if model_info['is_active']:
                                supported_classes = model_info['class_names']
                                node.get_logger().info(f"  감지 가능한 객체: {supported_classes}")
                            else:
                                node.get_logger().info(f"  감지 가능한 객체: 없음 (모델 비활성화)")
                            
                            node.get_logger().info("후방 카메라 모드: 0(대기), 1(등록), 2(추적)")
                            node.get_logger().info("전방 카메라 모드: 3(엘리베이터 외부), 4(엘리베이터 내부), 5(일반), 6(대기)")
                            node.get_logger().info("시뮬레이션 모드: 100(배송), 101(호출), 102(길안내), 103(복귀), 104(엘리베이터)")
                            node.get_logger().info("키보드: A(ArUco테스트), F(좌우반전), C(신뢰도조정)")
                        elif key == ord('a') or key == ord('A'):  # A키: ArUco 감지 테스트
                            node.test_aruco_detection()
                        elif key == ord('d') or key == ord('D'):  # D키: 방향등 위치 리셋
                            node.remembered_direction_positions = {'upper': None, 'lower': None}
                            node.last_position_update = None
                            node.brightness_history = {'upper': [], 'lower': []}
                            node.get_logger().info("🔄 방향등 위치와 밝기 히스토리를 리셋했습니다. 방향등 2개가 감지되면 다시 기억을 시작합니다.")
                        elif key == ord('l') or key == ord('L'):  # L키: 강제 방향등 위치 기억
                            # 마지막으로 감지된 direction_light 객체들로 강제 위치 기억
                            if node.last_detected_objects:
                                direction_objects = [obj for obj in node.last_detected_objects if obj.get('class_name') == 'direction_light']
                                if len(direction_objects) == 2:
                                    node.get_logger().info(f"🔥 [MANUAL] L키로 강제 위치 기억 시도! 방향등 {len(direction_objects)}개 감지됨")
                                    # 좌표 정보 출력
                                    for i, obj in enumerate(direction_objects):
                                        node.get_logger().info(f"🔥 [MANUAL] 방향등[{i}]: center={obj['center']}, bbox={obj['bbox']}")
                                    success = node._update_remembered_positions(direction_objects)
                                    node.get_logger().info(f"🔥 [MANUAL] 강제 위치 기억 결과: {success}")
                                elif len(direction_objects) == 0:
                                    node.get_logger().warn("🔥 [MANUAL] 방향등이 감지되지 않았습니다")
                                else:
                                    node.get_logger().warn(f"🔥 [MANUAL] 방향등이 2개가 아님: {len(direction_objects)}개")
                                    for i, obj in enumerate(direction_objects):
                                        node.get_logger().info(f"🔥 [MANUAL] 방향등[{i}]: center={obj['center']}")
                            else:
                                node.get_logger().warn("🔥 [MANUAL] 마지막 감지된 객체가 없습니다")
                        elif key != 255 and key != -1:  # 다른 키가 눌렸을 때 (헤드리스 모드 제외)
                            if 32 <= key <= 126:
                                node.get_logger().info(f"'{chr(key)}' 키 눌림")
                                node.get_logger().info("사용 가능한 키:")
                                node.get_logger().info("   R(추적시뮬레이션), T(추적이벤트), G(등록완료)")
                                node.get_logger().info("   B(버튼정보), M(상태확인), A(ArUco테스트)")
                                node.get_logger().info("   F(좌우반전), C(신뢰도), D(위치리셋), L(위치기억), ESC(종료)")
                            else:
                                node.get_logger().info(f"키 코드 {key} 눌림")
                            
                except Exception as e:
                    node.get_logger().error(f"프레임 처리 오류: {e}")
                    time.sleep(0.1)
                
                # GUI 처리 완료 후에 ROS2 콜백을 비중단적으로 처리
                try:
                    rclpy.spin_once(node, timeout_sec=0.001)  # 1ms만
                except Exception as ros_error:
                    if frame_count % 1000 == 1:
                        node.get_logger().warning(f"ROS2 콜백 처리 중 에러: {ros_error}")
                    
        except KeyboardInterrupt:
            node.get_logger().info("사용자에 의해 중단되었습니다")
        finally:
            # 정리
            if hasattr(node, 'camera_manager'):
                node.camera_manager.cleanup_all_cameras()
            
            # GUI 윈도우 정리 (헤드리스 모드가 아닐 때만)
            if hasattr(node, 'headless_mode') and not node.headless_mode:
                cv2.destroyAllWindows()
            node.destroy_node()
            
    except RuntimeError as e:
        print(f"카메라 초기화 실패: {e}")
        print("해결 방법:")
        print("   1. Astra 카메라가 USB에 제대로 연결되어 있는지 확인")
        print("   2. OpenNI2가 올바르게 설치되어 있는지 확인")
        print("   3. 카메라 드라이버가 설치되어 있는지 확인")
        print("   4. 다른 프로그램에서 카메라를 사용하고 있지 않은지 확인")
    except Exception as e:
        print(f"노드 실행 중 예상치 못한 에러: {e}")
        import traceback
        print(f"스택 트레이스: {traceback.format_exc()}")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            pass

if __name__ == '__main__':
    main() 