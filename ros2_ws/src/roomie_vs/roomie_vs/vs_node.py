#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import os
import numpy as np
import cv2
import cv2.aruco as aruco
from typing import Optional, Tuple, List

# ROS2 메시지 타입들
from geometry_msgs.msg import Point

# 커스텀 서비스
from roomie_msgs.srv import (
    ButtonStatus, 
    SetVSMode,
    ElevatorWidth,
    ElevatorStatus, 
    DoorStatus,
    SpaceAvailability,
    Location
)
from roomie_msgs.msg import TrackingEvent, Registered

# OpenNI2 환경변수 설정
import os

def setup_openni2_environment():
    """OpenNI2 실행을 위한 환경변수 설정"""
    openni_path = os.path.expanduser("~/Downloads/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux/ros2_astra_camera/astra_camera/openni2_redist/x64")
    
    if not os.path.exists(openni_path):
        print(f"❌ OpenNI2 경로를 찾을 수 없습니다: {openni_path}")
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
        
        # 카메라 내부 파라미터 (Astra 기본값)
        self.depth_fx = 570.3
        self.depth_fy = 570.3
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
    
    def pixel_to_3d(self, u: int, v: int, depth_mm: int) -> Tuple[float, float, float]:
        """2D 픽셀 좌표를 3D 월드 좌표로 변환"""
        if depth_mm <= 0:
            return 0.0, 0.0, 0.0
            
        z = depth_mm / 1000.0  # mm to meters
        x = (u - self.depth_cx) * z / self.depth_fx
        y = (v - self.depth_cy) * z / self.depth_fy
        
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
        
        # 전방 USB 웹캠인 경우
        if "USB" in self.camera_name:
            # 디바이스 이름에서 USB 웹캠 찾기
            for camera in available_cameras:
                device_name = camera['device_name'].lower()
                # USB 웹캠의 일반적인 키워드들
                usb_keywords = ['usb', 'webcam', 'c920', 'c922', 'c930', 'apc930', 'abko', 'logitech']
                
                if any(keyword in device_name for keyword in usb_keywords):
                    self.logger.info(f"USB 웹캠으로 선택: ID={camera['id']}, device='{camera['device_name']}'")
                    return camera
            
            # USB 웹캠을 못 찾았으면 0번이 아닌 카메라 우선
            for camera in available_cameras:
                if camera['id'] != 0:
                    self.logger.warning(f"USB 웹캠 디바이스명 미매칭, ID 기반 선택: ID={camera['id']}, device='{camera['device_name']}'")
                    return camera
            
            # 그래도 없으면 첫 번째
            self.logger.warning("USB 웹캠을 찾지 못해 첫 번째 카메라 사용")
            return available_cameras[0]
        
        # 후방 내장 카메라인 경우
        elif "Built-in" in self.camera_name:
            # 디바이스 이름에서 내장 카메라 찾기
            for camera in available_cameras:
                device_name = camera['device_name'].lower()
                # 내장 카메라의 일반적인 키워드들
                builtin_keywords = ['integrated', 'built-in', 'webcam', 'camera', 'hd']
                
                if camera['id'] == 0 or any(keyword in device_name for keyword in builtin_keywords):
                    self.logger.info(f"내장 카메라로 선택: ID={camera['id']}, device='{camera['device_name']}'")
                    return camera
            
            # 내장 카메라가 없으면 첫 번째 사용 가능한 카메라
            self.logger.warning("내장 카메라를 찾지 못해 첫 번째 카메라 사용")
            return available_cameras[0]
        
        # 기본적으로 첫 번째 사용 가능한 카메라 선택
        return available_cameras[0]
    
    def _try_camera_id(self, camera_id: int) -> bool:
        """특정 camera_id로 웹캠 초기화 시도"""
        try:
            self.logger.info(f"{self.camera_name} camera_id={camera_id} 시도 중...")
            
            cap = cv2.VideoCapture(camera_id)
            if not cap.isOpened():
                self.logger.debug(f"camera_id={camera_id} 열기 실패")
                cap.release()
                return False
            
            # 해상도 설정 (640x480)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
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
            height, width = frame.shape[:2]
            
            # 카메라 백엔드 정보 확인
            backend = cap.getBackendName()
            self.logger.info(f"✅ {self.camera_name} 초기화 성공: camera_id={camera_id}, {width}x{height}, backend={backend}")
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

class MultiModelDetector:
    """다중 YOLO 모델을 지원하는 탐지 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.models = {}
        self.current_model_name = None
        self.current_model = None
        
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
                    # 실제 모델 클래스 확인
                    if hasattr(self.models['normal'], 'names'):
                        actual_classes = list(self.models['normal'].names.values())
                        self.logger.info(f"✅ 일반 주행 모델 로딩 성공: {normal_model_path}")
                        self.logger.info(f"📋 실제 클래스: {actual_classes}")
                    else:
                        self.logger.info(f"✅ 일반 주행 모델 로딩 성공: {normal_model_path}")
                except Exception as e:
                    self.logger.warning(f"⚠️ 일반 주행 모델 로딩 실패: {e}")
            else:
                # 일반 주행용 모델이 없으면 COCO 사전훈련 모델 사용
                try:
                    self.models['normal'] = YOLO('yolov8n.pt')
                    # COCO 모델 클래스 확인
                    if hasattr(self.models['normal'], 'names'):
                        actual_classes = list(self.models['normal'].names.values())
                        self.logger.info("✅ 일반 주행용으로 COCO 사전훈련 모델(yolov8n.pt) 사용")
                        self.logger.info(f"📋 COCO 클래스 (전체 {len(actual_classes)}개): person, chair 등만 필터링 사용")
                    else:
                        self.logger.info("✅ 일반 주행용으로 COCO 사전훈련 모델(yolov8n.pt) 사용")
                except Exception as e:
                    self.logger.warning(f"⚠️ COCO 모델도 로딩 실패: {e}")
            
            # 2. 엘리베이터용 모델 (training/elevator/best.pt)
            elevator_model_path = self._find_model_in_subdir('elevator', 'best.pt')
            if elevator_model_path:
                try:
                    self.models['elevator'] = YOLO(elevator_model_path)
                    # 실제 모델 클래스 확인
                    if hasattr(self.models['elevator'], 'names'):
                        actual_classes = list(self.models['elevator'].names.values())
                        self.logger.info(f"✅ 엘리베이터 모델 로딩 성공: {elevator_model_path}")
                        self.logger.info(f"📋 실제 클래스: {actual_classes}")
                    else:
                        self.logger.info(f"✅ 엘리베이터 모델 로딩 성공: {elevator_model_path}")
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
            
            # 모드별 버튼 인식 처리
            if mode_id == 3:  # 엘리베이터 외부 - button_recog_1
                objects = self._apply_button_recog_1(objects)
            elif mode_id == 4:  # 엘리베이터 내부 - button_recog_2  
                objects = self._apply_button_recog_2(objects)
                
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
                        object_id = current_id_map.get(class_name, class_name.upper())
                        
                        if class_name == 'button' and depth_image is not None:
                            is_pressed = self._check_button_pressed(depth_image, center_x, center_y, radius)
                        
                        objects.append({
                            'center': (center_x, center_y),
                            'radius': radius,
                            'depth_mm': int(depth_value),
                            'is_pressed': is_pressed,
                            'class_name': class_name,
                            'class_id': class_id,
                            'object_id': object_id,
                            'confidence': float(conf),
                            'bbox': (x1, y1, x2, y2),
                            'is_button': class_name == 'button',
                            'model_name': self.current_model_name
                        })
            
            self.logger.debug(f"{self.current_model_name} 모델로 {len(objects)}개 객체 탐지")
            return objects
            
        except Exception as e:
            self.logger.error(f"{self.current_model_name} 모델 탐지 에러: {e}")
            return []
    
    def _check_button_pressed(self, depth_image: np.ndarray, cx: int, cy: int, radius: int) -> bool:
        """버튼 눌림 상태 확인 (기존 YOLOButtonDetector와 동일)"""
        try:
            center_depth = depth_image[cy, cx]
            if center_depth <= 0:
                return False
            
            y1, y2 = max(0, cy-radius), min(depth_image.shape[0], cy+radius)
            x1, x2 = max(0, cx-radius), min(depth_image.shape[1], cx+radius)
            
            surrounding_region = depth_image[y1:y2, x1:x2]
            valid_depths = surrounding_region[surrounding_region > 0]
            
            if valid_depths.size < 5:
                return False
                
            surrounding_depth = np.mean(valid_depths)
            
            # 중심이 주변보다 깊으면 눌린 것으로 판단
            return center_depth > surrounding_depth + 10  # 10mm 차이
            
        except Exception:
            return False
    
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
                center_y = obj['center'][1]
                
                # 상위 50%는 상행버튼, 하위 50%는 하행버튼
                if center_y <= button_objects[len(button_objects)//2]['center'][1]:
                    obj['button_id'] = 101  # 상행버튼
                    obj['floor_type'] = 'up'
                else:
                    obj['button_id'] = 100  # 하행버튼  
                    obj['floor_type'] = 'down'
                    
                obj['recognition_method'] = 'button_recog_1'
                
            updated_objects.append(obj)
            
        return updated_objects
    
    def _apply_button_recog_2(self, objects: List[dict]) -> List[dict]:
        """button_recog_2: 엘리베이터 내부 - 위치 기반 층수 매핑"""
        button_objects = [obj for obj in objects if obj.get('class_name') == 'button']
        
        if len(button_objects) == 0:
            return objects
            
        # 엘리베이터 내부 버튼 배치 매핑 (상대 위치 기반)
        # 102  |  1   |  4   |  7   | 10  |
        # 103  | 13   |  3   |  6   |  9  | 12
        #      | 14   |  2   |  5   |  8  | 11
        
        button_layout = {
            # (col, row): button_id
            (0, 0): 102,  # 열기
            (0, 1): 103,  # 닫기
            (1, 0): 1,    # 1층
            (1, 1): 13,   # B1층
            (1, 2): 14,   # B2층
            (2, 0): 4,    # 4층
            (2, 1): 3,    # 3층
            (2, 2): 2,    # 2층
            (3, 0): 7,    # 7층
            (3, 1): 6,    # 6층
            (3, 2): 5,    # 5층
            (4, 0): 10,   # 10층
            (4, 1): 9,    # 9층
            (4, 2): 8,    # 8층
            (5, 1): 12,   # 12층
            (5, 2): 11,   # 11층
        }
        
        # 버튼들의 위치를 기반으로 격자 생성
        x_coords = [obj['center'][0] for obj in button_objects]
        y_coords = [obj['center'][1] for obj in button_objects]
        
        if len(set(x_coords)) < 2 or len(set(y_coords)) < 2:
            # 격자를 만들 수 없으면 원본 반환
            return objects
            
        # X, Y 좌표를 열/행으로 변환
        x_sorted = sorted(set(x_coords))
        y_sorted = sorted(set(y_coords))
        
        updated_objects = []
        
        for obj in objects:
            if obj.get('class_name') == 'button':
                center_x, center_y = obj['center']
                
                # 가장 가까운 격자점 찾기
                col = min(range(len(x_sorted)), key=lambda i: abs(x_sorted[i] - center_x))
                row = min(range(len(y_sorted)), key=lambda i: abs(y_sorted[i] - center_y))
                
                # 매핑 테이블에서 button_id 찾기
                if (col, row) in button_layout:
                    button_id = button_layout[(col, row)]
                    obj['button_id'] = button_id
                    
                    # 버튼 종류 분류
                    if button_id in [100, 101]:
                        obj['floor_type'] = 'direction'
                    elif button_id in [102, 103]:
                        obj['floor_type'] = 'control'
                    elif button_id in [13, 14]:
                        obj['floor_type'] = 'basement'
                    else:
                        obj['floor_type'] = 'floor'
                        
                else:
                    # 매핑되지 않은 위치 - 기본값
                    obj['button_id'] = f"unknown_{col}_{row}"
                    obj['floor_type'] = 'unknown'
                    
                obj['recognition_method'] = 'button_recog_2'
                obj['grid_position'] = (col, row)
                
            updated_objects.append(obj)
            
        return updated_objects

    def get_current_model_info(self):
        """현재 모델 정보 반환"""
        return {
            'model_name': self.current_model_name,
            'available_models': list(self.models.keys()),
            'class_names': self.model_classes.get(self.current_model_name, []),
            'is_active': self.current_model is not None
        }

class YOLOButtonDetector:
    """YOLO 기반 엘리베이터 객체 탐지 클래스"""
    
    def __init__(self, logger):
        self.logger = logger
        self.yolo_model = None
        
        # 4개 클래스 정의
        self.class_names = [
            'button', 'direction_light', 'display', 'door'
        ]
        
        # 클래스별 ID 매핑
        self.button_id_map = {
            'button': 'BUTTON',
        }
        
        # YOLO 모델 초기화
        self._initialize_yolo_model()
        
    def _initialize_yolo_model(self):
        """YOLO 모델 초기화 및 로딩"""
        try:
            from ultralytics import YOLO
            
            model_path = self._find_best_model()
            if model_path:
                self.yolo_model = YOLO(model_path)
                self.logger.info(f"엘리베이터 감지 모델 로딩 성공: {model_path}")
                return True
            else:
                self.logger.error("엘리베이터 감지 YOLO 모델을 찾을 수 없습니다")
                self.logger.error("training/elevator/best.pt 파일이 있는지 확인하세요")
                raise FileNotFoundError("엘리베이터 감지 YOLO 모델 파일을 찾을 수 없습니다")
                
        except ImportError:
            self.logger.error("ultralytics 패키지가 필요합니다: pip install ultralytics")
            raise ImportError("ultralytics 패키지를 설치하세요")
        except Exception as e:
            self.logger.error(f"YOLO 모델 초기화 실패: {e}")
            raise RuntimeError(f"YOLO 모델 로딩 실패: {e}")
    
    def _find_best_model(self):
        """엘리베이터 감지 YOLO 모델 찾기"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        possible_training_dirs = [
            os.path.join(script_dir, "..", "training"),
            os.path.join(os.path.expanduser("~"), "project_ws", "Roomie", "ros2_ws", "src", "roomie_vs", "training"),
            os.path.join(os.getcwd(), "ros2_ws", "src", "roomie_vs", "training"),
            "ros2_ws/src/roomie_vs/training"
        ]
        
        training_dir = None
        for candidate in possible_training_dirs:
            if os.path.exists(candidate):
                training_dir = candidate
                break
        
        if training_dir is None:
            self.logger.error("training 디렉토리를 찾을 수 없습니다")
            return None
            
        self.logger.info(f"엘리베이터 감지 모델 검색: {training_dir}")
        
        # 엘리베이터 서브디렉토리에서 best.pt 찾기
        best_model_path = os.path.join(training_dir, "elevator", "best.pt")
        if os.path.exists(best_model_path):
            self.logger.info(f"엘리베이터 감지 모델 발견: {best_model_path}")
            return best_model_path
        
        self.logger.error(f"엘리베이터 감지 모델을 찾을 수 없습니다: {best_model_path}")
        return None
        
    def detect_buttons(self, color_image: np.ndarray, depth_image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """YOLO로 이미지에서 엘리베이터 객체들을 탐지"""
        if color_image is None or self.yolo_model is None:
            return []
            
        try:
            return self._detect_with_yolo(color_image, depth_image, conf_threshold)
        except Exception as e:
            self.logger.error(f"YOLO 버튼 탐지 에러: {e}")
            return []
    
    def _detect_with_yolo(self, color_image: np.ndarray, depth_image: np.ndarray, conf_threshold: float = 0.7) -> List[dict]:
        """YOLO 모델을 사용한 버튼 탐지"""
        try:
            results = self.yolo_model.predict(
                color_image, 
                conf=conf_threshold,
                verbose=False
            )
            
            buttons = []
            if results and len(results) > 0:
                result = results[0]
                
                if result.boxes is not None and len(result.boxes) > 0:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    confs = result.boxes.conf.cpu().numpy()
                    classes = result.boxes.cls.cpu().numpy()
                    
                    for box, conf, cls in zip(boxes, confs, classes):
                        x1, y1, x2, y2 = box.astype(int)
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        width = x2 - x1
                        height = y2 - y1
                        radius = int(max(width, height) / 2)
                        
                        # 클래스 정보
                        class_id = int(cls)
                        class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"unknown_{class_id}"
                        
                        # Depth 정보
                        depth_value = depth_image[center_y, center_x] if depth_image is not None else 1000
                        
                        # 버튼 눌림 상태 추정
                        is_pressed = False
                        button_id = None
                        
                        if class_name == 'button':
                            button_id = self.button_id_map.get(class_name, 'BUTTON')
                            if depth_image is not None:
                                is_pressed = self._check_button_pressed(depth_image, center_x, center_y, radius)
                        
                        buttons.append({
                            'center': (center_x, center_y),
                            'radius': radius,
                            'depth_mm': int(depth_value),
                            'is_pressed': is_pressed,
                            'class_name': class_name,
                            'class_id': class_id,
                            'button_id': button_id,
                            'confidence': float(conf),
                            'bbox': (x1, y1, x2, y2),
                            'is_button': class_name == 'button'
                        })
            
            self.logger.debug(f"엘리베이터 객체 탐지 결과: {len(buttons)}개")
            return buttons
            
        except Exception as e:
            self.logger.error(f"YOLO 탐지 에러: {e}")
            return []
    
    def _check_button_pressed(self, depth_image: np.ndarray, cx: int, cy: int, radius: int) -> bool:
        """버튼 눌림 상태 확인"""
        try:
            center_depth = depth_image[cy, cx]
            if center_depth <= 0:
                return False
            
            y1, y2 = max(0, cy-radius), min(depth_image.shape[0], cy+radius)
            x1, x2 = max(0, cx-radius), min(depth_image.shape[1], cx+radius)
            
            surrounding_region = depth_image[y1:y2, x1:x2]
            valid_depths = surrounding_region[surrounding_region > 0]
            
            if valid_depths.size < 5:
                return False
                
            surrounding_depth = np.mean(valid_depths)
            
            # 중심이 주변보다 깊으면 눌린 것으로 판단
            return center_depth > surrounding_depth + 10  # 10mm 차이
            
        except Exception:
            return False

class VSNode(Node):
    """OpenNI2 기반 Vision Service ROS2 노드"""
    
    def __init__(self):
        super().__init__('vs_node')
        
        # 멀티 카메라 매니저와 다중 모델 탐지기 초기화
        self.camera_manager = MultiCameraManager(self.get_logger())
        self.model_detector = MultiModelDetector(self.get_logger())
        
        # 현재 선택된 카메라들 (모드별로 변경됨)
        self.current_camera = None
        self.current_depth_camera = None
        self.current_camera_name = "None"
        
        # 이미지 처리 옵션
        self.flip_horizontal = True  # 좌우반전을 기본으로 켜기
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
            
            self.get_logger().info("🔍 ArUco 기본 파라미터 생성 시도...")
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.get_logger().info("✅ ArUco 파라미터 생성 성공")
            
            # ArucoDetector는 일단 생성하지 않음 (이 부분이 문제일 가능성)
            self.aruco_detector = None
            self.aruco_api_version = "basic"
            
        except Exception as e:
            self.get_logger().warning(f"초기화 중 오류: {e}")
            self.aruco_dict = None
            self.aruco_params = None
            self.aruco_detector = None
            self.aruco_api_version = "error"
        
        # 마지막으로 감지된 위치 저장
        self.last_detected_location_id = 0  # 기본값: LOB_WAITING
        self.last_detection_time = None
        
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
        
        self.elevator_width_service = self.create_service(
            ElevatorWidth,
            '/vs/command/elevator_width',
            self.elevator_width_callback
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
        
        self.space_availability_service = self.create_service(
            SpaceAvailability,
            '/vs/command/space_availability',
            self.space_availability_callback
        )
        
        self.location_service = self.create_service(
            Location,
            '/vs/command/location',
            self.location_callback
        )
        
        # ROS2 토픽 퍼블리셔들
        
        self.tracking_event_pub = self.create_publisher(
            TrackingEvent,
            '/vs/tracking_event',
            10
        )
        
        self.registered_pub = self.create_publisher(
            Registered,
            '/vs/registered',
            10
        )
        
        self.get_logger().info("모든 VS 인터페이스 초기화 완료!")
        self.get_logger().info("구현된 서비스 7개: set_vs_mode, elevator_width, button_status, elevator_status, door_status, space_availability, location")
        self.get_logger().info("구현된 토픽 2개: tracking_event, registered")
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
        if self.aruco_detector is None:
            self.get_logger().debug("ArUco 시스템이 초기화되지 않음")
            return self.last_detected_location_id
        
        try:
            # 입력 이미지가 제공되면 사용, 없으면 현재 카메라 프레임 사용
            if input_image is not None:
                current_color = input_image
            else:
                # 현재 카메라 프레임 획득
                with self.camera.frame_lock:
                    current_color = self.camera.current_color
            
            if current_color is None:
                self.get_logger().debug("카메라 프레임이 없음")
                return self.last_detected_location_id
            
            # 좌우반전은 이미 적용되었다고 가정 (main에서 처리)
            processed_image = current_color.copy()
            
            # 그레이스케일 변환
            gray = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)
            
            # A키 테스트와 동일한 관대한 파라미터로 감지
            test_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            test_params = cv2.aruco.DetectorParameters()
            test_params.minMarkerPerimeterRate = 0.03
            test_params.maxMarkerPerimeterRate = 4.0
            test_params.polygonalApproxAccuracyRate = 0.1
            test_params.maxErroneousBitsInBorderRate = 0.35
            test_params.errorCorrectionRate = 0.6
            test_detector = cv2.aruco.ArucoDetector(test_dict, test_params)
            
            # ArUco 마커 감지 (A키 테스트와 동일한 방식)
            corners, ids, rejected = test_detector.detectMarkers(gray)
            
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
                    self.get_logger().warning(f"⚠️ 알 수 없는 ArUco 마커: {detected_id} (매핑 테이블에 없음)")
                    self.get_logger().info(f"지원되는 마커 ID: {list(self.aruco_to_location.keys())}")
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
            with self.camera.frame_lock:
                current_color = self.camera.current_color
            
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
                    # 테스트용 ArUco 사전과 detector 생성
                    test_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                    test_params = cv2.aruco.DetectorParameters()
                    
                    # 관대한 파라미터 설정
                    test_params.minMarkerPerimeterRate = 0.03
                    test_params.maxMarkerPerimeterRate = 4.0
                    test_params.polygonalApproxAccuracyRate = 0.1
                    test_params.maxErroneousBitsInBorderRate = 0.35
                    test_params.errorCorrectionRate = 0.6
                    
                    test_detector = cv2.aruco.ArucoDetector(test_dict, test_params)
                    
                    # ArUco 마커 감지
                    corners, ids, rejected = test_detector.detectMarkers(gray)
                    
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
        """ArUco 마커 감지 결과를 이미지에 표시"""
        if self.aruco_detector is None:
            return
        
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # A키 테스트와 동일한 관대한 파라미터로 감지
            test_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            test_params = cv2.aruco.DetectorParameters()
            test_params.minMarkerPerimeterRate = 0.03
            test_params.maxMarkerPerimeterRate = 4.0
            test_params.polygonalApproxAccuracyRate = 0.1
            test_params.maxErroneousBitsInBorderRate = 0.35
            test_params.errorCorrectionRate = 0.6
            test_detector = cv2.aruco.ArucoDetector(test_dict, test_params)
            
            # ArUco 마커 감지
            corners, ids, rejected = test_detector.detectMarkers(gray)
            
            if ids is not None:
                # 감지된 마커 그리기
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                
                # 마커 정보 텍스트 표시
                cv2.putText(image, f"ArUco Markers: {len(ids)}", (10, 160), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # 첫 번째 마커의 location_id 표시
                if len(ids) > 0:
                    marker_id = int(ids[0][0])
                    location_id = self.aruco_to_location.get(marker_id, -1)
                    if location_id != -1:
                        location_names = {
                            0: "LOB_WAITING", 1: "LOB_CALL", 2: "RES_PICKUP", 3: "RES_CALL",
                            4: "SUP_PICKUP", 5: "ELE_1", 6: "ELE_2", 101: "ROOM_101",
                            102: "ROOM_102", 201: "ROOM_201", 202: "ROOM_202"
                        }
                        location_name = location_names.get(location_id, f"ID_{location_id}")
                        cv2.putText(image, f"Location: {location_name}", (10, 185), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        except Exception as e:
            pass
    
    def button_status_callback(self, request, response):
        """버튼 상태 요청 처리"""
        try:
            self.get_logger().info(f"버튼 상태 요청: robot_id={request.robot_id}, button_ids={request.button_ids}")
            
            response.robot_id = request.robot_id
            n_buttons = len(request.button_ids)
            
            if n_buttons == 0:
                response.xs = []
                response.ys = []
                response.depths = []
                response.is_pressed = []
                response.timestamp = []
                return response

            xs, ys, depths, is_pressed, timestamps = [], [], [], [], []
            
            try:
                # 현재 프레임 획득
                with self.camera.frame_lock:
                    current_depth = self.camera.current_depth
                    current_color = self.camera.current_color
                
                # 이미지 좌우반전
                if self.flip_horizontal:
                    if current_color is not None:
                        current_color = cv2.flip(current_color, 1)
                    if current_depth is not None:
                        current_depth = cv2.flip(current_depth, 1)
                
                if current_color is not None:
                    # 다중 모델로 객체 탐지 (현재 모드 전달)
                    detected_objects = self.model_detector.detect_objects(current_color, current_depth, self.confidence_threshold, self.current_front_mode_id)
                    
                    # 'button' 클래스 객체들만 필터링
                    detected_buttons = [obj for obj in detected_objects if obj.get('class_name') == 'button']
                    
                    for i, button_id in enumerate(request.button_ids):
                        timestamp = self.get_clock().now().to_msg()
                        
                        if i < len(detected_buttons):
                            btn = detected_buttons[i]
                            center = btn['center']
                            
                            # 3D 좌표로 변환
                            x_3d = (center[0] - 320.0) / 570.3 * (btn['depth_mm'] / 1000.0)
                            y_3d = (center[1] - 240.0) / 570.3 * (btn['depth_mm'] / 1000.0)
                            z_3d = btn['depth_mm'] / 1000.0
                            
                            xs.append(float(x_3d))
                            ys.append(float(y_3d))
                            depths.append(float(z_3d))
                            is_pressed.append(bool(btn['is_pressed']))
                            timestamps.append(timestamp)
                            
                            confidence = btn.get('confidence', 1.0)
                            self.get_logger().info(f"버튼 탐지 - button #{i+1}: "
                                                 f"x={x_3d:.3f}, y={y_3d:.3f}, z={z_3d:.3f}, "
                                                 f"pressed={btn['is_pressed']}, conf={confidence:.2f}")
                        else:
                            # 더미값 사용
                            dummy_x = 0.1 + (len(xs) * 0.05)
                            dummy_y = 0.2 + (len(xs) * 0.03)
                            dummy_z = 1.0
                            
                            xs.append(float(dummy_x))
                            ys.append(float(dummy_y))
                            depths.append(float(dummy_z))
                            is_pressed.append(bool(False))
                            timestamps.append(timestamp)
                            
                            self.get_logger().info(f"요청된 버튼 #{i+1} 미탐지 - 더미값 사용")
                else:
                    self.get_logger().warning("카메라 프레임이 없음 - 더미값 사용")
                    for i, button_id in enumerate(request.button_ids):
                        xs.append(float(0.1 + i * 0.05))
                        ys.append(float(0.2 + i * 0.03))
                        depths.append(float(0.8 + i * 0.1))
                        is_pressed.append(bool(False))
                        timestamps.append(self.get_clock().now().to_msg())
                        
            except Exception as detection_error:
                self.get_logger().error(f"버튼 탐지 중 에러: {detection_error}")
                # 탐지 실패 시 더미값 사용
                for i, button_id in enumerate(request.button_ids):
                    xs.append(float(0.1 + i * 0.05))
                    ys.append(float(0.2 + i * 0.03))
                    depths.append(float(0.8 + i * 0.1))
                    is_pressed.append(bool(False))
                    timestamps.append(self.get_clock().now().to_msg())
                    
            response.success = True
            response.xs = xs
            response.ys = ys
            response.depths = depths
            response.is_pressed = is_pressed
            response.timestamp = timestamps
            
            self.get_logger().info(f"엘리베이터 버튼 상태 응답 완료: {len(xs)}개 버튼")
                
        except Exception as e:
            self.get_logger().error(f"버튼 상태 서비스 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
            response.xs = []
            response.ys = []
            response.depths = []
            response.is_pressed = []
            response.timestamp = []
        
        return response
    
    # 토픽 퍼블리시 메소드들
    
    def publish_tracking_event(self, robot_id: int, tracking_event_id: int, task_id: int = 1):
        """추적 이벤트 발행 (추적모드에서만 동작)"""
        try:
            if self.current_rear_mode_id != 2:
                current_mode = self.mode_names.get(self.current_rear_mode_id, "Unknown")
                self.get_logger().warning(f"추적 이벤트 발행 실패: 현재 모드가 '{current_mode}'입니다")
                return False
            
            msg = TrackingEvent()
            msg.robot_id = robot_id
            msg.tracking_event_id = tracking_event_id
            msg.task_id = task_id
            msg.timestamp = self.get_clock().now().to_msg()
            
            self.tracking_event_pub.publish(msg)
            
            event_names = {
                0: "slow_down",
                1: "maintain", 
                2: "lost",
                3: "resume"
            }
            event_name = event_names.get(tracking_event_id, f"unknown({tracking_event_id})")
            self.get_logger().info(f"추적 이벤트 발행: {event_name} (robot_id={robot_id}, task_id={task_id})")
            return True
            
        except Exception as e:
            self.get_logger().error(f"추적 이벤트 발행 에러: {e}")
            return False
    
    def publish_registered_event(self, robot_id: int):
        """추적 대상 등록 완료 이벤트 발행 (등록모드에서만 동작)"""
        try:
            if self.current_rear_mode_id != 1:
                current_mode = self.mode_names.get(self.current_rear_mode_id, "Unknown")
                self.get_logger().warning(f"등록 완료 이벤트 발행 실패: 현재 모드가 '{current_mode}'입니다")
                return False
            
            msg = Registered()
            msg.robot_id = robot_id
            msg.timestamp = self.get_clock().now().to_msg()
            
            self.registered_pub.publish(msg)
            self.get_logger().info(f"등록 완료 이벤트 발행: robot_id={robot_id}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"등록 완료 이벤트 발행 에러: {e}")
            return False
    
    def simulate_tracking_sequence(self, robot_id: int = 1, task_id: int = 1):
        """추적 시뮬레이션 시퀀스"""
        import threading
        import time
        
        def tracking_simulation():
            self.get_logger().info(f"추적 시뮬레이션 시작: robot_id={robot_id}")
            
            # 등록모드로 자동 전환
            old_mode_id = self.current_rear_mode_id
            old_mode_name = self.mode_names.get(old_mode_id, "Unknown")
            
            self.get_logger().info(f"자동 모드 전환: {old_mode_name} → 등록모드")
            self.current_rear_mode_id = 1
            
            time.sleep(1)
            
            # 등록 완료 이벤트 발행
            self.get_logger().info("[1/6] 등록 완료 이벤트 발행")
            if self.publish_registered_event(robot_id):
                self.get_logger().info("등록 완료")
            
            time.sleep(2)
            
            # 추적모드로 자동 전환
            self.get_logger().info("자동 모드 전환: 등록모드 → 추적모드")
            self.current_rear_mode_id = 2
            
            time.sleep(1)
            
            # 추적 시퀀스 실행
            tracking_events = [
                (1, "maintain - 정상 추적"),
                (0, "slow_down - 속도 감소 요청"),
                (1, "maintain - 추적 재개"),
                (2, "lost - 추적 대상 상실"),
                (3, "resume - 추적 복구")
            ]
            
            for i, (event_id, description) in enumerate(tracking_events):
                time.sleep(2)
                self.get_logger().info(f"[{i+2}/6] {description}")
                if self.publish_tracking_event(robot_id, event_id, task_id):
                    self.get_logger().info(f"추적 이벤트 발행 성공")
            
            # 원래 모드로 복원
            time.sleep(1)
            if old_mode_id != self.current_rear_mode_id:
                self.get_logger().info(f"모드 복원: 추적모드 → {old_mode_name}")
                self.current_rear_mode_id = old_mode_id
            
            self.get_logger().info("추적 시뮬레이션 완료")
        
        threading.Thread(target=tracking_simulation, daemon=True).start()
    
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
        
        # 전방 카메라 체크
        if hasattr(self, 'current_camera') and self.current_camera is not None:
            # 모든 전방 모드에서 웹캠과 뎁스를 별도 창으로 분리 (카메라는 항상 켜두기)
            if self.current_front_mode_id in [3, 4, 5, 6]:
                # 웹캠 창
                if self.current_front_mode_id == 3:
                    webcam_name = 'Front USB Webcam (Elevator Out)'
                elif self.current_front_mode_id == 4:
                    webcam_name = 'Front USB Webcam (Elevator In)'
                elif self.current_front_mode_id == 5:
                    webcam_name = 'Front USB Webcam (ArUco)'
                else:  # mode_id == 6
                    webcam_name = 'Front USB Webcam (Standby)'
                    
                active_cameras.append({
                    'camera': self.current_camera,
                    'depth_camera': None,
                    'name': webcam_name,
                    'mode_id': self.current_front_mode_id,
                    'type': 'front_webcam'
                })
                
                # 뎁스 카메라 창
                if hasattr(self, 'current_depth_camera') and self.current_depth_camera is not None:
                    if self.current_front_mode_id == 3:
                        depth_name = 'Front Depth Camera (Elevator Out)'
                    elif self.current_front_mode_id == 4:
                        depth_name = 'Front Depth Camera (Elevator In)'
                    elif self.current_front_mode_id == 5:
                        depth_name = 'Front Depth Camera (YOLO)'
                    else:  # mode_id == 6
                        depth_name = 'Front Depth Camera (Standby)'
                        
                    active_cameras.append({
                        'camera': self.current_depth_camera,
                        'depth_camera': self.current_depth_camera,
                        'name': depth_name,
                        'mode_id': self.current_front_mode_id,
                        'type': 'front_depth'
                    })
            else:
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
            else:
                self.current_rear_camera = None
                self.current_rear_camera_name = "None"
                self.get_logger().warning(f"⚠️ 후방 모드 {mode_name}용 카메라가 없습니다")
                
        except Exception as e:
            self.get_logger().error(f"후방 카메라 업데이트 에러: {e}")
    
    def elevator_width_callback(self, request, response):
        """엘리베이터 입구 너비 감지 처리"""
        try:
            self.get_logger().info(f"엘리베이터 너비 감지 요청: robot_id={request.robot_id}")
            
            dummy_left = -0.85
            dummy_right = 0.85
            
            response.robot_id = request.robot_id
            response.success = True
            response.left_boundary = float(dummy_left)
            response.right_boundary = float(dummy_right)
            
            self.get_logger().info(f"엘리베이터 너비: left={dummy_left:.3f}m, right={dummy_right:.3f}m")
                
        except Exception as e:
            self.get_logger().error(f"엘리베이터 너비 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
            response.left_boundary = 0.0
            response.right_boundary = 0.0
        
        return response
    
    def elevator_status_callback(self, request, response):
        """엘리베이터 위치 및 방향 감지 처리"""
        try:
            self.get_logger().info(f"엘리베이터 상태 감지 요청: robot_id={request.robot_id}")
            
            import random
            dummy_direction = random.choice([0, 1])
            dummy_position = random.choice([1, 2, 3])
            
            response.robot_id = request.robot_id
            response.success = True
            response.direction = dummy_direction
            response.position = dummy_position
            
            direction_str = "상행" if dummy_direction == 0 else "하행"
            self.get_logger().info(f"엘리베이터 상태: {direction_str}, {dummy_position}층")
                
        except Exception as e:
            self.get_logger().error(f"엘리베이터 상태 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
            response.direction = 0
            response.position = 1
        
        return response
    
    def door_status_callback(self, request, response):
        """문 열림 감지 처리"""
        try:
            self.get_logger().info(f"문 상태 감지 요청: robot_id={request.robot_id}")
            
            import random
            dummy_door_opened = random.choice([True, False])
            
            response.robot_id = request.robot_id
            response.success = True
            response.door_opened = dummy_door_opened
            
            door_str = "열림" if dummy_door_opened else "닫힘"
            self.get_logger().info(f"문 상태: {door_str}")
                
        except Exception as e:
            self.get_logger().error(f"문 상태 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
            response.door_opened = False
        
        return response
    
    def space_availability_callback(self, request, response):
        """엘리베이터 탑승/하차시 공간 확보 여부 감지 처리"""
        try:
            self.get_logger().info(f"공간 가용성 감지 요청: robot_id={request.robot_id}")
            
            import random
            dummy_space_available = random.choice([True, False])
            
            response.robot_id = request.robot_id
            response.success = True
            response.space_availability = dummy_space_available
            
            space_str = "확보됨" if dummy_space_available else "확보 안됨"
            self.get_logger().info(f"공간 가용성: {space_str}")
                
        except Exception as e:
            self.get_logger().error(f"공간 가용성 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
            response.space_availability = False
        
        return response
    
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
                    self.get_logger().info(f"현재 위치: {location_name} (ArUco 기반, 마지막 감지: {time_diff:.1f}초 전)")
                else:
                    self.get_logger().info(f"현재 위치: {location_name} (ArUco 기반, 초기값)")
                    
            else:  # 기타 모드 - 기본 위치 반환
                response.location_id = self.last_detected_location_id  # 마지막 알려진 위치 유지
                mode_name = self.get_active_mode_name()
                self.get_logger().info(f"위치 서비스: {mode_name}에서는 ArUco 사용 안함 (마지막 위치 유지)")
                
        except Exception as e:
            self.get_logger().error(f"위치 감지 에러: {e}")
            response.robot_id = request.robot_id
            response.success = False
            response.location_id = self.last_detected_location_id
        
        return response

    def _draw_objects_on_image(self, image: np.ndarray, objects: List[dict]) -> np.ndarray:
        """YOLO로 탐지된 객체들을 이미지에 시각화"""
        import cv2
        
        # 객체 타입별 색상 정의
        color_map = {
            'person': (255, 0, 255),      # 보라색
            'chair': (0, 255, 255),       # 노란색  
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
                
                # 객체별 색상 선택 (버튼은 눌림 상태에 따라)
                if class_name == 'button' and is_pressed:
                    color = (0, 0, 255)  # 빨간색 (눌린 버튼)
                else:
                    color = color_map.get(class_name, (128, 128, 128))  # 기본 회색
                
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                
                # 클래스 이름과 신뢰도 표시
                if class_name == 'button' and 'button_id' in obj:
                    button_id = obj['button_id']
                    recognition_method = obj.get('recognition_method', '')
                    if isinstance(button_id, int):
                        if button_id == 100:
                            label = f"하행버튼: {confidence:.2f}"
                        elif button_id == 101:
                            label = f"상행버튼: {confidence:.2f}"
                        elif button_id == 102:
                            label = f"열기버튼: {confidence:.2f}"
                        elif button_id == 103:
                            label = f"닫기버튼: {confidence:.2f}"
                        elif button_id == 13:
                            label = f"B1층: {confidence:.2f}"
                        elif button_id == 14:
                            label = f"B2층: {confidence:.2f}"
                        else:
                            label = f"{button_id}층: {confidence:.2f}"
                    else:
                        label = f"{button_id}: {confidence:.2f}"
                else:
                    label = f"{class_name}: {confidence:.2f}"
                    
                cv2.putText(image, label, (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # 모델 이름 표시 (작게)
                model_text = f"[{model_name}]"
                cv2.putText(image, model_text, (x1, y1-30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                
                # 거리 정보 표시
                if depth_mm > 0:
                    distance_text = f"{depth_mm}mm"
                    cv2.putText(image, distance_text, (center[0]-20, center[1]+30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # 버튼 눌림 상태 표시
                if class_name == 'button' and is_pressed:
                    pressed_text = "PRESSED"
                    cv2.putText(image, pressed_text, (center[0]-30, center[1]+50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        return image

    def _add_info_text(self, image: np.ndarray, objects: List[dict], custom_title: str = None):
        """다중 모델 탐지 결과 및 시스템 정보를 영상에 표시"""
        import cv2
        
        # 현재 모드 정보
        mode_name = self.get_active_mode_name()
        
        # 상단에 제목 (custom_title이 있으면 사용)
        if custom_title:
            cv2.putText(image, f"Roomie VS - {custom_title}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        else:
            cv2.putText(image, f"Roomie Vision System v3 - {mode_name}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        
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
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 탐지된 객체 수
        cv2.putText(image, f"Objects Detected: {len(objects)}", (10, 85), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
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
                cv2.putText(image, f"Objects: {counts_text}", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 255, 128), 1)
        
        # 눌린 버튼 표시
        pressed_buttons = []
        for obj in objects:
            if obj.get('is_pressed', False) and obj.get('class_name') == 'button':
                pressed_buttons.append("BUTTON")
        
        if pressed_buttons:
            pressed_text = f"Pressed: {len(pressed_buttons)} button(s)"
            cv2.putText(image, pressed_text, (10, 135), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
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
            cv2.putText(image, f"Current Location: {current_location_name}", (10, 210), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)
        
        # 종료 안내
        cv2.putText(image, "ESC:Exit, B:Info, M:Status, F:Flip, C:Conf, A:ArUco Test", (10, image.shape[0]-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    def __del__(self):
        """소멸자 - 멀티 카메라 시스템 정리"""
        if hasattr(self, 'camera_manager'):
            self.camera_manager.cleanup_all_cameras()

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
                        
                        # 이미지 좌우반전
                        if node.flip_horizontal:
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
                                    objects = node.model_detector.detect_objects(color_image, depth_image, node.confidence_threshold, mode_id)
                                elif mode_id == 5:  # 일반 모드: ArUco만 (이미 위에서 처리)
                                    pass
                                elif mode_id == 6:  # 대기 모드: 영상만
                                    pass
                            elif camera_type == 'front_depth':
                                if mode_id == 5:  # 일반 모드: 뎁스에 일반 YOLO
                                    objects = node.model_detector.detect_objects(color_image, depth_image, node.confidence_threshold, mode_id)
                                elif mode_id in [3, 4, 6]:  # 엘리베이터/대기 모드: 뎁스는 영상만
                                    pass
                            elif camera_type in ['rear', 'front']:
                                # 후방 카메라나 기타 전방 카메라: 영상만
                                pass
                            
                            display_image = color_image.copy()
                            if objects:
                                display_image = node._draw_objects_on_image(display_image, objects)
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
                        elif key == ord('r') or key == ord('R'):  # R키: 추적 시뮬레이션
                            node.get_logger().info("'R' 키 눌림 - 추적 시뮬레이션 시작")
                            node.simulate_tracking_sequence(robot_id=1, task_id=1)
                        elif key == ord('t') or key == ord('T'):  # T키: 단일 추적 이벤트
                            current_mode = node.get_active_mode_name()
                            node.get_logger().info(f"'T' 키 눌림 - 추적 이벤트 발행 시도 (현재: {current_mode})")
                            import random
                            event_id = random.choice([0, 1, 2, 3])
                            success = node.publish_tracking_event(robot_id=1, tracking_event_id=event_id, task_id=1)
                            if not success:
                                node.get_logger().info("추적 이벤트를 발행하려면 '1t' 명령으로 추적모드로 변경하세요")
                        elif key == ord('g') or key == ord('G'):  # G키: 등록 완료 이벤트
                            current_mode = node.get_active_mode_name()
                            node.get_logger().info(f"'G' 키 눌림 - 등록 완료 이벤트 발행 시도 (현재: {current_mode})")
                            success = node.publish_registered_event(robot_id=1)
                            if not success:
                                node.get_logger().info("등록 완료 이벤트를 발행하려면 '1r' 명령으로 등록모드로 변경하세요")
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
                                                button_name = "하행버튼"
                                            elif button_id == 101:
                                                button_name = "상행버튼"
                                            elif button_id == 102:
                                                button_name = "열기버튼"
                                            elif button_id == 103:
                                                button_name = "닫기버튼"
                                            elif button_id == 13:
                                                button_name = "B1층"
                                            elif button_id == 14:
                                                button_name = "B2층"
                                            else:
                                                button_name = f"{button_id}층"
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
                        elif key != 255 and key != -1:  # 다른 키가 눌렸을 때 (헤드리스 모드 제외)
                            if 32 <= key <= 126:
                                node.get_logger().info(f"'{chr(key)}' 키 눌림")
                                node.get_logger().info("사용 가능한 키:")
                                node.get_logger().info("   R(추적시뮬레이션), T(추적이벤트), G(등록완료)")
                                node.get_logger().info("   B(버튼정보), M(상태확인), A(ArUco테스트)")
                                node.get_logger().info("   F(좌우반전), C(신뢰도), ESC(종료)")
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