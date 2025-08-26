
import os
import cv2
import numpy as np
import threading
from typing import Optional, Tuple

# OpenNI2 모듈 임포트 시도
try:
    from primesense import openni2
    from primesense import _openni2 as c_api
except ImportError:
    print("경고: primesense 모듈을 찾을 수 없습니다. OpenNI2 카메라를 사용할 수 없습니다.")
    openni2 = None

def setup_openni2_environment():
    """OpenNI2 실행에 필요한 환경 변수를 설정합니다."""
    # 다양한 경로에서 OpenNI2 SDK 위치를 탐색
    downloads_openni_path = os.path.expanduser("~/Downloads/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux/ros2_astra_camera/astra_camera/openni2_redist/x64")
    project_openni_path = os.path.expanduser("~/project_ws/Roomie/ros2_ws/src/roomie_vs/OpenNI_SDK_ROS2_v1.0.2_20220809_b32e47_linux/ros2_astra_camera/astra_camera/openni2_redist/x64")
    
    openni_path = None
    if os.path.exists(downloads_openni_path):
        openni_path = downloads_openni_path
        print(f"정보: OpenNI2 경로를 'Downloads'에서 찾았습니다: {openni_path}")
    elif os.path.exists(project_openni_path):
        openni_path = project_openni_path
        print(f"정보: OpenNI2 경로를 프로젝트 폴더에서 찾았습니다: {openni_path}")
    else:
        print("오류: OpenNI2 SDK 경로를 찾을 수 없습니다.")
        print(f"  - 확인한 경로 1: {downloads_openni_path}")
        print(f"  - 확인한 경로 2: {project_openni_path}")
        return False
    
    # 환경 변수 설정
    os.environ['OPENNI2_REDIST'] = openni_path
    os.environ['LD_LIBRARY_PATH'] = f"{os.environ.get('LD_LIBRARY_PATH', '')}:{openni_path}"
    
    # 사용자 라이브러리 경로를 PYTHONPATH에 추가
    user_lib_path = "/home/jinhyuk2me/.local/lib/python3.12/site-packages"
    os.environ['PYTHONPATH'] = f"{os.environ.get('PYTHONPATH', '')}:{user_lib_path}"
    
    print("정보: OpenNI2 환경 변수 설정이 완료되었습니다.")
    return True

class OpenNI2Camera:
    """OpenNI2 SDK를 직접 사용하여 Astra 깊이 카메라를 제어하는 클래스입니다."""
    
    def __init__(self, logger):
        self.logger = logger
        self.is_running = False
        self.device = None
        self.rgb_stream = None
        self.depth_stream = None
        
        # Astra 카메라의 추정된 내부 파라미터
        self.depth_fx = 1140.6
        self.depth_fy = 1140.6
        self.depth_cx = 320.0
        self.depth_cy = 240.0
        
        self.current_depth = None
        self.current_color = None
        self.frame_lock = threading.Lock()
        
    def initialize(self) -> bool:
        """OpenNI2를 통해 카메라를 초기화합니다."""
        if openni2 is None:
            self.logger.error("OpenNI2 모듈이 없어 카메라를 초기화할 수 없습니다.")
            return False
        try:
            self.logger.info("OpenNI2 카메라 초기화를 시작합니다...")
            openni2.initialize()
            
            self.device = openni2.Device.open_any()
            device_info = self.device.get_device_info()
            self.logger.info(f"카메라 장치 '{device_info.name.decode()}' ({device_info.vendor.decode()})'를 열었습니다.")
            
            # RGB 스트림 생성 시도
            try:
                self.rgb_stream = self.device.create_color_stream()
                self.rgb_stream.start()
                video_mode = self.rgb_stream.get_video_mode()
                self.logger.info(f"RGB 스트림을 시작합니다: {video_mode.resolutionX}x{video_mode.resolutionY} @ {video_mode.fps}fps")
            except Exception as e:
                self.logger.warning(f"RGB 스트림을 생성하지 못했습니다: {e}")
                self.rgb_stream = None
            
            # 깊이 스트림 생성 시도
            try:
                self.depth_stream = self.device.create_depth_stream()
                self.depth_stream.start()
                video_mode = self.depth_stream.get_video_mode()
                self.logger.info(f"깊이 스트림을 시작합니다: {video_mode.resolutionX}x{video_mode.resolutionY} @ {video_mode.fps}fps")
            except Exception as e:
                self.logger.warning(f"깊이 스트림을 생성하지 못했습니다: {e}")
                self.depth_stream = None
            
            if not self.rgb_stream and not self.depth_stream:
                self.logger.error("RGB와 깊이 스트림 모두 생성에 실패하여 초기화를 중단합니다.")
                return False
            
            self.is_running = True
            self.logger.info("OpenNI2 카메라가 성공적으로 초기화되었습니다.")
            return True
            
        except Exception as e:
            self.logger.error(f"OpenNI2 카메라 초기화 중 예외가 발생했습니다: {e}")
            return False
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """카메라에서 RGB와 깊이 프레임을 가져옵니다."""
        if not self.is_running:
            raise RuntimeError("카메라가 실행 중이 아닐 때 프레임을 요청할 수 없습니다.")
        
        try:
            depth_image, color_image = None, None
            
            if self.rgb_stream:
                try:
                    rgb_frame = self.rgb_stream.read_frame()
                    rgb_data = rgb_frame.get_buffer_as_uint8()
                    rgb_array = np.frombuffer(rgb_data, dtype=np.uint8)
                    color_image = rgb_array.reshape((rgb_frame.height, rgb_frame.width, 3))
                    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                except Exception as e:
                    self.logger.warning(f"RGB 프레임을 읽는 데 실패했습니다: {e}")
            
            if self.depth_stream:
                try:
                    depth_frame = self.depth_stream.read_frame()
                    depth_data = depth_frame.get_buffer_as_uint16()
                    depth_array = np.frombuffer(depth_data, dtype=np.uint16)
                    depth_image = depth_array.reshape((depth_frame.height, depth_frame.width))
                except Exception as e:
                    self.logger.warning(f"깊이 프레임을 읽는 데 실패했습니다: {e}")
            
            with self.frame_lock:
                self.current_depth = depth_image.copy() if depth_image is not None else None
                self.current_color = color_image.copy() if color_image is not None else None
            
            return depth_image, color_image
            
        except Exception as e:
            self.logger.error(f"프레임 획득 중 예외가 발생했습니다: {e}")
            raise RuntimeError(f"카메라 프레임 획득에 실패했습니다: {e}")
    
    def pixel_to_3d(self, u: int, v: int, depth_mm: int, is_flipped: bool = False) -> Tuple[float, float, float]:
        """2D 픽셀 좌표를 3D 월드 좌표로 변환합니다."""
        if depth_mm <= 0:
            return 0.0, 0.0, 0.0
        
        if is_flipped:
            u = int(self.depth_cx * 2) - u
            
        z = 1000.0 / depth_mm
        x = (u - self.depth_cx) * z / self.depth_fx
        y = (v - self.depth_cy) * z / self.depth_fy
        
        return x, y, z
    
    def cleanup(self):
        """카메라 리소스를 정리하고 해제합니다."""
        self.is_running = False
        try:
            if self.rgb_stream: self.rgb_stream.stop()
            if self.depth_stream: self.depth_stream.stop()
            if self.device: self.device.close()
            if openni2: openni2.unload()
            self.logger.info("OpenNI2 카메라 리소스가 성공적으로 정리되었습니다.")
        except Exception as e:
            self.logger.warning(f"카메라 리소스 정리 중 오류가 발생했습니다: {e}")

class WebCamCamera:
    """자동 탐지 기능을 포함한 일반 웹캠 제어 클래스입니다."""
    
    def __init__(self, logger, camera_id=None, camera_ids_to_try=None, camera_name="Webcam"):
        self.logger = logger
        self.preferred_camera_id = camera_id
        self.camera_ids_to_try = camera_ids_to_try or [0, 1, 2, 3]
        self.camera_name = camera_name
        self.actual_camera_id = None
        self.is_running = False
        self.cap = None
        
        self.current_color = None
        self.current_depth = None
        self.frame_lock = threading.Lock()
        
    def initialize(self) -> bool:
        """사용 가능한 웹캠을 자동으로 탐지하고 초기화합니다."""
        try:
            if self.preferred_camera_id is not None and self._try_camera_id(self.preferred_camera_id):
                return True
            
            if "ABKO" in self.camera_name:
                self.logger.info(f"'{self.camera_name}'는 전용 연결 로직을 사용합니다.")
                return self._try_abko_camera_directly()

            self.logger.info(f"'{self.camera_name}' 자동 탐지를 시작합니다... (시도할 ID: {self.camera_ids_to_try})")
            available_cameras = self._scan_available_cameras()
            
            if not available_cameras:
                self.logger.error(f"'{self.camera_name}' 자동 탐지 실패: 사용 가능한 카메라가 없습니다.")
                return False
            
            selected_camera = self._select_appropriate_camera(available_cameras)
            if selected_camera and self._try_camera_id(selected_camera['id']):
                return True
            
            self.logger.error(f"'{self.camera_name}' 자동 탐지 실패: 적절한 카메라를 찾거나 연결할 수 없습니다.")
            return False
            
        except Exception as e:
            self.logger.error(f"'{self.camera_name}' 초기화 중 예외가 발생했습니다: {e}")
            return False
    
    def _scan_available_cameras(self) -> list:
        """시스템에 연결된 모든 카메라를 스캔하여 정보를 수집합니다."""
        available_cameras = []
        for camera_id in self.camera_ids_to_try:
            if self.preferred_camera_id is not None and camera_id == self.preferred_camera_id:
                continue
            
            if "Front" in self.camera_name and camera_id == 0:
                self.logger.debug(f"ID={camera_id}는 후방 카메라용으로 예약되어 있어 건너뜁니다.")
                continue
                
            try:
                cap = cv2.VideoCapture(camera_id)
                if not cap.isOpened():
                    continue
                
                ret, frame = cap.read()
                if not ret or frame is None:
                    cap.release()
                    continue
                
                device_name = self._get_camera_device_name(camera_id)
                camera_info = {
                    'id': camera_id,
                    'backend': cap.getBackendName(),
                    'width': frame.shape[1],
                    'height': frame.shape[0],
                    'device_name': device_name
                }
                available_cameras.append(camera_info)
                self.logger.info(f"카메라 발견: ID={camera_id}, {camera_info['width']}x{camera_info['height']}, 백엔드={camera_info['backend']}, 장치명='{device_name}'")
                cap.release()
            except Exception as e:
                self.logger.debug(f"ID={camera_id} 스캔 중 오류 발생: {e}")
        return available_cameras
    
    def _get_camera_device_name(self, camera_id: int) -> str:
        """v4l2-ctl 명령어를 사용하여 카메라의 장치명을 가져옵니다."""
        try:
            import subprocess
            device_path = f"/dev/video{camera_id}"
            result = subprocess.run(
                ['v4l2-ctl', '--device', device_path, '--info'],
                capture_output=True, text=True, timeout=3
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'Card type' in line or 'Device name' in line:
                        return line.split(':', 1)[1].strip()
            return f"알 수 없음 (ID={camera_id})"
        except Exception as e:
            self.logger.debug(f"카메라 장치명을 가져오는 데 실패했습니다 (ID={camera_id}): {e}")
            return f"알 수 없음 (ID={camera_id})"
    
    def _select_appropriate_camera(self, available_cameras: list) -> dict:
        """수집된 장치 정보를 바탕으로 현재 역할에 가장 적합한 카메라를 선택합니다."""
        if not available_cameras:
            return None
        
        if "USB" in self.camera_name:
            self.logger.info("전방 USB 웹캠(HCAM01N) 선택 로직을 시작합니다.")
            for camera in available_cameras:
                if 'hcam01n' in camera['device_name'].lower():
                    self.logger.info(f"HCAM01N 전방 카메라를 선택했습니다: ID={camera['id']}, 장치명='{camera['device_name']}'")
                    return camera
            
            allowed_keywords = ['c920', 'c922', 'c930', 'logitech']
            for camera in available_cameras:
                device_name_lower = camera['device_name'].lower()
                if 'abko' in device_name_lower or device_name_lower.startswith('hd webcam'):
                    continue
                if any(keyword in device_name_lower for keyword in allowed_keywords):
                    self.logger.info(f"허용된 외부 USB 웹캠을 선택했습니다: ID={camera['id']}, 장치명='{camera['device_name']}'")
                    return camera
            
            raise RuntimeError("적합한 전방 카메라(HCAM01N)를 찾을 수 없습니다.")
        
        elif "ABKO" in self.camera_name:
            self.logger.info("후방 ABKO 카메라 선택 로직을 시작합니다.")
            abko_keywords = ['abko apc930', 'abko ap', 'apc930', 'abko']
            for camera in available_cameras:
                if any(keyword in camera['device_name'].lower() for keyword in abko_keywords):
                    self.logger.info(f"ABKO 후방 카메라를 선택했습니다: ID={camera['id']}, 장치명='{camera['device_name']}'")
                    return camera
            raise RuntimeError("후방 ABKO 카메라를 찾을 수 없습니다.")
        
        return available_cameras[0]
    
    def _get_optimal_resolution(self, camera_id: int) -> tuple:
        """카메라 장치명에 따라 최적의 해상도를 반환합니다."""
        try:
            device_name = self._get_camera_device_name(camera_id).lower()
            if 'hcam01n' in device_name:
                self.logger.info("HCAM01N에 최적화된 해상도(800x600)를 설정합니다.")
                return (800, 600)
            self.logger.info(f"기본 해상도(640x480)를 설정합니다 (ID={camera_id}).")
            return (640, 480)
        except Exception as e:
            self.logger.warning(f"해상도 설정 중 오류가 발생하여 기본값을 사용합니다: {e}")
            return (640, 480)
    
    def _try_camera_id(self, camera_id: int) -> bool:
        """주어진 ID로 웹캠 초기화를 시도합니다."""
        try:
            self.logger.info(f"'{self.camera_name}' (ID={camera_id}) 연결을 시도합니다...")
            cap = cv2.VideoCapture(camera_id)
            if not cap.isOpened():
                return False
            
            width, height = self._get_optimal_resolution(camera_id)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            ret, frame = cap.read()
            if not ret or frame is None:
                cap.release()
                return False
            
            self.cap = cap
            self.actual_camera_id = camera_id
            self.is_running = True
            actual_height, actual_width = frame.shape[:2]
            
            self.logger.info(f"'{self.camera_name}' 초기화 성공: ID={camera_id}, {actual_width}x{actual_height}, 백엔드={cap.getBackendName()}")
            if actual_width != width or actual_height != height:
                self.logger.warning(f"요청된 해상도({width}x{height})와 실제 해상도({actual_width}x{actual_height})가 다릅니다.")
            
            return True
        except Exception as e:
            self.logger.debug(f"ID={camera_id} 연결 시도 중 오류 발생: {e}")
            return False
    
    def _try_camera_id_with_formats(self, camera_id: int) -> bool:
        """ABKO 카메라에 대해 다양한 포맷과 해상도로 연결을 시도합니다."""
        formats_to_try = [(640, 480), (1280, 720), (800, 600)]
        for width, height in formats_to_try:
            try:
                self.logger.info(f"ABKO 카메라(ID={camera_id})에 {width}x{height} 해상도로 연결을 시도합니다.")
                cap = cv2.VideoCapture(camera_id)
                if not cap.isOpened(): continue
                
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                
                import time; time.sleep(0.1)
                
                for _ in range(3):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.cap = cap
                        self.actual_camera_id = camera_id
                        self.is_running = True
                        self.logger.info(f"ABKO 카메라 강제 연결 성공: ID={camera_id}, {frame.shape[1]}x{frame.shape[0]}, 백엔드={cap.getBackendName()}")
                        return True
                    time.sleep(0.05)
                cap.release()
            except Exception as e:
                self.logger.debug(f"ABKO 카메라(ID={camera_id}, {width}x{height}) 연결 시도 중 오류: {e}")
        
        self.logger.warning(f"ABKO 카메라(ID={camera_id})에 대한 모든 포맷 연결 시도에 실패했습니다.")
        return False
    
    def _try_abko_camera_directly(self) -> bool:
        """ID 0과 1에 대해 ABKO 카메라 직접 연결을 시도합니다."""
        self.logger.info("후방 ABKO 카메라(ID 0 또는 1)에 직접 연결을 시도합니다.")
        if self._try_camera_id_with_formats(0) or self._try_camera_id_with_formats(1):
            return True
        self.logger.error("후방 ABKO 카메라(ID 0, 1) 연결에 모두 실패했습니다.")
        return False
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """웹캠에서 컬러 프레임을 가져옵니다 (깊이 프레임은 항상 None)."""
        if not self.is_running or self.cap is None:
            raise RuntimeError("웹캠이 실행 중이 아닐 때 프레임을 요청할 수 없습니다.")
        
        try:
            ret, color_image = self.cap.read()
            if not ret:
                self.logger.warning("웹캠에서 프레임을 읽지 못했습니다.")
                return None, None
            
            with self.frame_lock:
                self.current_color = color_image.copy()
            
            return None, color_image
        except Exception as e:
            self.logger.error(f"웹캠 프레임 획득 중 예외가 발생했습니다: {e}")
            raise RuntimeError(f"웹캠 프레임 획득에 실패했습니다: {e}")
    
    def cleanup(self):
        """웹캠 리소스를 정리하고 해제합니다."""
        self.is_running = False
        try:
            if self.cap:
                self.cap.release()
                self.cap = None
            self.logger.info(f"'{self.camera_name}'(ID={self.actual_camera_id}) 리소스가 정리되었습니다.")
        except Exception as e:
            self.logger.warning(f"웹캠 리소스 정리 중 오류가 발생했습니다: {e}")

class MultiCameraManager:
    """여러 종류의 카메라(깊이, 웹캠)를 통합 관리하고, 서비스 모드에 따라 동적으로 초기화합니다."""
    
    def __init__(self, logger):
        self.logger = logger
        
        front_cam_id = int(os.getenv('FRONT_CAMERA_ID')) if os.getenv('FRONT_CAMERA_ID') else None
        rear_cam_id = int(os.getenv('REAR_CAMERA_ID')) if os.getenv('REAR_CAMERA_ID') else None
        
        self.front_webcam = WebCamCamera(logger, camera_id=front_cam_id or 2, camera_ids_to_try=[2, 3], camera_name="Front HCAM01N Webcam")
        self.front_depth = OpenNI2Camera(logger)
        self.rear_webcam = WebCamCamera(logger, camera_id=rear_cam_id or 0, camera_ids_to_try=[0, 1, 2, 3, 4, 5], camera_name="Rear ABKO Camera")
        
        self.front_webcam_initialized = False
        self.front_depth_initialized = False
        self.rear_webcam_initialized = False
        
    def get_required_cameras_for_mode(self, mode_id: int) -> list:
        """서비스 모드별로 필요한 카메라 장치 목록을 반환합니다."""
        camera_requirements = {
            0: ['rear_webcam'], 1: ['rear_webcam'], 2: ['rear_webcam'],
            3: ['front_webcam', 'front_depth'], 4: ['front_webcam', 'front_depth'],
            5: ['front_webcam', 'front_depth'], 6: ['front_webcam', 'front_depth'],
            100: [], 101: [], 102: [], 103: [], 104: []
        }
        return camera_requirements.get(mode_id, [])
    
    def initialize_cameras_for_mode(self, mode_id: int) -> bool:
        """주어진 서비스 모드에 필요한 카메라들을 초기화합니다."""
        required_cameras = self.get_required_cameras_for_mode(mode_id)
        self.logger.info(f"모드 {mode_id}에 필요한 카메라: {required_cameras}")
        
        if not required_cameras:
            self.logger.info(f"모드 {mode_id}는 시뮬레이션 모드로, 카메라 초기화가 필요 없습니다.")
            return True

        success = True
        initialized_cameras = []
        
        if 'rear_webcam' in required_cameras and not self.rear_webcam_initialized:
            if self._initialize_rear_camera():
                initialized_cameras.append('후방 웹캠')
            else:
                success = False
                    
        if 'front_webcam' in required_cameras and not self.front_webcam_initialized:
            if self._initialize_front_webcam():
                initialized_cameras.append('전방 웹캠')
            else:
                success = False
                    
        if 'front_depth' in required_cameras and not self.front_depth_initialized:
            if self._initialize_front_depth():
                initialized_cameras.append('전방 뎁스')
            else:
                success = False
        
        if initialized_cameras:
            self.logger.info(f"모드 {mode_id}를 위해 다음 카메라를 새로 초기화했습니다: {', '.join(initialized_cameras)}")
        else:
            self.logger.info(f"모드 {mode_id}에 필요한 카메라는 이미 모두 초기화되어 있습니다.")
            
        return success
    
    def get_camera_for_mode(self, mode_id: int) -> Tuple[Optional[object], Optional[object], str]:
        """서비스 모드에 맞는 활성화된 카메라 객체를 반환합니다."""
        if mode_id in [0, 1, 2]:  # 후방 카메라 모드
            if self.rear_webcam_initialized:
                name = "Rear Webcam" + (" (Standby)" if mode_id == 0 else "")
                return self.rear_webcam, None, name
            else:
                self.logger.warning(f"후방 웹캠이 초기화되지 않아 모드 {mode_id}를 실행할 수 없습니다.")
                return None, None, "None"
                
        elif mode_id in [3, 4, 5, 6]:  # 전방 카메라 모드
            cam, depth_cam = None, None
            name_parts = []
            if self.front_webcam_initialized:
                cam = self.front_webcam
                name_parts.append("Front Webcam")
            if self.front_depth_initialized:
                depth_cam = self.front_depth
                if not cam: cam = depth_cam # 웹캠 없으면 뎁스캠이 메인
                name_parts.append("Depth")

            if not name_parts:
                self.logger.warning("전방 카메라가 초기화되지 않아 모드를 실행할 수 없습니다.")
                return None, None, "None"

            name = " + ".join(name_parts)
            if mode_id == 3: name += " (Elevator Out)"
            elif mode_id == 4: name += " (Elevator In)"
            elif mode_id == 6: name += " (Standby)"
            return cam, depth_cam, name
        
        else: # 시뮬레이션 모드
            return None, None, "Simulation Mode"
    
    def _initialize_front_webcam(self) -> bool:
        """전방 웹캠만 개별적으로 초기화합니다."""
        try:
            if self.front_webcam.initialize():
                self.front_webcam_initialized = True
                self.logger.info("전방 웹캠이 성공적으로 초기화되었습니다.")
                return True
            else:
                self.logger.warning("전방 웹캠 초기화에 실패했습니다.")
                return False
        except Exception as e:
            self.logger.error(f"전방 웹캠 초기화 중 예외가 발생했습니다: {e}")
            return False
    
    def _initialize_front_depth(self) -> bool:
        """전방 깊이 카메라만 개별적으로 초기화합니다."""
        try:
            if self.front_depth.initialize():
                self.front_depth_initialized = True
                self.logger.info("전방 깊이 카메라가 성공적으로 초기화되었습니다.")
                return True
            else:
                self.logger.warning("전방 깊이 카메라 초기화에 실패했습니다.")
                return False
        except Exception as e:
            self.logger.error(f"전방 깊이 카메라 초기화 중 예외가 발생했습니다: {e}")
            return False

    def _initialize_rear_camera(self) -> bool:
        """후방 웹캠만 개별적으로 초기화합니다."""
        try:
            if self.rear_webcam.initialize():
                self.rear_webcam_initialized = True
                self.logger.info("후방 웹캠이 성공적으로 초기화되었습니다.")
                return True
            else:
                self.logger.warning("후방 웹캠 초기화에 실패했습니다.")
                return False
        except Exception as e:
            self.logger.error(f"후방 웹캠 초기화 중 예외가 발생했습니다: {e}")
            return False

    def cleanup_all_cameras(self):
        """초기화된 모든 카메라의 리소스를 정리하고 해제합니다."""
        self.logger.info("모든 카메라 리소스 정리를 시작합니다...")
        try:
            if self.front_webcam_initialized: self.front_webcam.cleanup()
            if self.front_depth_initialized: self.front_depth.cleanup()
            if self.rear_webcam_initialized: self.rear_webcam.cleanup()
        except Exception as e:
            self.logger.error(f"카메라 리소스 정리 중 예외가 발생했습니다: {e}")
        self.logger.info("모든 카메라 리소스 정리가 완료되었습니다.")
