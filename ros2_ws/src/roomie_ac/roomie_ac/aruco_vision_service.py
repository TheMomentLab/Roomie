# aruco_vision_service.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from roomie_msgs.srv import ButtonStatus, ButtonStatus2
import time 
import cv2
import cv2.aruco as aruco
import numpy as np
import threading
from . import config # config.py 파일을 import 합니다.

class ArucoVisionService(Node):
    """
    카메라로부터 ArUco 마커를 감지하고, 해당 정보를 서비스 요청에 따라 제공하는 ROS 2 노드.
    기존 YoloVisionService를 ArUco 기반으로 대체합니다.
    """
    def __init__(self):
        super().__init__('aruco_vision_service')

        # --- 콜백 그룹 및 서비스 서버 생성 (기존과 동일) ---
        self.callback_group = ReentrantCallbackGroup()
        self.normal_srv = self.create_service(
            ButtonStatus, '/vs/command/button_status',
            self.handle_request_normal, callback_group=self.callback_group
        )
        self.corner_srv = self.create_service(
            ButtonStatus2, '/vs/command/button_status2',
            self.handle_request_corner, callback_group=self.callback_group
        )

        # --- 카메라 및 ArUco 초기화 (기존과 동일) ---
        self.cap = cv2.VideoCapture(config.CAMERA_DEVICE_ID)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"카메라 {config.CAMERA_DEVICE_ID}번을 열 수 없습니다.")
            rclpy.shutdown(); return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.IMAGE_WIDTH_PX)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.IMAGE_HEIGHT_PX)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detected_markers = {}
        self.status_lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'aruco_annotated_image', 10)

        # --- [핵심 수정] 카메라 읽기 스레드 분리 ---
        # 1. 스레드 간 공유할 변수 초기화
        self.latest_frame = None
        self.is_running = True # 스레드의 안전한 종료를 위한 플래그

        # 2. 카메라 프레임을 읽어오는 전용 스레드 생성 및 시작
        self.camera_thread = threading.Thread(target=self._camera_reader_loop, daemon=True)
        self.camera_thread.start()
        self.get_logger().info("📷 독립된 카메라 리더 스레드 시작.")

        # 3. ROS 타이머는 이제 프레임 '처리'만 담당
        self.timer = self.create_timer(0.1, self.detect_and_publish_callback, callback_group=self.callback_group)
        self.get_logger().info("🟢 ArUco Vision Service 활성화 (카메라 스레드 분리).")

    def _camera_reader_loop(self):
        """
        [신규] ROS 2 Executor와 완전히 분리된 독립 스레드에서 실행됩니다.
        카메라에서 프레임을 읽어오는 블로킹(blocking) 작업을 전담합니다.
        """
        self.get_logger().info("카메라 리더 루프 시작.")
        while rclpy.ok() and self.is_running:
            ret, frame = self.cap.read()
            if ret:
                # 잠금을 사용하여 공유 변수에 안전하게 최신 프레임을 씁니다.
                with self.status_lock:
                    self.latest_frame = frame
            else:
                # 로그가 너무 많이 출력되지 않도록 5초에 한 번만 경고를 표시합니다.
                self.get_logger().warn("⚠️ 카메라 리더 스레드: 프레임 읽기 실패.", throttle_duration_sec=5)
            # CPU 사용량을 과도하게 점유하지 않도록 약간의 지연을 줍니다.
            time.sleep(0.01)
        self.get_logger().info("카메라 리더 루프 종료.")

    def detect_and_publish_callback(self):
        """
        [수정] 이제 이 함수는 블로킹되는 self.cap.read()를 호출하지 않습니다.
        카메라 스레드가 준비한 최신 프레임을 가져와 처리만 합니다.
        """
        current_frame = None
        # 잠금을 사용하여 공유 변수에서 안전하게 최신 프레임을 읽어옵니다.
        with self.status_lock:
            if self.latest_frame is not None:
                current_frame = self.latest_frame.copy()

        # 처리할 프레임이 없으면 즉시 반환하여 다른 작업에 영향을 주지 않습니다.
        if current_frame is None:
            self.get_logger().warn("⚠️ 감지 콜백: 처리할 프레임이 없습니다.", throttle_duration_sec=5)
            return

        # --- 이후의 ArUco 감지 및 발행 로직은 기존과 동일 ---
        # 변수 이름만 'frame'에서 'current_frame'으로 변경합니다.
        corners, ids, rejected_img_points = aruco.detectMarkers(current_frame, self.aruco_dict, parameters=self.aruco_params)
        
        annotated_frame = current_frame.copy()
        current_markers = {}

        if ids is not None:
            aruco.drawDetectedMarkers(annotated_frame, corners, ids)

            for i, marker_id in enumerate(ids):
                marker_id_int = int(marker_id[0])
                marker_corners = corners[i].reshape((4, 2))
                
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])
                pixel_area = cv2.contourArea(marker_corners)

                norm_x = center_x / config.IMAGE_WIDTH_PX
                norm_y = center_y / config.IMAGE_HEIGHT_PX
                norm_size = pixel_area / float(config.IMAGE_WIDTH_PX * config.IMAGE_HEIGHT_PX)

                current_markers[marker_id_int] = {
                    "button_id": marker_id_int,
                    "x": norm_x,
                    "y": norm_y,
                    "size": norm_size,
                    # [수정] 모서리점 원본 데이터(Pixel 좌표)도 저장
                    "corners": marker_corners.flatten().tolist(), # [x1,y1,x2,y2...]
                    "is_pressed": False,
                    "timestamp": self.get_clock().now().to_msg()
                }

                cv2.circle(annotated_frame, (int(center_x), int(center_y)), 4, (0, 0, 255), -1)
                cv2.putText(annotated_frame, f"ID: {marker_id_int}",
                            (int(marker_corners[0][0]), int(marker_corners[0][1]) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 멀티스레드 충돌 방지를 위해 잠금을 사용하여 공유 데이터 업데이트
        with self.status_lock:
            self.detected_markers = current_markers

        # 처리된 이미지를 ROS 토픽으로 발행
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")

    # handle_request_normal 함수
    def handle_request_normal(self, request, response):
        """[수정] 일반 모드(ButtonStatus) 요청을 처리하는 핸들러"""
        self.get_logger().info(f"--- 🔮 일반 모드 요청: id={request.button_id} ---")
        with self.status_lock:
            marker_data = self.detected_markers.get(request.button_id)
            if marker_data:
                response.success = True
                response.x = float(marker_data["x"])
                response.y = float(marker_data["y"])
                response.size = float(marker_data["size"])
                response.is_pressed = marker_data["is_pressed"]
            else:
                response.success = False
        response.robot_id = request.robot_id
        response.button_id = request.button_id
        response.timestamp = self.get_clock().now().to_msg()
        return response

    # handle_request_corner 함수 (신규)
    def handle_request_corner(self, request, response):
        """[신규] 모서리 모드(ButtonStatus2) 요청을 처리하는 핸들러"""
        self.get_logger().info(f"--- 🔮 모서리 모드 요청: id={request.button_id} ---")
        with self.status_lock:
            marker_data = self.detected_markers.get(request.button_id)
            if marker_data:
                response.success = True
                response.x = float(marker_data["x"])
                response.y = float(marker_data["y"])
                response.size = float(marker_data["size"])
                # [핵심] corners 필드를 채워줍니다.
                response.corners = [float(c) for c in marker_data["corners"]]
                response.is_pressed = marker_data["is_pressed"]
            else:
                response.success = False
        response.robot_id = request.robot_id
        response.button_id = request.button_id
        response.timestamp = self.get_clock().now().to_msg()
        return response

    def destroy_node(self):
        """[수정] 노드 종료 시 카메라 스레드를 안전하게 종료시킵니다."""
        self.get_logger().info("노드 종료 중... 카메라 스레드 및 리소스를 해제합니다.")
        # 1. 카메라 스레드 루프를 중지하도록 플래그 설정
        self.is_running = False
        # 2. 스레드가 완전히 종료될 때까지 최대 1초간 기다림 (권장)
        if hasattr(self, 'camera_thread') and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
        # 3. 카메라 리소스 해제
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArucoVisionService()
        # 멀티스레드 Executor를 사용하여 서비스 콜백과 타이머 콜백이 병렬로 처리되도록 함
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            if rclpy.ok():
                executor.shutdown()
                node.destroy_node()
    except Exception as e:
        print(f"노드 실행 중 심각한 오류 발생: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()