# aruco_vision_service.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from roomie_msgs.srv import ButtonStatus

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

        # --- 콜백 그룹 설정 (기존과 동일하게 유지하여 안정성 확보) ---
        # 서비스 요청 처리를 위한 ReentrantCallbackGroup
        self.service_callback_group = ReentrantCallbackGroup()
        # 주기적인 이미지 처리를 위한 MutuallyExclusiveCallbackGroup
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # --- 서비스 서버 생성 ---
        # ButtonStatus 서비스 타입을 사용하여 '/vs/command/button_status' 이름으로 서비스를 생성합니다.
        self.srv = self.create_service(
            ButtonStatus,
            '/vs/command/button_status',
            self.handle_request,
            callback_group=self.service_callback_group
        )

        # --- 카메라 및 ArUco 초기화 ---
        self.cap = cv2.VideoCapture(config.CAMERA_DEVICE_ID)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"카메라 {config.CAMERA_DEVICE_ID}번을 열 수 없습니다.")
            rclpy.shutdown()
            return

        # 카메라 해상도 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.IMAGE_WIDTH_PX)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.IMAGE_HEIGHT_PX)

        # ArUco 사전(Dictionary) 및 검출기 파라미터 설정
        # DICT_4X4_50: 4x4 크기의 마커를 사용하며, 총 50개의 고유 ID를 가집니다.
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

        # --- 상태 변수 및 동기화 잠금 ---
        # 감지된 마커 정보를 저장할 딕셔너리 {marker_id: {data}}
        self.detected_markers = {}
        # 멀티스레드 환경에서 self.detected_markers에 안전하게 접근하기 위한 잠금
        self.status_lock = threading.Lock()

        # --- 시각화 및 퍼블리싱 설정 ---
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'aruco_annotated_image', 10)

        # 0.1초마다 detect_and_publish_callback 함수를 실행하는 타이머 생성
        self.timer = self.create_timer(0.1, self.detect_and_publish_callback, callback_group=self.timer_callback_group)

        self.get_logger().info("🟢 ArUco Vision Service가 성공적으로 활성화되었습니다.")
        self.get_logger().info("서비스 '/vs/command/button_status'와 토픽 '/aruco_annotated_image'가 준비되었습니다.")

    def detect_and_publish_callback(self):
        """
        주기적으로 카메라에서 프레임을 읽어 ArUco 마커를 감지하고,
        결과를 시각화하여 토픽으로 퍼블리시하며, 감지된 정보를 내부 상태 변수에 저장합니다.
        """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ 카메라 프레임을 읽어오는 데 실패했습니다.")
            return

        # ArUco 마커 검출
        corners, ids, rejected_img_points = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        # 시각화를 위해 원본 프레임 복사
        annotated_frame = frame.copy()
        
        # 감지된 마커 정보를 저장할 새로운 딕셔너리
        current_markers = {}

        if ids is not None:
            aruco.drawDetectedMarkers(annotated_frame, corners, ids)

            for i, marker_id in enumerate(ids):
                marker_id_int = int(marker_id[0])
                marker_corners = corners[i].reshape((4, 2))
                
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])

                # ======================= [수정된 부분 시작] =======================
                # 크기(면적) 계산: cv2.contourArea로 4개 코너를 이용해 실제 픽셀 면적을 계산합니다.
                pixel_area = cv2.contourArea(marker_corners)

                # 0~1 사이의 값으로 정규화
                norm_x = center_x / config.IMAGE_WIDTH_PX
                norm_y = center_y / config.IMAGE_HEIGHT_PX

                # 'size'를 전체 이미지 면적 대비 '정규화된 면적'으로 계산합니다.
                total_pixel_area = float(config.IMAGE_WIDTH_PX * config.IMAGE_HEIGHT_PX)
                norm_size = pixel_area / total_pixel_area
                # ======================== [수정된 부분 끝] ========================

                current_markers[marker_id_int] = {
                    "button_id": marker_id_int,
                    "x": norm_x,
                    "y": norm_y,
                    "size": norm_size,
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

    def handle_request(self, request, response):
        """
        서비스 요청을 처리합니다. 클라이언트가 요청한 button_id(마커 ID)에 해당하는
        마커 정보를 찾아 응답합니다.
        """
        self.get_logger().info(f"--- 🔮 서비스 요청 수신: button_id(marker_id)={request.button_id} ---")

        # 공유 데이터에 접근하기 위해 잠금 획득
        with self.status_lock:
            # 요청된 ID의 마커가 현재 감지된 마커 목록에 있는지 확인
            marker_data = self.detected_markers.get(request.button_id)

            if marker_data:
                # 마커를 찾았을 경우, 응답 메시지에 정보 채우기
                response.success = True
                
                # ✨ [핵심 수정] numpy 타입을 파이썬 기본 float 타입으로 변환합니다.
                response.x = float(marker_data["x"])
                response.y = float(marker_data["y"])
                response.size = float(marker_data["size"])
                
                response.is_pressed = marker_data["is_pressed"]
                self.get_logger().info(f"✔️ 처리 성공: ID {request.button_id} 마커 정보를 응답합니다.")
            else:
                # 마커를 찾지 못했을 경우
                response.success = False
                self.get_logger().warn(f"⚠️ 처리 실패: ID {request.button_id} 마커가 현재 프레임에 없습니다.")

        # 공통 응답 필드 채우기
        response.robot_id = request.robot_id
        response.button_id = request.button_id
        response.timestamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"--- 🔮 최종 응답 전송: success={response.success} ---")
        return response

    def destroy_node(self):
        """노드 종료 시 호출되는 함수. 카메라 리소스를 해제합니다."""
        self.get_logger().info("노드 종료 중... 리소스를 해제합니다.")
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
