# yolo_vision_service.py - OCR 제외 및 원 검출 기능 추가

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from roomie_msgs.srv import ButtonStatus
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from . import config

class YoloVisionService(Node):
    def __init__(self):
        super().__init__('yolo_vision_service')
        self.srv = self.create_service(ButtonStatus, '/vs/command/button_status', self.handle_request)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().fatal("카메라를 열 수 없습니다.")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.IMAGE_WIDTH_PX)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.IMAGE_HEIGHT_PX)

        self.model = YOLO('/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/best.pt')
        self.model.conf = 0.5
        self.model.iou = 0.5

        # track_id를 키로 사용하는 버튼 상태 저장소
        self.button_status_map = {}

        self.timer = self.create_timer(0.05, self.detect_callback)
        self.get_logger().info("🟢 YOLO Vision Service (원 검출 모드)가 활성화되었습니다.")

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w, _ = frame.shape
        # YOLO의 track 기능을 사용하여 객체 추적
        results = self.model.track(frame, persist=True)
        boxes = results[0].boxes
        annotated_frame = frame.copy()

        for box in boxes:
            class_id = int(box.cls[0].item())
            class_name = self.model.names[class_id]

            # 클래스 이름이 'button'으로 시작하는 경우에만 처리
            # 이전 테스트에서 'button'으로 확인했으므로, 'button' 또는 'button_'을 사용
            if class_name != 'button':
                continue

            # button_id는 요청받은 값을 그대로 사용하므로, 여기서 특정하지 않음
            # 여기서는 감지된 객체의 정보만 추적하고 저장
            track_id = int(box.id[0].item()) if box.id is not None else -1
            if track_id == -1:
                continue

            # 1. YOLO가 감지한 영역(ROI) 추출
            xmin, ymin, xmax, ymax = map(int, box.xyxy[0].tolist())
            roi = frame[ymin:ymax, xmin:xmax]
            if roi.size == 0:
                continue

            # 2. 원 검출을 위한 전처리
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred_roi = cv2.medianBlur(gray_roi, 5)

            # 3. 허프 원 변환(HoughCircles)으로 원 검출
            # 파라미터는 환경에 따라 튜닝이 필요할 수 있습니다.
            circles = cv2.HoughCircles(blurred_roi, cv2.HOUGH_GRADIENT, dp=1.2,
                                       minDist=roi.shape[0], param1=100, param2=30,
                                       minRadius=5, maxRadius=int(roi.shape[0]))

            # 원이 검출된 경우에만 정보 업데이트
            if circles is not None:
                # 검출된 원들 중 가장 큰 원을 찾음
                circles = np.uint16(np.around(circles))
                best_circle = max(circles[0, :], key=lambda c: c[2]) # c[2]는 반지름
                
                cx, cy, r = best_circle
                
                # 4. 전체 이미지 기준 좌표로 변환
                full_frame_cx = xmin + cx
                full_frame_cy = ymin + cy
                
                # 5. 좌표와 크기 정규화 (0.0 ~ 1.0)
                # 서비스 응답에 사용할 값들
                norm_x = float(full_frame_cx) / w
                norm_y = float(full_frame_cy) / h
                norm_size = float(r * 2) / w  # 지름을 기준으로 크기 정규화

                # 화면에 그리기
                cv2.rectangle(annotated_frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2) # YOLO 영역(파란색)
                cv2.circle(annotated_frame, (full_frame_cx, full_frame_cy), r, (0, 255, 0), 3) # 검출된 원(초록색)
                cv2.putText(annotated_frame, f"Track {track_id}", (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # 6. 추적 ID에 해당하는 버튼 정보 업데이트
                self.button_status_map[track_id] = {
                    "x": norm_x,
                    "y": norm_y,
                    "size": norm_size,
                    "is_pressed": False, # 이 값은 외부에서 제어되어야 함
                    "timestamp": self.get_clock().now().to_msg(),
                    "detected_class": class_name # 어떤 클래스로 감지되었는지 저장
                }

        # ROS 메시지로 변환하여 이미지 퍼블리시
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(img_msg)
        if config.DEBUG:
            cv2.imshow("YOLO + Circle Detection View", annotated_frame)
            cv2.waitKey(1)

    def handle_request(self, request, response):
        # 현재 추적되고 있는 모든 객체를 확인
        # 요청된 button_id와 가장 유사한 객체를 찾음 (여기서는 'button' 클래스 객체)
        # 실제 구현에서는 button_id (예: 2층, 3층)를 구분하는 로직이 필요하지만,
        # 현재는 'button'으로 감지된 첫 번째 객체를 사용
        
        target_track_id = -1
        # 'button' 클래스로 감지된 객체를 찾음
        for track_id, status in self.button_status_map.items():
            if status["detected_class"] == 'button':
                target_track_id = track_id
                break

        if target_track_id != -1:
            status = self.button_status_map[target_track_id]
            response.success = True
            response.robot_id = request.robot_id
            response.button_id = request.button_id # 요청받은 ID를 그대로 반환
            response.x = status["x"]
            response.y = status["y"]
            response.size = status["size"]
            response.is_pressed = status["is_pressed"]
            response.timestamp = status["timestamp"]
            self.get_logger().info(f"Button ID {request.button_id} 요청에 대해 Track ID {target_track_id}의 정보로 응답합니다.")
        else:
            response.success = False
            response.timestamp = self.get_clock().now().to_msg()
            self.get_logger().warn(f"요청된 버튼 ID {request.button_id}에 해당하는 'button' 객체를 찾지 못했습니다.")

        return response

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()