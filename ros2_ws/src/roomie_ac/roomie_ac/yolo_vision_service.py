import rclpy
from rclpy.node import Node
from roomie_msgs.srv import ButtonStatus
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from cv_bridge import CvBridge
from datetime import datetime
from ultralytics import YOLO
import threading
import time
import easyocr
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32

class YoloVisionService(Node):
    '''
    YOLO + OCR → self.button_status_map[button_id]에 저장
           ↓
    handle_request() 호출 시 요청된 button_ids에 대해
            ↓
    각 ID의 상태를 응답 필드로 매핑
            ↓
    서비스 응답 완료
    '''
    def __init__(self):
        super().__init__('yolo_vision_service')
        self.srv = self.create_service(ButtonStatus, '/vs/command/button_status', self.handle_request)
        self.ocr_pub = self.create_publisher(String, '/vs/button_status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        # 카메라 초기화
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

        # 모델 로드
        self.model = YOLO('/home/mac/dev_ws/addinedu/project/ros-repo-2/ros2_ws/src/roomie_ac/roomie_ac/best.pt')
        self.model.conf = 0.5
        self.model.iou = 0.5
        self.last_x_pub = self.create_publisher(Float32, '/vs/last_button_x', 10)
        self.last_y_pub = self.create_publisher(Float32, '/vs/last_button_y', 10)
         # OCR 리더기
        self.reader = easyocr.Reader(['en'], gpu=True)

        # 버튼 상태 저장 dict
        self.button_status_map = {}  # key: int(button_id), value: dict

        # 동기화용 락
        self.lock = threading.Lock()

        # 감지 루프 실행
        self.running = True
        self.worker_thread = threading.Thread(target=self.detect_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("🟢 YOLO Vision Service + OCR 활성화됨")

    def detect_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            h, w, _ = frame.shape
            results = self.model(frame)
            boxes = results[0].boxes
            annotated = frame.copy()

            with self.lock:
                for box in boxes:
                    class_id = int(box.cls[0].item())
                    class_name = self.model.names[class_id]
                    if not class_name.startswith('button_'):
                        continue

                    try:
                        button_id = int(class_name.split('_')[1])
                    except:
                        continue

                    xmin, ymin, xmax, ymax = map(int, box.xyxy[0].tolist())
                    roi = frame[ymin:ymax, xmin:xmax]

                    text = self.reader.readtext(roi, detail=0)
                    ocr_text = ''.join(filter(str.isdigit, ''.join(text))).strip()

                    x_center = ((xmin + xmax) / 2) / w
                    y_center = ((ymin + ymax) / 2) / h
                    size = ((xmax - xmin) * (ymax - ymin)) / (w * h)

                    self.button_status_map[button_id] = {
                        "x": float(x_center),
                        "y": float(y_center),
                        "size": float(size),
                        "is_pressed": False,
                        "timestamp": self.get_clock().now().to_msg(),
                        "ocr": ocr_text
                    }

                    # Publish
                    msg = String()
                    msg.data = f"button_{button_id} x:{x_center:.3f} y:{y_center:.3f} size:{size:.3f} ocr:{ocr_text}"
                    self.ocr_pub.publish(msg)
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    self.image_pub.publish(image_msg)

                    cv2.putText(annotated, f'Btn {button_id}: {ocr_text}', (xmin, ymin - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.rectangle(annotated, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

            cv2.imshow("YOLO + OCR View", annotated)
            cv2.waitKey(1)
            time.sleep(0.5)

    def handle_request(self, request, response):
        xs, ys, sizes, is_pressed, timestamps = [], [], [], [], []

        with self.lock:
            for btn_id in request.button_ids:
                status = self.button_status_map.get(btn_id, None)

                if status:
                    xs.append(status["x"])
                    ys.append(status["y"])
                    sizes.append(status["size"])
                    is_pressed.append(status["is_pressed"])
                    timestamps.append(status["timestamp"])
                else:
                    xs.append(0.0)
                    ys.append(0.0)
                    sizes.append(0.0)
                    is_pressed.append(False)
                    timestamps.append(self.get_clock().now().to_msg())

        response.robot_id = request.robot_id
        response.success = True
        response.xs = xs
        response.ys = ys
        response.sizes = sizes
        response.is_pressed = is_pressed
        response.timestamp = timestamps
        return response
    
    def destroy_node(self):
        self.running = False
        self.worker_thread.join()
        self.cap.release()
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
