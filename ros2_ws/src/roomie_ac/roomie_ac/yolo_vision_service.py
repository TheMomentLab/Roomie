import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image  # ROS 이미지 메시지
from cv_bridge import CvBridge     # OpenCV <-> ROS 이미지 변환
from roomie_msgs.srv import ButtonStatus
from geometry_msgs.msg import Point
from ultralytics import YOLO
import cv2
import threading
from . import config

class YoloVisionService(Node):
    def __init__(self):
        super().__init__('yolo_vision_service')

        # 콜백 그룹 설정
        self.service_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # 서비스 서버 생성
        self.srv = self.create_service(
            ButtonStatus, '/vs/command/button_status', self.handle_request,
            callback_group=self.service_callback_group
        )

        # 카메라 및 YOLO 모델 초기화
        self.cap = cv2.VideoCapture(config.CAMERA_DEVICE_ID)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"카메라 {config.CAMERA_DEVICE_ID}번을 열 수 없습니다.")
            rclpy.shutdown()
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.IMAGE_WIDTH_PX)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.IMAGE_HEIGHT_PX)
        self.model = YOLO(config.YOLO_MODEL_PATH)

        # 상태 변수 및 동기화 잠금
        self.last_button_bbox = None
        self.status_lock = threading.Lock()

        # ✨ CvBridge와 이미지 Publisher는 그대로 유지합니다. (올바른 설정)
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'yolo_annotated_image', 10)
        
        self.timer = self.create_timer(0.1, self.detect_callback, callback_group=self.timer_callback_group)
        self.get_logger().info("🟢 YOLO Vision Service (Publisher 모드)가 활성화되었습니다.")

    def detect_callback(self):
        with self.status_lock:  # 전체 감싸기
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("⚠️ 카메라 프레임을 읽어오는 데 실패했습니다.")
                return

            h, w, _ = frame.shape
            results = self.model.track(source=frame, persist=True, verbose=False)
            annotated_frame = frame.copy()

            current_tracks = set()
            if results[0].boxes is not None and results[0].boxes.id is not None:
                # [Roomie 수정] with self.status_lock 블록 추가: 데이터 접근을 안전하게 만듭니다.
                with self.status_lock:
                    for box in results[0].boxes:
                        # [Roomie 수정] 감지된 모든 버튼에 대해 정보 추출 및 저장
                        class_id = int(box.cls[0].item())
                        class_name = self.model.names[class_id]
                        if 'button' not in class_name:
                            continue
                        
                        try:
                            button_id_from_class = int(class_name.split('_')[1])
                        except (ValueError, IndexError):
                            continue
                        
                        track_id = int(box.id[0].item())
                        current_tracks.add(track_id)
                        
                        xmin, ymin, xmax, ymax = map(int, box.xyxy[0].tolist())
                        
                        # ✨ [핵심 수정] 4점 좌표 대신 중심점과 크기 계산
                        center_x = (xmin + xmax) / 2.0
                        center_y = (ymin + ymax) / 2.0
                        width = xmax - xmin # 너비
                        height = ymax - ymin # 높이

                        # 0~1 값으로 정규화 (너비 기준)
                        norm_x = center_x / config.IMAGE_WIDTH_PX
                        norm_y = center_y / config.IMAGE_HEIGHT_PX
                        # size는 너비를 기준으로 정규화합니다.
                        norm_size = width / config.IMAGE_WIDTH_PX
                        
                        # ✨ [핵심 수정] 감지된 버튼 정보를 새 형식으로 맵에 저장
                        self.button_status_map[track_id] = {
                            "button_id": button_id_from_class,
                            "x": norm_x,
                            "y": norm_y,
                            "size": norm_size,
                            "is_pressed": False, # 이 값은 현재 사용되지 않음
                            "timestamp": self.get_clock().now().to_msg()
                        }

                        # [Roomie 수정] 시각화 로직
                        conf = float(box.conf[0].item()) * 100
                        cv2.rectangle(annotated_frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                        label = f"Btn {button_id_from_class} (ID:{track_id})"
                        cv2.putText(annotated_frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                    # [Roomie 수정] 추적이 사라진 버튼 정보는 맵에서 삭제
                    lost_tracks = set(self.button_status_map.keys()) - current_tracks
                    for track_id in lost_tracks:
                        del self.button_status_map[track_id]

            # 처리된 이미지를 ROS 토픽으로 발행
            try:
                img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.image_publisher.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"이미지 발행 실패: {e}")

    def handle_request(self, request, response):
        with self.status_lock:
            # [Roomie 수정] 요청된 ID와 일치하는 모든 버튼을 리스트에 담습니다.
            matches = []
            for status in self.button_status_map.values():
                if status["button_id"] == request.button_id:
                    matches.append(status)
            
            # [Roomie 수정] 매칭된 버튼의 개수에 따라 응답을 다르게 처리합니다.
            if len(matches) == 1:
                # ✨ [핵심 수정] 응답 필드를 새 형식에 맞게 채웁니다.
                matched_status = matches[0]
                response.success = True
                response.x = matched_status["x"]
                response.y = matched_status["y"]
                response.size = matched_status["size"]
                response.is_pressed = matched_status["is_pressed"]
                self.get_logger().info(f"✅ Button ID {request.button_id} 요청 처리 성공.")
                
            elif len(matches) > 1:
                # 2개 이상 중복으로 감지된 경우 (서비스 정의에 따름)
                response.success = False
                self.get_logger().warn(f"❌ Button ID {request.button_id}가 {len(matches)}개 중복 감지되어 처리 실패.")

            else: # len(matches) == 0
                # 하나도 감지되지 않은 경우
                response.success = False
                self.get_logger().warn(f"❌ Button ID {request.button_id}를 찾을 수 없어 처리 실패.")

        # [Roomie 수정] robot_id, button_id, timestamp는 성공/실패 여부와 관계없이 채워주는 것이 좋습니다.
        response.robot_id = request.robot_id
        response.button_id = request.button_id
        response.timestamp = self.get_clock().now().to_msg()
            
        return response
    
    def destroy_node(self):
        self.get_logger().info("노드 종료 중... 리소스를 해제합니다.")
        self.cap.release()
        # ✨ 로컬 GUI 코드가 없으므로 아래 두 줄도 삭제합니다.
        # cv2.destroyAllWindows()
        # for _ in range(4):
        #     cv2.waitKey(1)
        super().destroy_node()

def main(args=None):
    # 이 함수는 수정할 필요 없음 (기존과 동일)
    rclpy.init(args=args)
    node = YoloVisionService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            executor.shutdown()
            # 노드가 executor에 의해 관리되고 있었으므로, 노드 소멸자를 호출
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()