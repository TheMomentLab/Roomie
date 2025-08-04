#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class ButtonDetector(Node):
    def __init__(self):
        super().__init__('button_detector')
        
        # YOLO 모델 로드
        self.load_model()
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 카메라 초기화
        self.cap = cv2.VideoCapture(2)  # HCAM01N 카메라 (video2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다!')
            return
        
        # 퍼블리셔 초기화
        self.button_pub = self.create_publisher(Point, 'detected_buttons', 10)
        
        # 타이머 생성 (30 FPS)
        self.timer = self.create_timer(1.0/30.0, self.detect_buttons)
        
        self.get_logger().info('버튼 탐지 노드가 시작되었습니다.')
    
    def load_model(self):
        """YOLO 모델을 로드합니다."""
        try:
            # 모델 파일 경로
            package_share_dir = get_package_share_directory('roomie_ec')
            model_path = os.path.join(package_share_dir, 'models', 'best.pt')
            
            self.get_logger().info(f'모델 로드 중: {model_path}')
            
            # Ultralytics YOLO 모델 로드
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO 모델 로드 완료')
                
        except Exception as e:
            self.get_logger().error(f'모델 로드 중 오류: {str(e)}')
            return
    
    def detect_buttons(self):
        """카메라에서 버튼을 탐지합니다."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('카메라에서 프레임을 읽을 수 없습니다.')
            return
        
        # BGR to RGB 변환
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # YOLO 추론
        try:
            # Ultralytics YOLO 추론
            results = self.model(frame_rgb, verbose=False)
            
            # 결과 처리
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # 클래스 ID 확인 (0: button)
                        class_id = int(box.cls.item())
                        if class_id == 0:  # 버튼 클래스
                            # 바운딩 박스 좌표
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            confidence = box.conf.item()
                            
                            # x좌표와 bbox 크기 계산
                            center_x = (x1 + x2) / 2
                            bbox_width = x2 - x1
                            bbox_height = y2 - y1
                            bbox_size = bbox_width * bbox_height  # 면적
                            
                            # 좌표 정규화 (0-1 범위)
                            img_height, img_width = frame.shape[:2]
                            norm_x = center_x / img_width
                            norm_size = bbox_size / (img_width * img_height)  # 정규화된 크기
                            
                            # ROS2 메시지 생성 및 발행
                            button_msg = Point()
                            button_msg.x = norm_x  # 정규화된 x좌표
                            button_msg.y = norm_size  # 정규화된 bbox 크기
                            button_msg.z = confidence  # confidence를 z에 저장
                            
                            self.button_pub.publish(button_msg)
                            
                            self.get_logger().info(f'버튼 탐지: x={norm_x:.3f}, size={norm_size:.3f}, conf={confidence:.3f}')
                            
                            # OpenCV 창에 바운딩 박스 그리기
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(frame, f'Button: {confidence:.2f}', 
                                      (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 결과 화면 표시
            cv2.imshow('Button Detection', frame)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'추론 중 오류: {str(e)}')
    
    def __del__(self):
        """정리 작업"""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    detector = ButtonDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 