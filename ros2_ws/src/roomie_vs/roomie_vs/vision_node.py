#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
import threading
import time
import os
import numpy as np
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory

# Local modules
from .camera_manager import MultiCameraManager, setup_openni2_environment
from .model_manager import MultiModelDetector, CNNButtonClassifier, ButtonPressedCNN
from .obstacle_detector import ObstacleDetector
from .udp_streamer import UDPVideoStreamer
from .person_tracker import PersonTracker
from .display_ocr import DisplayOCR
from .gpu_monitor import GPUResourceMonitor

# ROS2 messages and services
from roomie_msgs.srv import (
    ButtonStatus, 
    SetVSMode,
    ElevatorStatus, 
    DoorStatus,
    Location
)
from roomie_msgs.msg import Obstacle, GlassDoorStatus, Tracking
from roomie_msgs.action import Enroll
from std_srvs.srv import Trigger

class VisionNode(Node):
    """
    Roomie 로봇의 비전 서비스를 담당하는 메인 ROS2 노드.
    카메라 관리, 객체 감지, 사람 추적, 장애물 감지 등 다양한 비전 관련 기능을 총괄합니다.
    """
    
    def __init__(self):
        """노드 초기화, 핵심 구성 요소 설정 및 메인 루프 시작"""
        super().__init__('vision_node')
        
        # 설정 파일 로드
        self.config = self._load_config()
        
        # 비전 기능 핵심 구성 요소 초기화
        self.camera_manager = MultiCameraManager(self.get_logger())
        self.model_detector = MultiModelDetector(self.get_logger(), self.config)
        self.person_tracker = PersonTracker(self.get_logger(), self.config, self.model_detector.models.get('normal'))
        self.display_ocr = DisplayOCR(self.get_logger())
        self.obstacle_detector = ObstacleDetector(self.get_logger())
        self.udp_streamer = UDPVideoStreamer(
            target_ip=self.config['udp_streamer']['target_ip'],
            target_port=self.config['udp_streamer']['target_port'],
            logger=self.get_logger()
        )
        
        # GPU 모니터링 활성화 여부 확인 및 시작
        if self.config['gpu_monitor']['enabled']:
            self.gpu_monitor = GPUResourceMonitor(self.get_logger(), self.config['gpu_monitor']['max_memory_mb'])
            self.gpu_monitor.start_monitoring()
        else:
            self.gpu_monitor = None

        # 노드 상태 변수
        self.current_camera = None
        self.current_depth_camera = None
        self.current_camera_name = "None"
        self.flip_horizontal = False  # 수평 뒤집기 여부
        self.last_glass_door_opened = None
        self.last_obstacle_publish_time = None
        self.obstacle_detection_history = {}

        # ROS2 서비스 및 토픽 인터페이스 초기화
        self._initialize_services()
        self._initialize_topics()
        
        # 별도 스레드에서 메인 처리 루프 시작
        self.main_loop_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_loop_thread.start()
        
        self.get_logger().info("비전 노드가 성공적으로 초기화되었습니다.")

    def _load_config(self):
        """vision_config.yaml 설정 파일을 로드합니다."""
        package_share_directory = get_package_share_directory('roomie_vs')
        config_file = os.path.join(package_share_directory, 'config', 'vision_config.yaml')
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)

    def _initialize_services(self):
        """비전 노드에서 제공하는 ROS2 서비스를 설정합니다."""
        self.create_service(SetVSMode, '/vs/command/set_vs_mode', self.set_vs_mode_callback)
        self.create_service(ButtonStatus, '/vs/command/button_status', self.get_button_status_callback)
        self.create_service(ElevatorStatus, '/vs/command/elevator_status', self.get_elevator_status_callback)
        self.create_service(DoorStatus, '/vs/command/door_status', self.get_door_status_callback)
        self.create_service(Location, '/vs/command/location', self.get_location_callback)
        self.create_service(Trigger, '/vs/command/stop_tracking', self.stop_tracking_callback)
        self.enroll_action_server = ActionServer(
            self,
            Enroll,
            '/vs/action/enroll',
            execute_callback=self.enroll_execute_callback,
            goal_callback=lambda goal_request: rclpy.action.GoalResponse.ACCEPT,
            cancel_callback=lambda goal_handle: rclpy.action.CancelResponse.ACCEPT
        )
        self.get_logger().info("ROS2 서비스가 준비되었습니다.")

    def _initialize_topics(self):
        """비전 노드에서 발행하는 ROS2 토픽을 설정합니다."""
        self.obstacle_pub = self.create_publisher(Obstacle, '/vs/obstacle', 10)
        self.glass_door_pub = self.create_publisher(GlassDoorStatus, '/vs/glass_door_status', 10)
        self.tracking_pub = self.create_publisher(Tracking, '/vs/tracking', 10)
        self.get_logger().info("ROS2 토픽이 준비되었습니다.")

    def set_vs_mode_callback(self, request, response):
        """요청에 따라 비전 시스템의 동작 모드를 변경합니다."""
        mode_id = request.mode_id
        self.get_logger().info(f"비전 모드 변경 요청 수신: {mode_id}")
        
        # 모드에 필요한 카메라 초기화
        if not self.camera_manager.initialize_cameras_for_mode(mode_id):
            self.get_logger().error(f"모드 {mode_id}에 대한 카메라 초기화에 실패했습니다.")
            response.success = False
            return response
        
        self.current_camera, self.current_depth_camera, self.current_camera_name = \
            self.camera_manager.get_camera_for_mode(mode_id)
        
        # 모드에 맞는 모델 및 추적기 설정
        self.model_detector.set_model_for_mode(mode_id)
        self.person_tracker.set_mode(mode_id)
        
        # 특정 모드에서만 수평 뒤집기 적용
        self.flip_horizontal = (mode_id in [0, 1, 2])
        
        # OCR 기능 활성화/비활성화
        if mode_id in [3, 4]:
            self.display_ocr.enable_ocr(use_gpu=True)
        else:
            self.display_ocr.disable_ocr()
        
        response.success = True
        self.get_logger().info(f"비전 모드가 {mode_id} ({self.current_camera_name}) (으)로 변경되었습니다.")
        return response

    def _main_loop(self):
        """
        메인 루프: 현재 활성화된 카메라에서 프레임을 받아와 모드에 맞는 비전 처리를 수행합니다.
        """
        while rclpy.ok():
            if self.current_camera is None:
                time.sleep(0.1)
                continue
            
            try:
                depth_frame, color_frame = self.current_camera.get_frames()
                
                if color_frame is None:
                    time.sleep(0.03)
                    continue
                
                # 필요한 경우, 컬러 및 깊이 프레임을 수평으로 뒤집습니다.
                if self.flip_horizontal:
                    color_frame = cv2.flip(color_frame, 1)
                    if depth_frame is not None:
                        depth_frame = cv2.flip(depth_frame, 1)
                
                # 후방 카메라: 사람 추적 수행
                if self.current_camera_name.startswith("Rear"):
                    tracking_events = self.person_tracker.process_frame(color_frame)
                    for event in tracking_events:
                        msg = Tracking(id=event['id'], event=event['event'])
                        self.tracking_pub.publish(msg)
                    
                    overlay_frame = self.person_tracker.get_overlay_frame(color_frame)
                    self.udp_streamer.stream_frame(overlay_frame)

                # 전방 카메라: 객체 및 장애물 감지 수행
                else:
                    objects = self.model_detector.detect_objects(color_frame, depth_frame, self.config['yolo']['confidence_threshold'])
                    self._process_obstacles(objects, self.current_depth_camera)
                    self._publish_glass_door_status(objects)

                time.sleep(0.03)
                
            except Exception as e:
                self.get_logger().error(f"메인 루프에서 예외 발생: {e}")
                time.sleep(1)

    def get_button_status_callback(self, request, response):
        # TODO: 모델 탐지기를 사용한 버튼 상태 확인 기능 구현 필요
        response.success = False
        return response

    def get_elevator_status_callback(self, request, response):
        # TODO: OCR을 사용한 엘리베이터 상태(층, 방향) 확인 기능 구현 필요
        response.position = "?"
        response.direction = 0
        return response

    def get_door_status_callback(self, request, response):
        # TODO: 모델 탐지기를 사용한 문 열림 상태 확인 기능 구현 필요
        response.door_opened = False
        return response

    def get_location_callback(self, request, response):
        # TODO: ArUco 마커를 사용한 위치 인식 기능 구현 필요
        response.location_id = 0
        return response

    def stop_tracking_callback(self, request, response):
        """사람 추적을 중지하고 관련 상태를 초기화합니다."""
        result = self.person_tracker.stop_tracking()
        response.success = result['success']
        response.message = result['message']
        return response

    async def enroll_execute_callback(self, goal_handle):
        """지정된 시간 동안 특정 대상을 등록하는 액션 콜백"""
        duration_sec = goal_handle.request.duration_sec
        self.person_tracker.register_target(duration_sec)
        
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Enroll.Result(success=False)
            
            # 진행률 피드백
            feedback_msg = Enroll.Feedback()
            feedback_msg.progress = (time.time() - start_time) / duration_sec
            goal_handle.publish_feedback(feedback_msg)
            await asyncio.sleep(0.1)
        
        self.person_tracker._finalize_registration()
        
        goal_handle.succeed()
        
        result = Enroll.Result()
        result.success = self.person_tracker.target_registered
        return result

    def _process_obstacles(self, objects, depth_camera):
        """
        감지된 객체 정보를 바탕으로 장애물을 처리하고, 일정 시간 동안의 감지 빈도를 고려하여 최종 장애물 정보를 발행합니다.
        """
        if depth_camera is None:
            return
        
        obstacles = self.obstacle_detector.detect_obstacles_from_objects(objects, depth_camera)
        
        current_time = time.time()
        if self.last_obstacle_publish_time is None:
            self.last_obstacle_publish_time = current_time
        
        # 장애물 감지 기록
        for obs in obstacles:
            class_name = obs['class_name']
            if class_name not in self.obstacle_detection_history:
                self.obstacle_detection_history[class_name] = [0, 0]
            self.obstacle_detection_history[class_name][0] += 1
        
        for class_name in self.obstacle_detection_history:
            self.obstacle_detection_history[class_name][1] += 1
        
        # 설정된 주기마다 장애물 정보 발행 여부 결정
        if current_time - self.last_obstacle_publish_time >= self.config['obstacle_detector']['publish_interval_sec']:
            for class_name, (count, total_frames) in self.obstacle_detection_history.items():
                detection_rate = count / total_frames if total_frames > 0 else 0
                
                # 특정 임계값 이상의 빈도로 감지된 경우에만 장애물로 판단하고 발행
                if detection_rate >= self.config['obstacle_detector']['detection_rate_threshold']:
                    for obs in obstacles:
                        if obs['class_name'] == class_name:
                            msg = Obstacle()
                            msg.robot_id = obs['robot_id']
                            msg.dynamic = obs['dynamic']
                            msg.x = obs['x']
                            msg.y = obs['y']
                            msg.depth = obs['depth']
                            self.obstacle_pub.publish(msg)
                            break
            
            self.obstacle_detection_history.clear()
            self.last_obstacle_publish_time = current_time

    def _publish_glass_door_status(self, objects):
        """유리문 객체 감지 결과를 바탕으로 문 열림 상태를 발행합니다."""
        door_opened = self._check_door_opened(objects)
        
        # 상태가 변경되었을 때만 메시지 발행
        if self.last_glass_door_opened is None or self.last_glass_door_opened != door_opened:
            msg = GlassDoorStatus()
            msg.opened = door_opened
            self.glass_door_pub.publish(msg)
            self.last_glass_door_opened = door_opened

    def _check_door_opened(self, objects) -> bool:
        """
        감지된 문 객체의 수와 위치를 기준으로 문이 열렸는지 여부를 판단합니다.
        - 문 객체가 2개 이상이면 열린 것으로 간주합니다.
        - 문 객체가 1개이고 화면 중앙에서 벗어나 있으면 열린 것으로 간주합니다.
        """
        door_objects = [obj for obj in objects if obj['class_name'] == 'door']
        
        if not door_objects:
            return False
        
        if len(door_objects) >= 2:
            return True
        
        if len(door_objects) == 1:
            door = door_objects[0]
            center_x = door['center'][0]
            
            # 화면 중앙에서 일정 비율 이상 벗어났는지 확인
            if abs(center_x - 320) > 320 * 0.3:
                return True
        
        return False

    def cleanup(self):
        """노드 종료 시 사용된 모든 리소스를 정리합니다."""
        self.get_logger().info("비전 노드 리소스를 정리합니다...")
        self.camera_manager.cleanup_all_cameras()
        self.udp_streamer.close()
        if self.gpu_monitor:
            self.gpu_monitor.stop_monitoring()
        cv2.destroyAllWindows()
        self.get_logger().info("비전 노드 리소스 정리가 완료되었습니다.")

def main(args=None):
    setup_openni2_environment()
    rclpy.init(args=args)
    
    try:
        vision_node = VisionNode()
        executor = MultiThreadedExecutor()
        executor.add_node(vision_node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            vision_node.cleanup()
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Vision 노드 실행 중 치명적 오류: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()