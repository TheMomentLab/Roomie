#ifndef TOPIC_MANAGER_H
#define TOPIC_MANAGER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "../controller/controller.h"

// 토픽 메시지 헤더
#include "roomie_msgs/msg/robot_state.h"
#include "roomie_msgs/msg/read_card_request.h"
#include "roomie_msgs/msg/read_card_response.h"

class TopicManager {
private:
  // 구독자
  rcl_subscription_t robot_state_subscriber;
  rcl_subscription_t read_card_request_subscriber;
  roomie_msgs__msg__RobotState robot_state_msg;
  roomie_msgs__msg__ReadCardRequest read_card_request_msg;
  
  // 발행자
  rcl_publisher_t read_card_response_publisher;
  roomie_msgs__msg__ReadCardResponse read_card_response_msg;
  
  // 컨트롤러 참조
  Controller* controller;
  
  // 카드 읽기 관련
  static const unsigned long CARD_READ_TIMEOUT_MS = 10000; // 10초 타임아웃
  bool card_read_requested;
  
  // 구독 콜백 함수
  static void robot_state_callback(const void* msg);
  static void read_card_request_callback(const void* msg);
  
  // 정적 참조 (콜백에서 사용)
  static TopicManager* instance;
  
public:
  TopicManager();
  
  bool init(rcl_node_t* node, rclc_executor_t* executor, 
            rclc_support_t* support, Controller* ctrl);
  
  // 실제 토픽 처리 함수
  void handleRobotState(const roomie_msgs__msg__RobotState* msg);
  void handleReadCardRequest(const roomie_msgs__msg__ReadCardRequest* msg);
  
  // 카드 읽기 응답 발행
  void publishReadCardResponse(int32_t robot_id, bool success, int32_t location_id);
  
  // 카드 읽기 상태 업데이트 (main loop에서 호출)
  void update();
};

#endif // TOPIC_MANAGER_H