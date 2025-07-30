#include "topic_manager.h"
#include <Arduino.h>

// 정적 멤버 초기화
TopicManager* TopicManager::instance = nullptr;

TopicManager::TopicManager() : controller(nullptr) {
  instance = this;
}

bool TopicManager::init(rcl_node_t* node, rclc_executor_t* executor, 
                       rclc_support_t* support, Controller* ctrl) {
  controller = ctrl;
  
  rcl_allocator_t allocator = rcl_get_default_allocator();
  
  Serial.println("토픽 매니저 초기화 시작...");
  
  // RobotState 구독자 생성
  if (rclc_subscription_init_default(
        &robot_state_subscriber, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(roomie_msgs, msg, RobotState),
        "/roomie/status/robot_state") != RCL_RET_OK) {
    Serial.println("RobotState 구독자 초기화 실패");
    return false;
  }
  
  // Executor에 구독자 추가
  if (rclc_executor_add_subscription(executor, &robot_state_subscriber,
                                    &robot_state_msg, robot_state_callback,
                                    ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("RobotState 구독자 executor 추가 실패");
    return false;
  }
  
  Serial.println("토픽 매니저 초기화 완료");
  return true;
}

// 콜백 함수
void TopicManager::robot_state_callback(const void* msg) {
  if (instance) {
    instance->handleRobotState((const roomie_msgs__msg__RobotState*)msg);
  }
}

// 실제 토픽 처리 함수
void TopicManager::handleRobotState(const roomie_msgs__msg__RobotState* msg) {
  Serial.printf("RobotState 수신: robot_id=%d, state_id=%d\n", 
                msg->robot_id, msg->robot_state_id);
  
  // 해당 로봇의 메시지인지 확인
  if (msg->robot_id == 1) {  // ROBOT_ID
    RobotState state = static_cast<RobotState>(msg->robot_state_id);
    controller->setRobotState(state);
    
    Serial.printf("로봇 상태 업데이트: %d\n", state);
  }
}