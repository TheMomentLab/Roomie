#include "topic_manager.h"
#include <Arduino.h>

// 정적 멤버 초기화
TopicManager* TopicManager::instance = nullptr;

TopicManager::TopicManager() : controller(nullptr), card_read_requested(false) {
  instance = this;
}

bool TopicManager::init(rcl_node_t* node, rclc_executor_t* executor, 
                       rclc_support_t* support, Controller* ctrl) {
  controller = ctrl;
  
  rcl_allocator_t allocator = rcl_get_default_allocator();
  
  Serial.println("토픽 초기화 시작...");
  
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

  // ReadCardRequest 구독자 생성
  if (rclc_subscription_init_default(
        &read_card_request_subscriber, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(roomie_msgs, msg, ReadCardRequest),
        "/ioc/read_card_request") != RCL_RET_OK) {
    Serial.println("ReadCardRequest 구독자 초기화 실패");
    return false;
  }
  
  // Executor에 구독자 추가
  if (rclc_executor_add_subscription(executor, &read_card_request_subscriber,
                                    &read_card_request_msg, read_card_request_callback,
                                    ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("ReadCardRequest 구독자 executor 추가 실패");
    return false;
  }

  // ReadCardResponse 발행자 생성
  if (rclc_publisher_init_default(
        &read_card_response_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(roomie_msgs, msg, ReadCardResponse),
        "/ioc/read_card_response") != RCL_RET_OK) {
    Serial.println("ReadCardResponse 발행자 초기화 실패");
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

void TopicManager::read_card_request_callback(const void* msg) {
  if (instance) {
    instance->handleReadCardRequest((const roomie_msgs__msg__ReadCardRequest*)msg);
  }
}

// 실제 토픽 처리 함수
void TopicManager::handleRobotState(const roomie_msgs__msg__RobotState* msg) {
  Serial.printf("RobotState 수신: robot_id=%d, state_id=%d\n", 
                msg->robot_id, msg->robot_state_id);
  
  // 해당 로봇의 메시지인지 확인
  if (msg->robot_id == 0) {  // ROBOT_ID
    RobotState state = static_cast<RobotState>(msg->robot_state_id);
    controller->setRobotState(state);
    
    Serial.printf("로봇 상태 업데이트: %d\n", state);
  }
}

void TopicManager::handleReadCardRequest(const roomie_msgs__msg__ReadCardRequest* msg) {
  Serial.printf("ReadCardRequest 수신: robot_id=%d\n", msg->robot_id);
  
  // 해당 로봇의 요청인지 확인
  if (msg->robot_id == 0) {  // ROBOT_ID
    Serial.println("카드 읽기 요청 처리 시작...");
    
    // 카드 리더 활성화
    controller->setCardReaderState(CardReaderState::ACTIVATED);
    card_read_requested = true;
    
    Serial.println("카드 리더가 활성화되었습니다. 카드를 대주세요.");
  }
}

void TopicManager::publishReadCardResponse(int32_t robot_id, bool success, int32_t location_id) {
  // 응답 메시지 설정
  read_card_response_msg.robot_id = robot_id;
  read_card_response_msg.success = success;
  read_card_response_msg.location_id = location_id;
  
  // 응답 발행
  rcl_ret_t ret = rcl_publish(&read_card_response_publisher, &read_card_response_msg, NULL);
  
  if (ret != RCL_RET_OK) {
    Serial.printf("ReadCardResponse 발행 실패! (오류 코드: %d)\n", ret);
  } else {
    Serial.printf("ReadCardResponse 발행 완료: robot_id=%d, success=%s, location_id=%d\n", 
                  robot_id, success ? "true" : "false", location_id);
  }
}

void TopicManager::update() {
  // 카드 읽기 요청이 활성화된 상태에서만 처리
  if (!card_read_requested || !controller->isCardReaderEnabled()) {
    return;
  }
  
  // 타임아웃 체크
  unsigned long current_time = millis();
  unsigned long start_time = controller->getCardReaderStartTime();
  
  if (current_time - start_time > CARD_READ_TIMEOUT_MS) {
    Serial.println("카드 읽기 타임아웃 발생");
    publishReadCardResponse(0, false, -1);  // 실패 응답
    
    // 상태 초기화
    controller->setCardReaderState(CardReaderState::DEACTIVATED);
    card_read_requested = false;
    return;
  }
  
  // 카드 읽기 시도
  int32_t location_id = -1;
  bool success = controller->readCardInfo(location_id);
  
  if (success) {
    Serial.println("카드 읽기 성공!");
    publishReadCardResponse(0, true, location_id);
    
    // 상태 초기화
    controller->setCardReaderState(CardReaderState::DEACTIVATED);
    card_read_requested = false;
  }
  // 실패 시에는 계속 시도 (타임아웃까지)
}