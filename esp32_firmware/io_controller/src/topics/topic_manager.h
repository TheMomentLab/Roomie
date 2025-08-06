#ifndef TOPIC_MANAGER_H
#define TOPIC_MANAGER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "../controller/controller.h"

// 토픽 메시지 헤더
#include "roomie_msgs/msg/robot_state.h"

class TopicManager {
private:
  // 구독자
  rcl_subscription_t robot_state_subscriber;
  roomie_msgs__msg__RobotState robot_state_msg;
  
  // 컨트롤러 참조
  Controller* controller;
  
  // 구독 콜백 함수
  static void robot_state_callback(const void* msg);
  
  // 정적 참조 (콜백에서 사용)
  static TopicManager* instance;
  
public:
  TopicManager();
  
  bool init(rcl_node_t* node, rclc_executor_t* executor, 
            rclc_support_t* support, Controller* ctrl);
  
  // 실제 토픽 처리 함수
  void handleRobotState(const roomie_msgs__msg__RobotState* msg);
};

#endif // TOPIC_MANAGER_H