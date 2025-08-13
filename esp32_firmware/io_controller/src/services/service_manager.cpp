#include "service_manager.h"
#include <Arduino.h>

#define MAX_INIT_ATTEMPTS 5
#define RETRY_DELAY_MS 500  // 100ms에서 500ms로 증가
#define SERVICE_INIT_DELAY_MS 500  // 서비스 간 지연 시간 증가

ServiceManager* ServiceManager::instance = nullptr;

ServiceManager::ServiceManager() {
  instance = this;
}

bool ServiceManager::init(rcl_node_t* node, rclc_executor_t* executor, 
                         rclc_support_t* support, Controller* ctrl) {
  controller = ctrl;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  
  Serial.println("서비스 초기화 시작...");
  
  // Agent 연결 안정화를 위한 초기 대기
  delay(1000);
  
  rcl_ret_t ret;
  int attempt;

  // ControlLock 서비스 - 재시도 로직 추가
  for (attempt = 0; attempt < MAX_INIT_ATTEMPTS; attempt++) {
    ret = rclc_service_init_default(
          &control_lock_service, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(roomie_msgs, srv, ControlLock),
          "/ioc/control_lock");
    
    if (ret == RCL_RET_OK) {
      Serial.println("ControlLock 서비스 생성 성공");
      break;
    }
    
    Serial.printf("ControlLock 서비스 초기화 실패 (시도 %d/%d, 오류: %d)\n", 
                  attempt + 1, MAX_INIT_ATTEMPTS, ret);
    rcl_reset_error();
    delay(RETRY_DELAY_MS);
  }
  
  if (ret != RCL_RET_OK) {
    Serial.println("ControlLock 서비스 초기화 최종 실패");
    return false;
  }

  ret = rclc_executor_add_service(executor, &control_lock_service, 
                               &control_lock_req, &control_lock_res, 
                               control_lock_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("ControlLock 서비스 executor 추가 실패 (오류 코드: %d)\n", ret);
    return false;
  }
  Serial.println("ControlLock 서비스 executor 추가 성공");

  delay(SERVICE_INIT_DELAY_MS);

  // CheckItemLoaded 서비스 - 재시도 로직 추가
  for (attempt = 0; attempt < MAX_INIT_ATTEMPTS; attempt++) {
    ret = rclc_service_init_default(
          &check_item_service, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(roomie_msgs, srv, CheckItemLoaded),
          "/ioc/check_item_loaded");
    
    if (ret == RCL_RET_OK) {
      Serial.println("CheckItemLoaded 서비스 생성 성공");
      break;
    }
    
    Serial.printf("CheckItemLoaded 서비스 초기화 실패 (시도 %d/%d, 오류: %d)\n", 
                  attempt + 1, MAX_INIT_ATTEMPTS, ret);
    rcl_reset_error();
    delay(RETRY_DELAY_MS);
  }
  
  if (ret != RCL_RET_OK) {
    Serial.println("CheckItemLoaded 서비스 초기화 최종 실패");
    return false;
  }

  ret = rclc_executor_add_service(executor, &check_item_service,
                               &check_item_req, &check_item_res, 
                               check_item_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("CheckItemLoaded 서비스 executor 추가 실패 (오류 코드: %d)\n", ret);
    return false;
  }
  Serial.println("CheckItemLoaded 서비스 executor 추가 성공");

  delay(SERVICE_INIT_DELAY_MS);

  // CheckDoorState 서비스 - 재시도 로직 추가
  for (attempt = 0; attempt < MAX_INIT_ATTEMPTS; attempt++) {
    ret = rclc_service_init_default(
          &check_door_service, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(roomie_msgs, srv, CheckDoorState),
          "/ioc/check_door_state");
    
    if (ret == RCL_RET_OK) {
      Serial.println("CheckDoorState 서비스 생성 성공");
      break;
    }
    
    Serial.printf("CheckDoorState 서비스 초기화 실패 (시도 %d/%d, 오류: %d)\n", 
                  attempt + 1, MAX_INIT_ATTEMPTS, ret);
    rcl_reset_error();
    delay(RETRY_DELAY_MS);
  }
  
  if (ret != RCL_RET_OK) {
    Serial.println("CheckDoorState 서비스 초기화 최종 실패");
    return false;
  }

  ret = rclc_executor_add_service(executor, &check_door_service,
                               &check_door_req, &check_door_res, 
                               check_door_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("CheckDoorState 서비스 executor 추가 실패 (오류 코드: %d)\n", ret);
    return false;
  }
  Serial.println("CheckDoorState 서비스 executor 추가 성공");

  Serial.println("모든 서비스 초기화 완료");
  return true;
}

// 콜백 함수들 (변경 없음)
void ServiceManager::control_lock_callback(const void* req, void* res) {
  if (instance) {
    instance->handleControlLock(
      (const roomie_msgs__srv__ControlLock_Request*)req,
      (roomie_msgs__srv__ControlLock_Response*)res);
  }
}

void ServiceManager::check_door_callback(const void* req, void* res) {
  if (instance) {
    instance->handleCheckDoor(
      (const roomie_msgs__srv__CheckDoorState_Request*)req,
      (roomie_msgs__srv__CheckDoorState_Response*)res);
  }
}

void ServiceManager::check_item_callback(const void* req, void* res) {
  if (instance) {
    instance->handleCheckItem(
      (const roomie_msgs__srv__CheckItemLoaded_Request*)req,
      (roomie_msgs__srv__CheckItemLoaded_Response*)res);
  }
}

// 실제 서비스 처리 함수들 (변경 없음)
void ServiceManager::handleControlLock(const roomie_msgs__srv__ControlLock_Request* req,
                                      roomie_msgs__srv__ControlLock_Response* res) {
  Serial.printf("ControlLock 요청: robot_id=%d, locked=%s\n", 
                req->robot_id, req->locked ? "true" : "false");
  
  bool result = controller->setLockState(req->locked);
  
  res->robot_id = req->robot_id;
  res->success = result;
  
  Serial.printf("ControlLock 응답: success=%s\n", res->success ? "true" : "false");
}

void ServiceManager::handleCheckDoor(const roomie_msgs__srv__CheckDoorState_Request* req,
                                    roomie_msgs__srv__CheckDoorState_Response* res) {
  Serial.printf("CheckDoor 요청: robot_id=%d\n", req->robot_id);
  
  res->robot_id = req->robot_id;
  res->is_opened = controller->isDoorOpen();
  
  Serial.printf("CheckDoor 응답: is_opened=%s\n", res->is_opened ? "true" : "false");
}

void ServiceManager::handleCheckItem(const roomie_msgs__srv__CheckItemLoaded_Request* req,
                                    roomie_msgs__srv__CheckItemLoaded_Response* res) {
  Serial.printf("CheckItem 요청: robot_id=%d\n", req->robot_id);
  
  res->robot_id = req->robot_id;
  res->item_loaded = controller->isItemLoaded();
  
  Serial.printf("CheckItem 응답: item_loaded=%s\n", res->item_loaded ? "true" : "false");
}