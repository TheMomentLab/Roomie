#include "service_manager.h"
#include <Arduino.h>

ServiceManager* ServiceManager::instance = nullptr;

ServiceManager::ServiceManager() {
  instance = this;
}

bool ServiceManager::init(rcl_node_t* node, rclc_executor_t* executor, 
                         rclc_support_t* support, Controller* ctrl) {
  controller = ctrl;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  
  Serial.println("서비스 초기화 시작...");
  Serial.printf("서비스 초기화 전 여유 메모리: %d bytes (최대 블록: %d bytes)\n", 
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  Serial.printf("스택 여유 공간: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));
  
  rcl_ret_t ret;

  // --- 1. 모든 서비스 초기화 단계 ---
  Serial.println("\n--- 단계 1: 모든 서비스 초기화 ---");

  // 1.1 ControlLock 서비스
  Serial.println("ControlLock 서비스 초기화 시작...");
  ret = rclc_service_init_default(
        &control_lock_service, node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(roomie_msgs, srv, ControlLock),
        "/ioc/control_lock");
  Serial.printf("ControlLock init 반환값: %d\n", ret);
  if (ret != RCL_RET_OK) {
    Serial.printf("ControlLock 서비스 초기화 실패 (오류 코드: %d)\n", ret);
    Serial.printf("상세 오류: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    return false;
  }
  Serial.println("✅ ControlLock 서비스 생성 성공");
  Serial.printf("ControlLock 후 스택 여유: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));

  // 1.2 CheckItemLoaded 서비스
  Serial.println("CheckItemLoaded 서비스 초기화 시작...");
  ret = rclc_service_init_default(
        &check_item_service, node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(roomie_msgs, srv, CheckItemLoaded),
        "/ioc/check_item_loaded");
  Serial.printf("CheckItemLoaded init 반환값: %d\n", ret);
  if (ret != RCL_RET_OK) {
    Serial.printf("CheckItemLoaded 서비스 초기화 실패 (오류 코드: %d)\n", ret);
    Serial.printf("상세 오류: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    return false;
  }
  Serial.println("✅ CheckItemLoaded 서비스 생성 성공");
  Serial.printf("CheckItemLoaded 후 스택 여유: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));

  // 1.3 CheckDoorState 서비스
  Serial.println("CheckDoorState 서비스 초기화 시작...");
  ret = rclc_service_init_default(
        &check_door_service, node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(roomie_msgs, srv, CheckDoorState),
        "/ioc/check_door_state");
  Serial.printf("CheckDoorState init 반환값: %d\n", ret);
  if (ret != RCL_RET_OK) {
    Serial.printf("CheckDoorState 서비스 초기화 실패 (오류 코드: %d)\n", ret);
    Serial.printf("상세 오류: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
    return false;
  }
  Serial.println("✅ CheckDoorState 서비스 생성 성공");
  Serial.printf("모든 서비스 초기화 후 여유 메모리: %d bytes (최대 블록: %d bytes)\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  Serial.printf("모든 서비스 초기화 후 스택 여유: %d bytes\n", uxTaskGetStackHighWaterMark(NULL));

  // --- 2. 모든 서비스를 Executor에 추가 단계 ---
  Serial.println("\n--- 단계 2: 모든 서비스를 Executor에 추가 ---");
  
  // 2.1 ControlLock 서비스 추가
  ret = rclc_executor_add_service(executor, &control_lock_service, 
                               &control_lock_req, &control_lock_res, 
                               control_lock_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("ControlLock 서비스 executor 추가 실패 (오류 코드: %d)\n", ret);
    return false;
  }
  Serial.println("✅ ControlLock 서비스 executor 추가 성공");

  // 2.2 CheckItemLoaded 서비스 추가
  ret = rclc_executor_add_service(executor, &check_item_service,
                               &check_item_req, &check_item_res, 
                               check_item_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("CheckItemLoaded 서비스 executor 추가 실패 (오류 코드: %d)\n", ret);
    return false;
  }
  Serial.println("✅ CheckItemLoaded 서비스 executor 추가 성공");

  // 2.3 CheckDoorState 서비스 추가
  ret = rclc_executor_add_service(executor, &check_door_service,
                               &check_door_req, &check_door_res, 
                               check_door_callback);
  if (ret != RCL_RET_OK) {
    Serial.printf("CheckDoorState 서비스 executor 추가 실패 (오류 코드: %d)\n", ret);
    return false;
  }
  Serial.println("✅ CheckDoorState 서비스 executor 추가 성공");
  Serial.printf("모든 서비스 executor 추가 후 여유 메모리: %d bytes (최대 블록: %d bytes)\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  Serial.println("\n모든 서비스 초기화 완료");
  return true;
}

// 콜백 함수들
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

// 실제 서비스 처리 함수들
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