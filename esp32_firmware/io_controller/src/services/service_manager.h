#ifndef SERVICE_MANAGER_H
#define SERVICE_MANAGER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include "../controller/controller.h"

// 서비스 메시지 헤더
#include "roomie_msgs/srv/control_lock.h"
#include "roomie_msgs/srv/check_door_state.h"
#include "roomie_msgs/srv/check_item_loaded.h"

// 정적 메모리 할당: 서비스 메시지를 전역/정적 변수로 선언하여 힙 단편화 방지
static roomie_msgs__srv__ControlLock_Request control_lock_req;
static roomie_msgs__srv__ControlLock_Response control_lock_res;
static roomie_msgs__srv__CheckDoorState_Request check_door_req;
static roomie_msgs__srv__CheckDoorState_Response check_door_res;
static roomie_msgs__srv__CheckItemLoaded_Request check_item_req;
static roomie_msgs__srv__CheckItemLoaded_Response check_item_res;

class ServiceManager {
private:
  // 서비스 인스턴스들
  rcl_service_t control_lock_service;
  rcl_service_t check_door_service;
  rcl_service_t check_item_service;
  
  // 컨트롤러 참조
  Controller* controller;
  
  // 서비스 콜백 함수들
  static void control_lock_callback(const void* req, void* res);
  static void check_door_callback(const void* req, void* res);
  static void check_item_callback(const void* req, void* res);
  
  // 정적 참조 (콜백에서 사용)
  static ServiceManager* instance;
  
public:
  ServiceManager();
  
  bool init(rcl_node_t* node, rclc_executor_t* executor, 
            rclc_support_t* support, Controller* ctrl);
  
  // 실제 서비스 처리 함수들
  void handleControlLock(const roomie_msgs__srv__ControlLock_Request* req,
                        roomie_msgs__srv__ControlLock_Response* res);
  
  void handleCheckDoor(const roomie_msgs__srv__CheckDoorState_Request* req,
                      roomie_msgs__srv__CheckDoorState_Response* res);
  
  void handleCheckItem(const roomie_msgs__srv__CheckItemLoaded_Request* req,
                      roomie_msgs__srv__CheckItemLoaded_Response* res);
};

#endif // SERVICE_MANAGER_H