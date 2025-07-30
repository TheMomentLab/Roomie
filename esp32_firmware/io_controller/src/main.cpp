#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <IPAddress.h>
#include <cstdlib>

#include "controller/controller.h"
#include "services/service_manager.h"
#include "topics/topic_manager.h"

// ==================== 설정 상수 ====================
const char* WIFI_SSID = "AIE_509_2.4G";
const char* WIFI_PASSWORD = "addinedu_class1";
const byte MICRO_ROS_AGENT_IP[] = {192, 168, 0, 47};
const int MICRO_ROS_AGENT_PORT = 8888;
const char* NODE_NAME = "roomie_ioc";
const unsigned long WIFI_TIMEOUT_MS = 20000;


// ==================== 네트워크 관리 클래스 ====================
class NetworkManager {
private:
  char* ssid;
  char* password;
  IPAddress agent_ip;
  uint16_t agent_port;
  bool is_connected;

  void printNetworkInfo() {
    Serial.printf("로컬 IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("게이트웨이: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("서브넷 마스크: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("신호 강도: %d dBm\n", WiFi.RSSI());
    Serial.printf("micro-ROS Agent: %s:%d\n", agent_ip.toString().c_str(), agent_port);
  }
  
public:
  NetworkManager(char* wifi_ssid, char* wifi_password, 
                IPAddress micro_ros_agent_ip, uint16_t micro_ros_agent_port)
    : ssid(wifi_ssid), password(wifi_password), agent_ip(micro_ros_agent_ip), 
      agent_port(micro_ros_agent_port), is_connected(false) {}
  
  bool connect() {
    Serial.println("WiFi 연결 시작...");
    Serial.printf("SSID: %s\n", ssid);
    WiFi.begin(ssid, password);
    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < WIFI_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      is_connected = true;
      Serial.println("\nWiFi 연결 성공!");
      printNetworkInfo();
      return true;
    } else {
      is_connected = false;
      Serial.println("\nWiFi 연결 실패!");
      return false;
    }
  }
  
  bool isConnected() { return is_connected && (WiFi.status() == WL_CONNECTED); }
  void update() { if(WiFi.status() != WL_CONNECTED) is_connected = false; }
  IPAddress getAgentIP() const { return agent_ip; }
  uint16_t getAgentPort() const { return agent_port; }
};


// ==================== 전역 객체 ====================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

Controller controller;
ServiceManager services;
TopicManager topics;
NetworkManager network(const_cast<char*>(WIFI_SSID), const_cast<char*>(WIFI_PASSWORD), 
                       IPAddress(MICRO_ROS_AGENT_IP),
                       MICRO_ROS_AGENT_PORT);

// ==================== 함수 프로토타입 ====================
void micro_ros_task(void * pvParameters);
bool init_micro_ros();
void error_loop();


// ==================== 메인 프로그램 ====================
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n=== Roomie IOC WiFi 버전 시작 ===");

  Serial.println("1. 컨트롤러 초기화...");
  controller.init();
  Serial.println("✅ 컨트롤러 초기화 완료");

  Serial.println("2. WiFi 네트워크 연결...");
  if (!network.connect()) {
      Serial.println("❌ WiFi 연결 실패! 시스템 정지");
      error_loop();
  }
  Serial.println("✅ WiFi 연결 완료");

  xTaskCreate(
    micro_ros_task,
    "micro_ros_task",
    20480, // 스택 크기 (Bytes) - 20KB로 대폭 늘림
    NULL,
    5,
    NULL
  );
  Serial.println("✅ micro-ROS 태스크 생성 및 시작");
}

void loop() {
  network.update();
  controller.update();
  delay(10);
}


// ==================== micro-ROS 태스크 ====================
void micro_ros_task(void * pvParameters) {
  Serial.println("3. micro-ROS 초기화 (전용 태스크)...");
  if (!init_micro_ros()) {
    Serial.println("❌ micro-ROS 초기화 실패! 태스크 종료.");
    vTaskDelete(NULL);
    return;
  }
  Serial.printf("✅ micro-ROS 초기화 완료\n\n");

  Serial.println("4. 서비스 초기화...");
  if (!services.init(&node, &executor, &support, &controller)) {
    Serial.println("❌ 서비스 초기화 실패!");
    error_loop();
  }
  Serial.println("✅ 서비스 초기화 완료");

  Serial.println("5. 토픽 초기화...");
  if (!topics.init(&node, &executor, &support, &controller)) {
    Serial.println("❌ 토픽 초기화 실패!");
    error_loop();
  }
  Serial.println("✅ 토픽 초기화 완료");

  Serial.println("\n🎉 IOC 시스템 초기화 완료!");
  controller.setRobotState(RobotState::WAITING);

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    delay(10);
  }
}


// ==================== 유틸리티 함수 ====================
bool init_micro_ros() {
  // 1. 통신 트랜스포트 설정 (가장 먼저!)
  set_microros_wifi_transports(const_cast<char*>(WIFI_SSID), const_cast<char*>(WIFI_PASSWORD), 
                               network.getAgentIP(), network.getAgentPort());

  allocator = rcl_get_default_allocator();

  // 2. Init Options 생성 및 도메인 ID 설정
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    Serial.println("❌ rcl_init_options_init 실패!");
    return false;
  }
  if (rcl_init_options_set_domain_id(&init_options, 200) != RCL_RET_OK) {
    Serial.println("❌ 도메인 ID 설정 실패!");
    return false;
  }
  Serial.println("✅ 도메인 ID 200으로 설정 완료");

  // 3. 옵션을 사용하여 Support 구조체 초기화
  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
    Serial.println("❌ micro-ROS 지원 구조체 초기화 실패!");
    return false;
  }

  // 4. 노드 생성
  if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) {
    Serial.println("❌ 노드 초기화 실패!");
    return false;
  }

  // 5. Executor 생성
  if (rclc_executor_init(&executor, &support.context, 16, &allocator) != RCL_RET_OK) {
    Serial.println("❌ Executor 초기화 실패!");
    return false;
  }
  
  return true;
}

void error_loop() {
  controller.setRobotState(RobotState::ERROR);
  Serial.println("💥 치명적 오류 발생! 시스템 정지");
  while(1) {
    controller.update();
    delay(100);
  }
}