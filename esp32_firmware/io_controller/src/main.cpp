#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define SERVO_PIN 18

// WiFi 설정
char* ssid = "AIE_509_2.4G";
char* password = "addinedu_class1";

// micro-ROS Agent PC의 IP + 포트
IPAddress agent_ip(192, 168, 0, 47);
uint16_t agent_port = 8888;

// 서보 관련
Servo myServo;

rcl_subscription_t servo_sub;
std_msgs__msg__Int32 recv_msg;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

void servo_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int angle = constrain(msg->data, 0, 180);
  myServo.write(angle);
  Serial.printf("서보 각도: %d\n", angle);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // WiFi 연결 및 상태 확인 추가
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("WiFi 연결됨! IP: %s\n", WiFi.localIP().toString().c_str());

  // micro-ROS 초기화
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  // ROS Doima ID 설정
  setenv("ROS_DOMAIN_ID", "200", 1);

  myServo.attach(SERVO_PIN);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "servo_node", "", &support);

  rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "servo_angle");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &servo_sub, &recv_msg, &servo_callback, ON_NEW_DATA);
  
  Serial.println("micro-ROS 노드 초기화 완료");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}