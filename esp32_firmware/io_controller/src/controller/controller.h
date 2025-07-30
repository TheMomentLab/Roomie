#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESP32Servo.h>

// 핀 정의
#define SERVO_LOCK_PIN 18
#define STATUS_LED_PIN 2
#define RGB_LED_R_PIN 25
#define RGB_LED_G_PIN 26
#define RGB_LED_B_PIN 27
#define DOOR_IR_SENSOR_PIN 21       // 문 상태 IR 센서 (디지털 출력)
#define ITEM_IR_SENSOR_PIN 22       // 적재 IR 센서 (디지털 출력)

// 로봇 ID
#define ROBOT_ID 1

// 로봇 상태 정의 (확장된 상태 ID)
enum RobotState {
  INITIAL = 0,          // 초기화 - 빨간색
  CHARGING = 1,         // 충전상태 - 초록색
  WAITING = 2,          // 작업대기 - 초록색 깜빡임
  PICKUP_MOVING = 10,   // 픽업위치 이동 - 초록색 깜빡임
  PICKUP_WAITING = 11,  // 픽업대기 - 초록색 깜빡임
  DELIVERY_MOVING = 12,  // 배송장소 이동 - 파란색
  DELIVERY_WAITING = 13,  // 수령대기 - 초록색 깜빡임
  CALL_MOVING = 20, // 호출위치 이동 - 파란색
  GUIDE_WAITING = 21, // 길안내 목적지 업데이트 - 초록색 깜빡임
  GUIDE_MOVING = 22,  // 길안내 이동 - 파란색
  TARGET_SEARCHING = 23, // 대상 탐색 - 초록색 깜빡임
  RETURN_MOVING = 30,   // 대기위치로 이동 - 파란색
  ELEVATOR_RIDING = 31, // 엘리베이터 탑승 - 파란색
  ERROR = 90            // 오류 - 빨간색
};

class Controller {
private:
  Servo lockServo;
  bool servoAttached; // 서보 attach 여부 플래그
  
  bool lockState;
  RobotState currentRobotState;
  unsigned long lastUpdate;
  
  // LED 제어
  void setRGB(int r, int g, int b);
  void updateStatusLED();
  
public:
  Controller();
  
  // 초기화
  bool init();
  
  // 업데이트 (주기적 호출)
  void update();
  
  // 잠금 제어
  bool setLockState(bool locked);
  bool getLockState() const { return lockState; }
  
  // 센서 읽기
  bool isDoorOpen();
  bool isItemLoaded();
  
  // 로봇 상태 설정
  void setRobotState(RobotState state);
  RobotState getRobotState() const { return currentRobotState; }
  
  // 시스템 상태 표시
  void setSystemStatus(bool ready);
};

#endif // CONTROLLER_H