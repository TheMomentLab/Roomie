#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESP32Servo.h>

// 핀 정의
#define SERVO_LOCK_PIN 18
#define STATUS_LED_PIN 2
#define RGB_LED_R_PIN 25
#define RGB_LED_G_PIN 26
#define RGB_LED_B_PIN 27
#define DOOR_TRIG_PIN 32
#define DOOR_ECHO_PIN 33
#define ITEM_TRIG_PIN 19
#define ITEM_ECHO_PIN 23

// 로봇 ID
#define ROBOT_ID 0

// 로봇 상태 정의
enum RobotState {
  INITIAL = 0,
  CHARGING = 1,
  WAITING = 2,
  PICKUP_MOVING = 10,
  PICKUP_WAITING = 11,
  DELIVERY_MOVING = 12,
  DELIVERY_WAITING = 13,
  CALL_MOVING = 20,
  GUIDE_WAITING = 21,
  GUIDE_MOVING = 22,
  DESTINATION_SEARCHING = 23,
  RETURN_MOVING = 30,
  ELEVATOR_RIDING = 31,
  ERROR = 90
};

const char* robotStateToString(RobotState state);

class Controller {
private:
  Servo lockServo;
  bool servoAttached;
  
  bool lockState;
  RobotState currentRobotState;
  unsigned long lastUpdate;
  
  void setRGB(int r, int g, int b);
  void updateStatusLED();
  
public:
  Controller();
  
  bool init();
  void update();
  
  bool setLockState(bool locked);
  bool getLockState() const { return lockState; }
  
  bool isDoorOpen();
  bool isItemLoaded();
  
  void setRobotState(RobotState state);
  RobotState getRobotState() const { return currentRobotState; }
  
  void setSystemStatus(bool ready);
};

#endif