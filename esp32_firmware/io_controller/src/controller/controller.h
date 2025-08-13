#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESP32Servo.h>
#include <MFRC522.h>

// 핀 정의
#define SERVO_LOCK_PIN 13
#define STATUS_LED_PIN 2
#define RGB_LED_R_PIN 25
#define RGB_LED_G_PIN 26
#define RGB_LED_B_PIN 27
#define DOOR_TRIG_PIN 32
#define DOOR_ECHO_PIN 33
#define ITEM_TRIG_PIN 12
#define ITEM_ECHO_PIN 14
#define RFID_RST_PIN 22
#define RFID_SS_PIN 5

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

// 카드 리더 상태 정의
enum class CardReaderState {
  DEACTIVATED,  // 카드 리더 비활성화
  ACTIVATED     // 카드 리더 활성화 (카드 대기)
};

const char* robotStateToString(RobotState state);
const char* cardReaderStateToString(CardReaderState state);

class Controller {
private:
  Servo lockServo;
  bool servoAttached;

  MFRC522 rfid;
  bool rfidInitialized;
  
  bool lockState;
  RobotState currentRobotState;
  CardReaderState cardReaderState;
  unsigned long lastUpdate;
  unsigned long cardReaderStartTime;  // 카드 리더 활성화 시작 시간
  
  void setRGB(int r, int g, int b);
  void updateStatusLED();

  float getDistanceCM(int trigPin, int echoPin);
  
public:
  Controller();
  
  bool init();
  void update();
  
  bool setLockState(bool locked);
  bool getLockState() const { return lockState; }
  
  bool isDoorOpen();
  bool isItemLoaded();

  bool readCardInfo(int32_t& location_id);
  
  void setRobotState(RobotState state);
  RobotState getRobotState() const { return currentRobotState; }
  
  // 카드 리더 상태 관리
  void setCardReaderState(CardReaderState state);
  CardReaderState getCardReaderState() const { return cardReaderState; }
  bool isCardReaderEnabled() const { return cardReaderState == CardReaderState::ACTIVATED; }
  unsigned long getCardReaderStartTime() const { return cardReaderStartTime; }
  
  void setSystemStatus(bool ready);
};

#endif