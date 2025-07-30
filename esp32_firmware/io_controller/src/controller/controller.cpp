#include "controller.h"
#include <Arduino.h>

Controller::Controller() : lockState(false), servoAttached(false), currentRobotState(RobotState::INITIAL), lastUpdate(0) {}

bool Controller::init() {
  Serial.println("컨트롤러 초기화 시작...");
  
  Serial.printf("서보모터 핀 %d 초기화 시도...\n", SERVO_LOCK_PIN);
  if (lockServo.attach(SERVO_LOCK_PIN)) {
    Serial.println("✅ 서보모터 초기화 성공");
    servoAttached = true;
    setLockState(true); // 초기 상태는 잠금
  } else {
    Serial.println("⚠️ 서보모터 초기화 실패!");
    servoAttached = false;
  }

  // IR 센서 핀 초기화
  pinMode(DOOR_IR_SENSOR_PIN, INPUT);
  pinMode(ITEM_IR_SENSOR_PIN, INPUT);
  Serial.printf("IR 센서 초기 상태 - 문: %s, 적재: %s\n",
                isDoorOpen() ? "열림" : "닫힘",
                isItemLoaded() ? "감지됨" : "감지안됨");

  // LED 핀 초기화
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(RGB_LED_R_PIN, OUTPUT);
  pinMode(RGB_LED_G_PIN, OUTPUT);
  pinMode(RGB_LED_B_PIN, OUTPUT);
  
  setRobotState(RobotState::INITIAL);
  Serial.println("컨트롤러 초기화 완료");
  return true;
}

void Controller::update() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate < 100) return;  // 100ms마다 업데이트
  
  updateStatusLED();
  lastUpdate = currentTime;
}

bool Controller::setLockState(bool locked) {
  if (!servoAttached) {
    Serial.println("⚠️ 서보모터가 초기화되지 않아 제어할 수 없습니다.");
    return false;
  }

  if (lockServo.attached()) {
    if (locked) {
      lockServo.write(90);
    } else {
      lockServo.write(0);
    }
    lockState = locked;
    return true;
  } else {
    Serial.println("⚠️ 서보모터 attach 상태가 아닙니다.");
    return false;
  }
}

bool Controller::isDoorOpen() {
  // IR 센서 읽기 (디지털 신호)
  // 문이 닫혀있을 때 센서가 문을 감지해서 LOW
  // 문이 열려있을 때 센서가 아무것도 감지하지 못해서 HIGH
  bool sensorState = digitalRead(DOOR_IR_SENSOR_PIN);
  
  // LOW = 문 닫힌 상태 (문이 감지됨), HIGH = 문 열린 상태 (문이 감지되지 않음)
  bool doorOpen = (sensorState == HIGH);
  
  Serial.printf("문 상태 센서: %s → 문 %s\n", 
                sensorState ? "HIGH" : "LOW", 
                doorOpen ? "열림" : "닫힘");
  
  return doorOpen;
}

bool Controller::isItemLoaded() {
  // IR 센서 읽기 (디지털 신호)
  // 물건이 있을 때 센서가 물건을 감지해서 LOW
  // 물건이 없을 때 센서가 아무것도 감지하지 못해서 HIGH
  bool sensorState = digitalRead(ITEM_IR_SENSOR_PIN);
  
  // LOW = 물건 있음 (적재됨), HIGH = 물건 없음 (미적재)
  bool itemLoaded = (sensorState == LOW);
  
  Serial.printf("적재 센서: %s → %s\n", 
                sensorState ? "HIGH" : "LOW", 
                itemLoaded ? "적재됨" : "미적재");
  
  return itemLoaded;
}

void Controller::setRobotState(RobotState state) {
  currentRobotState = state;
  Serial.printf("로봇 상태 변경: %d\n", state);
}

void Controller::setSystemStatus(bool ready) {
  if (ready) {
    // 시스템 준비 완료 신호 (3회 깜빡임)
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(200);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(200);
    }
  }
}

void Controller::setRGB(int r, int g, int b) {
  // 공통 양극(Common Anode) RGB LED 스트립용 - 반전 로직
  // HIGH = LED 꺼짐, LOW = LED 켜짐
  analogWrite(RGB_LED_R_PIN, 255 - (r * 0.8));  // 반전 + 80% 밝기
  analogWrite(RGB_LED_G_PIN, 255 - (g * 0.8));
  analogWrite(RGB_LED_B_PIN, 255 - (b * 0.8));
}

void Controller::updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  // 깜빡임 주기 (500ms)
  if (millis() - lastBlink > 500) {
    blinkState = !blinkState;
    lastBlink = millis();
  }
  
  switch (currentRobotState) {
    case INITIAL:         // 0 - 빨간색 점멸
      setRGB(255, 0, 0);
      digitalWrite(STATUS_LED_PIN, blinkState ? HIGH : LOW);
      break;

    case ERROR:           // 90 - 빨간색 점등
      setRGB(255, 0, 0);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
      
    case CHARGING:        // 1 - 초록색 점등
      setRGB(0, 255, 0);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
      
    case WAITING:         // 2 - 파란색 점등
    case PICKUP_MOVING:   // 10 - 파란색 점등  
    case PICKUP_WAITING:  // 11 - 파란색 점등
    case DELIVERY_WAITING:  // 13 - 파란색 점등
    case GUIDE_WAITING: // 21 - 파란색 점등
    case TARGET_SEARCHING: // 23 - 파란색 점등
      setRGB(0, 0, 255);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
      
    case DELIVERY_MOVING: // 12 - 파란색 점멸
    case CALL_MOVING: // 20 - 파란색 점멸
    case GUIDE_MOVING:  // 22 - 파란색 점멸
    case RETURN_MOVING:   // 30 - 파란색 점멸
    case ELEVATOR_RIDING: // 31 - 파란색 점멸
      setRGB(0, 0, 255);
      digitalWrite(STATUS_LED_PIN, blinkState ? HIGH : LOW);
      break;
      
    default:
      // 알 수 없는 상태는 노란색으로 표시
      setRGB(255, 255, 0);
      digitalWrite(STATUS_LED_PIN, blinkState ? HIGH : LOW);
      break;
  }
}