#include "controller.h"
#include <Arduino.h>

const char* robotStateToString(RobotState state) {
  switch (state) {
    case INITIAL:          return "초기화";
    case CHARGING:         return "충전상태";
    case WAITING:          return "작업대기";
    case PICKUP_MOVING:    return "픽업위치 이동";
    case PICKUP_WAITING:   return "픽업대기";
    case DELIVERY_MOVING:  return "배송장소 이동";
    case DELIVERY_WAITING: return "수령대기";
    case CALL_MOVING:      return "호출위치 이동";
    case GUIDE_WAITING:    return "길안내 목적지 업데이트";
    case GUIDE_MOVING:     return "길안내 이동";
    case DESTINATION_SEARCHING: return "대상 탐색";
    case RETURN_MOVING:    return "대기위치로 이동";
    case ELEVATOR_RIDING:  return "엘리베이터 탑승";
    case ERROR:            return "오류";
    default:               return "UNKNOWN";
  }
}

// 초음파 센서 거리 측정 헬퍼 함수
float getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms 타임아웃
  if (duration == 0) {
    return -1.0; // 감지 실패 시 -1 반환
  }
  return duration * 0.034 / 2.0; // cm 단위로 변환
}

Controller::Controller() : lockState(false), servoAttached(false), currentRobotState(RobotState::INITIAL), lastUpdate(0) {}

bool Controller::init() {
  Serial.println("컨트롤러 초기화 시작...");
  
  // 서보모터 초기화
  lockServo.attach(SERVO_LOCK_PIN);
  if (lockServo.attached()) {
    Serial.println("서보모터 초기화 성공");
    servoAttached = true;
    setLockState(true);
  } else {
    Serial.println("⚠️ 서보모터 초기화 실패!");
    servoAttached = false;
  }

  // 초음파 센서 핀 초기화
  pinMode(DOOR_TRIG_PIN, OUTPUT);
  pinMode(DOOR_ECHO_PIN, INPUT);
  pinMode(ITEM_TRIG_PIN, OUTPUT);
  pinMode(ITEM_ECHO_PIN, INPUT);
  Serial.println("초음파 센서 핀 초기화 완료");

  // LED 핀 초기화
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(RGB_LED_R_PIN, OUTPUT);
  pinMode(RGB_LED_G_PIN, OUTPUT);
  pinMode(RGB_LED_B_PIN, OUTPUT);
  Serial.println("LED 핀 초기화 완료");
  
  setRobotState(RobotState::INITIAL);
  Serial.println("컨트롤러 초기화 완료");
  return true;
}

void Controller::update() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate < 100) return;
  
  updateStatusLED();
  lastUpdate = currentTime;
}

bool Controller::setLockState(bool locked) {
  if (!servoAttached) {
    Serial.println("⚠️ 서보모터가 초기화되지 않아 제어할 수 없습니다.");
    return false;
  }

  if (lockServo.attached()) {
    // 잠금: 0도, 해제: 90도 로직 적용
    if (locked) {
      lockServo.write(0);
    } else {
      lockServo.write(90);
    }
    lockState = locked;
    return true;
  } else {
    Serial.println("⚠️ 서보모터 attach 상태가 아닙니다.");
    return false;
  }
}

bool Controller::isDoorOpen() {
  const float DOOR_OPEN_THRESHOLD_CM = 5.0;
  float distance = getDistanceCM(DOOR_TRIG_PIN, DOOR_ECHO_PIN);

  if (distance < 0) {
    Serial.println("경고: 문 센서 감지 실패.");
    return false; // 측정 실패 시 안전을 위해 '닫힘'으로 판단
  }

  bool doorOpen = (distance > DOOR_OPEN_THRESHOLD_CM);
  
  Serial.printf("문 센서 거리: %.1f cm → 문 %s\n", distance, doorOpen ? "열림" : "닫힘");
  return doorOpen;
}

bool Controller::isItemLoaded() {
  const float ITEM_LOADED_THRESHOLD_CM = 25.0;
  float distance = getDistanceCM(ITEM_TRIG_PIN, ITEM_ECHO_PIN);

  if (distance < 0) {
    Serial.println("경고: 적재 센서 감지 실패.");
    return false; // 측정 실패 시 안전을 위해 '미적재'로 판단
  }

  bool itemLoaded = (distance < ITEM_LOADED_THRESHOLD_CM);
  
  Serial.printf("적재 센서 거리: %.1f cm → %s\n", distance, itemLoaded ? "적재" : "미적재");
  return itemLoaded;
}

void Controller::setRobotState(RobotState state) {
  currentRobotState = state;
  Serial.printf("로봇 상태 변경: %d (%s)\n", state, robotStateToString(state));
}

void Controller::setSystemStatus(bool ready) {
  if (ready) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(200);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(200);
    }
  }
}

void Controller::setRGB(int r, int g, int b) {
  analogWrite(RGB_LED_R_PIN, 255 - r);
  analogWrite(RGB_LED_G_PIN, 255 - g);
  analogWrite(RGB_LED_B_PIN, 255 - b);
}

void Controller::updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  if (millis() - lastBlink > 500) {
    blinkState = !blinkState;
    lastBlink = millis();
  }
  
  // LED 로직
  switch (currentRobotState) {
    case INITIAL:
      setRGB(0, 255, 255);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    case ERROR:
      setRGB(255, 0, 0);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    case CHARGING:
      setRGB(0, 255, 0);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    case WAITING: case PICKUP_WAITING: case DELIVERY_WAITING: 
    case GUIDE_WAITING: case DESTINATION_SEARCHING:
      setRGB(255, 255, 0);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    case PICKUP_MOVING: case DELIVERY_MOVING: case CALL_MOVING: 
    case GUIDE_MOVING: case RETURN_MOVING: case ELEVATOR_RIDING:
      setRGB(0, 0, 255);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    default:
      setRGB(255, 255, 255);
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
  }
}