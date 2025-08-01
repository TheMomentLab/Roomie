#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18

Servo myServo;

void setup() {
  Serial.begin(115200);      // 시리얼 통신 시작
  myServo.attach(SERVO_PIN); // 서보 초기화
  Serial.println("각도 입력 (0 ~ 180):");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // 줄바꿈까지 입력 읽기
    input.trim(); // 공백 제거

    int angle = input.toInt(); // 문자열을 정수로 변환

    // 유효 범위 확인
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      Serial.print("이동: ");
      Serial.println(angle);
    } else {
      Serial.println("⚠️ 유효한 각도는 0~180도입니다.");
    }

    Serial.println("다음 각도 입력 (0 ~ 180):");
  }
}
