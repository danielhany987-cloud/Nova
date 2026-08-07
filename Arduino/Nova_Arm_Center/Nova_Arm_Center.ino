#include <Arduino.h>
#include <ESP32Servo.h>

constexpr uint8_t PIN_BASE_SERVO     = 26;
constexpr uint8_t PIN_SHOULDER_SERVO = 18;
constexpr uint8_t PIN_ELBOW_SERVO    = 19;
constexpr uint8_t PIN_CLAW_SERVO     = 14;

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo clawServo;

void setup() {
  baseServo.setPeriodHertz(50);
  shoulderServo.setPeriodHertz(50);
  elbowServo.setPeriodHertz(50);
  clawServo.setPeriodHertz(50);

  baseServo.attach(PIN_BASE_SERVO, 500, 2400);
  shoulderServo.attach(PIN_SHOULDER_SERVO, 500, 2400);
  elbowServo.attach(PIN_ELBOW_SERVO, 500, 2400);
  clawServo.attach(PIN_CLAW_SERVO, 500, 2400);

  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  clawServo.write(90);
}

void loop() {
}
