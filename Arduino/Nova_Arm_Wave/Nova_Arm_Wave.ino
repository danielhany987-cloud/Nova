#include <ESP32Servo.h>

// --- PIN SETUP ---
const int BASE_PIN = 26;
const int SHOULDER_PIN = 18;

Servo baseServo;
Servo shoulderServo;

int pos = 0; 

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  // Note: 500 to 2500 pulse width is standard for giving 270-degree servos their full range
  baseServo.attach(BASE_PIN, 500, 2500); 
  shoulderServo.attach(SHOULDER_PIN, 500, 2500);

  // Set initial starting positions before the movement begins
  baseServo.write(90);  
  shoulderServo.write(70); 
  
  // Wait 3 seconds so you have time to step back
  delay(3000); 

  // --- 1. MOVE BASE TO 270 ONCE AND STOP ---
  for (pos = 90; pos <= 270; pos += 1) { 
    baseServo.write(pos);
    delay(30); 
  }
  
  // The base servo is now at 270 and the code will never tell it to move again.
}

void loop() {
  // --- 2. KEEP THE WAVE GOING FOREVER ---
  // The base is completely removed from this loop. It won't reset to 90.

  // Sweep shoulder left/up to 110
  for (pos = 70; pos <= 110; pos += 1) { 
    shoulderServo.write(pos);
    delay(30);
  }

  // Sweep shoulder right/down to 70
  for (pos = 110; pos >= 70; pos -= 1) { 
    shoulderServo.write(pos);
    delay(30);
  }
}