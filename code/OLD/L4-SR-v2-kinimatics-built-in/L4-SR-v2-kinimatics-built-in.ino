#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVOMIN = 102;  // Pulse for 0 degrees
const int SERVOMAX = 512;  // Pulse for 180 degrees



// Translate angle (0–180) to pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz

  // Set all servos to 90° on startup
  for (int ch = 0; ch < 16; ch++) {
    pwm.setPWM(ch, 0, angleToPulse(65));
  }
   Serial.println("Servos set to start possision");

}

void loop(){



}