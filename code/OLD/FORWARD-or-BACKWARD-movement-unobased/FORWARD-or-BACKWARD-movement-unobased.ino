/* Servo motor driver board control for ESP32-CAM
   Home Page
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

#define servoMIN 200
#define servoMAX 600
#define stepDelay 10   // Delay between each step (in milliseconds)
#define stepSize 5

    // Size of each step

void setup() {
  Serial.begin(9600);
  // Initialize I2C with SDA on GPIO 14 and SCL on GPIO 12
  srituhobby.begin();
  srituhobby.setPWMFreq(60);
}

void moveTwoServosSmooth(int servo1, int startPos1, int endPos1, int servo2, int startPos2, int endPos2) {
  int pos1 = startPos1;
  int pos2 = startPos2;

  while (pos1 != endPos1 || pos2 != endPos2) {
    if (pos1 < endPos1) pos1 += stepSize;
    else if (pos1 > endPos1) pos1 -= stepSize;

    if (pos2 < endPos2) pos2 += stepSize;
    else if (pos2 > endPos2) pos2 -= stepSize;

    srituhobby.setPWM(servo1, 0, pos1);
    srituhobby.setPWM(servo2, 0, pos2);
    
    delay(stepDelay);
  }
}

void loop() {
  // Move front left (servo4) and back right (servo2)
  moveTwoServosSmooth(3, servoMIN, servoMAX, 1, servoMAX, servoMIN);
  delay(300);
  
  // Move front right (servo1) and back left (servo3)
  moveTwoServosSmooth(0, servoMAX, servoMIN, 2, servoMIN, servoMAX);
  delay(300);

  // Move front left (servo4) and back right (servo2) back to initial position
  moveTwoServosSmooth(3, servoMAX, servoMIN, 1, servoMIN, servoMAX);
  delay(300);
  
  // Move front right (servo1) and back left (servo3) back to initial position
  moveTwoServosSmooth(0, servoMIN, servoMAX, 2, servoMAX, servoMIN);
  delay(300);
}