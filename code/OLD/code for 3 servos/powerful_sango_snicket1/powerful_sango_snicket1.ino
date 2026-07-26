// C++ code
//
#include <Servo.h>

Servo servo_2;

Servo servo_3;

Servo servo_4;

void setup()
{
  servo_2.attach(2, 500, 2500);
  servo_3.attach(3, 500, 2500);
  servo_4.attach(4, 500, 2500);
  pinMode(8, INPUT);
  Serial.begin(9600);
  pinMode(9, INPUT);
  pinMode(10, INPUT);

  servo_2.write(0);
  servo_3.write(0);
  servo_4.write(0);
}

void loop()
{
  Serial.println(digitalRead(8));
  Serial.println(digitalRead(9));
  Serial.println(digitalRead(10));
  if (digitalRead(8) == 1) {
    servo_2.write(180);
    delay(1000); // Wait for 1000 millisecond(s)
  } else {
    servo_2.write(0);
  }
  if (digitalRead(9) == 1) {
    servo_3.write(180);
    delay(1000); // Wait for 1000 millisecond(s)
  } else {
    servo_3.write(0);
  }
  if (digitalRead(10) == 1) {
    servo_4.write(180);
    delay(1000); // Wait for 1000 millisecond(s)
  } else {
    servo_4.write(0);
  }
}