#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_now.h>
#include <WiFi.h>

#define Num_servos 12

// --- Servo Setup ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int servo_pin[4][3] = { {0, 4, 8}, {1, 5, 9}, {2, 6, 10}, {3, 7, 11} };

int angleToPulse(int angle) {
  int pulse = 500 + (angle * 10);  // More stable than 11.11
  pulse = (pulse / 10) * 10;       // round to nearest 10 µs
  return pulse / 4.88;             // convert to PWM step
}

void setServoAngle(int servoNum, int angle) {
  pwm.setPWM(servoNum, 0, angleToPulse(angle));
}

// --- Control Mode Management ---
bool useSerialControl = false;
unsigned long lastSerialCommandTime = 0;
const unsigned long serialTimeout = 5000; // ms

// --- ESP-NOW Data Struct ---
struct PacketData {
  byte lxAxisValue;
  byte lyAxisValue;
  byte rxAxisValue;
  byte ryAxisValue;
  byte switch1Value;
  byte switch2Value;
  byte switch3Value;
  byte switch4Value;
  byte switch5Value;
  byte switch6Value;
};
PacketData receivedData;

// --- ESP-NOW Receive Handler ---
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (useSerialControl && (millis() - lastSerialCommandTime < serialTimeout)) return;

  memcpy(&receivedData, incomingData, sizeof(receivedData));

  // Front-left leg
  setServoAngle(servo_pin[0][0], map(receivedData.lxAxisValue, 0, 254, 0, 180));
  setServoAngle(servo_pin[0][1], map(receivedData.lyAxisValue, 0, 254, 0, 180));
  setServoAngle(servo_pin[0][2], map(receivedData.ryAxisValue, 0, 254, 0, 180));

  // Front-right leg (inverted)
  setServoAngle(servo_pin[1][0], map(receivedData.lxAxisValue, 0, 254, 0, 180));
  setServoAngle(servo_pin[1][1], map(receivedData.lyAxisValue, 254, 0, 0, 180));
  setServoAngle(servo_pin[1][2], map(receivedData.ryAxisValue, 254, 0, 0, 180));
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  Serial.println("Ready. Use joystick or serial commands.");
}

// --- Main Loop ---
void loop() {
  if (Serial.available()) {
    useSerialControl = true;
    lastSerialCommandTime = millis();

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("s")) {
      int servoNum, angle;
      int matched = sscanf(cmd.c_str(), "s %d %d", &servoNum, &angle);
      if (matched == 2 && servoNum >= 0 && servoNum < Num_servos && angle >= 0 && angle <= 180) {
        setServoAngle(servoNum, angle);
        Serial.printf("Moved servo %d to %d degrees\n", servoNum, angle);
      } else {
        Serial.println("Invalid command. Usage: s [0-11] [0-180]");
      }

    } else if (cmd.startsWith("S")) {
      int angle;
      int matched = sscanf(cmd.c_str(), "S %d", &angle);
      if (matched == 1 && angle >= 0 && angle <= 180) {
        for (int i = 0; i < Num_servos; i++) {
          setServoAngle(i, angle);
          delay(100); // stagger servo movements
        }
        Serial.printf("Moved all servos to %d degrees\n", angle);
      } else {
        Serial.println("Invalid command. Usage: S [0-180]");
      }

    } else {
      Serial.println("Unknown command. Use 's' or 'S'.");
    }
  }

  // Timeout: revert back to ESP-NOW if no serial input
  if (useSerialControl && (millis() - lastSerialCommandTime > serialTimeout)) {
    useSerialControl = false;
    Serial.println("Resuming joystick control.");
  }
}
