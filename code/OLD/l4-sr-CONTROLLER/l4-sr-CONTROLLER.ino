#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SDA_PIN 19
#define SCL_PIN 18

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button GPIOs
const int btnGreen = 5;
const int btnRed = 15;
const int btnYellow = 13;
const int btnBlue = 12;
const int btnRB = 4;
const int btnLB = 14;


// Joystick pins // the data has been rotated due to oreantation of the joystick
const int lxPin = 34; //vry
const int lyPin = 35; //vrx
const int rxPin = 32; //vry
const int ryPin = 33; //vrx

// ESP-NOW
uint8_t robotAddress[] = { 0xD0, 0xEF, 0x76, 0x5D, 0xA0, 0x38 };

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
PacketData data;

struct Pose {
  int angles[12]; // 12 servos
};

Pose poses[] = {
  {{90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90}}, // Neutral
  {{0, 45, 90, 0, 45, 90, 0, 45, 90, 0, 45, 90}},     // Sit
  {{180, 135, 90, 180, 135, 90, 180, 135, 90, 180, 135, 90}} // Stretch
};

int selectedPoseIndex = 0;
// Menu system
int selectedIndex = 0;
int currentPage = 0;
bool inSubMenu = false;
String currentSubMenu = "";

struct ServoState {
  int angles[12];
};
ServoState receivedState;

// Menu item list
std::vector<String> mainMenu = {
  "Servos", "Lights", "Poses", "Settings",
  "Serial Monitor", "Joystick Monitor", "Device Manager"
};

// Input history
#define MAX_INPUT_HISTORY 5
String inputHistory[MAX_INPUT_HISTORY];
int inputIndex = 0;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 100;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // OLED init
if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  Serial.println(F("SSD1306 allocation failed"));
  while(true); // Don't continue
}
display.clearDisplay();
display.display();
  esp_now_register_recv_cb(OnDataRecv);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  pinMode(btnGreen, INPUT_PULLUP);
  pinMode(btnRed, INPUT_PULLUP);
  pinMode(btnYellow, INPUT_PULLUP);
  pinMode(btnBlue, INPUT_PULLUP);
  pinMode(btnRB, INPUT_PULLUP);
  pinMode(btnLB, INPUT_PULLUP);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  drawMenu();
}

void loop() {
  updateJoystickData();

  esp_now_send(robotAddress, (uint8_t *)&data, sizeof(data));
  delay(100);

  if (millis() - lastDebounceTime > debounceDelay) {
    if (digitalRead(btnGreen) == LOW) {
      lastDebounceTime = millis();
      if (!inSubMenu) {
        currentSubMenu = mainMenu[selectedIndex];
        inSubMenu = true;
        openSubMenu(currentSubMenu);
      }
    }
    else if (digitalRead(btnRed) == LOW) {
      lastDebounceTime = millis();
      if (inSubMenu) {
        inSubMenu = false;
        drawMenu();
      }
    }
    else if (digitalRead(btnRB) == LOW) {
      lastDebounceTime = millis();
      inSubMenu = false;
      currentPage = 0;
      selectedIndex = 0;
      drawMenu();
    }
    else if (!inSubMenu && digitalRead(btnYellow) == LOW) {
      lastDebounceTime = millis();
      selectedIndex++;
      if (selectedIndex >= mainMenu.size()) selectedIndex = 0;
      currentPage = selectedIndex / 4;
      drawMenu();
    }
    else if (!inSubMenu && digitalRead(btnBlue) == LOW) {
      lastDebounceTime = millis();
      selectedIndex--;
      if (selectedIndex < 0) selectedIndex = mainMenu.size() - 1;
      currentPage = selectedIndex / 4;
      drawMenu();
    }
  }

  if (inSubMenu && currentSubMenu == "Joystick Monitor") drawJoystickMonitor();
  if (inSubMenu && currentSubMenu == "Serial Monitor") drawSerialMonitor();
  if (inSubMenu && currentSubMenu == "Poses") {
    drawPoseMenu();
    if (millis() - lastDebounceTime > debounceDelay) {
      if (digitalRead(btnYellow) == LOW) {
        lastDebounceTime = millis();
        selectedPoseIndex = (selectedPoseIndex + 1) % (sizeof(poses) / sizeof(poses[0]));
      }
      if (digitalRead(btnBlue) == LOW) {
        lastDebounceTime = millis();
        selectedPoseIndex = (selectedPoseIndex - 1 + 3) % 3;
      }
      if (digitalRead(btnGreen) == LOW) {
        lastDebounceTime = millis();
        esp_now_send(robotAddress, (uint8_t *)&poses[selectedPoseIndex], sizeof(Pose));
      }
    }
  }
  delay(50);
}

void updateJoystickData() {
  data.lxAxisValue = mapJoystick(analogRead(lxPin), true);
  data.lyAxisValue = mapJoystick(analogRead(lyPin), true);
  data.rxAxisValue = mapJoystick(analogRead(rxPin), true);
  data.ryAxisValue = mapJoystick(analogRead(ryPin), true);

  data.switch1Value = !digitalRead(btnYellow);
  data.switch2Value = !digitalRead(btnBlue);
  data.switch3Value = !digitalRead(btnRB);
  data.switch4Value = !digitalRead(btnRed);
  data.switch5Value = !digitalRead(btnGreen);
  data.switch6Value = !digitalRead(btnLB);

  logInput();
}

int mapJoystick(int value, bool reverse) {
  if (value >= 2200) return map(value, 2200, 4095, 127, 254);
  else if (value <= 1800) return (value == 0 ? 0 : map(value, 1800, 0, 127, 0));
  return 127;
}

void logInput() {
  String input = "";

  if (digitalRead(btnGreen) == LOW) input = "Green Pressed";
  else if (digitalRead(btnRed) == LOW) input = "Red Pressed";
  else if (digitalRead(btnYellow) == LOW) input = "Yellow Pressed";
  else if (digitalRead(btnBlue) == LOW) input = "Blue Pressed";
  else if (digitalRead(btnRB) == LOW) input = "RB Pressed";
  else if (digitalRead(btnLB) == LOW) input = "LB Pressed";
  else if (data.lxAxisValue < 50) input = "LX Left";
  else if (data.lxAxisValue > 200) input = "LX Right";
  else if (data.lyAxisValue > 200) input = "LY Up";
  else if (data.lyAxisValue < 50) input = "LY Down";
  else if (data.rxAxisValue < 50) input = "RX Left";
  else if (data.rxAxisValue > 200) input = "RX Right";
  else if (data.ryAxisValue > 200) input = "RY Up";
  else if (data.ryAxisValue < 50) input = "RY Down";

  if (input != "") {
    inputHistory[inputIndex] = input;
    inputIndex = (inputIndex + 1) % MAX_INPUT_HISTORY;
  }
}

void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("= L4-SR CONTROLLER ="));

  for (int i = 0; i < 4; i++) {
    int menuIndex = i + currentPage * 4;
    if (menuIndex < mainMenu.size()) {
      display.setCursor(10, 16 + i * 12);
      if (menuIndex == selectedIndex) display.print(">");
      else display.print(" ");
      display.println(mainMenu[menuIndex]);
    }
  }
  display.display();
}

void drawServoMonitor() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("= Servo Monitor =");

  for (int leg = 0; leg < 4; leg++) {
    int y = 12 + leg * 12;
    display.setCursor(0, y);
    display.printf("Leg %d: %3d %3d %3d",
      leg + 1,
      receivedState.angles[leg * 3],
      receivedState.angles[leg * 3 + 1],
      receivedState.angles[leg * 3 + 2]);
  }

  display.display();
}

void drawSerialMonitor() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(4, 0);
  display.println("= Serial Monitor =");

  for (int i = 0; i < MAX_INPUT_HISTORY; i++) {
    if (inputHistory[i] != "") {
      display.setCursor(0, 16 + i * 10);
      display.println(inputHistory[i]);
    }
  }
  display.display();
}

void drawJoystickMonitor() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(4, 0);
  display.println("= Joystick Monitor =");

  display.setCursor(0, 20);
  display.print("LX: "); display.print(data.lxAxisValue);
  display.setCursor(0, 30);
  display.print("LY: "); display.print(data.lyAxisValue);
  display.setCursor(0, 40);
  display.print("RX: "); display.print(data.rxAxisValue);
  display.setCursor(0, 50);
  display.print("RY: "); display.print(data.ryAxisValue);

  display.display();
}

void drawPoseMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("= Select Pose =");

  const char* poseNames[] = {"Neutral", "Sit", "Stretch"};

  for (int i = 0; i < 3; i++) {
    display.setCursor(10, 16 + i * 12);
    if (i == selectedPoseIndex) display.print(">");
    else display.print(" ");
    display.println(poseNames[i]);
  }
  display.display();
}

void openSubMenu(String subMenu) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("SubMenu: ");
  display.println(subMenu);
  display.display();
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
  Serial.print("Sent: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}


void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  if (len == sizeof(ServoState)) {
    memcpy(&receivedState, incomingData, sizeof(receivedState));
  }
}
