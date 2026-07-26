// Includes
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define buttons
#define BTN_GREEN 2
#define BTN_RED 15
#define BTN_YELLOW 13
#define BTN_BLUE 12
#define BTN_LB 4
#define BTN_RB 14

// Joystick pins
#define JOY_LX 32
#define JOY_LY 33
#define JOY_RX 34
#define JOY_RY 35

// ESP-NOW config
uint8_t receiverMacAddress[] = {0xAC, 0x67, 0xB2, 0x36, 0x7F, 0x28};
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

// Menu system
const char* menuItems[] = {"Servos", "Lights", "Poses", "Settings", "Serial Monitor", "Joystick Monitor"};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
int menuIndex = 0;
int page = 0;
bool inSubMenu = false;
bool joystickMonitorActive = false;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}

int mapAndAdjustJoystickDeadBandValues(int value, bool reverse) {
  if (value >= 2200) value = map(value, 2200, 4095, 127, 254);
  else if (value <= 1800) value = (value == 0 ? 0 : map(value, 1800, 0, 127, 0));
  else value = 127;
  if (reverse) value = 254 - value;
  return value;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Buttons
  pinMode(BTN_GREEN, INPUT_PULLUP);
  pinMode(BTN_RED, INPUT_PULLUP);
  pinMode(BTN_YELLOW, INPUT_PULLUP);
  pinMode(BTN_BLUE, INPUT_PULLUP);
  pinMode(BTN_LB, INPUT_PULLUP);
  pinMode(BTN_RB, INPUT_PULLUP);

  drawMenu();
}

void loop() {
  // Read joysticks for serial
  int lx = analogRead(JOY_LX);
  int ly = analogRead(JOY_LY);
  int rx = analogRead(JOY_RX);
  int ry = analogRead(JOY_RY);

  data.lxAxisValue = mapAndAdjustJoystickDeadBandValues(lx, false);
  data.lyAxisValue = mapAndAdjustJoystickDeadBandValues(ly, false);
  data.rxAxisValue = mapAndAdjustJoystickDeadBandValues(rx, false);
  data.ryAxisValue = mapAndAdjustJoystickDeadBandValues(ry, false);

  data.switch1Value = !digitalRead(BTN_YELLOW);
  data.switch2Value = !digitalRead(BTN_BLUE);
  data.switch3Value = !digitalRead(BTN_RB);
  data.switch4Value = !digitalRead(BTN_RED);
  data.switch5Value = !digitalRead(BTN_GREEN);
  data.switch6Value = !digitalRead(BTN_LB);

  esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));

  Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d\n", lx, ly, rx, ry);

  if (!inSubMenu) {
    if (!digitalRead(BTN_YELLOW)) {
      menuIndex++;
      if (menuIndex >= menuLength) menuIndex = 0;
      drawMenu();
      delay(200);
    }
    if (!digitalRead(BTN_BLUE)) {
      menuIndex--;
      if (menuIndex < 0) menuIndex = menuLength - 1;
      drawMenu();
      delay(200);
    }
    if (!digitalRead(BTN_GREEN)) {
      if (strcmp(menuItems[menuIndex], "Joystick Monitor") == 0) {
        inSubMenu = true;
        joystickMonitorActive = true;
      } else {
        inSubMenu = true;
        selectSubMenu();
      }
      delay(200);
    }
  } else {
    if (!digitalRead(BTN_RED) || !digitalRead(BTN_RB)) {
      inSubMenu = false;
      joystickMonitorActive = false;
      drawMenu();
      delay(200);
    } else if (joystickMonitorActive) {
      drawJoystickMonitor();
    }
  }

  delay(50);
}

void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("== CrabBot Controller =="));
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);

  page = menuIndex / 4;

  for (int i = 0; i < 4; i++) {
    int item = page * 4 + i;
    if (item >= menuLength) break;
    display.setCursor(5, 12 + i * 13);
    if (item == menuIndex) display.print("> ");
    else display.print("  ");
    display.println(menuItems[item]);
  }

  display.display();
}

void drawJoystickMonitor() {
  static int last_lx = 25, last_ly = 35;
  static int last_rx = 95, last_ry = 35;

  int lx = analogRead(JOY_LX);
  int ly = analogRead(JOY_LY);
  int rx = analogRead(JOY_RX);
  int ry = analogRead(JOY_RY);

  int l_x = map(ly, 0, 4095, 5, 45);
  int l_y = map(lx, 0, 4095, 15, 55);
  int r_x = map(ry, 0, 4095, 75, 115);
  int r_y = map(rx, 0, 4095, 15, 55);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println(F("Joystick Monitor"));

  // Left stick
  display.drawRect(5, 15, 40, 40, SSD1306_WHITE);
  display.fillCircle(last_lx, last_ly, 1, SSD1306_BLACK);
  display.fillCircle(l_x, l_y, 2, SSD1306_WHITE);

  // Right stick
  display.drawRect(75, 15, 40, 40, SSD1306_WHITE);
  display.fillCircle(last_rx, last_ry, 1, SSD1306_BLACK);
  display.fillCircle(r_x, r_y, 2, SSD1306_WHITE);

  last_lx = l_x;
  last_ly = l_y;
  last_rx = r_x;
  last_ry = r_y;

  display.display();
}

void selectSubMenu() {
  if (strcmp(menuItems[menuIndex], "Servos") == 0) servosSubMenu();
  else if (strcmp(menuItems[menuIndex], "Lights") == 0) lightsSubMenu();
  else if (strcmp(menuItems[menuIndex], "Poses") == 0) posesSubMenu();
  else if (strcmp(menuItems[menuIndex], "Settings") == 0) settingsSubMenu();
  else if (strcmp(menuItems[menuIndex], "Serial Monitor") == 0) serialMonitorSubMenu();
}

void servosSubMenu() {
  // Future code here
}

void lightsSubMenu() {
  // Future code here
}

void posesSubMenu() {
  // Future code here
}

void settingsSubMenu() {
  // Future code here
}

void serialMonitorSubMenu() {
  // Future code here
}