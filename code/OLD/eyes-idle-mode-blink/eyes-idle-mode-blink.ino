#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
   display.display();
  delay(2000);

  display.clearDisplay();
  
  // Draw yellow area (top 16 pixels)
  display.setTextSize(1);      
  display.setTextColor(SSD1306_WHITE);  
  display.setCursor(0, 0);    
  display.println(F("      IDLE MODE"));  
  display.setCursor(0, 8);    
  display.println(F("        HELLO!"));  
  display.display();
}

void loop() {
  static unsigned long previousMillis = 0;
  static bool eyesOpen = true;
  static int eyeXOffset = 0;
  static int eyeYOffset = 0;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= random(1000, 3000)) {
    previousMillis = currentMillis;

    // Randomly decide whether to blink or move eyes
    if (random(10) < 4) {  // 20% chance to blink
      eyesOpen = !eyesOpen;
      if (!eyesOpen) {
        drawEyes(false, eyeXOffset, eyeYOffset);
        delay(200); // Blink duration
        eyesOpen = !eyesOpen;
      }
    } else {
      eyeXOffset = random(-20, 20);
      eyeYOffset = random(-10,10); // Random horizontal offset
    }

    drawEyes(eyesOpen, eyeXOffset, eyeYOffset);
  }
}

void drawEyes(bool open, int xOffset, int yOffset) {
  // Clear the blue area
  display.fillRect(0, 16, SCREEN_WIDTH, SCREEN_HEIGHT - 16, SSD1306_BLACK);

  if (open) {
    // Draw eyes open with offset
    display.drawCircle(40 + xOffset, 40 + yOffset, 10, SSD1306_WHITE); // Left eye
    display.fillCircle(40 + xOffset, 40 + yOffset, 5, SSD1306_WHITE);  // Left eye pupil
    display.drawCircle(88 + xOffset, 40 + yOffset, 10, SSD1306_WHITE); // Right eye
    display.fillCircle(88 + xOffset, 40 + yOffset, 5, SSD1306_WHITE);  // Right eye pupil
  } else {
    // Draw eyes closed
    display.drawLine(30 + xOffset, 40 + yOffset, 50 + xOffset, 40 + yOffset, SSD1306_WHITE); // Left eye
    display.drawLine(78 + xOffset, 40 + yOffset, 98 + xOffset, 40 + yOffset, SSD1306_WHITE); // Right eye
  }

  display.display();
}