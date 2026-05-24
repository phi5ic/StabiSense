#include "Display_Module.h"
#include "Globals.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static ScreenPage currentPage = PAGE_SPLASH;
static int rollHistory[128];
static int pitchHistory[128];
static int motionHistory[128];
static bool displayReady = false;

static const char* className(int c) {
  switch (c) {
    case 0: return "IDLE";
    case 1: return "TREMOR";
    case 2: return "LANDSLIDE";
    case 3: return "SEISMIC";
    default: return "UNKNOWN";
  }
}

static const char* sourceName(int s) {
  switch (s) {
    case 1: return "MODEL";
    case 2: return "RULE";
    case 3: return "OVERRIDE";
    default: return "WARM";
  }
}

static void drawDashboard(SystemData data) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("AI DASH ");
  display.print(data.aiReady ? "ML" : "NO-ML");
  display.print(" ");
  display.print(sourceName(data.detectionSource));

  display.setCursor(0, 12);
  display.print("R:"); display.print(data.roll, 1);
  display.print(" P:"); display.print(data.pitch, 1);
  display.print(" T:"); display.print(data.tempC, 0);

  display.setCursor(0, 25);
  display.print("State:");
  display.print(className(data.aiClass));

  display.setCursor(0, 38);
  display.print("Conf:");
  display.print(data.aiConfidence * 100.0f, 0);
  display.print("% M:");
  display.print(data.motionScore, 2);

  display.setCursor(0, 51);
  display.print("A:");
  display.print(data.linearAcc, 2);
  display.print(" G:");
  display.print(data.gyroMag, 0);
  display.print(" Z:");
  display.print(data.accZ, 2);
}

static void drawOscilloscope(SystemData data) {
  for (int i = 0; i < 127; i++) {
    rollHistory[i] = rollHistory[i + 1];
    pitchHistory[i] = pitchHistory[i + 1];
    motionHistory[i] = motionHistory[i + 1];
  }

  rollHistory[127] = map((long)(data.roll * 100.0f), -4500, 4500, 31, 0);
  pitchHistory[127] = map((long)(data.pitch * 100.0f), -4500, 4500, 63, 32);
  motionHistory[127] = map((long)(data.motionScore * 100.0f), 0, 100, 63, 0);

  rollHistory[127] = constrain(rollHistory[127], 0, 31);
  pitchHistory[127] = constrain(pitchHistory[127], 32, 63);
  motionHistory[127] = constrain(motionHistory[127], 0, 63);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("ROLL/PITCH/MOTION");
  display.drawLine(0, 16, 127, 16, SSD1306_WHITE);
  display.drawLine(0, 48, 127, 48, SSD1306_WHITE);

  for (int i = 0; i < 127; i++) {
    display.drawLine(i, rollHistory[i], i + 1, rollHistory[i + 1], SSD1306_WHITE);
    display.drawLine(i, pitchHistory[i], i + 1, pitchHistory[i + 1], SSD1306_WHITE);
    if (i % 2 == 0) {
      display.drawPixel(i, motionHistory[i], SSD1306_WHITE);
    }
  }
}

static void drawCalibrationLevel(SystemData data) {
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2 + 4;

  display.drawCircle(centerX, centerY, 4, SSD1306_WHITE);
  display.drawCircle(centerX, centerY, 24, SSD1306_WHITE);
  display.drawLine(centerX - 30, centerY, centerX + 30, centerY, SSD1306_WHITE);
  display.drawLine(centerX, centerY - 30, centerX, centerY + 30, SSD1306_WHITE);

  int dotX = centerX + map((long)data.roll, -45, 45, -30, 30);
  int dotY = centerY + map((long)data.pitch, -45, 45, 30, -30);

  dotX = constrain(dotX, centerX - 30, centerX + 30);
  dotY = constrain(dotY, centerY - 30, centerY + 30);

  display.fillCircle(dotX, dotY, 3, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("X:"); display.print(data.roll, 1);
  display.setCursor(70, 0);
  display.print("Y:"); display.print(data.pitch, 1);
  display.setCursor(0, 56);
  display.print("UP = TARE");
}

void initDisplay() {
  displayReady = false;

  if (i2cMutex == NULL) {
    Serial.println("OLED skipped: mutex missing");
    return;
  }

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

    if (!displayReady) {
      Serial.println("OLED allocation failed");
      xSemaphoreGive(i2cMutex);
      return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(18, 18);
    display.setTextSize(2);
    display.print("SEISMIC");
    display.setCursor(26, 42);
    display.setTextSize(1);
    display.print("BOOTING...");
    display.display();
    xSemaphoreGive(i2cMutex);
  }

  for (int i = 0; i < 128; i++) {
    rollHistory[i] = 15;
    pitchHistory[i] = 47;
    motionHistory[i] = 63;
  }
}

void setScreenPage(ScreenPage newPage) {
  currentPage = newPage;
}

void updateDisplay(SystemData data) {
  if (!displayReady) {
    return;
  }

  display.clearDisplay();

  // Lower warning threshold for real testing. You can raise 0.60f later.
  if ((data.aiClass == 2 || data.aiClass == 3) && data.aiConfidence > 0.55f) {
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(8, 4);
    display.print("WARNING");

    display.setTextSize(1);
    display.setCursor(8, 28);
    display.print(className(data.aiClass));
    display.print(" DETECTED");

    display.setCursor(8, 42);
    display.print("Conf:");
    display.print(data.aiConfidence * 100.0f, 0);
    display.print("% ");
    display.print(sourceName(data.detectionSource));

    display.setCursor(8, 55);
    display.print("Motion:");
    display.print(data.motionScore, 2);

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      display.display();
      xSemaphoreGive(i2cMutex);
    }
    return;
  }

  switch (currentPage) {
    case PAGE_SPLASH:
      display.setTextSize(1);
      display.setCursor(20, 28);
      display.print("SEISMIC MONITOR");
      break;
    case PAGE_DASHBOARD:
      drawDashboard(data);
      break;
    case PAGE_OSCILLOSCOPE:
      drawOscilloscope(data);
      break;
    case PAGE_CALIBRATION_LEVEL:
      drawCalibrationLevel(data);
      break;
  }

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    display.display();
    xSemaphoreGive(i2cMutex);
  }
}
