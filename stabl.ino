#include <Wire.h>
#include "Globals.h"
#include "IMU_Module.h"
#include "BMP_Module.h"
#include "Display_Module.h"
#include "LightProximityAndGesture.h"

#define BUZZER_PIN 12

SemaphoreHandle_t i2cMutex;
unsigned long lastDisplayUpdate = 0;
unsigned long lastAlarmBeep = 0;

LightProximityAndGesture gestureSensor;

int pageIndex = 1;
bool telemetryPaused = false;
bool myosaGestureReady = false;

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

void playBeep(int frequency, int duration_ms) {
  tone(BUZZER_PIN, frequency, duration_ms);
}

void changePage(int newPage) {
  pageIndex = newPage;
  if (pageIndex < 1) pageIndex = 3;
  if (pageIndex > 3) pageIndex = 1;

  if (pageIndex == 1) setScreenPage(PAGE_DASHBOARD);
  else if (pageIndex == 2) setScreenPage(PAGE_OSCILLOSCOPE);
  else if (pageIndex == 3) setScreenPage(PAGE_CALIBRATION_LEVEL);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("===== SEISMIC MONITOR BOOT =====");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(BUZZER_PIN, OUTPUT);

  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println(">>> FATAL: Failed to create I2C mutex.");
    while (true) {
      delay(1000);
    }
  }

  initDisplay();
  initBMP();
  initIMU();

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    if (gestureSensor.ping()) {
      gestureSensor.enableGestureSensor();
      myosaGestureReady = true;
      Serial.println(">>> MYOSA Gesture Sensor Ready.");
    } else {
      myosaGestureReady = false;
      Serial.println(">>> MYOSA Gesture Sensor not found / disabled.");
    }
    xSemaphoreGive(i2cMutex);
  }

  delay(1200);
  changePage(1);

  playBeep(1000, 100);
  delay(150);
  playBeep(1500, 150);

  Serial.println("===== BOOT COMPLETE =====");
}

void loop() {
  SystemData sysData;
  sysData.roll = getRoll();
  sysData.pitch = getPitch();
  sysData.tempC = getTemperature();
  sysData.accX = getAccX();
  sysData.accY = getAccY();
  sysData.accZ = getAccZ();
  sysData.aiClass = getSeismicClass();
  sysData.aiConfidence = getSeismicConfidence();
  sysData.motionScore = getMotionScore();
  sysData.linearAcc = getLinearAcceleration();
  sysData.gyroMag = getGyroMagnitude();
  sysData.tiltMag = getTiltMagnitude();
  sysData.aiReady = isAIModelReady();
  sysData.detectionSource = getDetectionSource();

  String swipe = "NONE";
  if (myosaGestureReady && xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    swipe = String(gestureSensor.getGesture(false));
    xSemaphoreGive(i2cMutex);
  }

  if (swipe != "NONE") {
    if (swipe == "UP") {
      tareIMU();
      playBeep(2000, 200);
      Serial.println(">>> EVENT: IMU tared to zero.");
    } else if (swipe == "DOWN") {
      telemetryPaused = !telemetryPaused;
      playBeep(telemetryPaused ? 500 : 1200, telemetryPaused ? 300 : 150);
      Serial.println(telemetryPaused ? ">>> EVENT: Telemetry paused." : ">>> EVENT: Telemetry resumed.");
    } else if (swipe == "LEFT") {
      changePage(pageIndex + 1);
      playBeep(1500, 50);
      Serial.print(">>> EVENT: Page "); Serial.println(pageIndex);
    } else if (swipe == "RIGHT") {
      changePage(pageIndex - 1);
      playBeep(1500, 50);
      Serial.print(">>> EVENT: Page "); Serial.println(pageIndex);
    }
  }

  if (millis() - lastDisplayUpdate > 100) {
    bool danger = (sysData.aiClass == 2 || sysData.aiClass == 3) && sysData.aiConfidence > 0.55f;
    bool tremor = (sysData.aiClass == 1) && sysData.aiConfidence > 0.50f;

    if ((danger || tremor) && millis() - lastAlarmBeep > 350) {
      if (sysData.aiClass == 1) playBeep(1800, 40);
      else playBeep(2600, 70);
      lastAlarmBeep = millis();
    }

    Serial.print("R:"); Serial.print(sysData.roll, 1);
    Serial.print(" P:"); Serial.print(sysData.pitch, 1);
    Serial.print(" Ax:"); Serial.print(sysData.accX, 2);
    Serial.print(" Ay:"); Serial.print(sysData.accY, 2);
    Serial.print(" Az:"); Serial.print(sysData.accZ, 2);
    Serial.print(" LinA:"); Serial.print(sysData.linearAcc, 3);
    Serial.print(" GMag:"); Serial.print(sysData.gyroMag, 1);
    Serial.print(" Motion:"); Serial.print(sysData.motionScore, 2);
    Serial.print(" Class:"); Serial.print(className(sysData.aiClass));
    Serial.print(" Conf:"); Serial.print(sysData.aiConfidence * 100.0f, 0); Serial.print("%");
    Serial.print(" Src:"); Serial.print(sourceName(sysData.detectionSource));
    Serial.print(" Pg:"); Serial.print(pageIndex);
    Serial.print(" Pause:"); Serial.println(telemetryPaused ? "Y" : "N");

    if (!telemetryPaused) {
      updateDisplay(sysData);
    }

    lastDisplayUpdate = millis();
  }

  delay(10);
}
