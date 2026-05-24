#pragma once
#include <Arduino.h>

enum ScreenPage {
  PAGE_SPLASH,
  PAGE_DASHBOARD,
  PAGE_OSCILLOSCOPE,
  PAGE_CALIBRATION_LEVEL
};

struct SystemData {
  float roll;
  float pitch;
  float tempC;
  float accX;
  float accY;
  float accZ;

  int aiClass;             // 0 Idle, 1 Tremor, 2 Landslide, 3 Seismic
  float aiConfidence;      // 0.0 to 1.0
  float motionScore;       // rule-based motion strength
  float linearAcc;         // abs(acc magnitude - 1g)
  float gyroMag;           // deg/s magnitude
  float tiltMag;           // abs roll + abs pitch
  bool aiReady;            // TinyML loaded successfully
  int detectionSource;     // 0 none/warming, 1 TinyML, 2 rule fallback, 3 rule override
};

void initDisplay();
void setScreenPage(ScreenPage newPage);
void updateDisplay(SystemData data);
