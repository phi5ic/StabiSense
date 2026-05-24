#pragma once
#include <Arduino.h>

// Call once from setup()
void initIMU();

// Sets the current roll/pitch position as zero
void tareIMU();

// Orientation values
float getRoll();
float getPitch();

// Raw acceleration in g units
float getAccX();
float getAccY();
float getAccZ();

// AI / detection output
// 0: Idle, 1: Tremor, 2: Landslide, 3: Seismic
int getSeismicClass();
float getSeismicConfidence();

// Debug / display helpers
float getMotionScore();
float getLinearAcceleration();
float getGyroMagnitude();
float getTiltMagnitude();
bool isAIModelReady();
int getDetectionSource(); // 0: none/warming, 1: TinyML model, 2: rule fallback, 3: rule override
