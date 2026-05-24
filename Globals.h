#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Shared I2C pins for the MyOSA kit / ESP32
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// Global lock for the I2C bus
extern SemaphoreHandle_t i2cMutex;
