#include "BMP_Module.h"
#include "Globals.h"
#include <Adafruit_BMP085.h>

static Adafruit_BMP085 bmp;
static bool bmpReady = false;

bool initBMP() {
  bmpReady = false;

  if (i2cMutex == NULL) {
    return false;
  }

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    bmpReady = bmp.begin();
    xSemaphoreGive(i2cMutex);
  }

  Serial.println(bmpReady ? ">>> BMP ready" : ">>> BMP not found / disabled");
  return bmpReady;
}

float getTemperature() {
  if (!bmpReady || i2cMutex == NULL) {
    return 0.0f;
  }

  float temp = 0.0f;
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    temp = bmp.readTemperature();
    xSemaphoreGive(i2cMutex);
  }
  return temp;
}

float getPressure() {
  if (!bmpReady || i2cMutex == NULL) {
    return 0.0f;
  }

  float pressure = 0.0f;
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    pressure = bmp.readPressure();
    xSemaphoreGive(i2cMutex);
  }
  return pressure;
}
