#include "Gesture_Module.h"
#include "Globals.h"
#include <Adafruit_APDS9960.h>

static Adafruit_APDS9960 apds;
static bool gestureReady = false;

bool initGesture() {
  gestureReady = false;

  if (i2cMutex == NULL) {
    return false;
  }

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    if (apds.begin()) {
      apds.enableProximity(true);
      apds.enableGesture(true);
      gestureReady = true;
    }
    xSemaphoreGive(i2cMutex);
  }

  Serial.println(gestureReady ? ">>> APDS9960 gesture ready" : ">>> APDS9960 gesture not found / disabled");
  return gestureReady;
}

SwipeDirection getGesture() {
  if (!gestureReady || i2cMutex == NULL) {
    return SWIPE_NONE;
  }

  SwipeDirection result = SWIPE_NONE;

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    if (apds.gestureValid()) {
      uint8_t gesture = apds.readGesture();

      switch (gesture) {
        case APDS9960_UP:    result = SWIPE_UP;    break;
        case APDS9960_DOWN:  result = SWIPE_DOWN;  break;
        case APDS9960_LEFT:  result = SWIPE_LEFT;  break;
        case APDS9960_RIGHT: result = SWIPE_RIGHT; break;
        default:             result = SWIPE_NONE;  break;
      }
    }
    xSemaphoreGive(i2cMutex);
  }

  return result;
}
