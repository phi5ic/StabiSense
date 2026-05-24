#pragma once

// Optional APDS9960 module. Your main sketch currently uses LightProximityAndGesture directly,
// but these files are kept complete in case you switch back.
enum SwipeDirection {
  SWIPE_NONE,
  SWIPE_UP,
  SWIPE_DOWN,
  SWIPE_LEFT,
  SWIPE_RIGHT
};

bool initGesture();
SwipeDirection getGesture();
