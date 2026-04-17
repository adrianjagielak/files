#pragma once
#include <Arduino.h>

// Simple non-blocking status LED with named patterns.
namespace led {

enum class Mode {
  Off,
  SolidOn,
  SlowBlink,    // 1 Hz — waiting for configuration.
  FastBlink,    // 4 Hz — pairing in progress.
  Heartbeat,    // double-blink once per 2 s — connected, healthy.
  ErrorFlash,   // rapid triple-flash burst.
};

void begin();
void setMode(Mode m);
void loop();      // call from main loop

}  // namespace led
