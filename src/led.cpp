#include "led.h"
#include "config.h"

namespace led {
namespace {

Mode g_mode = Mode::Off;
uint32_t g_phase_start = 0;

void write(bool on) {
  digitalWrite(cfg::kLedPin, (on ^ cfg::kLedActiveLow) ? HIGH : LOW);
}

}  // namespace

void begin() {
  pinMode(cfg::kLedPin, OUTPUT);
  write(false);
}

void setMode(Mode m) {
  if (m == g_mode) return;
  g_mode = m;
  g_phase_start = millis();
  write(false);
}

void loop() {
  const uint32_t t = millis() - g_phase_start;
  switch (g_mode) {
    case Mode::Off:       write(false); break;
    case Mode::SolidOn:   write(true);  break;
    case Mode::SlowBlink: write((t / 500) & 1); break;
    case Mode::FastBlink: write((t / 125) & 1); break;
    case Mode::Heartbeat: {
      const uint32_t c = t % 2000;
      write(c < 100 || (c >= 200 && c < 300));
      break;
    }
    case Mode::ErrorFlash: {
      const uint32_t c = t % 1500;
      write(c < 80 || (c >= 160 && c < 240) || (c >= 320 && c < 400));
      break;
    }
  }
}

}  // namespace led
