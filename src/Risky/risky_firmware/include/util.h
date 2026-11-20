#pragma once

#include <Arduino.h>

class Periodic {
 public:
  explicit Periodic(uint32_t periodMs) : periodMs_(periodMs) {}
  bool due(uint32_t nowMs) {
    if (nowMs - lastMs_ < periodMs_) return false;
    lastMs_ = nowMs;
    return true;
  }

 private:
  uint32_t periodMs_;
  uint32_t lastMs_ = 0;
};

[[noreturn]] inline void halt(const char* reason) {
  Serial.printf("ERROR: %s\n", reason);
  while (true) delay(1000);
}
