#include "Esc.h"

Esc::Esc(uint8_t pin, uint8_t ledcChannel, uint16_t minUs, uint16_t maxUs)
    : pin_(pin), channel_(ledcChannel), minUs_(minUs), maxUs_(maxUs) {}

bool Esc::begin(uint32_t frameHz) {
  if (ledcSetup(channel_, frameHz, kResolutionBits) == 0) return false;
  frameHz_ = frameHz;
  ledcAttachPin(pin_, channel_);
  writeUs(minUs_);
  return true;
}

void Esc::writeUs(uint16_t us) {
  us = constrain(us, minUs_, maxUs_);
  const uint64_t fullScale = (1u << kResolutionBits) - 1;
  ledcWrite(channel_, static_cast<uint32_t>(us * fullScale * frameHz_ / 1000000));
}

void Esc::setLevel(float level) {
  level = constrain(level, 0.f, 1.f);
  writeUs(static_cast<uint16_t>(minUs_ + level * (maxUs_ - minUs_) + 0.5f));
}
