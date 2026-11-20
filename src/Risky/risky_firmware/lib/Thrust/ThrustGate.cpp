#include "ThrustGate.h"

ThrustGate::ThrustGate(float knobOffLevel) : knobOffLevel_(knobOffLevel) {}

float ThrustGate::shape(float knob) const {
  if (knob <= knobOffLevel_) return 0.f;
  const float level = (knob - knobOffLevel_) / (1.f - knobOffLevel_);
  return level > 1.f ? 1.f : level;
}

ThrustLevels ThrustGate::update(bool rcOk, ThrustLevels knobs) {
  if (!rcOk) return last_;
  const ThrustLevels shaped{shape(knobs.front), shape(knobs.rear)};
  if (!armed_ && shaped.front == 0.f && shaped.rear == 0.f) armed_ = true;
  last_ = armed_ ? shaped : ThrustLevels{0.f, 0.f};
  return last_;
}
