#include "MotorSide.h"

MotorSide::MotorSide(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, uint8_t ledcChannel,
                     bool inverted)
    : pwmPin_(pwmPin), in1Pin_(in1Pin), in2Pin_(in2Pin), channel_(ledcChannel), inverted_(inverted) {}

bool MotorSide::begin(uint32_t pwmHz, uint8_t resolutionBits) {
  if (ledcSetup(channel_, pwmHz, resolutionBits) == 0) return false;
  ledcAttachPin(pwmPin_, channel_);
  maxDuty_ = (1u << resolutionBits) - 1;
  pinMode(in1Pin_, OUTPUT);
  pinMode(in2Pin_, OUTPUT);
  set(0.f);
  return true;
}

void MotorSide::set(float command) {
  command = constrain(command, -1.f, 1.f);
  if (inverted_) command = -command;
  const bool brake = command == 0.f;
  digitalWrite(in1Pin_, brake || command > 0.f ? HIGH : LOW);
  digitalWrite(in2Pin_, brake || command < 0.f ? HIGH : LOW);
  ledcWrite(channel_, static_cast<uint32_t>(fabsf(command) * maxDuty_));
}
