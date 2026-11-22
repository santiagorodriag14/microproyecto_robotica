#include "DifferentialDrive.h"

namespace {
float approach(float current, float target, float maxStep) {
  return target > current ? fminf(current + maxStep, target) : fmaxf(current - maxStep, target);
}
}  // namespace

DifferentialDrive::DifferentialDrive(MotorSide& left, MotorSide& right, float slewPerS)
    : left_(left), right_(right), slewPerS_(slewPerS) {}

void DifferentialDrive::setTarget(float throttle, float turn) {
  float left = throttle + turn;
  float right = throttle - turn;
  // Escalar ambos lados por igual conserva la curva pedida cuando uno se satura.
  const float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > 1.f) {
    left /= peak;
    right /= peak;
  }
  targetLeft_ = left;
  targetRight_ = right;
}

void DifferentialDrive::update(float dtS) {
  const float step = slewPerS_ * dtS;
  outLeft_ = approach(outLeft_, targetLeft_, step);
  outRight_ = approach(outRight_, targetRight_, step);
  left_.set(outLeft_);
  right_.set(outRight_);
}

void DifferentialDrive::brake() {
  targetLeft_ = targetRight_ = outLeft_ = outRight_ = 0.f;
  left_.set(0.f);
  right_.set(0.f);
}
