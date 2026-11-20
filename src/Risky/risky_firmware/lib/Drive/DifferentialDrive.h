#pragma once

#include "MotorSide.h"

// Mezcla avance y giro en una velocidad por lado y limita su tasa de cambio:
// un tirón de torque puede vencer la fricción y despegar el robot de la pared.
class DifferentialDrive {
 public:
  DifferentialDrive(MotorSide& left, MotorSide& right, float slewPerS);
  // throttle y turn en [-1, 1]; turn > 0 gira a la derecha.
  void setTarget(float throttle, float turn);
  void update(float dtS);
  // Freno inmediato, sin rampa.
  void brake();

  float left() const { return outLeft_; }
  float right() const { return outRight_; }

 private:
  MotorSide& left_;
  MotorSide& right_;
  float slewPerS_;
  float targetLeft_ = 0.f;
  float targetRight_ = 0.f;
  float outLeft_ = 0.f;
  float outRight_ = 0.f;
};
