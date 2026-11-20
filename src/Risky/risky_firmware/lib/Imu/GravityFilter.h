#pragma once

#include "ImuSample.h"

// Filtro complementario sobre el vector "arriba" expresado en los ejes del sensor.
// No usa ángulos de Euler, así que no se degenera al pasar por pared (90°) ni techo (180°).
class GravityFilter {
 public:
  // accelWeight: cuánto se corrige hacia el acelerómetro en cada paso (0.02 a 100 Hz ≈ 0.5 s).
  explicit GravityFilter(float accelWeight);
  void reset(const ImuSample& sample);
  void update(const ImuSample& sample, float dtS);
  // Ángulo entre el eje z del sensor y la vertical: 0° suelo, 90° pared, 180° techo.
  float tiltDeg() const;

 private:
  float accelWeight_;
  float upX_ = 0.f;
  float upY_ = 0.f;
  float upZ_ = 1.f;
};
