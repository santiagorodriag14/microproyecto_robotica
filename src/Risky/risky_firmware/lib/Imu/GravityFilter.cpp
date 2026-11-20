#include "GravityFilter.h"

#include <math.h>

namespace {
// Si |a| se aleja de 1 g hay aceleración o vibración fuerte: en ese paso solo se integra el giroscopio.
constexpr float kAccelTrustBandG = 0.15f;
constexpr float kRadToDeg = 57.2957795f;
}  // namespace

GravityFilter::GravityFilter(float accelWeight) : accelWeight_(accelWeight) {}

void GravityFilter::reset(const ImuSample& s) {
  const float norm = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  if (norm <= 0.f) return;
  upX_ = s.ax / norm;
  upY_ = s.ay / norm;
  upZ_ = s.az / norm;
}

void GravityFilter::update(const ImuSample& s, float dtS) {
  // Un vector fijo en el mundo, visto desde el sensor, gira con -ω: v' = v - (ω × v)·dt.
  float x = upX_ - (s.gy * upZ_ - s.gz * upY_) * dtS;
  float y = upY_ - (s.gz * upX_ - s.gx * upZ_) * dtS;
  float z = upZ_ - (s.gx * upY_ - s.gy * upX_) * dtS;

  // En reposo el acelerómetro mide +1 g hacia arriba.
  const float accelNorm = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  if (fabsf(accelNorm - 1.f) < kAccelTrustBandG) {
    const float w = accelWeight_;
    x = (1.f - w) * x + w * s.ax / accelNorm;
    y = (1.f - w) * y + w * s.ay / accelNorm;
    z = (1.f - w) * z + w * s.az / accelNorm;
  }

  const float norm = sqrtf(x * x + y * y + z * z);
  upX_ = x / norm;
  upY_ = y / norm;
  upZ_ = z / norm;
}

float GravityFilter::tiltDeg() const {
  return atan2f(sqrtf(upX_ * upX_ + upY_ * upY_), upZ_) * kRadToDeg;
}
