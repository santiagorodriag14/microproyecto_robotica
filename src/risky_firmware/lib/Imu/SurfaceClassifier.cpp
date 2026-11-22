#include "SurfaceClassifier.h"

SurfaceClassifier::SurfaceClassifier(const SurfaceBands& bands) : bands_(bands) {}

Surface SurfaceClassifier::band(float tiltDeg) const {
  if (tiltDeg <= bands_.floorMaxDeg) return Surface::Floor;
  if (tiltDeg >= bands_.ceilingMinDeg) return Surface::Ceiling;
  if (tiltDeg >= bands_.wallMinDeg && tiltDeg <= bands_.wallMaxDeg) return Surface::Wall;
  return Surface::Transition;
}

Surface SurfaceClassifier::update(float tiltDeg, uint32_t nowMs) {
  const Surface raw = band(tiltDeg);
  if (raw == Surface::Transition || raw == current_) {
    current_ = raw;
    candidate_ = raw;
    return current_;
  }
  if (raw != candidate_) {
    candidate_ = raw;
    candidateSinceMs_ = nowMs;
  } else if (nowMs - candidateSinceMs_ >= bands_.settleMs) {
    current_ = raw;
  }
  return current_;
}
