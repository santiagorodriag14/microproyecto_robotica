#pragma once

#include <stdint.h>

enum class Surface : uint8_t { Floor, Transition, Wall, Ceiling };

inline const char* toString(Surface s) {
  switch (s) {
    case Surface::Floor: return "suelo";
    case Surface::Transition: return "transicion";
    case Surface::Wall: return "pared";
    case Surface::Ceiling: return "techo";
  }
  return "?";
}

struct SurfaceBands {
  float floorMaxDeg;
  float wallMinDeg;
  float wallMaxDeg;
  float ceilingMinDeg;
  // Tiempo que el ángulo debe quedarse en una banda estable antes de aceptarla.
  uint32_t settleMs;
};

// Clasifica la superficie según la inclinación. Entrar en transición es inmediato
// (las turbinas deben reaccionar ya); asentarse en suelo, pared o techo exige settleMs.
class SurfaceClassifier {
 public:
  explicit SurfaceClassifier(const SurfaceBands& bands);
  Surface update(float tiltDeg, uint32_t nowMs);
  Surface current() const { return current_; }

 private:
  Surface band(float tiltDeg) const;

  SurfaceBands bands_;
  Surface current_ = Surface::Floor;
  Surface candidate_ = Surface::Floor;
  uint32_t candidateSinceMs_ = 0;
};
