#pragma once

struct ThrustLevels {
  float front;
  float rear;
};

// Convierte las perillas en niveles de turbina [0, 1] con las reglas de seguridad:
// - no arma hasta ver ambas perillas en apagado, para que nada arranque de golpe al encender;
// - sin señal RC mantiene la última salida, porque cortar las turbinas en la pared hace caer el robot.
class ThrustGate {
 public:
  explicit ThrustGate(float knobOffLevel);
  ThrustLevels update(bool rcOk, ThrustLevels knobs);
  bool armed() const { return armed_; }

 private:
  // Por debajo de knobOffLevel da 0; arriba re-escala a [0, 1] para no saltar al salir del apagado.
  float shape(float knob) const;

  float knobOffLevel_;
  bool armed_ = false;
  ThrustLevels last_{0.f, 0.f};
};
