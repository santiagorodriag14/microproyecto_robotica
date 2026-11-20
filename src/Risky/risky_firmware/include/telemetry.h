#pragma once

#include "ThrustGate.h"

// Lo que el lazo de control comparte con la telemetría.
struct ControlState {
  float left;   // salida de cada lado de la base, [-1, 1]
  float right;
  ThrustLevels thrust;
};

namespace telemetry {
// Arranca la tarea de telemetría (WiFi, IMU, ToF y micro-ROS) en el núcleo 0. No bloquea.
void start();
// Solo copia el estado: es seguro llamarlo en cada ciclo de control.
void share(const ControlState& state);
}  // namespace telemetry
