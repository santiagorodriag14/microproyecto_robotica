#pragma once

#include <VL53L0X.h>
#include <Wire.h>

// VL53L0X en modo continuo, leído sin bloquear para no detener el lazo de control.
class Tof {
 public:
  static constexpr uint16_t kNoTarget = UINT16_MAX;

  explicit Tof(TwoWire& wire);
  // offsetMm se suma a cada medida (calibración en bench_tof).
  bool begin(uint32_t timingBudgetUs, uint32_t periodMs, int16_t offsetMm);
  // true cuando hay una medida nueva. mm = kNoTarget si no hay superficie dentro del alcance.
  bool poll(uint16_t& mm);

 private:
  VL53L0X sensor_;
  int16_t offsetMm_ = 0;
};
