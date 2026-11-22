#include "Tof.h"

namespace {
// El sensor reporta 8190 u 8191 cuando no ve objetivo.
constexpr uint16_t kOutOfRangeMm = 8190;
constexpr uint16_t kI2cTimeoutMs = 50;
}  // namespace

Tof::Tof(TwoWire& wire) { sensor_.setBus(&wire); }

bool Tof::begin(uint32_t timingBudgetUs, uint32_t periodMs, int16_t offsetMm) {
  offsetMm_ = offsetMm;
  sensor_.setTimeout(kI2cTimeoutMs);
  if (!sensor_.init() || !sensor_.setMeasurementTimingBudget(timingBudgetUs)) return false;
  sensor_.startContinuous(periodMs);
  return true;
}

bool Tof::poll(uint16_t& mm) {
  // Con el bit de dato listo activo, readRangeContinuousMillimeters() ya no espera.
  if ((sensor_.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  const uint16_t raw = sensor_.readRangeContinuousMillimeters();
  if (sensor_.timeoutOccurred()) return false;
  mm = raw >= kOutOfRangeMm ? kNoTarget : static_cast<uint16_t>(max(0, raw + offsetMm_));
  return true;
}
