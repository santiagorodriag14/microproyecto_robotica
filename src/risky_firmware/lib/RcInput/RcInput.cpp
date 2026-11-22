#include "RcInput.h"

namespace {
// Un ancho fuera de este rango es ruido o un flanco perdido.
constexpr uint32_t kMinPulseUs = 800;
constexpr uint32_t kMaxPulseUs = 2200;
}  // namespace

void RcChannel::begin(uint8_t pin) {
  pin_ = pin;
  pinMode(pin_, INPUT);
  attachInterruptArg(pin_, onEdge, this, CHANGE);
}

void ARDUINO_ISR_ATTR RcChannel::onEdge(void* arg) {
  auto* self = static_cast<RcChannel*>(arg);
  const uint32_t now = micros();
  if (digitalRead(self->pin_) == HIGH) {
    self->riseUs_ = now;
    return;
  }
  const uint32_t width = now - self->riseUs_;
  if (width >= kMinPulseUs && width <= kMaxPulseUs) {
    self->widths_[self->next_] = width;
    self->next_ = (self->next_ + 1) % kWindow;
    if (self->count_ < kWindow) ++self->count_;
    self->lastPulseUs_ = now;
  }
}

bool RcChannel::read(uint16_t& widthUs, uint32_t timeoutUs) const {
  // Cada lectura de 32 bits es atómica en el ESP32. Si la interrupción escribe mientras se copian
  // las tres, se mezclan pulsos de tramas vecinas, que siguen siendo muestras válidas.
  const uint32_t last = lastPulseUs_;
  if (count_ < kWindow || micros() - last > timeoutUs) return false;
  const uint32_t a = widths_[0], b = widths_[1], c = widths_[2];
  const uint32_t median = max(min(a, b), min(max(a, b), c));
  widthUs = static_cast<uint16_t>(median);
  return true;
}

float rcBipolar(uint16_t us, const RcRange& range, float deadband) {
  const float offset = static_cast<float>(us) - range.centerUs;
  const float span = offset >= 0.f ? range.maxUs - range.centerUs : range.centerUs - range.minUs;
  const float x = constrain(offset / span, -1.f, 1.f);
  const float magnitude = fabsf(x);
  if (magnitude <= deadband) return 0.f;
  return copysignf((magnitude - deadband) / (1.f - deadband), x);
}

float rcUnipolar(uint16_t us, const RcRange& range) {
  const float x = (static_cast<float>(us) - range.minUs) / (range.maxUs - range.minUs);
  return constrain(x, 0.f, 1.f);
}
