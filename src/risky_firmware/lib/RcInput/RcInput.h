#pragma once

#include <Arduino.h>

struct RcRange {
  uint16_t minUs;
  uint16_t centerUs;
  uint16_t maxUs;
};

// Mide el ancho de pulso de un canal PWM del receptor con una interrupción por flanco.
// Entrega la mediana de los últimos 3 pulsos: un pico aislado se descarta en vez de promediarse.
class RcChannel {
 public:
  void begin(uint8_t pin);
  // false hasta juntar 3 pulsos válidos, o si el último es más viejo que timeoutUs.
  bool read(uint16_t& widthUs, uint32_t timeoutUs) const;

 private:
  static constexpr uint8_t kWindow = 3;

  static void onEdge(void* arg);

  uint8_t pin_ = 0;
  volatile uint32_t riseUs_ = 0;
  volatile uint32_t widths_[kWindow] = {};
  volatile uint8_t next_ = 0;
  volatile uint8_t count_ = 0;
  volatile uint32_t lastPulseUs_ = 0;
};

// [-1, 1] alrededor del centro. La zona muerta se re-escala para no saltar al salir de ella.
float rcBipolar(uint16_t us, const RcRange& range, float deadband);
// [0, 1] de minUs a maxUs.
float rcUnipolar(uint16_t us, const RcRange& range);
