#pragma once

#include <Arduino.h>

// Señal tipo servo para un ESC: pulso de minUs (parado) a maxUs (máximo).
class Esc {
 public:
  Esc(uint8_t pin, uint8_t ledcChannel, uint16_t minUs, uint16_t maxUs);
  // Arranca enviando minUs: el ESC solo se arma si ve el mínimo al energizarse.
  bool begin(uint32_t frameHz);
  // Se limita a [minUs, maxUs].
  void writeUs(uint16_t us);
  // level en [0, 1].
  void setLevel(float level);

 private:
  static constexpr uint8_t kResolutionBits = 16;

  uint8_t pin_;
  uint8_t channel_;
  uint16_t minUs_;
  uint16_t maxUs_;
  uint32_t frameHz_ = 0;
};
