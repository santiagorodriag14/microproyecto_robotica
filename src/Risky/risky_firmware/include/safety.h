#pragma once

#include <Arduino.h>

#include "config.h"

// Para los bancos que no usan motores ni turbinas. Con STBY del TB6612 fijo en alto, un pin flotante
// puede mover los motores y un ESC puede leer ruido como pulso; en bajo, ruedas libres y ESC sin señal.
inline void parkActuators() {
  for (uint8_t p : {pin::kLeftPwm, pin::kLeftIn1, pin::kLeftIn2, pin::kRightPwm, pin::kRightIn1,
                    pin::kRightIn2, pin::kEsc1, pin::kEsc2}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}
