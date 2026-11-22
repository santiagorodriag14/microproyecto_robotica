#pragma once

#include <Arduino.h>

// Un lado del robot: dos TB6612FNG en paralelo que comparten PWM, IN1 e IN2.
class MotorSide {
 public:
  MotorSide(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, uint8_t ledcChannel, bool inverted);
  bool begin(uint32_t pwmHz, uint8_t resolutionBits);
  // command en [-1, 1]. Con 0 el lado queda en freno corto (IN1 = IN2 = alto), no en rueda libre.
  void set(float command);

 private:
  uint8_t pwmPin_;
  uint8_t in1Pin_;
  uint8_t in2Pin_;
  uint8_t channel_;
  bool inverted_;
  uint32_t maxDuty_ = 0;
};
