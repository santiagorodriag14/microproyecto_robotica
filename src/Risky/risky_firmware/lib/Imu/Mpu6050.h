#pragma once

#include <Arduino.h>

#include "ImuSample.h"

// MPU-6050 o su clon MPU-6500 (el del robot) en el bus Wire: ±4 g, ±500 °/s, 200 Hz, filtro de 44 Hz.
// El acceso al bus es el de Prueba_IMU.ino, la versión que se probó con el chip del robot.
class Mpu6050 {
 public:
  explicit Mpu6050(uint8_t address = 0x68);
  // Wire ya debe estar iniciado. Tarda ~1,1 s: el chip necesita asentarse antes de la primera lectura.
  bool begin();
  // 0x68 en el MPU-6050 original, 0x70 en el MPU-6500.
  uint8_t whoAmI() const { return whoAmI_; }
  // Promedia el giroscopio con el robot quieto y resta ese bias en cada lectura.
  bool calibrateGyro(uint16_t samples);
  bool read(ImuSample& sample);

 private:
  uint8_t address_;
  uint8_t whoAmI_ = 0;
  float biasX_ = 0.f;
  float biasY_ = 0.f;
  float biasZ_ = 0.f;
};
