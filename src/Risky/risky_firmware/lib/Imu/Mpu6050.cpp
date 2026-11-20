#include "Mpu6050.h"

#include <Wire.h>

namespace {
constexpr uint8_t kRegSampleRateDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXout = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

constexpr uint8_t kDeviceReset = 0x80;
constexpr uint8_t kClockPllGyroX = 0x01;
constexpr uint8_t kRate200Hz = 4;  // 1 kHz / (1 + 4)
constexpr uint8_t kDlpf44Hz = 3;
constexpr uint8_t kGyro500Dps = 1 << 3;
constexpr uint8_t kAccel4G = 1 << 3;

constexpr float kAccelLsbPerG = 8192.f;
constexpr float kGyroLsbPerRadS = 65.5f * RAD_TO_DEG;

bool escribir(uint8_t mpu, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpu);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool leer(uint8_t mpu, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(mpu);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpu, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
}  // namespace

Mpu6050::Mpu6050(uint8_t address) : address_(address) {}

bool Mpu6050::begin() {
  if (!leer(address_, kRegWhoAmI, &whoAmI_, 1)) return false;
  bool ok = escribir(address_, kRegPwrMgmt1, kDeviceReset);
  delay(100);
  ok &= escribir(address_, kRegPwrMgmt1, kClockPllGyroX);
  delay(10);
  ok &= escribir(address_, kRegSampleRateDiv, kRate200Hz);
  ok &= escribir(address_, kRegConfig, kDlpf44Hz);
  ok &= escribir(address_, kRegGyroConfig, kGyro500Dps);
  ok &= escribir(address_, kRegAccelConfig, kAccel4G);
  delay(1000);
  return ok;
}

bool Mpu6050::calibrateGyro(uint16_t samples) {
  biasX_ = biasY_ = biasZ_ = 0.f;
  float sumX = 0.f, sumY = 0.f, sumZ = 0.f;
  ImuSample sample;
  for (uint16_t i = 0; i < samples; ++i) {
    if (!read(sample)) return false;
    sumX += sample.gx;
    sumY += sample.gy;
    sumZ += sample.gz;
    delay(3);
  }
  biasX_ = sumX / samples;
  biasY_ = sumY / samples;
  biasZ_ = sumZ / samples;
  return true;
}

bool Mpu6050::read(ImuSample& sample) {
  uint8_t b[14];
  if (!leer(address_, kRegAccelXout, b, 14)) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  sample.ax = raw[0] / kAccelLsbPerG;
  sample.ay = raw[1] / kAccelLsbPerG;
  sample.az = raw[2] / kAccelLsbPerG;
  sample.gx = raw[4] / kGyroLsbPerRadS - biasX_;
  sample.gy = raw[5] / kGyroLsbPerRadS - biasY_;
  sample.gz = raw[6] / kGyroLsbPerRadS - biasZ_;
  return true;
}
