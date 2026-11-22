// Banco 4b: el driver Mpu6050 que usan el firmware principal y bench_ros. Debe dar lo mismo que bench_imu.
#include <Arduino.h>
#include <Wire.h>

#include "Mpu6050.h"
#include "config.h"
#include "safety.h"
#include "util.h"

namespace {
Mpu6050 imuSensor(imu::kAddress);
Periodic report(200);
}  // namespace

void setup() {
  parkActuators();
  Serial.begin(115200);
  Wire.begin(pin::kSda, pin::kScl, kI2cHz);
  if (!imuSensor.begin()) halt("el MPU no responde por I2C");
  Serial.printf("WHO_AM_I = 0x%02X\n", imuSensor.whoAmI());
  Serial.println("Calibrando giroscopio, no mover...");
  if (!imuSensor.calibrateGyro(imu::kGyroCalibrationSamples)) halt("falló la lectura del MPU");
}

void loop() {
  if (!report.due(millis())) return;
  ImuSample s;
  if (!imuSensor.read(s)) {
    Serial.println("lectura fallida");
    return;
  }
  Serial.printf("A[g] %+6.3f %+6.3f %+6.3f | G[rad/s] %+6.2f %+6.2f %+6.2f\n", s.ax, s.ay, s.az, s.gx, s.gy,
                s.gz);
}
