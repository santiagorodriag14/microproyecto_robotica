// Banco 6: telemetría micro-ROS. Publica /risky/imu/data_raw y /risky/tof/range con los sensores reales.
#include <Arduino.h>
#include <Wire.h>

#include "Mpu6050.h"
#include "RosLink.h"
#include "Tof.h"
#include "config.h"
#include "safety.h"
#include "util.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copie include/secrets.example.h como include/secrets.h y llene la red y la IP del agente"
#endif

namespace {
constexpr uint32_t kImuPublishPeriodMs = 50;

Mpu6050 imuSensor(imu::kAddress);
Tof tofSensor(Wire);
RosLink ros;
Periodic tofPoll(kControlPeriodMs);
Periodic imuPublish(kImuPublishPeriodMs);
bool wasConnected = false;
}  // namespace

void setup() {
  parkActuators();
  Serial.begin(115200);
  Wire.begin(pin::kSda, pin::kScl, kI2cHz);
  if (!imuSensor.begin()) halt("el MPU-6050 no responde por I2C");
  if (!tofSensor.begin(tof::kTimingBudgetUs, tof::kPeriodMs, tof::kOffsetMm)) halt("el VL53L0X no responde por I2C");
  if (!imuSensor.calibrateGyro(imu::kGyroCalibrationSamples)) halt("falló la lectura del MPU-6050");

  Serial.printf("Conectando a la red %s...\n", secrets::kWifiSsid);
  ros.begin({secrets::kWifiSsid, secrets::kWifiPassword, secrets::kAgentIp, secrets::kAgentPort});
  Serial.println("WiFi conectado. Esperando al agente micro-ROS...");
}

void loop() {
  ros.spin();
  if (ros.connected() != wasConnected) {
    wasConnected = ros.connected();
    Serial.println(wasConnected ? "Agente conectado, publicando." : "Agente perdido, reintentando...");
  }

  const uint32_t now = millis();
  uint16_t mm;
  if (tofPoll.due(now) && tofSensor.poll(mm)) {
    ros.publishRange(mm == Tof::kNoTarget ? INFINITY : mm / 1000.f);
  }
  ImuSample sample;
  if (imuPublish.due(now) && imuSensor.read(sample)) ros.publishImu(sample);
}
