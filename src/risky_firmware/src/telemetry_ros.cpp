// Telemetría del firmware principal: publica IMU, ToF, comando de la base y turbinas por micro-ROS.
// Corre en su propia tarea para que el WiFi y un agente caído nunca frenen el lazo de control.
#include <Arduino.h>
#include <Wire.h>

#include "Mpu6050.h"
#include "RosLink.h"
#include "Tof.h"
#include "config.h"
#include "telemetry.h"
#include "util.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copie include/secrets.example.h como include/secrets.h y llene la red y la IP del agente"
#endif

namespace {
constexpr uint32_t kStackBytes = 16384;
constexpr UBaseType_t kPriority = 1;
constexpr BaseType_t kCore = 0;  // loop() de Arduino, y con él el control, corre en el núcleo 1

Mpu6050 imuSensor(imu::kAddress);
Tof tofSensor(Wire);
RosLink ros;

portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;
ControlState sharedState{};

ControlState latestState() {
  portENTER_CRITICAL(&stateLock);
  const ControlState state = sharedState;
  portEXIT_CRITICAL(&stateLock);
  return state;
}

void publishState() {
  const ControlState s = latestState();
  const float linear = (s.left + s.right) / 2.f * drive::kMaxWheelSpeedMps;
  // turn > 0 gira a la derecha; en ROS el giro positivo es antihorario.
  const float angular = (s.right - s.left) * drive::kMaxWheelSpeedMps / drive::kTrackWidthM;
  ros.publishCmdVel(linear, angular);
  ros.publishThrust(s.thrust.front, s.thrust.rear);
}

[[noreturn]] void run(void*) {
  // Solo esta tarea usa el bus I2C.
  Wire.begin(pin::kSda, pin::kScl, kI2cHz);
  // La calibración del giroscopio asume el robot quieto durante el primer par de segundos.
  const bool imuOk = imuSensor.begin() && imuSensor.calibrateGyro(imu::kGyroCalibrationSamples);
  const bool tofOk = tofSensor.begin(tof::kTimingBudgetUs, tof::kPeriodMs, tof::kOffsetMm);
  if (!imuOk) Serial.println("AVISO: el MPU-6050 no responde, no se publica /risky/imu/data_raw");
  if (!tofOk) Serial.println("AVISO: el VL53L0X no responde, no se publica /risky/tof/range");

  Serial.printf("Conectando a la red %s...\n", secrets::kWifiSsid);
  ros.begin({secrets::kWifiSsid, secrets::kWifiPassword, secrets::kAgentIp, secrets::kAgentPort});
  Serial.println("WiFi conectado. Esperando al agente micro-ROS...");

  Periodic tofPoll(kControlPeriodMs);
  Periodic imuPublish(telemetry::kImuPeriodMs);
  Periodic statePublish(telemetry::kStatePeriodMs);
  bool wasConnected = false;
  while (true) {
    ros.spin();
    if (ros.connected() != wasConnected) {
      wasConnected = ros.connected();
      Serial.println(wasConnected ? "Agente conectado, publicando." : "Agente perdido, reintentando...");
    }

    const uint32_t now = millis();
    uint16_t mm;
    if (tofOk && tofPoll.due(now) && tofSensor.poll(mm)) {
      ros.publishRange(mm == Tof::kNoTarget ? INFINITY : mm / 1000.f);
    }
    ImuSample sample;
    if (imuOk && imuPublish.due(now) && imuSensor.read(sample)) ros.publishImu(sample);
    if (statePublish.due(now)) publishState();
    vTaskDelay(1);
  }
}
}  // namespace

void telemetry::start() {
  if (xTaskCreatePinnedToCore(run, "telemetry", kStackBytes, nullptr, kPriority, nullptr, kCore) != pdPASS) {
    Serial.println("AVISO: no se pudo crear la tarea de telemetría; el robot sigue en modo manual");
  }
}

void telemetry::share(const ControlState& state) {
  portENTER_CRITICAL(&stateLock);
  sharedState = state;
  portEXIT_CRITICAL(&stateLock);
}
