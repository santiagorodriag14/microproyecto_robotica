// Banco 5: VL53L0X. Cada 0,5 s imprime el promedio de la distancia corregida y de la cruda.
// Calibración: con el robot en el suelo, tof::kOffsetMm = distancia real del sensor al suelo − promedio crudo.
#include <Arduino.h>
#include <Wire.h>

#include "Tof.h"
#include "config.h"
#include "safety.h"
#include "util.h"

namespace {
constexpr uint32_t kReporteMs = 500;

Tof tofSensor(Wire);
Periodic poll(kControlPeriodMs);
Periodic reporte(kReporteMs);
uint32_t ultimaMs = 0;
uint32_t suma = 0;
uint16_t medidas = 0;
uint16_t sinSuperficie = 0;
}  // namespace

void setup() {
  parkActuators();
  Serial.begin(115200);
  Wire.begin(pin::kSda, pin::kScl, kI2cHz);
  if (!tofSensor.begin(tof::kTimingBudgetUs, tof::kPeriodMs, tof::kOffsetMm)) {
    halt("el VL53L0X no responde por I2C");
  }
  Serial.printf("Offset actual: %d mm\n", tof::kOffsetMm);
}

void loop() {
  const uint32_t ahora = millis();
  uint16_t mm;
  if (poll.due(ahora) && tofSensor.poll(mm)) {
    ultimaMs = ahora;
    if (mm == Tof::kNoTarget) {
      ++sinSuperficie;
    } else {
      suma += mm;
      ++medidas;
    }
  }
  if (!reporte.due(ahora)) return;
  if (ahora - ultimaMs > kReporteMs) Serial.println("AVISO: sin medidas nuevas");
  if (medidas > 0) {
    const float promedio = static_cast<float>(suma) / medidas;
    Serial.printf("corregida %6.1f mm | cruda %6.1f mm | %u medidas\n", promedio, promedio - tof::kOffsetMm,
                  medidas);
  } else if (sinSuperficie > 0) {
    Serial.println("sin superficie a la vista");
  }
  suma = medidas = sinSuperficie = 0;
}
