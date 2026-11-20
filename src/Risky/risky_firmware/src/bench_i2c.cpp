// Banco I2C: revisa las líneas del bus y busca dispositivos cada segundo a 400 kHz y a 100 kHz.
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "safety.h"
#include "util.h"

namespace {
constexpr uint32_t kSlowHz = 100000;

Periodic scanTimer(1000);
unsigned scans = 0;

const char* deviceName(uint8_t address) {
  switch (address) {
    case 0x29: return "VL53L0X";
    case 0x68:
    case 0x69: return "MPU-6050";
    default: return "desconocido";
  }
}

// En reposo SDA y SCL deben quedar en alto por los pull-ups. Una línea en bajo es un dispositivo
// trabado a mitad de una transferencia o un corto, y con eso ningún escaneo funciona.
void checkIdleLines() {
  pinMode(pin::kSda, INPUT);
  pinMode(pin::kScl, INPUT);
  delay(5);
  Serial.printf("Líneas en reposo: SDA %s, SCL %s\n", digitalRead(pin::kSda) ? "alto" : "BAJO",
                digitalRead(pin::kScl) ? "alto" : "BAJO");
}

void scan(uint32_t hz) {
  Wire.setClock(hz);
  Serial.printf("  %3u kHz:", static_cast<unsigned>(hz / 1000));
  unsigned found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X %s", address, deviceName(address));
      ++found;
    }
  }
  Serial.println(found ? "" : "  ninguno");
}
}  // namespace

void setup() {
  parkActuators();
  Serial.begin(115200);
  delay(500);
  checkIdleLines();
  Wire.begin(pin::kSda, pin::kScl, kI2cHz);
}

void loop() {
  if (!scanTimer.due(millis())) return;
  Serial.printf("Escaneo %u\n", ++scans);
  scan(kI2cHz);
  scan(kSlowHz);
}
