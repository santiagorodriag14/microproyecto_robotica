// Banco 3: ESC por consola serial. SIN HÉLICES.
#include <Arduino.h>

#include "Esc.h"
#include "config.h"
#include "util.h"

namespace {
Esc esc1(pin::kEsc1, ledc::kEsc1, esc::kMinUs, esc::kMaxUs);
Esc esc2(pin::kEsc2, ledc::kEsc2, esc::kMinUs, esc::kMaxUs);

void printHelp() {
  Serial.println("Comandos:");
  Serial.println("  1 30 | 2 30 | a 30   ESC1, ESC2 o ambos al 30 %");
  Serial.println("  max | min            pulso máximo o mínimo en ambos (calibración)");
  Serial.println("  cualquier otra cosa  apaga ambos");
}

void writeBoth(uint16_t us) {
  esc1.writeUs(us);
  esc2.writeUs(us);
}

void handle(const String& line) {
  if (line == "max") {
    writeBoth(esc::kMaxUs);
    Serial.println("Máximo enviado. Encienda SW1, espere los pitidos de calibración y envíe 'min'.");
    return;
  }
  if (line == "min") {
    writeBoth(esc::kMinUs);
    Serial.println("Mínimo enviado.");
    return;
  }
  char target;
  float percent;
  const bool valid = sscanf(line.c_str(), "%c %f", &target, &percent) == 2 &&
                     (target == '1' || target == '2' || target == 'a') && percent >= 0.f &&
                     percent <= 100.f;
  if (!valid) {
    writeBoth(esc::kMinUs);
    Serial.println("Apagado.");
    printHelp();
    return;
  }
  if (target != '2') esc1.setLevel(percent / 100.f);
  if (target != '1') esc2.setLevel(percent / 100.f);
  Serial.printf("ESC %c al %.0f %%\n", target, percent);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  if (!esc1.begin(esc::kFrameHz) || !esc2.begin(esc::kFrameHz)) {
    halt("no se pudo configurar el PWM de los ESC");
  }
  printHelp();
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  handle(line);
}
