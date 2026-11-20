// Banco 3b: turbinas con el radio, SIN HÉLICES. Knob A (CH5) delantera, Knob B (CH6) trasera.
// Calibrar antes el rango de los ESC con bench_esc.
#include <Arduino.h>

#include "Esc.h"
#include "RcInput.h"
#include "ThrustGate.h"
#include "config.h"
#include "util.h"

namespace {
RcChannel frontInput;
RcChannel rearInput;
Esc frontEsc(thrust::kFrontPin, thrust::kFrontLedc, esc::kMinUs, esc::kMaxUs);
Esc rearEsc(thrust::kRearPin, thrust::kRearLedc, esc::kMinUs, esc::kMaxUs);
ThrustGate gate(thrust::kKnobOffLevel);
Periodic control(kControlPeriodMs);
Periodic report(200);
ThrustLevels output{0.f, 0.f};
bool rcOk = false;

const char* status() {
  if (!rcOk) return "SIN SEÑAL RC: mantiene la última salida";
  return gate.armed() ? "armado" : "esperando ambas perillas en 0 para armar";
}
}  // namespace

void setup() {
  Serial.begin(115200);
  frontInput.begin(pin::kRcCh5);
  rearInput.begin(pin::kRcCh6);
  if (!frontEsc.begin(esc::kFrameHz) || !rearEsc.begin(esc::kFrameHz)) {
    halt("no se pudo configurar el PWM de los ESC");
  }
}

void loop() {
  const uint32_t now = millis();
  if (control.due(now)) {
    uint16_t frontUs, rearUs;
    rcOk = frontInput.read(frontUs, rc::kTimeoutUs) && rearInput.read(rearUs, rc::kTimeoutUs);
    ThrustLevels knobs{0.f, 0.f};
    if (rcOk) knobs = {rcUnipolar(frontUs, rc::kCh5), rcUnipolar(rearUs, rc::kCh6)};
    output = gate.update(rcOk, knobs);
    frontEsc.setLevel(output.front);
    rearEsc.setLevel(output.rear);
  }
  if (report.due(now)) {
    Serial.printf("delantera %3.0f %%  trasera %3.0f %%  %s\n", output.front * 100.f, output.rear * 100.f,
                  status());
  }
}
