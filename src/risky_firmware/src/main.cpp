// Firmware de Risky, teleoperación manual: palanca derecha (CH1 giro, CH2 avance) mueve las ruedas,
// Knob A (CH5) la turbina delantera y Knob B (CH6) la trasera.
// Sin señal RC las ruedas frenan y las turbinas sostienen su última potencia para no despegarse de la pared.
// Este lazo corre en el núcleo 1 y no depende de ROS: la telemetría va aparte, en el núcleo 0.
#include <Arduino.h>

#include "DifferentialDrive.h"
#include "Esc.h"
#include "MotorSide.h"
#include "RcInput.h"
#include "ThrustGate.h"
#include "config.h"
#include "telemetry.h"
#include "util.h"

namespace {
struct RcCommand {
  bool ok;
  float throttle;
  float turn;
  ThrustLevels knobs;
};

RcChannel turnInput;
RcChannel throttleInput;
RcChannel frontKnobInput;
RcChannel rearKnobInput;
MotorSide leftSide(pin::kLeftPwm, pin::kLeftIn1, pin::kLeftIn2, ledc::kLeft, drive::kLeftInverted);
MotorSide rightSide(pin::kRightPwm, pin::kRightIn1, pin::kRightIn2, ledc::kRight,
                    drive::kRightInverted);
DifferentialDrive base(leftSide, rightSide, drive::kSlewPerS);
Esc frontEsc(thrust::kFrontPin, thrust::kFrontLedc, esc::kMinUs, esc::kMaxUs);
Esc rearEsc(thrust::kRearPin, thrust::kRearLedc, esc::kMinUs, esc::kMaxUs);
ThrustGate thrustGate(thrust::kKnobOffLevel);
Periodic control(kControlPeriodMs);
Periodic report(250);
RcCommand command{};
ThrustLevels thrustOutput{0.f, 0.f};

// Si falta cualquier canal se trata como pérdida total: no se maneja el robot con datos parciales.
RcCommand readRc() {
  uint16_t turnUs, throttleUs, frontUs, rearUs;
  const bool ok = turnInput.read(turnUs, rc::kTimeoutUs) && throttleInput.read(throttleUs, rc::kTimeoutUs) &&
                  frontKnobInput.read(frontUs, rc::kTimeoutUs) && rearKnobInput.read(rearUs, rc::kTimeoutUs);
  if (!ok) return {false, 0.f, 0.f, {0.f, 0.f}};
  return {true, rcBipolar(throttleUs, rc::kCh2, rc::kDeadband), rcBipolar(turnUs, rc::kCh1, rc::kDeadband),
          {rcUnipolar(frontUs, rc::kCh5), rcUnipolar(rearUs, rc::kCh6)}};
}

void printStatus() {
  if (!command.ok) {
    Serial.printf("SIN SEÑAL RC: ruedas frenadas, turbinas sostienen del %3.0f %% tras %3.0f %%\n",
                  thrustOutput.front * 100.f, thrustOutput.rear * 100.f);
    return;
  }
  Serial.printf("avance %+.2f giro %+.2f -> izq %+.2f der %+.2f | turbinas del %3.0f %% tras %3.0f %%%s\n",
                command.throttle, command.turn, base.left(), base.right(), thrustOutput.front * 100.f,
                thrustOutput.rear * 100.f, thrustGate.armed() ? "" : " (esperando perillas en 0)");
}
}  // namespace

void setup() {
  Serial.begin(115200);
  turnInput.begin(pin::kRcCh1);
  throttleInput.begin(pin::kRcCh2);
  frontKnobInput.begin(pin::kRcCh5);
  rearKnobInput.begin(pin::kRcCh6);
  if (!leftSide.begin(drive::kPwmHz, drive::kPwmBits) || !rightSide.begin(drive::kPwmHz, drive::kPwmBits)) {
    halt("no se pudo configurar el PWM de las ruedas");
  }
  if (!frontEsc.begin(esc::kFrameHz) || !rearEsc.begin(esc::kFrameHz)) {
    halt("no se pudo configurar el PWM de los ESC");
  }
  telemetry::start();
}

void loop() {
  const uint32_t now = millis();
  if (control.due(now)) {
    command = readRc();
    if (command.ok) {
      base.setTarget(command.throttle, command.turn);
      base.update(kControlPeriodMs / 1000.f);
    } else {
      base.brake();
    }
    thrustOutput = thrustGate.update(command.ok, command.knobs);
    frontEsc.setLevel(thrustOutput.front);
    rearEsc.setLevel(thrustOutput.rear);
    telemetry::share({base.left(), base.right(), thrustOutput});
  }
  if (report.due(now)) printStatus();
}
