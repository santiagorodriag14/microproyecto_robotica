// Banco 2: ruedas con el radio (CH2 avance, CH1 giro). Probar con el robot sobre un soporte.
#include <Arduino.h>

#include "DifferentialDrive.h"
#include "MotorSide.h"
#include "RcInput.h"
#include "config.h"
#include "util.h"

namespace {
RcChannel turnInput;
RcChannel throttleInput;
MotorSide leftSide(pin::kLeftPwm, pin::kLeftIn1, pin::kLeftIn2, ledc::kLeft, drive::kLeftInverted);
MotorSide rightSide(pin::kRightPwm, pin::kRightIn1, pin::kRightIn2, ledc::kRight,
                    drive::kRightInverted);
DifferentialDrive base(leftSide, rightSide, drive::kSlewPerS);
Periodic control(kControlPeriodMs);
Periodic report(200);
bool rcOk = false;
float throttle = 0.f;
float turn = 0.f;
}  // namespace

void setup() {
  Serial.begin(115200);
  turnInput.begin(pin::kRcCh1);
  throttleInput.begin(pin::kRcCh2);
  if (!leftSide.begin(drive::kPwmHz, drive::kPwmBits) ||
      !rightSide.begin(drive::kPwmHz, drive::kPwmBits)) {
    halt("no se pudo configurar el PWM de las ruedas");
  }
}

void loop() {
  const uint32_t now = millis();
  if (control.due(now)) {
    uint16_t turnUs, throttleUs;
    rcOk = turnInput.read(turnUs, rc::kTimeoutUs) && throttleInput.read(throttleUs, rc::kTimeoutUs);
    if (rcOk) {
      throttle = rcBipolar(throttleUs, rc::kCh2, rc::kDeadband);
      turn = rcBipolar(turnUs, rc::kCh1, rc::kDeadband);
      base.setTarget(throttle, turn);
      base.update(kControlPeriodMs / 1000.f);
    } else {
      base.brake();
    }
  }
  if (report.due(now)) {
    if (rcOk) {
      Serial.printf("avance %+.2f giro %+.2f -> izq %+.2f der %+.2f\n", throttle, turn, base.left(),
                    base.right());
    } else {
      Serial.println("sin señal RC: freno");
    }
  }
}
