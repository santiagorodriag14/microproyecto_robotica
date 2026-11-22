#pragma once

#include <stdint.h>

#include "RcInput.h"
#include "SurfaceClassifier.h"

// Pines según Esquematico_Risky_v5, hoja 2.
namespace pin {
constexpr uint8_t kRcCh1 = 34;  // palanca derecha ↔: giro
constexpr uint8_t kRcCh2 = 35;  // palanca derecha ↕: avance
constexpr uint8_t kRcCh5 = 32;  // Knob A: turbina delantera
constexpr uint8_t kRcCh6 = 33;  // Knob B: turbina trasera
constexpr uint8_t kEsc1 = 25;
constexpr uint8_t kEsc2 = 26;
constexpr uint8_t kSda = 21;
constexpr uint8_t kScl = 22;
constexpr uint8_t kLeftPwm = 17;  // drivers 1 y 2
constexpr uint8_t kLeftIn1 = 19;
constexpr uint8_t kLeftIn2 = 18;
constexpr uint8_t kRightPwm = 23;  // drivers 3 y 4
constexpr uint8_t kRightIn1 = 4;
constexpr uint8_t kRightIn2 = 16;
}  // namespace pin

// Cada par de canales LEDC (0-1, 2-3, ...) comparte temporizador,
// así que los ESC (50 Hz) y las ruedas (20 kHz) van en pares distintos.
namespace ledc {
constexpr uint8_t kEsc1 = 0;
constexpr uint8_t kEsc2 = 1;
constexpr uint8_t kLeft = 2;
constexpr uint8_t kRight = 3;
}  // namespace ledc

constexpr uint32_t kControlPeriodMs = 10;
constexpr uint32_t kI2cHz = 100000;  // a 400 kHz el bus falló en bench_i2c (pull-ups débiles)

namespace rc {
constexpr uint32_t kTimeoutUs = 100000;  // 5 tramas de 20 ms sin pulso válido
constexpr float kDeadband = 0.06f;
// Reemplazar con los extremos y el centro medidos en bench_rc.
constexpr RcRange kCh1{1080, 1564, 1986};
constexpr RcRange kCh2{996, 1486, 1980};
constexpr RcRange kCh5{994, 1500, 1986};
constexpr RcRange kCh6{994, 1500, 1986};
}  // namespace rc

namespace drive {
constexpr uint32_t kPwmHz = 20000;  // por encima del rango audible; el TB6612FNG admite hasta 100 kHz
constexpr uint8_t kPwmBits = 10;    // 80 MHz / 20 kHz deja como máximo 11 bits
constexpr float kSlewPerS = 2.f;    // de 0 a máxima velocidad en 0,5 s
// Cambiar si en bench_drive un lado gira al revés con la palanca hacia adelante.
constexpr bool kLeftInverted = false;
constexpr bool kRightInverted = false;
// Solo para convertir el comando a m/s en /risky/cmd_vel; no afectan el control.
// Estimado: medir la distancia recorrida en 5 s a fondo sobre el suelo y dividir entre 5.
constexpr float kMaxWheelSpeedMps = 0.10f;
constexpr float kTrackWidthM = 0.255f;  // promedio entre ruedas izquierda y derecha, del URDF
}  // namespace drive

namespace esc {
constexpr uint32_t kFrameHz = 50;
constexpr uint16_t kMinUs = 1000;
constexpr uint16_t kMaxUs = 2000;
}  // namespace esc

namespace thrust {
// Se asume ESC1 en la turbina delantera; si en bench_thrust Knob A mueve la trasera, intercambiar.
constexpr uint8_t kFrontPin = pin::kEsc1;
constexpr uint8_t kFrontLedc = ledc::kEsc1;
constexpr uint8_t kRearPin = pin::kEsc2;
constexpr uint8_t kRearLedc = ledc::kEsc2;
// Las perillas no tienen un cero exacto: por debajo de este nivel la turbina queda apagada.
constexpr float kKnobOffLevel = 0.05f;
}  // namespace thrust

namespace imu {
constexpr uint8_t kAddress = 0x68;  // AD0 a GND
constexpr uint16_t kGyroCalibrationSamples = 500;
constexpr float kAccelWeight = 0.02f;
constexpr SurfaceBands kBands{25.f, 65.f, 115.f, 155.f, 300};
}  // namespace imu

namespace tof {
// Distancia real al suelo menos el promedio crudo de bench_tof, con el robot en el suelo.
constexpr int16_t kOffsetMm = -54;
constexpr uint32_t kTimingBudgetUs = 33000;
constexpr uint32_t kPeriodMs = 50;
}  // namespace tof

namespace telemetry {
constexpr uint32_t kImuPeriodMs = 50;
constexpr uint32_t kStatePeriodMs = 50;  // /risky/cmd_vel y /risky/thrust
}  // namespace telemetry
