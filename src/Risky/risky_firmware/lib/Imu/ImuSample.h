#pragma once

// Aceleración en g y velocidad angular en rad/s, en los ejes del sensor.
struct ImuSample {
  float ax, ay, az;
  float gx, gy, gz;
};
