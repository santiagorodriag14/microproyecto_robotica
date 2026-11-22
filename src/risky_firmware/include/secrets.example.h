// Copiar como include/secrets.h (ignorado por git) y llenar con los datos reales.
#pragma once

#include <IPAddress.h>

namespace secrets {
constexpr char kWifiSsid[] = "nombre-de-la-red";
constexpr char kWifiPassword[] = "clave-de-la-red";
// IP del PC donde corre el agente micro-ROS.
const IPAddress kAgentIp(192, 168, 1, 100);
constexpr uint16_t kAgentPort = 8888;
}  // namespace secrets
