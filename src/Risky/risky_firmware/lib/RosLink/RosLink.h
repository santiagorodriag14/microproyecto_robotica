#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "ImuSample.h"

struct RosLinkConfig {
  const char* ssid;
  const char* password;
  IPAddress agentIp;
  uint16_t agentPort;
};

// Telemetría por micro-ROS (UDP sobre WiFi), con los tópicos del contrato real/simulación. Si el agente se cae, destruye las entidades
// y las vuelve a crear cuando reaparece, sin reiniciar el ESP32.
class RosLink {
 public:
  // Bloquea hasta conectarse al WiFi.
  void begin(const RosLinkConfig& config);
  // Llamar seguido. Si el agente no responde puede bloquear hasta ~300 ms.
  void spin();
  bool connected() const { return state_ == State::Connected; }
  // +inf cuando no hay superficie a la vista (REP 117).
  void publishRange(float meters);
  void publishImu(const ImuSample& sample);
  // Velocidad que se le está pidiendo a la base, en m/s y rad/s.
  void publishCmdVel(float linearMps, float angularRadS);
  // Niveles de turbina [0, 1].
  void publishThrust(float front, float rear);

 private:
  enum class State { WaitingAgent, Connected };

  bool createEntities();
  void destroyEntities();

  State state_ = State::WaitingAgent;
  uint32_t lastCheckMs_ = 0;
  rcl_allocator_t allocator_{};
  rclc_support_t support_{};
  rcl_node_t node_{};
  rcl_publisher_t rangePub_{};
  rcl_publisher_t imuPub_{};
  rcl_publisher_t cmdVelPub_{};
  rcl_publisher_t thrustPub_{};
  sensor_msgs__msg__Range rangeMsg_{};
  sensor_msgs__msg__Imu imuMsg_{};
  geometry_msgs__msg__Twist cmdVelMsg_{};
  std_msgs__msg__Float32MultiArray thrustMsg_{};
  float thrustData_[2] = {0.f, 0.f};
};
