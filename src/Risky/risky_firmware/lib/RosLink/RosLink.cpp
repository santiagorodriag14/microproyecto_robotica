#include "RosLink.h"

#include <rmw_microros/rmw_microros.h>

namespace {
constexpr char kNodeName[] = "risky";
constexpr char kNamespace[] = "risky";
constexpr uint32_t kRetryPeriodMs = 500;
constexpr uint32_t kAlivePeriodMs = 1000;
constexpr int kPingTimeoutMs = 100;
// Con el agente conectado se tolera perder pings sueltos por WiFi antes de declararlo caído.
constexpr uint8_t kAlivePingAttempts = 3;
constexpr int kTimeSyncTimeoutMs = 1000;
constexpr float kGravity = 9.80665f;

// VL53L0X: cono de 25° y alcance útil de 1,2 m en el modo por defecto.
constexpr float kTofFieldOfViewRad = 0.436f;
constexpr float kTofMinRangeM = 0.03f;
constexpr float kTofMaxRangeM = 1.2f;

template <typename T>
void ignoreResult(T) {}

void setFrameId(rosidl_runtime_c__String& field, const char* id) {
  field.data = const_cast<char*>(id);
  field.size = strlen(id);
  field.capacity = field.size + 1;
}

void stamp(builtin_interfaces__msg__Time& time) {
  const int64_t ns = rmw_uros_epoch_nanos();
  time.sec = static_cast<int32_t>(ns / 1000000000);
  time.nanosec = static_cast<uint32_t>(ns % 1000000000);
}
}  // namespace

void RosLink::begin(const RosLinkConfig& config) {
  set_microros_wifi_transports(const_cast<char*>(config.ssid), const_cast<char*>(config.password),
                               config.agentIp, config.agentPort);

  setFrameId(rangeMsg_.header.frame_id, "tof_link");
  rangeMsg_.radiation_type = sensor_msgs__msg__Range__INFRARED;
  rangeMsg_.field_of_view = kTofFieldOfViewRad;
  rangeMsg_.min_range = kTofMinRangeM;
  rangeMsg_.max_range = kTofMaxRangeM;

  setFrameId(imuMsg_.header.frame_id, "imu_link");
  imuMsg_.orientation_covariance[0] = -1.0;  // convención ROS: orientación no disponible

  thrustMsg_.data.data = thrustData_;
  thrustMsg_.data.size = 2;
  thrustMsg_.data.capacity = 2;
}

void RosLink::spin() {
  const uint32_t now = millis();
  if (now - lastCheckMs_ < (connected() ? kAlivePeriodMs : kRetryPeriodMs)) return;
  lastCheckMs_ = now;

  const uint8_t attempts = connected() ? kAlivePingAttempts : 1;
  const bool agentUp = rmw_uros_ping_agent(kPingTimeoutMs, attempts) == RMW_RET_OK;
  if (!connected() && agentUp && createEntities()) {
    state_ = State::Connected;
  } else if (connected() && !agentUp) {
    destroyEntities();
    state_ = State::WaitingAgent;
  }
}

bool RosLink::createEntities() {
  allocator_ = rcl_get_default_allocator();
  if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK) return false;

  node_ = rcl_get_zero_initialized_node();
  rangePub_ = rcl_get_zero_initialized_publisher();
  imuPub_ = rcl_get_zero_initialized_publisher();
  cmdVelPub_ = rcl_get_zero_initialized_publisher();
  thrustPub_ = rcl_get_zero_initialized_publisher();
  // Best effort: un dato de telemetría viejo no sirve y así publicar nunca espera confirmación.
  const bool ok =
      rclc_node_init_default(&node_, kNodeName, kNamespace, &support_) == RCL_RET_OK &&
      rclc_publisher_init_best_effort(&rangePub_, &node_,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                      "tof/range") == RCL_RET_OK &&
      rclc_publisher_init_best_effort(&imuPub_, &node_,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                      "imu/data_raw") == RCL_RET_OK &&
      rclc_publisher_init_best_effort(&cmdVelPub_, &node_,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                      "cmd_vel") == RCL_RET_OK &&
      rclc_publisher_init_best_effort(&thrustPub_, &node_,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                      "thrust") == RCL_RET_OK;
  if (!ok) {
    destroyEntities();
    return false;
  }
  // Sincroniza el reloj con el agente para que los sellos de tiempo sean de la hora del PC.
  ignoreResult(rmw_uros_sync_session(kTimeSyncTimeoutMs));
  return true;
}

void RosLink::destroyEntities() {
  // El agente puede no estar: destruir sin esperar su respuesta.
  rmw_context_t* context = rcl_context_get_rmw_context(&support_.context);
  if (context != nullptr) {
    ignoreResult(rmw_uros_set_context_entity_destroy_session_timeout(context, 0));
  }
  ignoreResult(rcl_publisher_fini(&thrustPub_, &node_));
  ignoreResult(rcl_publisher_fini(&cmdVelPub_, &node_));
  ignoreResult(rcl_publisher_fini(&imuPub_, &node_));
  ignoreResult(rcl_publisher_fini(&rangePub_, &node_));
  ignoreResult(rcl_node_fini(&node_));
  ignoreResult(rclc_support_fini(&support_));
}

void RosLink::publishRange(float meters) {
  if (!connected()) return;
  stamp(rangeMsg_.header.stamp);
  rangeMsg_.range = meters;
  ignoreResult(rcl_publish(&rangePub_, &rangeMsg_, nullptr));
}

void RosLink::publishImu(const ImuSample& s) {
  if (!connected()) return;
  stamp(imuMsg_.header.stamp);
  imuMsg_.angular_velocity.x = s.gx;
  imuMsg_.angular_velocity.y = s.gy;
  imuMsg_.angular_velocity.z = s.gz;
  imuMsg_.linear_acceleration.x = s.ax * kGravity;
  imuMsg_.linear_acceleration.y = s.ay * kGravity;
  imuMsg_.linear_acceleration.z = s.az * kGravity;
  ignoreResult(rcl_publish(&imuPub_, &imuMsg_, nullptr));
}

void RosLink::publishCmdVel(float linearMps, float angularRadS) {
  if (!connected()) return;
  cmdVelMsg_.linear.x = linearMps;
  cmdVelMsg_.angular.z = angularRadS;
  ignoreResult(rcl_publish(&cmdVelPub_, &cmdVelMsg_, nullptr));
}

void RosLink::publishThrust(float front, float rear) {
  if (!connected()) return;
  thrustData_[0] = front;
  thrustData_[1] = rear;
  ignoreResult(rcl_publish(&thrustPub_, &thrustMsg_, nullptr));
}
