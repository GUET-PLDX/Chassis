#ifndef MODULES_CHASSIS_CHASSIS_MOTION_STATE_HPP_
#define MODULES_CHASSIS_CHASSIS_MOTION_STATE_HPP_

#include <cstdint>

enum class ChassisMotionMode : uint8_t { NON_ROTOR, ROTOR };

static constexpr const char* CHASSIS_MOTION_STATE_TOPIC_NAME =
    "chassis_motion_state";
static constexpr bool CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER = true;

struct ChassisMotionState {
  float yaw_rate_rad_s = 0.0f;
  bool yaw_rate_valid = false;
  bool online = false;
  ChassisMotionMode mode = ChassisMotionMode::NON_ROTOR;
};

#endif  // MODULES_CHASSIS_CHASSIS_MOTION_STATE_HPP_
