#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace Pldx::OmniCommandContract {

inline constexpr float MAX_WHEEL_ANGULAR_VELOCITY_RAD_S = 52.0F;

struct WheelTargets {
  std::array<float, 4U> angular_velocity{};
  float scale = 1.0F;
  bool valid = false;
};

inline WheelTargets Resolve(float vx, float vy, float wz, float wheel_radius,
                            float wheel_to_center) {
  WheelTargets result{};
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
      !std::isfinite(wheel_radius) || !std::isfinite(wheel_to_center) ||
      wheel_radius <= 0.0F || wheel_to_center <= 0.0F) {
    return result;
  }

  constexpr float INV_SQRT2 = 0.70710678118F;
  result.angular_velocity[0] =
      (-INV_SQRT2 * vx - INV_SQRT2 * vy + wz * wheel_to_center) / wheel_radius;
  result.angular_velocity[1] =
      (INV_SQRT2 * vx - INV_SQRT2 * vy + wz * wheel_to_center) / wheel_radius;
  result.angular_velocity[2] =
      (INV_SQRT2 * vx + INV_SQRT2 * vy + wz * wheel_to_center) / wheel_radius;
  result.angular_velocity[3] =
      (-INV_SQRT2 * vx + INV_SQRT2 * vy + wz * wheel_to_center) / wheel_radius;

  float maximum = 0.0F;
  for (const float value : result.angular_velocity) {
    maximum = std::max(maximum, std::fabs(value));
  }
  if (maximum > MAX_WHEEL_ANGULAR_VELOCITY_RAD_S) {
    result.scale = MAX_WHEEL_ANGULAR_VELOCITY_RAD_S / maximum;
    for (float& value : result.angular_velocity) {
      value *= result.scale;
    }
  }
  result.valid = true;
  return result;
}

}  // namespace Pldx::OmniCommandContract
