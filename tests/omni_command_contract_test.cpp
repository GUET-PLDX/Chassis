#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include "OmniCommandContract.hpp"

int main() {
  const auto feasible =
      Pldx::OmniCommandContract::Resolve(1.0F, -0.5F, 0.2F, 0.075F, 0.245F);
  assert(feasible.valid && feasible.scale == 1.0F);

  const auto limited =
      Pldx::OmniCommandContract::Resolve(5.0F, 5.0F, 5.0F, 0.075F, 0.245F);
  assert(limited.valid && limited.scale > 0.0F && limited.scale < 1.0F);
  float maximum = 0.0F;
  for (const float value : limited.angular_velocity) {
    maximum = std::max(maximum, std::fabs(value));
  }
  assert(
      std::fabs(maximum -
                Pldx::OmniCommandContract::MAX_WHEEL_ANGULAR_VELOCITY_RAD_S) <
      1.0e-4F);

  assert(
      !Pldx::OmniCommandContract::Resolve(
           std::numeric_limits<float>::infinity(), 0.0F, 0.0F, 0.075F, 0.245F)
           .valid);
}
