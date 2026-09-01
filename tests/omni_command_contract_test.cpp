#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include "OmniCommandContract.hpp"

namespace {

void LeavesFeasibleTargetsUnchanged() {
  const auto RESULT =
      Pldx::OmniCommandContract::Resolve(0.5F, -0.25F, 0.2F, 0.1F, 0.3F);
  assert(RESULT.valid);
  assert(RESULT.scale == 1.0F);
}

void UniformlyDesaturatesCombinationLimit() {
  const auto UNLIMITED =
      Pldx::OmniCommandContract::Resolve(2.5F, 2.5F, 1.8F, 1.0F, 0.3F);
  assert(UNLIMITED.valid);
  assert(UNLIMITED.scale == 1.0F);

  const auto LIMITED =
      Pldx::OmniCommandContract::Resolve(2.5F, 2.5F, 1.8F, 0.05F, 0.3F);
  assert(LIMITED.valid);
  assert(LIMITED.scale > 0.0F && LIMITED.scale < 1.0F);
  float maximum = 0.0F;
  for (const float value : LIMITED.angular_velocity) {
    maximum = std::max(maximum, std::fabs(value));
  }
  assert(
      std::fabs(maximum -
                Pldx::OmniCommandContract::MAX_WHEEL_ANGULAR_VELOCITY_RAD_S) <
      1.0e-4F);

  const auto RECOMPUTED = Pldx::OmniCommandContract::Resolve(
      2.5F * LIMITED.scale, 2.5F * LIMITED.scale, 1.8F * LIMITED.scale, 0.05F,
      0.3F);
  assert(RECOMPUTED.valid);
  for (size_t index = 0U; index < LIMITED.angular_velocity.size(); ++index) {
    assert(std::fabs(RECOMPUTED.angular_velocity[index] -
                     LIMITED.angular_velocity[index]) < 1.0e-4F);
  }
}

void RejectsInvalidGeometryAndInputs() {
  assert(
      !Pldx::OmniCommandContract::Resolve(0.0F, 0.0F, 0.0F, 0.0F, 0.3F).valid);
  assert(!Pldx::OmniCommandContract::Resolve(
              std::numeric_limits<float>::infinity(), 0.0F, 0.0F, 0.1F, 0.3F)
              .valid);
}

}  // namespace

int main() {
  LeavesFeasibleTargetsUnchanged();
  UniformlyDesaturatesCombinationLimit();
  RejectsInvalidGeometryAndInputs();
}
