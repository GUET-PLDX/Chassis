#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "../ChassisWheelTelemetry.hpp"

namespace {

constexpr float EPSILON = 1.0e-5f;

bool Near(float lhs, float rhs) { return std::fabs(lhs - rhs) < EPSILON; }

ChassisWheelTelemetryDetail::SampleInput NominalInput() {
  ChassisWheelTelemetryDetail::SampleInput input;
  input.now_us = 100000U;
  input.feedback_max_age_us = 30000U;
  input.max_sample_skew_us = 3000U;
  input.reduction_ratio = 14.0f;
  input.wheel_radius = 0.07f;
  input.wheel_to_center = 0.31f;
  input.power_state_valid = true;
  input.chassis_power_on = true;
  for (uint8_t index = 0U; index < 4U; ++index) {
    input.wheel[index].received_time_us = 99000U + index * 100U;
    input.wheel[index].sequence = static_cast<uint16_t>(index + 1U);
    input.wheel[index].motor_omega = static_cast<float>(index + 1U) * 14.0f;
    input.wheel[index].temperature = 40.0f;
    input.wheel[index].has_feedback = true;
    input.wheel[index].online = true;
  }
  return input;
}

void TestWheelOrderAndForwardKinematics() {
  constexpr float RADIUS = 0.07f;
  constexpr float CENTER = 0.31f;
  constexpr float C = 0.70710678118f;
  struct Fixture {
    float vx;
    float vy;
    float wz;
  };
  constexpr Fixture FIXTURES[] = {
      {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
      {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 2.0f},  {0.0f, 0.0f, -2.0f},
      {0.4f, -0.2f, 0.7f},
  };
  for (const auto& fixture : FIXTURES) {
    const float WHEELS[4] = {
        (-C * fixture.vx - C * fixture.vy + CENTER * fixture.wz) / RADIUS,
        (C * fixture.vx - C * fixture.vy + CENTER * fixture.wz) / RADIUS,
        (C * fixture.vx + C * fixture.vy + CENTER * fixture.wz) / RADIUS,
        (-C * fixture.vx + C * fixture.vy + CENTER * fixture.wz) / RADIUS,
    };
    const auto RESULT =
        ChassisWheelTelemetryDetail::ForwardKinematics(WHEELS, RADIUS, CENTER);
    assert(Near(RESULT.vx, fixture.vx));
    assert(Near(RESULT.vy, fixture.vy));
    assert(Near(RESULT.wz, fixture.wz));
  }
}

void TestNominalSample() {
  const auto SAMPLE =
      ChassisWheelTelemetryDetail::BuildSample(NominalInput(), 42U);
  assert(SAMPLE.sequence == 42U);
  assert(SAMPLE.sample_time_us == 99150U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::CHASSIS_POWER_ON) !=
         0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::POWER_STATE_VALID) !=
         0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::DIAGNOSTIC_VALID) !=
         0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::SAMPLE_SKEW_OK) != 0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::TRANSPORT_VALID) != 0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::TIME_SYNC_VALID) == 0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::STALE) == 0U);
  for (uint8_t index = 0U; index < 4U; ++index) {
    assert(Near(SAMPLE.wheel_angular_velocity[index],
                static_cast<float>(index + 1U)));
    assert((SAMPLE.wheel_status[index] & ChassisWheelTelemetry::ONLINE) != 0U);
    assert((SAMPLE.wheel_status[index] & ChassisWheelTelemetry::FRESH) != 0U);
  }
  const float WHEELS[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  const auto EXPECTED =
      ChassisWheelTelemetryDetail::ForwardKinematics(WHEELS, 0.07f, 0.31f);
  assert(Near(SAMPLE.vx, EXPECTED.vx));
  assert(Near(SAMPLE.vy, EXPECTED.vy));
  assert(Near(SAMPLE.wz, EXPECTED.wz));
}

void TestFreshnessPowerSkewAndFaults() {
  auto input = NominalInput();
  input.power_state_valid = false;
  auto sample = ChassisWheelTelemetryDetail::BuildSample(input, 1U);
  assert((sample.global_status & ChassisWheelTelemetry::POWER_STATE_VALID) ==
         0U);
  assert((sample.wheel_status[0] & ChassisWheelTelemetry::FRESH) != 0U);

  input = NominalInput();
  input.chassis_power_on = false;
  sample = ChassisWheelTelemetryDetail::BuildSample(input, 2U);
  assert((sample.global_status & ChassisWheelTelemetry::STALE) != 0U);
  for (const auto STATUS : sample.wheel_status) {
    assert((STATUS & ChassisWheelTelemetry::FRESH) == 0U);
  }

  input = NominalInput();
  input.wheel[0].received_time_us = 69999U;
  sample = ChassisWheelTelemetryDetail::BuildSample(input, 3U);
  assert((sample.global_status & ChassisWheelTelemetry::STALE) != 0U);
  assert((sample.wheel_status[0] & ChassisWheelTelemetry::FRESH) == 0U);
  assert((sample.wheel_status[1] & ChassisWheelTelemetry::FRESH) != 0U);

  input = NominalInput();
  input.wheel[3].received_time_us = 95000U;
  sample = ChassisWheelTelemetryDetail::BuildSample(input, 4U);
  assert((sample.global_status & ChassisWheelTelemetry::SAMPLE_SKEW_OK) == 0U);
  assert((sample.global_status & ChassisWheelTelemetry::STALE) != 0U);
  for (const auto STATUS : sample.wheel_status) {
    assert((STATUS & ChassisWheelTelemetry::FRESH) != 0U);
  }

  input = NominalInput();
  input.wheel[1].motor_omega = std::numeric_limits<float>::quiet_NaN();
  sample = ChassisWheelTelemetryDetail::BuildSample(input, 5U);
  assert((sample.wheel_status[1] & ChassisWheelTelemetry::MOTOR_FAULT) != 0U);
  assert((sample.wheel_status[1] & ChassisWheelTelemetry::FRESH) == 0U);
  assert((sample.global_status & ChassisWheelTelemetry::TRANSPORT_VALID) == 0U);

  input = NominalInput();
  input.wheel[0].motor_torque =
      ChassisWheelTelemetryDetail::M3508_HIGH_CURRENT_TORQUE_NM;
  input.wheel[0].temperature =
      ChassisWheelTelemetryDetail::M3508_OVER_TEMPERATURE_C;
  input.wheel[0].error_id = 7U;
  sample = ChassisWheelTelemetryDetail::BuildSample(input, 6U);
  assert((sample.wheel_status[0] & ChassisWheelTelemetry::HIGH_CURRENT) != 0U);
  assert((sample.wheel_status[0] & ChassisWheelTelemetry::OVER_TEMPERATURE) !=
         0U);
  assert((sample.wheel_status[0] & ChassisWheelTelemetry::MOTOR_FAULT) != 0U);
}

void TestSequenceWrap() {
  uint16_t sequence = std::numeric_limits<uint16_t>::max();
  ++sequence;
  assert(sequence == 0U);
  auto input = NominalInput();
  for (auto& wheel : input.wheel) {
    wheel.sequence = sequence;
  }
  const auto SAMPLE = ChassisWheelTelemetryDetail::BuildSample(input, sequence);
  assert(SAMPLE.sequence == 0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::TRANSPORT_VALID) != 0U);
  assert((SAMPLE.global_status & ChassisWheelTelemetry::STALE) == 0U);
  for (const auto STATUS : SAMPLE.wheel_status) {
    assert((STATUS & ChassisWheelTelemetry::FRESH) != 0U);
  }
}

}  // namespace

int main() {
  static_assert(std::is_trivially_copyable_v<ChassisWheelTelemetry>);
  TestWheelOrderAndForwardKinematics();
  TestNominalSample();
  TestFreshnessPowerSkewAndFaults();
  TestSequenceWrap();
}
