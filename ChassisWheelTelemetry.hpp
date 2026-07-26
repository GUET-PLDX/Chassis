#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>

struct ChassisWheelTelemetry {
  static constexpr uint8_t ONLINE = 1U << 0;
  static constexpr uint8_t FRESH = 1U << 1;
  static constexpr uint8_t HIGH_CURRENT = 1U << 2;
  static constexpr uint8_t OVER_TEMPERATURE = 1U << 3;
  static constexpr uint8_t ENCODING_SATURATED = 1U << 4;
  static constexpr uint8_t MOTOR_FAULT = 1U << 5;

  static constexpr uint8_t CHASSIS_POWER_ON = 1U << 0;
  static constexpr uint8_t POWER_STATE_VALID = 1U << 1;
  static constexpr uint8_t DIAGNOSTIC_VALID = 1U << 2;
  static constexpr uint8_t SAMPLE_SKEW_OK = 1U << 3;
  static constexpr uint8_t TRANSPORT_VALID = 1U << 4;
  static constexpr uint8_t TIME_SYNC_VALID = 1U << 5;
  static constexpr uint8_t STALE = 1U << 6;

  uint64_t sample_time_us = 0U;
  uint16_t sequence = 0U;
  float wheel_angular_velocity[4]{};
  uint8_t wheel_status[4]{};
  uint8_t global_status = 0U;
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
};

static_assert(std::is_trivially_copyable_v<ChassisWheelTelemetry>);

namespace ChassisWheelTelemetryDetail {

/* 90% of the M3508 rated current expressed through its torque constant. */
static constexpr float M3508_HIGH_CURRENT_TORQUE_NM = 0.2812032f;
static constexpr float M3508_OVER_TEMPERATURE_C = 75.0f;

struct WheelFeedback {
  uint64_t received_time_us = 0U;
  uint16_t sequence = 0U;
  float motor_omega = 0.0f;
  float motor_torque = 0.0f;
  float temperature = 0.0f;
  uint8_t error_id = 0U;
  bool has_feedback = false;
  bool online = false;
};

struct SampleInput {
  uint64_t now_us = 0U;
  uint32_t feedback_max_age_us = 30000U;
  uint32_t max_sample_skew_us = 3000U;
  float reduction_ratio = 1.0f;
  float wheel_radius = 0.0f;
  float wheel_to_center = 0.0f;
  bool power_state_valid = false;
  bool chassis_power_on = false;
  WheelFeedback wheel[4]{};
};

struct DiagnosticTwist {
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
};

inline DiagnosticTwist ForwardKinematics(const float wheel_omega[4],
                                         float wheel_radius,
                                         float wheel_to_center) {
  constexpr float SQRT2 = 1.41421356237f;
  DiagnosticTwist result;
  result.vx =
      (-wheel_omega[0] + wheel_omega[1] + wheel_omega[2] - wheel_omega[3]) *
      SQRT2 * wheel_radius / 4.0f;
  result.vy =
      (-wheel_omega[0] - wheel_omega[1] + wheel_omega[2] + wheel_omega[3]) *
      SQRT2 * wheel_radius / 4.0f;
  result.wz =
      (wheel_omega[0] + wheel_omega[1] + wheel_omega[2] + wheel_omega[3]) *
      wheel_radius / (4.0f * wheel_to_center);
  return result;
}

inline ChassisWheelTelemetry BuildSample(const SampleInput& input,
                                         uint16_t sequence) {
  ChassisWheelTelemetry sample;
  sample.sequence = sequence;

  bool all_feedback_valid =
      std::isfinite(input.reduction_ratio) && input.reduction_ratio > 0.0f;
  bool all_fresh = all_feedback_valid;
  uint64_t oldest_time_us = UINT64_MAX;
  uint64_t newest_time_us = 0U;
  uint8_t valid_feedback_count = 0U;

  for (uint8_t index = 0U; index < 4U; ++index) {
    const auto& feedback = input.wheel[index];
    uint8_t status = feedback.online ? ChassisWheelTelemetry::ONLINE : 0U;
    const bool FINITE = std::isfinite(feedback.motor_omega) &&
                        std::isfinite(feedback.motor_torque) &&
                        std::isfinite(feedback.temperature);
    const bool VALID = feedback.has_feedback && FINITE;
    const bool AGE_VALID = feedback.received_time_us <= input.now_us;
    const bool FRESH =
        VALID && feedback.online && AGE_VALID &&
        input.now_us - feedback.received_time_us <= input.feedback_max_age_us;

    if (FRESH) {
      status |= ChassisWheelTelemetry::FRESH;
    }
    if (FINITE &&
        std::fabs(feedback.motor_torque) >= M3508_HIGH_CURRENT_TORQUE_NM) {
      status |= ChassisWheelTelemetry::HIGH_CURRENT;
    }
    if (FINITE && feedback.temperature >= M3508_OVER_TEMPERATURE_C) {
      status |= ChassisWheelTelemetry::OVER_TEMPERATURE;
    }
    if (!FINITE || feedback.error_id != 0U) {
      status |= ChassisWheelTelemetry::MOTOR_FAULT;
    }

    sample.wheel_status[index] = status;
    sample.wheel_angular_velocity[index] =
        VALID && input.reduction_ratio > 0.0f
            ? feedback.motor_omega / input.reduction_ratio
            : 0.0f;
    all_feedback_valid = all_feedback_valid && VALID;
    all_fresh = all_fresh && FRESH;
    if (VALID) {
      ++valid_feedback_count;
      if (feedback.received_time_us < oldest_time_us) {
        oldest_time_us = feedback.received_time_us;
      }
      if (feedback.received_time_us > newest_time_us) {
        newest_time_us = feedback.received_time_us;
      }
    }
  }

  if (valid_feedback_count > 0U) {
    sample.sample_time_us =
        oldest_time_us + (newest_time_us - oldest_time_us) / 2U;
  }
  if (all_feedback_valid) {
    sample.global_status |= ChassisWheelTelemetry::TRANSPORT_VALID;
    if (newest_time_us - oldest_time_us <= input.max_sample_skew_us) {
      sample.global_status |= ChassisWheelTelemetry::SAMPLE_SKEW_OK;
    } else {
      all_fresh = false;
    }
  }

  if (input.power_state_valid) {
    sample.global_status |= ChassisWheelTelemetry::POWER_STATE_VALID;
    if (input.chassis_power_on) {
      sample.global_status |= ChassisWheelTelemetry::CHASSIS_POWER_ON;
    } else {
      all_fresh = false;
      for (auto& status : sample.wheel_status) {
        status &= static_cast<uint8_t>(~ChassisWheelTelemetry::FRESH);
      }
    }
  }

  if (!all_fresh) {
    sample.global_status |= ChassisWheelTelemetry::STALE;
  }

  const bool GEOMETRY_VALID = std::isfinite(input.wheel_radius) &&
                              std::isfinite(input.wheel_to_center) &&
                              input.wheel_radius > 0.0f &&
                              input.wheel_to_center > 0.0f;
  if (all_fresh && GEOMETRY_VALID) {
    const auto DIAGNOSTIC =
        ForwardKinematics(sample.wheel_angular_velocity, input.wheel_radius,
                          input.wheel_to_center);
    if (std::isfinite(DIAGNOSTIC.vx) && std::isfinite(DIAGNOSTIC.vy) &&
        std::isfinite(DIAGNOSTIC.wz)) {
      sample.vx = DIAGNOSTIC.vx;
      sample.vy = DIAGNOSTIC.vy;
      sample.wz = DIAGNOSTIC.wz;
      sample.global_status |= ChassisWheelTelemetry::DIAGNOSTIC_VALID;
    }
  }

  return sample;
}

}  // namespace ChassisWheelTelemetryDetail
