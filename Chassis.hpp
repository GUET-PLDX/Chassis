#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
depends:
- id: GUET-PLDX/BMI088
- id: GUET-PLDX/Motor
- id: GUET-PLDX/RMMotor
- id: GUET-PLDX/CMD
- id: GUET-PLDX/MadgwickAHRS
- id: GUET-PLDX/PowerControl
- id: GUET-PLDX/Referee
- id: GUET-PLDX/SuperPower
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include <numbers>
#include <type_traits>

/* 功率控制数组按当前最大底盘需求预留 */
static constexpr int CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT = 6;

struct MotorData {
  float output_current_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};

  float output_current_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
};

#include "CMD.hpp"
#include "Helm.hpp"
#include "Mecanum.hpp"
#include "Motor.hpp"
#include "Omni.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "pid.hpp"

template <typename ChassisType>
class Chassis {
 public:
  using ChassisMode = typename ChassisType::ChassisMode;
  struct ChassisParam {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float reduction_ratio = 0.0f;
    float rotor_speed_scale = 1.0f;
    float max_acceleration_mps2 = 2.0f;
    float max_angular_acceleration_rad_s2 = 4.0f;
    float mass_kg = 0.0f;
    float yaw_inertia_kg_m2 = 0.0f;
    float gravity_height = 0.0f;
    float wheel_resistance = 0.0f;
    float error_compensation = 0.0f;
    float gravity = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float rotor_omega_min_scale = 0.55f;
    float rotor_buffer_low_j = 35.0f;
    float rotor_buffer_high_j = 70.0f;
    float rotor_scale_lpf_alpha = 0.2f;
    /* 0 按质量/惯量和加速度限幅值推导，正数覆盖，负值关闭。 */
    float ff_max_force_n = 0.0f;
    float ff_max_torque_nm = 0.0f;
  };

  Chassis(
      Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
      Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
      Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd,
      PowerControl* power_control, Referee* referee,
      uint32_t task_stack_depth = 1536,
      ChassisParam chassis_param = {.wheel_radius = 0.065,
                                    .wheel_to_center = 0.26,
                                    .reduction_ratio = 15.7647,
                                    .rotor_speed_scale = 0.95},
      LibXR::PID<float>::Param pid_follow_ = {.k = 1.0,
                                              .p = 20.0,
                                              .i = 0.0,
                                              .d = 0.0,
                                              .i_limit = 0.0,
                                              .out_limit = 30.0,
                                              .cycle = true},
      LibXR::PID<float>::Param pid_velocity_x_ = {.k = 1.0,
                                                  .p = 20.0,
                                                  .i = 0.0,
                                                  .d = 0.0,
                                                  .i_limit = 0.0,
                                                  .out_limit = 30.0,
                                                  .cycle = false},
      LibXR::PID<float>::Param pid_velocity_y_ = {.k = 1.0,
                                                  .p = 20.0,
                                                  .i = 0.0,
                                                  .d = 0.0,
                                                  .i_limit = 0.0,
                                                  .out_limit = 30.0,
                                                  .cycle = false},
      LibXR::PID<float>::Param pid_omega_ = {.k = 1.0,
                                             .p = 20.0,
                                             .i = 0.0,
                                             .d = 0.0,
                                             .i_limit = 0.0,
                                             .out_limit = 30.0,
                                             .cycle = false},
      LibXR::PID<float>::Param pid_wheel_speed_0_ = {.k = 1.0,
                                                     .p = 0.3,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 5.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_wheel_speed_1_ = {.k = 1.0,
                                                     .p = 0.3,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 5.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_wheel_speed_2_ = {.k = 1.0,
                                                     .p = 0.3,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 5.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_wheel_speed_3_ = {.k = 1.0,
                                                     .p = 0.3,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 5.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_angle_0_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_angle_1_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_angle_2_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_angle_3_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_speed_0_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_speed_1_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_speed_2_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::PID<float>::Param pid_steer_speed_3_ = {.k = 1.0,
                                                     .p = 0.0,
                                                     .i = 0.0,
                                                     .d = 0.0,
                                                     .i_limit = 0.0,
                                                     .out_limit = 0.0,
                                                     .cycle = false},
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : chassis_(motor_wheel_0, motor_wheel_1, motor_wheel_2, motor_wheel_3,
                 motor_steer_0, motor_steer_1, motor_steer_2, motor_steer_3,
                 cmd, power_control, referee, task_stack_depth,
                 MakeParam(chassis_param), pid_follow_, pid_velocity_x_,
                 pid_velocity_y_, pid_omega_, pid_wheel_speed_0_,
                 pid_wheel_speed_1_, pid_wheel_speed_2_, pid_wheel_speed_3_,
                 pid_steer_angle_0_, pid_steer_angle_1_, pid_steer_angle_2_,
                 pid_steer_angle_3_, pid_steer_speed_0_, pid_steer_speed_1_,
                 pid_steer_speed_2_, pid_steer_speed_3_, thread_priority) {
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Chassis* chassis, uint32_t event_id) {
          UNUSED(in_isr);
          chassis->EventHandler(event_id);
        },
        this);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::RELAX),
                            callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::INDEPENDENT),
                            callback);

    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::ROTOR),
                            callback);
    chassis_event_.Register(static_cast<uint32_t>(ChassisMode::FOLLOW),
                            callback);
    if constexpr (std::is_same<ChassisType, Omni>::value) {
      chassis_event_.Register(static_cast<uint32_t>(ChassisMode::NAVIGATION),
                              callback);
    }
    /*
     * TRACK_START 只属于麦轮
     * 编译期判断可以让 Omni 和 Helm 继续使用各自的枚举
     */
    if constexpr (std::is_same<ChassisType, Mecanum>::value) {
      chassis_event_.Register(static_cast<uint32_t>(ChassisMode::TRACK_START),
                              callback);
    }
  }

  /**
   * @brief 获取底盘的事件处理器
   * @details 通过此事件处理器可以向底盘发送事件消息，控制底盘的行为模式
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event& GetEvent() { return chassis_event_; }

  /** @brief 获取底盘实现的实际模式。 */
  ChassisMode GetMode() { return chassis_.GetMode(); }

  /** @brief 外壳参数转为具体底盘参数；Omni 默认由运动约束推导前馈限幅。 */
  static typename ChassisType::ChassisParam MakeParam(
      const ChassisParam& chassis_param) {
    typename ChassisType::ChassisParam param{
        .wheel_radius = chassis_param.wheel_radius,
        .wheel_to_center = chassis_param.wheel_to_center,
        .gravity_height = chassis_param.gravity_height,
        .reduction_ratio = chassis_param.reduction_ratio,
        .wheel_resistance = chassis_param.wheel_resistance,
        .error_compensation = chassis_param.error_compensation,
        .gravity = chassis_param.gravity,
        .length = chassis_param.length,
        .width = chassis_param.width,
        .rotor_speed_scale = chassis_param.rotor_speed_scale,
        .rotor_omega_min_scale = chassis_param.rotor_omega_min_scale,
        .rotor_buffer_low_j = chassis_param.rotor_buffer_low_j,
        .rotor_buffer_high_j = chassis_param.rotor_buffer_high_j,
        .rotor_scale_lpf_alpha = chassis_param.rotor_scale_lpf_alpha};
    if constexpr (std::is_same<ChassisType, Omni>::value) {
      const auto positive = [](float value) {
        return std::isfinite(value) && value > 0.0f ? value : 0.0f;
      };
      // 用配置的逐轴加速度限幅值推导导航合力上限。
      param.ff_max_force_n = positive(
          chassis_param.ff_max_force_n == 0.0f
              ? std::numbers::sqrt2_v<float> * positive(chassis_param.mass_kg) *
                    positive(chassis_param.max_acceleration_mps2)
              : chassis_param.ff_max_force_n);
      param.ff_max_torque_nm = positive(
          chassis_param.ff_max_torque_nm == 0.0f
              ? positive(chassis_param.yaw_inertia_kg_m2) *
                    positive(chassis_param.max_angular_acceleration_rad_s2)
              : chassis_param.ff_max_torque_nm);
    }
    return param;
  }

  /** @brief 轮速正解算的实测速度快照；仅实现了该接口的底盘（Omni）可调用。 */
  auto GetMeasuredState() { return chassis_.GetMeasuredState(); }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) { chassis_.SetMode(event_id); }

  void OnMonitor() {}

 private:
  /**
   * @brief 具体底盘实现（`Omni` / `Mecanum` /
   * `Helm`），持有**全部**运行期状态。
   *
   * @note `Chassis<ChassisType>` 本身只是**适配层**，只干三件事：
   *       ① 提供 `OnMonitor`，供静态装配监控循环调用；
   *       ② 提供 Event 事件总线（`chassis_event_` + `GetEvent()`，供
   * `EventBinder` 绑定遥控通道、以及 `DualBoard` 里
   * `chassis_->GetEvent().Active(mode)` 从底盘板远程切模式）； ③ 把 manifest
   * 那份**与底盘类型无关**的 `ChassisParam` 转译成
   *          `ChassisType::ChassisParam`（各具体底盘的 manifest 是空的
   *          `standalone: false`，不能直接被 xrobot 实例化）。
   *
   *       所以 `PARAM`、`wheel_*`、`pid_*`、`dt_` 这些都在 `chassis_` 里面。
   *       ⚠️ 调试时请直接 watch
   * `chassis.chassis_`（或在已展开的节点上复制表达式），
   *       否则要先多展开一层。改成继承**并不能**省掉这一层——基类子对象在调试器里
   *       同样是一个独立节点，只是名字会变成 `<inherited
   * members>`，反而更难找。
   */
  ChassisType chassis_;
  LibXR::Event chassis_event_;
};
