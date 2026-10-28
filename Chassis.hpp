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

/// 功率控制数组中的电机数量上限
static constexpr int CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT = 6;

/**
 * @brief 提交给 PowerControl 的电机数据，3508 与 6020 各一组。
 */
struct MotorData {
  float output_current_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_3508[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};

  float output_current_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
  float rotorspeed_rpm_6020[CHASSIS_POWER_CONTROL_MAX_MOTOR_COUNT] = {};
};

/**
 * @brief 底盘订阅的 Topic 名称。
 *        Chassis::Param 的三个 Topic 名称字段在构造时打包成此结构传给内层底盘。
 */
struct ChassisTopicNames {
  const char* chassis_cmd = "chassis_cmd";  ///< 底盘控制命令 Topic
  const char* chassis_ref = "chassis_ref";  ///< 裁判系统底盘数据 Topic
  const char* gimbal_euler =
      "chassis_euler";  ///< 本板欧拉角 Topic，由 Omni 使用
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

/**
 * @brief 底盘控制模块：模板参数选择麦轮、全向轮或舵轮实现。
 *
 * @tparam ChassisType 底盘实现：Mecanum、Omni 或 Helm。
 */
template <typename ChassisType>
class Chassis {
 public:
  /// 所选底盘实现的模式枚举
  using ChassisMode = typename ChassisType::ChassisMode;

  /**
   * @brief 底盘几何、动力与小陀螺缩放参数。
   */
  struct ChassisParam {
    float wheel_radius = 0.0f;       ///< 轮半径，m
    float wheel_to_center = 0.0f;    ///< 轮心到底盘中心的距离，m
    float reduction_ratio = 0.0f;    ///< 轮电机减速比
    float rotor_speed_scale = 1.0f;  ///< 平移输入下的小陀螺转速缩放比例
    float max_acceleration_mps2 =
        2.0f;  ///< 推导导航合力限幅用逐轴加速度，m/s^2
    float max_angular_acceleration_rad_s2 =
        4.0f;                         ///< 推导导航力矩限幅用角加速度，rad/s^2
    float mass_kg = 0.0f;             ///< 整车质量，kg
    float yaw_inertia_kg_m2 = 0.0f;   ///< 绕质心偏航惯量，kg*m^2
    float wheel_resistance = 0.0f;    ///< 轮侧阻力补偿扭矩，N*m
    float error_compensation = 0.0f;  ///< 阻力补偿斜坡轮速尺度，rad/s
    float length = 0.0f;              ///< 轮距的长，m
    float width = 0.0f;               ///< 轮距的宽，m
    float rotor_omega_min_scale = 0.55f;  ///< 小陀螺动态缩放的下限
    float rotor_buffer_low_j = 35.0f;     ///< 缓冲能量低阈值，J
    float rotor_buffer_high_j = 70.0f;    ///< 缓冲能量高阈值，J
    float rotor_scale_lpf_alpha = 0.2f;   ///< 动态缩放的一阶低通系数
    /* 0 按质量/惯量和加速度限幅值推导，正数覆盖，负值关闭。 */
    float ff_max_force_n = 0.0f;    ///< 前馈合力限幅，N
    float ff_max_torque_nm = 0.0f;  ///< 前馈力矩限幅，N*m
  };

  /**
   * @brief 底盘配置参数。
   */
  struct Param {
    ChassisParam chassis_param;           ///< 底盘几何、动力与小陀螺缩放参数
    LibXR::PID<float>::Param pid_follow;  ///< 跟随云台的角度环 PID
    LibXR::PID<float>::Param pid_velocity_x;     ///< x 方向速度环 PID
    LibXR::PID<float>::Param pid_velocity_y;     ///< y 方向速度环 PID
    LibXR::PID<float>::Param pid_omega;          ///< 角速度环 PID
    LibXR::PID<float>::Param pid_wheel_speed_0;  ///< 轮 0 速度环 PID
    LibXR::PID<float>::Param pid_wheel_speed_1;  ///< 轮 1 速度环 PID
    LibXR::PID<float>::Param pid_wheel_speed_2;  ///< 轮 2 速度环 PID
    LibXR::PID<float>::Param pid_wheel_speed_3;  ///< 轮 3 速度环 PID
    LibXR::PID<float>::Param pid_steer_angle_0;  ///< 舵 0 角度环 PID
    LibXR::PID<float>::Param pid_steer_angle_1;  ///< 舵 1 角度环 PID
    LibXR::PID<float>::Param pid_steer_angle_2;  ///< 舵 2 角度环 PID
    LibXR::PID<float>::Param pid_steer_angle_3;  ///< 舵 3 角度环 PID
    LibXR::PID<float>::Param pid_steer_speed_0;  ///< 舵 0 速度环 PID
    LibXR::PID<float>::Param pid_steer_speed_1;  ///< 舵 1 速度环 PID
    LibXR::PID<float>::Param pid_steer_speed_2;  ///< 舵 2 速度环 PID
    LibXR::PID<float>::Param pid_steer_speed_3;  ///< 舵 3 速度环 PID
    LibXR::Thread::Priority thread_priority;     ///< 控制线程优先级
    const char* chassis_cmd_topic_name;  ///< 订阅的底盘控制命令 Topic 名称
    const char* chassis_ref_topic_name;  ///< 订阅的裁判系统底盘数据 Topic 名称
    const char*
        gimbal_euler_topic_name;  ///< 订阅的本板欧拉角 Topic 名称（Omni 使用）
  };

  /**
   * @brief 构造 Chassis，创建所选底盘的控制线程并注册模式事件。
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   * @param motor_wheel_1 第 1 个驱动轮电机。
   * @param motor_wheel_2 第 2 个驱动轮电机。
   * @param motor_wheel_3 第 3 个驱动轮电机。
   * @param motor_steer_0 舵向电机 0；Mecanum 为履带电机，可为 nullptr；Helm
   * 须非空； Omni 填 nullptr。
   * @param motor_steer_1 舵向电机 1；Helm 须非空，其他底盘填 nullptr。
   * @param motor_steer_2 舵向电机 2；Helm 须非空，其他底盘填 nullptr。
   * @param motor_steer_3 舵向电机 3；Helm 须非空，其他底盘填 nullptr。
   * @param cmd 控制命令模块实例，提供控制命令、控制模式与 CMD 事件。
   * @param power_control 功率控制模块实例。
   * @param referee 裁判系统模块实例。
   * @param task_stack_depth 控制线程栈深，默认 1536。
   * @param param 配置参数，默认值见 Param 与 ChassisParam。
   * @note xrobot 1.x 按构造参数名检查 YAML args；param 的 YAML
   * 映射须按结构体声明顺序列出全部字段。
   */
  Chassis(
      Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
      Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
      Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd,
      PowerControl* power_control, Referee* referee,
      uint32_t task_stack_depth = 1536,
      const Param& param = {.chassis_param = {},
                            .pid_follow = {},
                            .pid_velocity_x = {},
                            .pid_velocity_y = {},
                            .pid_omega = {},
                            .pid_wheel_speed_0 = {},
                            .pid_wheel_speed_1 = {},
                            .pid_wheel_speed_2 = {},
                            .pid_wheel_speed_3 = {},
                            .pid_steer_angle_0 = {},
                            .pid_steer_angle_1 = {},
                            .pid_steer_angle_2 = {},
                            .pid_steer_angle_3 = {},
                            .pid_steer_speed_0 = {},
                            .pid_steer_speed_1 = {},
                            .pid_steer_speed_2 = {},
                            .pid_steer_speed_3 = {},
                            .thread_priority = LibXR::Thread::Priority::HIGH,
                            .chassis_cmd_topic_name = "chassis_cmd",
                            .chassis_ref_topic_name = "chassis_ref",
                            .gimbal_euler_topic_name = "chassis_euler"})
      : chassis_(
            motor_wheel_0, motor_wheel_1, motor_wheel_2, motor_wheel_3,
            motor_steer_0, motor_steer_1, motor_steer_2, motor_steer_3, cmd,
            power_control, referee, task_stack_depth,
            MakeParam(param.chassis_param), param.pid_follow,
            param.pid_velocity_x, param.pid_velocity_y, param.pid_omega,
            param.pid_wheel_speed_0, param.pid_wheel_speed_1,
            param.pid_wheel_speed_2, param.pid_wheel_speed_3,
            param.pid_steer_angle_0, param.pid_steer_angle_1,
            param.pid_steer_angle_2, param.pid_steer_angle_3,
            param.pid_steer_speed_0, param.pid_steer_speed_1,
            param.pid_steer_speed_2, param.pid_steer_speed_3,
            param.thread_priority,
            ChassisTopicNames{.chassis_cmd = param.chassis_cmd_topic_name,
                              .chassis_ref = param.chassis_ref_topic_name,
                              .gimbal_euler = param.gimbal_euler_topic_name}) {
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
   * @brief 外壳参数转为具体底盘参数；Omni 默认由运动约束推导前馈限幅。
   * @note
   * 各底盘类型的字段含义一致但类型不同，故在构造时统一转译后送入内层底盘。
   */
  static typename ChassisType::ChassisParam MakeParam(
      const ChassisParam& chassis_param) {
    typename ChassisType::ChassisParam param{
        .wheel_radius = chassis_param.wheel_radius,
        .wheel_to_center = chassis_param.wheel_to_center,
        .reduction_ratio = chassis_param.reduction_ratio,
        .wheel_resistance = chassis_param.wheel_resistance,
        .error_compensation = chassis_param.error_compensation,
        .mass_kg = chassis_param.mass_kg,
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

  /**
   * @brief 具体底盘实现（`Omni` / `Mecanum` /
   * `Helm`），持有**全部**运行期状态。
   *
   * @note `Chassis<ChassisType>` 本身只是**适配层**，只干四件事：
   *       ① 提供 `OnMonitor`，供静态装配监控循环调用；
   *       ② 提供 Event 事件总线（`chassis_event_` + `GetEvent()`，供
   * `EventBinder` 绑定遥控通道、以及 `DualBoard` 里
   * `chassis_->GetEvent().Active(mode)` 从底盘板远程切模式）； ③ 把 manifest
   * 那份**与底盘类型无关**的 `ChassisParam` 转译成
   *          `ChassisType::ChassisParam`（各具体底盘的 manifest 是空的
   *          `standalone: false`，不能直接被 xrobot 实例化）；
   *       ④ 把三个 Topic 名称字段打包成 `ChassisTopicNames` 交给内层底盘。
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
