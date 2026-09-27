#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "CMD.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "timebase.hpp"
#include "transform.hpp"

#define M3508_NM_TO_LSB_RATIO 52437.5f /* 3508 转子扭矩到电机控制值的比例 */

#define OMNI_MOTOR_MAX_OMEGA 52 /* 全向轮输出轴最大角速度 rad/s */

#define OMNI_CHASSIS_MAX_POWER 100 /* 裁判系统离线时的默认功率上限 W */

template <typename ChassisType>
class Chassis;

class Omni {
 public:
  struct ChassisParam {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float gravity_height = 0.0f;
    float reduction_ratio = 0.0f;
    float wheel_resistance = 0.0f; /* Wheel-side resistance torque, N*m. */
    float error_compensation =
        0.0f; /* Full-compensation wheel speed, rad/s; zero disables ramping. */
    float gravity = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float rotor_speed_scale =
        1.0f; /* 小陀螺转速缩放比例，降低可给平移留出更多功率 */
    float rotor_omega_min_scale = 0.55f; /* 功率受限时小陀螺最小转速比例 */
    float rotor_buffer_low_j = 35.0f;    /* 缓冲能量低阈值 J */
    float rotor_buffer_high_j = 70.0f;   /* 缓冲能量高阈值 J */
    float rotor_scale_lpf_alpha = 0.2f;  /* 动态缩放一阶低通系数 */
    /* 变速小陀螺（ROTOR_VARIABLE）：扭矩域滞环参数 */
    float spin_torque_amp_nm = 15.0f;   /* DRIVE 相底盘偏航扭矩幅值 N·m */
    float spin_omega_hi_rad_s = 14.0f;  /* 滞环上限，进入滑行 rad/s */
    float spin_omega_lo_rad_s = 6.0f;   /* 滞环下限，恢复驱动 rad/s */
    float spin_omega_max_rad_s = 16.0f; /* 硬安全上限，超限强制滑行 rad/s */
    float spin_drive_timeout_s = 2.0f;  /* 单次 DRIVE 相最长时间 s */
  };
  enum class ChassisMode : uint8_t {
    RELAX,
    INDEPENDENT,
    ROTOR,
    FOLLOW,
    NAVIGATION,
    ROTOR_VARIABLE, /* 变速小陀螺：扭矩域滞环（追加尾部，线值=5） */
  };

  /**
   * @brief 构造函数，初始化全向轮底盘控制对象
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令引用
   * @param motor_wheel_0 第0个驱动轮电机指针
   * @param motor_wheel_1 第1个驱动轮电机指针
   * @param motor_wheel_2 第2个驱动轮电机指针
   * @param motor_wheel_3 第3个驱动轮电机指针
   * @param motor_steer_0 第0个舵向电机指针（本底盘未使用）
   * @param motor_steer_1 第1个舵向电机指针（本底盘未使用）
   * @param motor_steer_2 第2个舵向电机指针（本底盘未使用）
   * @param motor_steer_3 第3个舵向电机指针（本底盘未使用）
   * @param task_stack_depth 控制线程栈深度
   * @param chassis_param 全向轮底盘参数
   * @param pid_follow 跟随控制PID参数
   * @param pid_velocity_x X方向速度PID参数
   * @param pid_velocity_y Y方向速度PID参数
   * @param pid_omega 角速度PID参数
   * @param pid_wheel_omega_0 轮子0角速度PID参数
   * @param pid_wheel_omega_1 轮子1角速度PID参数
   * @param pid_wheel_omega_2 轮子2角速度PID参数
   * @param pid_wheel_omega_3 轮子3角速度PID参数
   * @param pid_steer_angle_0 舵机0角度PID参数（本底盘未使用）
   * @param pid_steer_angle_1 舵机1角度PID参数（本底盘未使用）
   * @param pid_steer_angle_2 舵机2角度PID参数（本底盘未使用）
   * @param pid_steer_angle_3 舵机3角度PID参数（本底盘未使用）
   */
  Omni(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
       Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
       Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
       Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd,
       PowerControl* power_control, Referee* referee, uint32_t task_stack_depth,
       ChassisParam chassis_param, LibXR::PID<float>::Param pid_follow,
       LibXR::PID<float>::Param pid_velocity_x,
       LibXR::PID<float>::Param pid_velocity_y,
       LibXR::PID<float>::Param pid_omega,
       LibXR::PID<float>::Param pid_wheel_speed_0,
       LibXR::PID<float>::Param pid_wheel_speed_1,
       LibXR::PID<float>::Param pid_wheel_speed_2,
       LibXR::PID<float>::Param pid_wheel_speed_3,
       LibXR::PID<float>::Param pid_steer_angle_0,
       LibXR::PID<float>::Param pid_steer_angle_1,
       LibXR::PID<float>::Param pid_steer_angle_2,
       LibXR::PID<float>::Param pid_steer_angle_3,
       LibXR::PID<float>::Param pid_steer_speed_0,
       LibXR::PID<float>::Param pid_steer_speed_1,
       LibXR::PID<float>::Param pid_steer_speed_2,
       LibXR::PID<float>::Param pid_steer_speed_3,
       LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : PARAM(chassis_param),
        /* 电机布局：
         *  wheel0   ▲ y  wheel3
         *      ↙    │     ↖
         *           │
         *  ─────────┼────────▶x
         *           │
         *      ↘    │     ↗
         *  wheel1   │    wheel2 */
        wheel_{.motor = {motor_wheel_0, motor_wheel_1, motor_wheel_2,
                         motor_wheel_3}},
        pid_follow_(pid_follow),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1,
                         pid_wheel_speed_2, pid_wheel_speed_3},
        cmd_(cmd),
        power_control_(power_control) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(motor_steer_0);
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    UNUSED(pid_steer_speed_0);
    UNUSED(pid_steer_speed_1);
    UNUSED(pid_steer_speed_2);
    UNUSED(pid_steer_speed_3);
    UNUSED(pid_steer_angle_0);
    UNUSED(pid_steer_angle_1);
    UNUSED(pid_steer_angle_2);
    UNUSED(pid_steer_angle_3);

    for (int i = 0; i < 4; i++) {
      wheel_.command[i].mode = Motor::ControlMode::MODE_TORQUE;
      wheel_.command[i].reduction_ratio = chassis_param.reduction_ratio;
      wheel_.command[i].torque = 0.0f;
      wheel_.command[i].position = 0.0f;
      wheel_.command[i].velocity = 0.0f;
      wheel_.command[i].kp = 0.0f;
      wheel_.command[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "OmniChassisThread", task_stack_depth,
                   thread_priority);
    /* 本底盘类型无裁判系统 UI，referee 参数仅为与共享外壳签名一致而保留 */
    UNUSED(referee);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Omni* omni, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          omni->mutex_.Lock();
          omni->chassis_event_ = ChassisMode::RELAX;
          omni->mutex_.Unlock();
        },
        this);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Omni* omni, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          omni->mutex_.Lock();
          omni->chassis_event_ = ChassisMode::RELAX;
          omni->LostCtrl();
          omni->mutex_.Unlock();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 全向轮底盘控制线程函数
   * @param omni Omni对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Omni* omni) {
    omni->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        "chassis_ref");
    /* 底盘倾角直接取自本板 AHRS，不再跨板取云台板 IMU 姿态做反推 */
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> chassis_euler_suber(
        "chassis_euler");
    LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>> gyro_suber(
        "chassis_gyro");
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    chassis_euler_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();
    gyro_suber.StartWaiting();
    omni->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();
    omni->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        omni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available()) {
        omni->referee_chassis_pack_ = referee_suber.GetData();
        referee_suber.StartWaiting();
      }

      if (chassis_euler_suber.Available()) {
        omni->motion_.euler = chassis_euler_suber.GetData();
        chassis_euler_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available()) {
        omni->motion_.yawmotor_angle = yawmotor_angle_suber.GetData();
        yawmotor_angle_suber.StartWaiting();
      }
      if (gyro_suber.Available()) {
        omni->motion_.gyro_wz = gyro_suber.GetData().z();
        omni->last_gyro_time_ = LibXR::Timebase::GetMicroseconds();
        gyro_suber.StartWaiting();
      }
      omni->mutex_.Lock();
      omni->Update();
      omni->UpdateCMD();
      omni->SelfResolution();
      omni->InverseKinematicsSolution();
      omni->DynamicInverseSolution();
      omni->FeedForward();
      omni->CalculateMotorCurrent();
      omni->PowerControlUpdate();
      omni->OutputToDynamics();
      omni->mutex_.Unlock();

      omni->thread_.SleepUntil(last_time, 1);
    }
  }

  /**
   * @brief 更新电机状态
   * @details 获取当前时间戳并更新所有驱动轮电机的状态
   */
  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    for (int i = 0; i < 4; i++) {
      const bool WHEEL_UPDATE_OK =
          wheel_.motor[i]->Update() == LibXR::ErrorCode::OK;
      wheel_.feedback[i] = wheel_.motor[i]->GetFeedback();
      wheel_.online[i] = WHEEL_UPDATE_OK && wheel_.feedback[i].state != 0U;
    }
  }

  /** @brief 在互斥保护下读取实际模式，供双板链路确认切换结果。 */
  ChassisMode GetMode() {
    LibXR::Mutex::LockGuard lock(mutex_);
    return chassis_event_;
  }

  /**
   * @brief 设置底盘模式 (由 Chassis 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    const ChassisMode NEXT_MODE = static_cast<ChassisMode>(mode);
    chassis_event_ = NEXT_MODE;
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; i++) {
      pid_wheel_speed_[i].Reset();
    }
    spin_.driving = false;
    spin_.tau_cmd = 0.0f;
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 从CMD获取底盘控制指令，并转换为目标速度
   */
  void UpdateCMD() {
    const float max_v =
        PARAM.wheel_radius * static_cast<float>(OMNI_MOTOR_MAX_OMEGA);

    if (cmd_data_.source == CMD::ChassisCommandSource::NAVIGATION) {
      /* 导航来源的速度已是物理量且已在主机侧旋到车体系，直接整组取用 */
      motion_.setpoint = cmd_data_.navigation_velocity;
      switch (chassis_event_) {
        case ChassisMode::RELAX:
          motion_.setpoint.vx_mps = 0.0F;
          motion_.setpoint.vy_mps = 0.0F;
          motion_.setpoint.wz_rad_s = 0.0F;
          break;
        case ChassisMode::NAVIGATION:
          motion_.setpoint.wz_rad_s = 0.0F;
          break;
        case ChassisMode::ROTOR:
          motion_.setpoint.wz_rad_s =
              -static_cast<float>(max_v / PARAM.wheel_to_center);
          break;
        case ChassisMode::FOLLOW:
          motion_.setpoint.wz_rad_s =
              -pid_follow_.Calculate(0.0f, motion_.yawmotor_angle, dt_);
          break;
        default:
          motion_.setpoint.vx_mps = 0.0F;
          motion_.setpoint.vy_mps = 0.0F;
          motion_.setpoint.wz_rad_s = 0.0F;
          break;
      }
    } else {
      /* 先生成目标角速度 */
      switch (chassis_event_) {
        case ChassisMode::RELAX:
          motion_.setpoint.wz_rad_s = 0.0f;
          break;

        case ChassisMode::INDEPENDENT:
          motion_.setpoint.wz_rad_s =
              max_v * cmd_data_.operator_input.z / PARAM.wheel_to_center;
          break;

        /* 扭矩域：速度由积分自然形成，速度设定占位为 0，
         * 旋转分量在逆运动学处改用实测值回填 */
        case ChassisMode::ROTOR_VARIABLE:
          motion_.setpoint.wz_rad_s = 0.0f;
          break;

        case ChassisMode::ROTOR:
          motion_.setpoint.wz_rad_s =
              -static_cast<float>(max_v / PARAM.wheel_to_center);
          break;

          /* 正方向跟随云台 */
        case ChassisMode::FOLLOW:
          motion_.setpoint.wz_rad_s =
              -pid_follow_.Calculate(0.0f, motion_.yawmotor_angle, dt_);
          break;

        case ChassisMode::NAVIGATION:
          motion_.setpoint.wz_rad_s = 0.0F;
          break;

        default:
          break;
      }

      /* 再生成目标平移速度 */
      switch (chassis_event_) {
        case ChassisMode::RELAX:
          motion_.setpoint.vx_mps = 0.0f;
          motion_.setpoint.vy_mps = 0.0f;
          break;
        case ChassisMode::ROTOR:
        case ChassisMode::ROTOR_VARIABLE:
        case ChassisMode::FOLLOW: {
          float beta = motion_.yawmotor_angle;
          float cos_beta = cosf(beta);
          float sin_beta = sinf(beta);
          motion_.setpoint.vx_mps =
              (cos_beta * cmd_data_.operator_input.x * max_v -
               sin_beta * cmd_data_.operator_input.y * max_v);
          motion_.setpoint.vy_mps =
              (sin_beta * cmd_data_.operator_input.x * max_v +
               cos_beta * cmd_data_.operator_input.y * max_v);
        } break;
        case ChassisMode::INDEPENDENT: {
          /* 独立模式用菱形限幅适配摇杆边界 */
          float s = fabsf(cmd_data_.operator_input.x) +
                    fabsf(cmd_data_.operator_input.y);
          float k = (s <= 1.0f) ? max_v : (max_v / s);
          motion_.setpoint.vx_mps = SQRT2 * k * cmd_data_.operator_input.x;
          motion_.setpoint.vy_mps = SQRT2 * k * cmd_data_.operator_input.y;
        } break;
        case ChassisMode::NAVIGATION:
          motion_.setpoint.vx_mps = 0.0F;
          motion_.setpoint.vy_mps = 0.0F;
          break;
        default:
          break;
      }
    }

    /* 小陀螺模式按平移输入和功率状态动态压低转速 */
    float rotor_translation_scale = 1.0f;
    if (chassis_event_ == ChassisMode::ROTOR) {
      float translation_magnitude =
          sqrtf(motion_.setpoint.vx_mps * motion_.setpoint.vx_mps +
                motion_.setpoint.vy_mps * motion_.setpoint.vy_mps);
      float translation_ratio = 0.0f;
      if (max_v > 1e-3f) {
        translation_ratio =
            std::clamp(translation_magnitude / max_v, 0.0f, 1.0f);
      }
      rotor_translation_scale =
          1.0f - (1.0f - PARAM.rotor_speed_scale) * translation_ratio;
      motion_.setpoint.wz_rad_s *=
          rotor_translation_scale * motion_.rotor_dynamic_scale;
    }
  }

  /**
   * @brief 前馈死区软限幅
   * @param x 输入值
   * @param dz 死区范围
   * @return 软限幅后的输出
   */
  float SoftDeadzone(float x, float dz) {
    if (fabs(x) < dz) {
      return 0.0f;
    } else {
      return (fabs(x) - dz) * (x > 0.0f ? 1.0f : -1.0f);
    }
  }

  /* Calculate wheel-side resistance compensation torque in N*m. */
  float ResistanceTorque(float target_omega) const {
    constexpr float RESISTANCE_EPSILON = 1e-6f;
    if (PARAM.wheel_resistance <= RESISTANCE_EPSILON ||
        fabsf(target_omega) <= RESISTANCE_EPSILON) {
      return 0.0f;
    }

    if (PARAM.error_compensation > RESISTANCE_EPSILON) {
      return PARAM.wheel_resistance *
             std::clamp(target_omega / PARAM.error_compensation, -1.0f, 1.0f);
    }

    return copysignf(PARAM.wheel_resistance, target_omega);
  }

  /**
   * @brief 计算姿态前馈
   * @details 底盘倾角直接取自本板 AHRS（`chassis_euler`），无需再借云台板 IMU
   *          姿态与云台关节角做跨板反推，也不再做反号适配。
   *          所有中间量（GX_FF/GY_FF/PX/PY/length 等）均为本函数局部量，
   *          每周期从 `motion_.euler` 与 PARAM 重新推导，结果直写
   *          `output_.feedforward_torque`。
   * @warning gx/gy/px/py 的符号由此处的符号约定决定。当前
   *          `sentry_chassis.yaml` 的 gravity 与 gravity_height 均为 0，前馈
   *          输出恒为 0，符号变更不可观测；填入实车值前必须先确认符号。
   */
  void FeedForward() {
    /* 四个轮位相对车体中心的坐标，只由几何参数决定 */
    const float HALF_WIDTH = PARAM.width / 2;
    const float HALF_LENGTH = PARAM.length / 2;
    const float POST_X[4] = {-HALF_WIDTH, -HALF_WIDTH, HALF_WIDTH, HALF_WIDTH};
    const float POST_Y[4] = {HALF_LENGTH, -HALF_LENGTH, -HALF_LENGTH,
                             HALF_LENGTH};

    const float K = static_cast<float>(LibXR::PI / 50.0);
    const float GY_FF =
        -PARAM.gravity * SoftDeadzone(sinf(motion_.euler.Pitch()), sinf(K));
    const float GX_FF =
        -PARAM.gravity * SoftDeadzone(sinf(motion_.euler.Roll()), sinf(K));
    const float PY = -PARAM.gravity_height *
                     SoftDeadzone(sinf(motion_.euler.Pitch()), sin(K));
    const float PX = -PARAM.gravity_height *
                     SoftDeadzone(sinf(motion_.euler.Roll()), sin(K));

    const float DIR[4] = {GX_FF + GY_FF, -GX_FF + GY_FF, -GX_FF - GY_FF,
                          GX_FF - GY_FF};

    float length[4];
    float length_inv_sum = 0.0f;
    for (int i = 0; i < 4; i++) {
      const float DX = POST_X[i] - PX;
      const float DY = POST_Y[i] - PY;
      length[i] = sqrtf(DX * DX + DY * DY);
      length_inv_sum += 1.0f / length[i];
    }

    for (int i = 0; i < 4; i++) {
      const float TORQUE_N = (1.0f / length[i]) / length_inv_sum;
      output_.feedforward_torque[i] =
          DIR[i] * SQRT2 * TORQUE_N * PARAM.wheel_radius;
    }
  }

  /**
   * @brief 全向轮底盘正运动学解算
   * @details 根据四个全向轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    motion_.measured.vx_mps =
        -(wheel_.feedback[0].omega / PARAM.reduction_ratio -
          wheel_.feedback[1].omega / PARAM.reduction_ratio -
          wheel_.feedback[2].omega / PARAM.reduction_ratio +
          wheel_.feedback[3].omega / PARAM.reduction_ratio) *
        SQRT2 * PARAM.wheel_radius / 4.0f;

    motion_.measured.vy_mps =
        -(wheel_.feedback[0].omega / PARAM.reduction_ratio +
          wheel_.feedback[1].omega / PARAM.reduction_ratio -
          wheel_.feedback[2].omega / PARAM.reduction_ratio -
          wheel_.feedback[3].omega / PARAM.reduction_ratio) *
        SQRT2 * PARAM.wheel_radius / 4.0f;

    motion_.measured.wz_rad_s =
        (wheel_.feedback[0].omega / PARAM.reduction_ratio +
         wheel_.feedback[1].omega / PARAM.reduction_ratio +
         wheel_.feedback[2].omega / PARAM.reduction_ratio +
         wheel_.feedback[3].omega / PARAM.reduction_ratio) *
        PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 全向轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个全向轮的目标角速度
   */
  void InverseKinematicsSolution() {
    const float VX = motion_.setpoint.vx_mps;
    const float VY = motion_.setpoint.vy_mps;
    /* 扭矩域无旋转速度设定：旋转分量按实测回填，
     * 使轮速误差仅反映平移需求（功率分配语义正确） */
    const float WZ = (chassis_event_ == ChassisMode::ROTOR_VARIABLE)
                         ? motion_.measured.wz_rad_s
                         : motion_.setpoint.wz_rad_s;
    output_.omega_setpoint[0] =
        (-INV_SQRT2 * VX - INV_SQRT2 * VY + WZ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    output_.omega_setpoint[1] =
        (INV_SQRT2 * VX - INV_SQRT2 * VY + WZ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    output_.omega_setpoint[2] =
        (INV_SQRT2 * VX + INV_SQRT2 * VY + WZ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    output_.omega_setpoint[3] =
        (-INV_SQRT2 * VX + INV_SQRT2 * VY + WZ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
  }

  /**
   * @brief 计算 PID 输出电流
   */
  void CalculateMotorCurrent() {
    if (chassis_event_ == ChassisMode::RELAX) {
      LostCtrl();
    } else if (chassis_event_ == ChassisMode::ROTOR_VARIABLE) {
      /* 扭矩域：无轮速闭环（与扭矩波形互斥），力设定 + 倾角前馈直通；
       * 滑行相力设定为 0 → 请求电流≈0，摩擦自然减速 */
      for (int i = 0; i < 4; i++) {
        output_.torque_output[i] =
            output_.force_setpoint[i] * PARAM.wheel_radius +
            output_.feedforward_torque[i];
      }
    } else {
      /* 轮速 PID 输出（仅本函数写读，局部化，不入实例） */
      float speed_pid_output[4];
      for (int i = 0; i < 4; i++) {
        speed_pid_output[i] = pid_wheel_speed_[i].Calculate(
            output_.omega_setpoint[i],
            wheel_.feedback[i].omega / PARAM.reduction_ratio, dt_);
      }

      /* 合成动力学输出和上坡前馈 */
      for (int i = 0; i < 4; i++) {
        output_.torque_output[i] =
            output_.force_setpoint[i] * PARAM.wheel_radius +
            speed_pid_output[i] + output_.feedforward_torque[i] +
            ResistanceTorque(output_.omega_setpoint[i]);
      }
    }
  }

  /**
   * @brief 功率控制更新
   */
  void PowerControlUpdate() {
    for (int i = 0; i < 4; i++) {
      power_.motor_data_.rotorspeed_rpm_3508[i] = wheel_.feedback[i].velocity;
      power_.motor_data_.output_current_3508[i] =
          wheel_.feedback[i].torque * M3508_NM_TO_LSB_RATIO;
    }

    power_control_->SetMotorFeedback3508(power_.motor_data_.output_current_3508,
                                         power_.motor_data_.rotorspeed_rpm_3508,
                                         4, wheel_.online);

    float wheel_speed_error[4];
    for (int i = 0; i < 4; i++) {
      wheel_speed_error[i] = output_.omega_setpoint[i] -
                             wheel_.feedback[i].omega / PARAM.reduction_ratio;
      power_.motor_data_.output_current_3508[i] =
          std::clamp(output_.torque_output[i] * M3508_NM_TO_LSB_RATIO /
                         PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }

    power_control_->SetMotorData3508(power_.motor_data_.output_current_3508,
                                     power_.motor_data_.rotorspeed_rpm_3508,
                                     wheel_speed_error, 4, wheel_.online);

    power_control_->SetBoostRequested(cmd_data_.self_define ==
                                      CMD::ChasStat::BOOST);
    power_control_->OutputLimit();
    power_.limited = power_control_->GetPowerControlData();

    float req_current_abs_sum = 0.0f;
    float lim_current_abs_sum = 0.0f;
    for (int i = 0; i < 4; i++) {
      float req_current_abs = fabsf(power_.motor_data_.output_current_3508[i]);
      float lim_current_abs =
          power_.limited.is_power_limited
              ? fabsf(power_.limited.new_output_current_3508[i])
              : req_current_abs;
      req_current_abs_sum += req_current_abs;
      lim_current_abs_sum += lim_current_abs;
    }

    float power_limit_ratio = 1.0f;
    if (req_current_abs_sum > 1e-3f) {
      power_limit_ratio =
          std::clamp(lim_current_abs_sum / req_current_abs_sum, 0.0f, 1.0f);
    }

    float buffer_scale = 1.0f;
    if (power_.limited.referee_energy_buffer_online) {
      float buffer_range =
          std::max(PARAM.rotor_buffer_high_j - PARAM.rotor_buffer_low_j, 1.0f);
      float referee_buffer_j =
          static_cast<float>(referee_chassis_pack_.power_buffer);
      float buffer_norm = std::clamp(
          (referee_buffer_j - PARAM.rotor_buffer_low_j) / buffer_range, 0.0f,
          1.0f);
      buffer_scale = PARAM.rotor_omega_min_scale +
                     (1.0f - PARAM.rotor_omega_min_scale) * buffer_norm;
    }

    float limit_scale =
        power_.limited.is_power_limited
            ? std::clamp(power_limit_ratio, PARAM.rotor_omega_min_scale, 1.0f)
            : 1.0f;
    float target_dynamic_scale = std::clamp(buffer_scale * limit_scale,
                                            PARAM.rotor_omega_min_scale, 1.0f);
    float lpf_alpha = std::clamp(PARAM.rotor_scale_lpf_alpha, 0.0f, 1.0f);
    motion_.rotor_dynamic_scale +=
        (target_dynamic_scale - motion_.rotor_dynamic_scale) * lpf_alpha;
    motion_.rotor_dynamic_scale = std::clamp(motion_.rotor_dynamic_scale,
                                             PARAM.rotor_omega_min_scale, 1.0f);

    if (chassis_event_ != ChassisMode::ROTOR &&
        chassis_event_ != ChassisMode::ROTOR_VARIABLE) {
      motion_.rotor_dynamic_scale = 1.0f;
    }
  }

  /**
   * @brief 变速小陀螺扭矩波形机（滞环：恒扭矩驱动 / 零扭矩滑行）
   * @details 扭矩域控制，速度是力的积分，不做连续速度闭环。
   *          ω 优先取本板 IMU 陀螺 z（轮速里程计在打滑时失真），
   *          陀螺 100 ms 未更新则回退正解算值。转向与现有 ROTOR 一致（负向）。
   */
  void UpdateSpinCommand() {
    const auto NOW = LibXR::Timebase::GetMicroseconds();
    const float OMEGA_MEASURED = (NOW - last_gyro_time_ < 100000)
                                     ? motion_.gyro_wz
                                     : motion_.measured.wz_rad_s;
    const float OMEGA_ABS = fabsf(OMEGA_MEASURED);

    if (spin_.driving) {
      if (OMEGA_ABS >= PARAM.spin_omega_hi_rad_s ||
          OMEGA_ABS > PARAM.spin_omega_max_rad_s ||
          NOW - spin_.phase_start_us >=
              static_cast<LibXR::MicrosecondTimestamp>(
                  PARAM.spin_drive_timeout_s * 1.0e6f)) {
        spin_.driving = false; /* 到上限/超硬限/超时 → 滑行 */
      }
    } else {
      if (OMEGA_ABS <= PARAM.spin_omega_lo_rad_s) {
        spin_.driving = true; /* 降到下限 → 恒扭矩驱动 */
        spin_.phase_start_us = NOW;
      }
    }

    spin_.tau_cmd = spin_.driving ? -PARAM.spin_torque_amp_nm : 0.0f;
  }

  /**
   * @brief 全向轮底盘逆动力学解算
   * @details
   * 通过运动学正解算出底盘现在的运动状态，并与目标状态进行PID控制，获得目标前馈力矩
   */
  void DynamicInverseSolution() {
    const float FORCE_X = pid_velocity_x_.Calculate(
        motion_.setpoint.vx_mps, motion_.measured.vx_mps, dt_);
    const float FORCE_Y = pid_velocity_y_.Calculate(
        motion_.setpoint.vy_mps, motion_.measured.vy_mps, dt_);

    /* 扭矩域：跳过 pid_omega_（速度环与扭矩波形互斥），
     * FORCE_Z 由波形机直接给定；速度域：pid_omega_ 闭环 */
    float force_z;
    if (chassis_event_ == ChassisMode::ROTOR_VARIABLE) {
      UpdateSpinCommand();
      force_z = spin_.tau_cmd / PARAM.wheel_to_center;
    } else {
      force_z = pid_omega_.Calculate(motion_.setpoint.wz_rad_s,
                                     motion_.measured.wz_rad_s, dt_);
    }

    /* 按全向轮受力方向分配前馈力 */
    output_.force_setpoint[0] =
        (-SQRT2 * FORCE_X - SQRT2 * FORCE_Y + force_z) / 4;
    output_.force_setpoint[1] =
        (SQRT2 * FORCE_X - SQRT2 * FORCE_Y + force_z) / 4;
    output_.force_setpoint[2] =
        (SQRT2 * FORCE_X + SQRT2 * FORCE_Y + force_z) / 4;
    output_.force_setpoint[3] =
        (-SQRT2 * FORCE_X + SQRT2 * FORCE_Y + force_z) / 4;
  }

  /**
   * @brief 全向轮底盘动力学输出
   * @details 限幅并输出四个全向轮的电流控制指令
   */
  void OutputToDynamics() {
    for (int i = 0; i < 4; i++) {
      output_.torque_output[i] =
          std::clamp(power_.limited.new_output_current_3508[i] /
                         M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                     -6.0f, 6.0f);
    }
    if (chassis_event_ == ChassisMode::RELAX) {
      LostCtrl();
      return;
    } else {
      for (int i = 0; i < 4; i++) {
        wheel_.command[i].torque =
            std::clamp(output_.torque_output[i], -6.0f, 6.0f);
      }
      for (int i = 0; i < 4; i++) {
        wheel_.motor[i]->Control(wheel_.command[i]);
      }
    }
  }
  /**
   * @brief 失去控制时的处理
   *
   */
  void LostCtrl() {
    for (int i = 0; i < 4; i++) {
      wheel_.motor[i]->Relax();
    }
  }

 private:
  /* === 状态分层（对齐 Gimbal 范式）===
   * 1. PARAM    构造期配置，const；
   * 2. wheel_   执行器层：电机句柄 + 每轮反馈/命令/在线状态；
   * 3. motion_  运动状态：跨周期、watch 主要观测入口；
   * 4. output_  输出链：单周期内跨函数生产/消费的中间量；
   * 5. power_   功率遥测：与 PowerControl 的往返数据。
   * 其余平铺成员与 Gimbal 同构：PID 组 / 时序 / 模式 / 框架句柄。
   * 调试：watch `chassis.chassis_` 按声明顺序展开以上节点。 */

  /* 几何/运动学常量（来自 <numbers>，与旧字面量逐位一致）。
     注意：本工具链 libc++ 无 std::numbers::inv_sqrt2，用精确的 /2 派生。 */
  static constexpr float SQRT2 = static_cast<float>(std::numbers::sqrt2);
  static constexpr float INV_SQRT2 = SQRT2 / 2.0f;

  const ChassisParam PARAM;

  /* 执行器层。电机布局：
   *  wheel0   ▲ y  wheel3
   *      ↙    │     ↖
   *           │
   *  ─────────┼────────▶x
   *           │
   *      ↘    │     ↗
   *  wheel1   │    wheel2 */
  struct Wheels {
    Motor* motor[4]{};
    Motor::Feedback feedback[4]{};
    Motor::MotorCmd command[4]{}; /* 恒 MODE_TORQUE，构造期置参 */
    bool online[4]{};
  };
  Wheels wheel_;

  /* 车体系运动状态（m/s、rad/s）。setpoint 由 UpdateCMD() 解析，
   * measured 由 SelfResolution() 正解；euler 来自本板 chassis_euler
   * 主题，是底盘倾角前馈的直接来源。yawmotor_angle 仅用于
   * FOLLOW/ROTOR 的指令旋转变换，勿动其 wrap/zero 语义。 */
  struct MotionState {
    CMD::NavigationVelocity setpoint{};
    CMD::NavigationVelocity measured{};
    LibXR::EulerAngle<float> euler{};
    float yawmotor_angle = 0.0f;
    float gyro_wz = 0.0f; /* 本板 IMU 陀螺 z（rad/s），变速小陀螺 ω 源 */
    float rotor_dynamic_scale = 1.0f; /* 功率相关动态缩放 */
  };
  MotionState motion_;

  /* 输出链：每周期依序经 InverseKinematicsSolution() →
   * DynamicInverseSolution() → CalculateMotorCurrent() →
   * PowerControlUpdate() → OutputToDynamics() 各级生产/消费，
   * 跨函数因此保留为成员；同函数生灭的中间量一律局部化。 */
  struct OutputStage {
    float omega_setpoint[4]{};
    float force_setpoint[4]{};
    float torque_output[4]{};
    float feedforward_torque[4]{}; /* FeedForward() 写 → CalculateMotorCurrent
                                      读 */
  };
  OutputStage output_;

  /* 功率遥测：motor_data_ 为送入 PowerControl 的电机数据往返，
   * limited 为限幅后的输出快照。 */
  struct PowerStage {
    MotorData motor_data_{};
    PowerControlData limited{};
  };
  PowerStage power_;

  /* 变速小陀螺（ROTOR_VARIABLE）扭矩波形机状态 */
  struct SpinState {
    bool driving = false; /* true=DRIVE 恒扭矩，false=COAST 滑行 */
    LibXR::MicrosecondTimestamp phase_start_us = 0;
    float tau_cmd = 0.0f; /* 底盘级偏航扭矩指令 N·m（含符号） */
  };
  SpinState spin_;

  LibXR::PID<float> pid_follow_;
  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_speed_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  LibXR::MicrosecondTimestamp last_gyro_time_ = 0; /* 陀螺新鲜度判据 */

  ChassisMode chassis_event_ = ChassisMode::RELAX;

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_{};

  PowerControl* power_control_;
  Referee::ChassisPack referee_chassis_pack_{};

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
};
