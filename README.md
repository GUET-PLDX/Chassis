# Chassis

## 1. 模块作用
底盘总控模块。通过模板参数选择 Omni、Mecanum 或 Helm 实现，并统一提供事件接口。

## 2. 主要函数说明
1. Chassis::GetEvent / EventHandler: 对外事件入口与模式切换代理。
2. Omni/Mecanum/Helm::ThreadFunction: 控制线程主循环。
3. Update / UpdateCMD / SetMode: 状态更新、命令解析与模式管理。
4. InverseKinematicsSolution / Helmcontrol / Output: 运动学解算与输出下发。
5. PowerControlUpdate / LostCtrl: 功率限制与失控保护。

## 3. 接入步骤
1. 添加模块并在 template_args 里选择 ChassisType。
2. 绑定轮电机、舵向电机、CMD、PowerControl。
3. 先验证 RELAX/FOLLOW/ROTOR 等模式切换，再联调整车。

底盘内部订阅的公共 topic：
- `chassis_cmd`：CMD 发布的底盘控制命令。
- `chassis_ref`：裁判系统底盘功率数据。
- `chassis_euler`：本板 AHRS 融合姿态，Omni 取 roll/pitch 做底盘倾角重力前馈。
- `yawmotor_angle`：云台 yaw 电机角度反馈，Omni 在 FOLLOW/ROTOR 模式下用于对齐底盘指令方向。

标准命令流程：
    xrobot_add_mod Chassis --instance-id chassis
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 参数配置（XRobot 1.0.1）

`Chassis::ChassisParam` 统一包含 22 项参数。以下为底盘实例的 `args` 配置片段，完整电机和 PID 参数见工程的 `User/RobotConfig/sentry_chassis.yaml`：

```yaml
  - chassis_param:
      wheel_radius: 0.075
      wheel_to_center: 0.245
      reduction_ratio: 19.2
      rotor_speed_scale: 0.80
      max_acceleration_mps2: 2.0
      max_angular_acceleration_rad_s2: 4.0
      max_jerk_mps3: 10.0
      max_angular_jerk_rad_s3: 20.0
      mass_kg: 28.0
      yaw_inertia_kg_m2: 0.58
      gravity_height: 0.0
      wheel_resistance: 0.0
      error_compensation: 0.0
      gravity: 0.0
      length: 0.0
      width: 0.0
      rotor_omega_min_scale: 0.55
      rotor_buffer_low_j: 35.0
      rotor_buffer_high_j: 70.0
      rotor_scale_lpf_alpha: 0.2
      ff_max_force_n: 0.0
      ff_max_torque_nm: 0.0
```

所有字段都填写在 `chassis_param`。默认关闭阻力/倾斜补偿，保留小陀螺能量缩放的 `0.55`、`35/70 J`、`0.2` 设置。XRobot 1.0.1 要求结构体映射字段完整匹配。

Omni 前馈限幅默认由 `sqrt(2)*mass_kg*max_acceleration_mps2` 与 `yaw_inertia_kg_m2*max_angular_acceleration_rad_s2` 推导。`chassis_param` 中的 `ff_max_force_n` / `ff_max_torque_nm` 为 0 时自动推导，正数指定独立限幅，负数关闭对应前馈。较小正限幅同时收紧规划加速度。

## 5. 依赖与硬件
Required Hardware:
  - dr16
  - motor
  - can
  - bmi088

Depends:
  - qdu-future/BMI088
  - qdu-future/RMMotor
  - qdu-future/CMD
  - xrobot-org/MadgwickAHRS

## 6. 代码入口
Modules/Chassis/Chassis.hpp

## LibXR 2.0 静态装配

使用 xrobot 1.0.1 的 `module` / 有序 `args`；构造函数显式注入外设与服务，实例按依赖顺序创建。参数名和顺序以头文件中的构造函数为准，固件示例见 `User/RobotConfig/`。

保留 PLDX 控制、协议、超时与离线保护。 `OnMonitor()` 由生成的监控循环直接调用。
