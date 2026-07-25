#pragma once

#include <array>
#include <string>

namespace moving_deck_sim
{

/**
 * @brief 传统基线支持的解析甲板运动场景。
 */
enum class Scenario
{
  kStatic,
  kConstantXy,
  kSinusoidalXy,
  kHeave,
  kRollPitch,
  kCombined,
};

/**
 * @brief 甲板解析运动参数。
 *
 * 位置和线速度使用 Gazebo world ENU；姿态使用 world ENU 下的 ZYX roll/pitch/yaw，
 * 角度单位为弧度；`angular_velocity_body` 使用甲板机体系角速度。
 */
struct MotionParameters
{
  Scenario scenario{Scenario::kStatic};
  std::array<double, 3> initial_position_enu{0.0, 0.0, 2.0};
  std::array<double, 2> velocity_xy{0.0, 0.0};
  std::array<double, 2> amplitude_xy{0.0, 0.0};
  std::array<double, 2> period_xy{1.0, 1.0};
  double amplitude_z_m{0.0};
  double period_z_s{1.0};
  std::array<double, 3> initial_rpy_rad{0.0, 0.0, 0.0};
  std::array<double, 3> amplitude_rpy_rad{0.0, 0.0, 0.0};
  std::array<double, 3> period_rpy_s{1.0, 1.0, 1.0};
  double update_rate_hz{50.0};
};

/**
 * @brief 某一仿真时刻的甲板解析位姿、线速度和机体系角速度。
 */
struct MotionSample
{
  std::array<double, 3> position_enu{};
  std::array<double, 3> orientation_rpy_enu{};
  std::array<double, 3> velocity_enu{};
  std::array<double, 3> angular_velocity_body{};
};

/**
 * @brief 计算静止、水平、升沉、倾斜和组合甲板解析轨迹。
 *
 * 该类不依赖 ROS 或 Gazebo。旧 `S0`～`S2` 行为保持不变；`S3` 只启用升沉，
 * `S4` 只启用姿态周期运动，`S5` 同时启用 XY 正弦、升沉和姿态周期运动。
 */
class MotionProfile
{
public:
  /**
   * @brief 从 YAML 场景名解析运动类型。
   *
   * @param name 支持 `S0_STATIC`、`S1_CONSTANT_XY`、`S2_SINUSOIDAL_XY`、
   *        `S3_HEAVE`、`S4_ROLL_PITCH` 和 `S5_COMBINED`。
   * @return 对应的场景枚举。
   * @throws std::invalid_argument 场景名不受支持时抛出。
   */
  static Scenario parse_scenario(const std::string & name);

  /**
   * @brief 创建并校验解析轨迹。
   *
   * @throws std::invalid_argument 任一数值非有限、周期非正、姿态幅值达到或超过
   *         90°，或更新频率非法时抛出。
   */
  explicit MotionProfile(MotionParameters parameters);

  /**
   * @brief 计算从本轮场景开始计时后的解析状态。
   *
   * @param elapsed_s 自场景开始后的仿真时间，单位为秒，必须为有限非负数。
   * @return Gazebo world ENU 位姿、线速度和甲板机体系角速度。
   * @throws std::invalid_argument elapsed_s 非法时抛出。
   */
  MotionSample sample(double elapsed_s) const;

private:
  MotionParameters parameters_;
};

}  // namespace moving_deck_sim
