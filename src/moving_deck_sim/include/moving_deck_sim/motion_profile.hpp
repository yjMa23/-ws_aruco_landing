#pragma once

#include <array>
#include <string>

namespace moving_deck_sim
{

/**
 * @brief P1 阶段支持的水平甲板运动场景。
 */
enum class Scenario
{
  kStatic,
  kConstantXy,
  kSinusoidalXy,
};

/**
 * @brief 甲板水平运动参数，位置和速度均使用 Gazebo world ENU 坐标系。
 */
struct MotionParameters
{
  Scenario scenario{Scenario::kStatic};
  std::array<double, 3> initial_position_enu{0.0, 0.0, 2.0};
  std::array<double, 2> velocity_xy{0.0, 0.0};
  std::array<double, 2> amplitude_xy{0.0, 0.0};
  std::array<double, 2> period_xy{1.0, 1.0};
  double update_rate_hz{50.0};
};

/**
 * @brief 某一仿真时刻的甲板解析位置和速度。
 */
struct MotionSample
{
  std::array<double, 3> position_enu{};
  std::array<double, 3> velocity_enu{};
};

/**
 * @brief 计算静止、水平匀速和水平正弦甲板轨迹。
 *
 * 该类不依赖 ROS 或 Gazebo，便于独立验证轨迹公式。构造时会校验全部输入，
 * 避免 NaN、Inf 或非法周期进入仿真控制链路。
 */
class MotionProfile
{
public:
  /**
   * @brief 从 YAML 场景名解析运动类型。
   *
   * @param name 支持 `S0_STATIC`、`S1_CONSTANT_XY`、`S2_SINUSOIDAL_XY`。
   * @return 对应的场景枚举。
   * @throws std::invalid_argument 场景名不受支持时抛出。
   */
  static Scenario parse_scenario(const std::string & name);

  /**
   * @brief 创建并校验水平运动轨迹。
   *
   * @param parameters ENU 坐标系下的轨迹参数；周期和更新频率必须大于零。
   * @throws std::invalid_argument 任一数值非有限、周期或更新频率非法时抛出。
   */
  explicit MotionProfile(MotionParameters parameters);

  /**
   * @brief 计算从本轮场景开始计时后的解析状态。
   *
   * @param elapsed_s 自场景开始后的仿真时间，单位为秒，必须为有限非负数。
   * @return Gazebo world ENU 坐标系下的位置（米）和速度（米每秒）。
   * @throws std::invalid_argument elapsed_s 非法时抛出。
   */
  MotionSample sample(double elapsed_s) const;

private:
  MotionParameters parameters_;
};

}  // namespace moving_deck_sim
