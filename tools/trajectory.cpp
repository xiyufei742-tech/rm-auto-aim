// 【学习导读｜后续参考：简化抛体模型】
// 输入 v0 初速(m/s)、d 水平距离(m)、h 目标相对高度(m)；输出 pitch(rad)、fly_time(s)。
// 假设只有重力、没有空气阻力；g=9.7833 m/s² 为代码采用值。
// 由 h=d·tanθ-g·d²/(2v0² cos²θ)，令 u=tanθ，得 a·u²+b·u+c=0。
// 此文件不决定开火，不计算目标运动，也未用于当前像素跟随。

#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  if (!std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(h) || v0 <= 0 || d <= 0)
    return;

  // 中间量用 long double，避免先算 v^4、d^2 时 double 溢出/下溢。
  // 仍使用项目原来的 g=9.7833 与无空气阻力模型，不引入实测补偿参数。
  const long double speed = v0;
  const long double distance = d;
  const long double height = h;
  const long double gravity = g;
  const auto speed2 = speed * speed;
  const auto range = std::hypot(distance, height);

  // 判别式 D=v^4-2*g*h*v^2-g^2*d^2。
  // 写成 [v^2-g*(h+r)]*[v^2+g*(r-h)]，r=hypot(d,h)，
  // 使临界射程附近的符号判断不必相减两个四次幂。
  // h 为负时 h+r 可能接近 0，使用 (r+h)(r-h)=d^2 有理化。
  const auto range_plus_height =
    height >= 0 ? range + height : distance * distance / (range - height);
  const auto range_minus_height =
    height <= 0 ? range - height : distance * distance / (range + height);
  const auto reach_margin = speed2 - gravity * range_plus_height;
  const auto other_factor = speed2 + gravity * range_minus_height;
  if (!std::isfinite(reach_margin) || !std::isfinite(other_factor) || reach_margin < 0)
    return;
  const auto sqrt_delta = std::sqrt(reach_margin) * std::sqrt(other_factor);

  // 低根原式是 tan(pitch)=(v^2-sqrt(D))/(g*d)，近水平时存在严重消减误差。
  // 乘以共轭后使用下面的等价式；两个根之和为正，低根的绝对值不大于高根，
  // 因此在 d>0、v>0 的前向抛物线中，它也对应较短飞行时间。
  const auto tan_pitch =
    (gravity * distance * distance + 2 * height * speed2) /
    (distance * (speed2 + sqrt_delta));
  const auto solved_pitch = std::atan(tan_pitch);
  const auto solved_time = (distance / speed) * std::hypot(1.0L, tan_pitch);
  const auto pitch_double = static_cast<double>(solved_pitch);
  const auto time_double = static_cast<double>(solved_time);
  if (!std::isfinite(tan_pitch) || !std::isfinite(pitch_double) ||
      !std::isfinite(time_double) || time_double <= 0 ||
      std::abs(pitch_double) >= std::acos(-1.0) / 2)
    return;

  pitch = pitch_double;
  fly_time = time_double;
  unsolvable = false;
}

}  // namespace tools
