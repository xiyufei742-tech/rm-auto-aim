// 无设备的数值回归：确认模型解能代回抛物线，以及非法输入不会冒充可解。
// 独立编译示例：g++ -std=c++17 -O2 -I. contest/trajectory_selfcheck.cpp
//                tools/trajectory.cpp -o /tmp/trajectory_selfcheck
#include "tools/trajectory.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
constexpr double g = 9.7833;
int checks = 0;

void require(bool condition, const std::string & message)
{
  ++checks;
  if (!condition) throw std::runtime_error(message);
}

void expect_failure(double speed, double distance, double height, const char * label)
{
  const tools::Trajectory result(speed, distance, height);
  require(result.unsolvable, std::string(label) + ": 应明确返回不可解");
  require(result.pitch == 0.0 && result.fly_time == 0.0,
          std::string(label) + ": 失败结果必须确定初始化");
}

void expect_solution(double speed, double distance, double height, const char * label)
{
  const tools::Trajectory result(speed, distance, height);
  require(!result.unsolvable, std::string(label) + ": 应可解");
  require(std::isfinite(result.pitch) && std::isfinite(result.fly_time) && result.fly_time > 0,
          std::string(label) + ": 解必须有限且时间为正");

  // 用返回的 double 角度与时间正向积分，而不是把求根公式再抄一遍来验证。
  const long double time = result.fly_time;
  const auto x = speed * std::cos(static_cast<long double>(result.pitch)) * time;
  const auto vertical_motion = speed * std::sin(static_cast<long double>(result.pitch)) * time;
  const auto drop = 0.5L * g * time * time;
  const auto z = vertical_motion - drop;
  require(std::abs(x - distance) <= 1e-10L * distance,
          std::string(label) + ": 水平位移回代残差过大");
  const auto height_scale = std::abs(static_cast<long double>(height)) + std::abs(vertical_motion) + drop;
  require(std::abs(z - height) <= 1e-10L * height_scale + 1e-300L,
          std::string(label) + ": 竖直位移回代残差过大");

  // 对普通范围独立计算较慢的高角度解，检查本解确实没有选到长飞行时间。
  if (distance > 1e-6 && distance < 100 && std::abs(height) < 50) {
    const long double v2 = static_cast<long double>(speed) * speed;
    const auto discriminant = v2 * v2 - 2 * g * height * v2 - g * g * distance * distance;
    if (discriminant >= 0) {
      const auto high_tan = (v2 + std::sqrt(discriminant)) / (g * distance);
      const auto high_time = distance / speed * std::hypot(1.0L, high_tan);
      require(time <= high_time * (1 + 1e-12L), std::string(label) + ": 应选择短飞行时间解");
    }
  }
}
}  // namespace

int main()
{
  try {
    expect_solution(25, 5, 0, "同高 5m");
    expect_solution(25, 5, 1.2, "目标较高");
    expect_solution(25, 5, -1.2, "目标较低");
    expect_solution(25, 5, 1e-12, "近水平高度");
    expect_solution(25, 5, -g * 25 / (2 * 625), "接近水平出射角");
    expect_solution(25, 1e-10, 0, "极小水平距离");
    expect_solution(25, 1e-7, 1e-6, "小距离正高度");
    expect_solution(25, 1e-7, -1e-6, "小距离负高度");
    expect_solution(0.1, 0.0001, 0, "低速短距离");
    expect_solution(g, g, 0, "恰好临界射程");
    expect_solution(25, (625 / g) * (1 - 1e-9), 0, "临界射程内侧");
    expect_failure(25, (625 / g) * (1 + 1e-9), 0, "超过临界射程");
    expect_failure(25, 1, 40, "超过最大高度");
    expect_failure(0, 5, 0, "零速度");
    expect_failure(-25, 5, 0, "负速度");
    expect_failure(25, 0, 1, "不支持垂直射击");
    expect_failure(25, -5, 0, "负水平距离");
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const auto inf = std::numeric_limits<double>::infinity();
    expect_failure(nan, 5, 0, "NaN速度");
    expect_failure(25, nan, 0, "NaN距离");
    expect_failure(25, 5, nan, "NaN高度");
    expect_failure(inf, 5, 0, "无限速度");
    expect_failure(25, inf, 0, "无限距离");
    expect_failure(25, 5, inf, "正无限高度");
    expect_failure(25, 5, -inf, "负无限高度");
    expect_failure(25, std::numeric_limits<double>::denorm_min(), 0, "时间无法用double表示");
    std::cout << "PASS: trajectory " << checks
              << " checks; finite inputs, physical feasibility, short-time root, forward residuals\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
