// 无设备的角度归约回归；CTest 应设置短超时，以捕捉旧实现的死循环。
#include "tools/math_tools.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
const double pi = std::acos(-1.0);
int checks = 0;

void require(bool condition, const std::string & message)
{
  ++checks;
  if (!condition) throw std::runtime_error(message);
}

// 只给正常的小角度用旧公式作行为对照，绝不能把巨大值传入此参考循环。
double ordinary_reference(double angle)
{
  while (angle > pi) angle -= 2 * pi;
  while (angle <= -pi) angle += 2 * pi;
  return angle;
}

void check_ordinary(double angle)
{
  const auto actual = tools::limit_rad(angle);
  require(std::isfinite(actual) && actual > -pi && actual <= pi, "普通输入未归约到(-pi,pi]");
  require(std::abs(actual - ordinary_reference(angle)) < 1e-12, "普通输入与原边界约定不一致");
}
}  // namespace

int main()
{
  try {
    for (const auto angle : {0.0, -0.0, pi, -pi, 3 * pi, -3 * pi}) check_ordinary(angle);
    require(tools::limit_rad(-pi) == pi, "-pi 必须映射成 +pi");
    require(tools::limit_rad(3 * pi) == pi, "+3pi 必须映射成 +pi");
    require(tools::limit_rad(-3 * pi) == pi, "-3pi 必须映射成 +pi");
    for (const auto boundary : {pi, -pi, 3 * pi, -3 * pi}) {
      check_ordinary(std::nextafter(boundary, -std::numeric_limits<double>::infinity()));
      check_ordinary(std::nextafter(boundary, std::numeric_limits<double>::infinity()));
    }
    // 普通多周角度与旧逻辑近等价，避免只验证零值和特殊端点。
    for (int index = -400; index <= 400; ++index) check_ordinary(index * 0.137);

    for (const auto large : {1e20, -1e20, std::numeric_limits<double>::max(),
                             -std::numeric_limits<double>::max()}) {
      const auto wrapped = tools::limit_rad(large);
      require(std::isfinite(wrapped) && wrapped > -pi && wrapped <= pi,
              "巨大有限输入应及时返回范围内角度");
    }
    for (const auto invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
      require(std::isnan(tools::limit_rad(invalid)), "非有限输入应返回 NaN 供上游拒绝");
    }
    std::cout << "PASS: angle wrap " << checks
              << " checks; (-pi,pi] boundaries, ordinary parity, huge finite and nonfinite inputs\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
