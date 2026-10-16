#include "xy_follow_policy.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool okay, const char * message) {
  if (!okay) throw std::runtime_error(message);
}

int main() {
  using autoaim::plan_follow_xy;
  constexpr double sx = 32.35;
  constexpr double sy = -32.55;
  const auto right_below = plan_follow_xy(730, 550, 1440, 1080,
                                         40, 3, 40, 3, sx, sy);
  require(right_below.commandable() &&
          std::abs(right_below.yaw_target_deg - 39.85) < 1e-9 &&
          std::abs(right_below.pitch_target_deg - 3.15) < 1e-9,
          "右下目标应减 yaw、增 pitch，且每轴单次最多 0.15°");
  const auto left_above = plan_follow_xy(709, 529, 1440, 1080,
                                        40, 3, 40, 3, sx, sy);
  require(left_above.commandable() &&
          left_above.yaw_target_deg > 40 && left_above.pitch_target_deg < 3,
          "左上目标应增 yaw、减 pitch");
  const auto centered = plan_follow_xy(719.5, 539.5, 1440, 1080,
                                       40, 3, 40, 3, sx, sy);
  require(centered.commandable() &&
          std::abs(centered.yaw_target_deg - 40) < 1e-9 &&
          std::abs(centered.pitch_target_deg - 3) < 1e-9,
          "双轴均在死区应保持实际角度");
  const auto at_bound = plan_follow_xy(730, 550, 1440, 1080,
                                       39, 4, 40, 3, sx, sy);
  require(!at_bound.commandable(), "行程边界仍要求继续向外移动时必须中止");
  const auto past_bound = plan_follow_xy(719.5, 539.5, 1440, 1080,
                                         41.01, 3, 40, 3, sx, sy);
  require(!past_bound.commandable(), "实际角度越过 ±1° 不可继续控制");
  const auto invalid = plan_follow_xy(std::numeric_limits<double>::quiet_NaN(),
                                      539.5, 1440, 1080, 40, 3, 40, 3, sx, sy);
  require(!invalid.commandable(), "无效像素不得发出双轴目标");
  std::cout << "PASS: 双轴方向、死区、单次限幅、总行程与无效输入\n";
}
