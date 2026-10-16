// 【学习导读｜连续像素策略自检】
// 检查左右方向、正负斜率、死区、单次修正、总行程与绝对角度边界及非法数值。
// near用于带浮点容差的数值比较；invalid验证无效输入不能生成可发送计划。
// 通过自检不等于控制参数已适合所有真实速度/延迟，需要现场验证。

#include "pixel_follow_policy.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const char * message) {
  require(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, message);
}

void invalid(double target_x, int width, double measured, double base, double slope) {
  const auto plan = autoaim::plan_follow_yaw(target_x, width, measured, base, slope);
  require(plan.status == autoaim::PixelFollowStatus::Invalid &&
              !plan.commandable() && !std::isfinite(plan.yaw_target_deg) &&
              !plan.reason.empty(),
          "invalid input must never produce a command");
}

}  // namespace

int main() {
  using autoaim::PixelFollowStatus;
  using autoaim::plan_follow_yaw;
  constexpr int width = 1440;
  constexpr double center = (width - 1) * 0.5;
  constexpr double slope = 32.1;  // 现场实测近似值。
  constexpr double base = -40.0;

  const auto centered = plan_follow_yaw(center, width, base, base, slope);
  require(centered.status == PixelFollowStatus::Hold && centered.commandable(),
          "centered target must hold yaw");
  near(centered.yaw_target_deg, base, "hold target must equal measured yaw");
  const auto edge_of_deadband = plan_follow_yaw(center + 3.0, width, base, base, slope);
  require(edge_of_deadband.status == PixelFollowStatus::Hold,
          "3-pixel deadband must include its edge");

  const auto right = plan_follow_yaw(center + 4.0, width, base, base, slope);
  require(right.status == PixelFollowStatus::Move && right.commandable(),
          "right-side target must produce a correction");
  near(right.yaw_target_deg, base - 4.0 / slope,
       "positive measured slope requires decreasing yaw for a right target");
  const auto left = plan_follow_yaw(center - 4.0, width, base, base, slope);
  near(left.yaw_target_deg, base + 4.0 / slope,
       "positive measured slope requires increasing yaw for a left target");
  const auto reversed = plan_follow_yaw(center + 4.0, width, base, base, -slope);
  near(reversed.yaw_target_deg, base + 4.0 / slope,
       "a negative measured slope must reverse the correction direction");

  const auto large_error = plan_follow_yaw(center + 330.0, width, base, base, slope);
  require(large_error.status == PixelFollowStatus::Move,
          "large visible error should begin a bounded correction");
  near(large_error.yaw_target_deg, base - 0.15, "one update must be capped at 0.15 deg");

  // 假设静止目标随云台转动按实测斜率移动，验证能逐步进入死区。
  double measured = base;
  const double initial_target_x = center + slope;
  bool reached_deadband = false;
  for (int step = 0; step < 20; ++step) {
    const double observed_x = initial_target_x + slope * (measured - base);
    const auto plan = plan_follow_yaw(observed_x, width, measured, base, slope);
    require(plan.commandable(), "reachable static target should not saturate");
    require(std::abs(plan.yaw_target_deg - measured) <= 0.150000001,
            "continuous update exceeded 0.15 deg");
    require(std::abs(plan.yaw_target_deg - base) <= 2.000000001,
            "continuous target exceeded 2 deg from base");
    measured = plan.yaw_target_deg;  // 假设下位机准确跟随。
    if (plan.status == PixelFollowStatus::Hold) {
      reached_deadband = true;
      break;
    }
  }
  require(reached_deadband, "reachable static target did not enter deadband");

  // 若初始偏差需要超过 2° 才能消除，允许逐步到界，但到界后必须停止。
  measured = base;
  bool saturated = false;
  for (int step = 0; step < 30; ++step) {
    const double observed_x = center + 100.0 + slope * (measured - base);
    const auto plan = plan_follow_yaw(observed_x, width, measured, base, slope);
    if (plan.status == PixelFollowStatus::Saturated) {
      require(!plan.commandable() && !std::isfinite(plan.yaw_target_deg) &&
                  !plan.reason.empty(),
              "saturation must stop commands and report why");
      saturated = true;
      break;
    }
    require(plan.status == PixelFollowStatus::Move,
            "unreachable off-center target must not be reported centered");
    require(std::abs(plan.yaw_target_deg - measured) <= 0.150000001,
            "one update exceeded 0.15 deg during saturation approach");
    require(plan.yaw_target_deg >= base - 2.000000001,
            "target crossed negative total yaw limit");
    measured = plan.yaw_target_deg;
  }
  require(saturated, "unreachable target did not report saturation");
  near(measured, base - 2.0, "saturation should happen at the 2 deg limit");

  const auto absolute_edge = plan_follow_yaw(center + 100.0, width,
                                              -169.95, -169.95, slope);
  require(absolute_edge.status == PixelFollowStatus::Move,
          "first correction near absolute limit should move to the limit");
  near(absolute_edge.yaw_target_deg, -170.0,
       "correction must stop at the conservative absolute yaw limit");
  const auto blocked_edge = plan_follow_yaw(center + 100.0, width,
                                             -170.0, -169.95, slope);
  require(blocked_edge.status == PixelFollowStatus::Saturated &&
              !blocked_edge.commandable(),
          "further movement beyond -170 deg must saturate");
  const auto positive_edge = plan_follow_yaw(center - 100.0, width,
                                              169.95, 169.95, slope);
  near(positive_edge.yaw_target_deg, 170.0,
       "correction must stop at the positive absolute yaw limit");
  const auto blocked_positive = plan_follow_yaw(center - 100.0, width,
                                                 170.0, 169.95, slope);
  require(blocked_positive.status == PixelFollowStatus::Saturated &&
              !blocked_positive.commandable(),
          "further movement beyond +170 deg must saturate");

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  invalid(nan, width, base, base, slope);
  invalid(inf, width, base, base, slope);
  invalid(-1.0, width, base, base, slope);
  invalid(static_cast<double>(width), width, base, base, slope);
  invalid(center, 1, base, base, slope);
  invalid(center, 8193, base, base, slope);
  invalid(center, width, nan, base, slope);
  invalid(center, width, base, inf, slope);
  invalid(center, width, 171.0, base, slope);
  invalid(center, width, base, -171.0, slope);
  invalid(center, width, base, base, nan);
  invalid(center, width, base, base, 0.0);
  invalid(center, width, base, base, 4.99);
  invalid(center, width, base, base, 200.01);

  const auto escaped = plan_follow_yaw(center, width, base - 2.1, base, slope);
  require(escaped.status == PixelFollowStatus::Saturated &&
              !escaped.commandable(),
          "out-of-envelope feedback must stop tracking even inside pixel deadband");

  std::cout << "pixel follow policy: direction, deadband, update/travel/absolute limits, "
               "saturation, invalid inputs OK\n";
}
