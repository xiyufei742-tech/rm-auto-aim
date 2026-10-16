#pragma once

#include "pitch_follow_policy.hpp"
#include "pixel_follow_policy.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace autoaim {

// 静止靶双轴联调：两轴使用各自现场实测的像素/角度斜率。
// 只计算绝对角度目标；不判断发射，也不处理弹道、距离或运动预测。
struct XYFollowPlan {
  PixelFollowPlan yaw;
  PitchFollowPlan pitch;
  double yaw_target_deg = NAN;
  double pitch_target_deg = NAN;
  std::string reason;

  bool commandable() const { return yaw.commandable() && pitch.commandable(); }
};

inline XYFollowPlan plan_follow_xy(double x, double y, int width, int height,
                                   double yaw_deg, double pitch_deg,
                                   double base_yaw_deg, double base_pitch_deg,
                                   double dx_dyaw, double dy_dpitch) {
  constexpr double kMaxYawTravelDeg = 1.0;
  constexpr double kMaxPitchTravelDeg = 1.0;
  XYFollowPlan plan;
  plan.yaw = plan_follow_yaw(x, width, yaw_deg, base_yaw_deg, dx_dyaw);
  plan.pitch = plan_follow_pitch(y, height, pitch_deg, base_pitch_deg, dy_dpitch);
  if (std::isfinite(yaw_deg) && std::isfinite(base_yaw_deg) &&
      std::isfinite(pitch_deg) && std::isfinite(base_pitch_deg) &&
      (std::abs(yaw_deg - base_yaw_deg) > kMaxYawTravelDeg ||
       std::abs(pitch_deg - base_pitch_deg) > kMaxPitchTravelDeg)) {
    plan.yaw.status = PixelFollowStatus::Saturated;
    plan.reason = "实际双轴角度离起点超过 ±1°";
    return plan;
  }
  if (!plan.commandable()) {
    plan.reason = !plan.yaw.commandable() ? plan.yaw.reason : plan.pitch.reason;
    return plan;
  }
  plan.yaw_target_deg = std::clamp(plan.yaw.yaw_target_deg,
                                   base_yaw_deg - kMaxYawTravelDeg,
                                   base_yaw_deg + kMaxYawTravelDeg);
  plan.pitch_target_deg = std::clamp(plan.pitch.pitch_target_deg,
                                     base_pitch_deg - kMaxPitchTravelDeg,
                                     base_pitch_deg + kMaxPitchTravelDeg);
  if ((std::abs(plan.yaw.error_px) > 3.0 &&
       std::abs(plan.yaw_target_deg - yaw_deg) < 1e-9) ||
      (std::abs(plan.pitch.error_px) > 3.0 &&
       std::abs(plan.pitch_target_deg - pitch_deg) < 1e-9)) {
    plan.yaw.status = PixelFollowStatus::Saturated;
    plan.reason = "仍偏离中心，但双轴测试的起点 ±1° 行程已用尽";
    return plan;
  }
  plan.reason = "双轴朝图像中心小幅修正";
  return plan;
}

}  // namespace autoaim
