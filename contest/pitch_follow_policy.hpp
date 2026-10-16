#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace autoaim {

enum class PitchFollowStatus { Invalid, Hold, Move, Saturated };

struct PitchFollowPlan {
  PitchFollowStatus status = PitchFollowStatus::Invalid;
  double pitch_target_deg = std::numeric_limits<double>::quiet_NaN();
  double error_px = std::numeric_limits<double>::quiet_NaN();
  std::string reason;

  bool commandable() const {
    return status == PitchFollowStatus::Hold || status == PitchFollowStatus::Move;
  }
};

// 固定靶首次无弹 pitch 控制：只处理图像纵坐标，绝不决定开火。
// 现场只读扫描初步得到 dy/dpitch≈−32.55 px/°；需保持相机设置和安装不变。
inline PitchFollowPlan plan_follow_pitch(double target_y, int frame_height,
                                        double measured_pitch_deg, double base_pitch_deg,
                                        double slope_px_per_deg) {
  constexpr double kMaxAbsPitchDeg = 50.0;
  constexpr double kMaxFromBaseDeg = 1.0;
  constexpr double kMaxUpdateDeg = 0.15;
  constexpr double kDeadbandPx = 3.0;

  PitchFollowPlan plan;
  if (frame_height < 2 || frame_height > 8192 || !std::isfinite(target_y) ||
      target_y < 0 || target_y >= frame_height ||
      !std::isfinite(measured_pitch_deg) || !std::isfinite(base_pitch_deg) ||
      !std::isfinite(slope_px_per_deg) ||
      std::abs(slope_px_per_deg) < 5 || std::abs(slope_px_per_deg) > 200) {
    plan.reason = "图像、角度或实测斜率无效";
    return plan;
  }
  if (std::abs(measured_pitch_deg) > kMaxAbsPitchDeg ||
      std::abs(base_pitch_deg) > kMaxAbsPitchDeg) {
    plan.reason = "pitch 超出保守绝对角度边界";
    return plan;
  }
  plan.error_px = target_y - (frame_height - 1) * 0.5;
  const double minimum = std::max(-kMaxAbsPitchDeg, base_pitch_deg - kMaxFromBaseDeg);
  const double maximum = std::min(kMaxAbsPitchDeg, base_pitch_deg + kMaxFromBaseDeg);
  if (measured_pitch_deg < minimum || measured_pitch_deg > maximum) {
    plan.status = PitchFollowStatus::Saturated;
    plan.reason = "实际 pitch 离起始角度超过 1°";
    return plan;
  }
  if (std::abs(plan.error_px) <= kDeadbandPx) {
    plan.status = PitchFollowStatus::Hold;
    plan.pitch_target_deg = measured_pitch_deg;
    plan.reason = "目标在纵向 3 像素死区内，保持当前 pitch";
    return plan;
  }
  // Δy≈slope·Δpitch。要让下一次纵向误差靠近 0，令 Δpitch≈−error/slope。
  const double correction = std::clamp(-plan.error_px / slope_px_per_deg,
                                       -kMaxUpdateDeg, kMaxUpdateDeg);
  const double proposed = measured_pitch_deg + correction;
  const double bounded = std::clamp(proposed, minimum, maximum);
  if (std::abs(bounded - measured_pitch_deg) < 1e-9) {
    plan.status = PitchFollowStatus::Saturated;
    plan.reason = "目标仍偏离中心，但 pitch 已到起点 ±1° 或绝对 ±50° 边界";
    return plan;
  }
  plan.status = PitchFollowStatus::Move;
  plan.pitch_target_deg = bounded;
  plan.reason = bounded == proposed ? "向图像中心小幅修正 pitch" :
                                      "本次修正受总行程或绝对角度边界限制";
  return plan;
}

}  // namespace autoaim
