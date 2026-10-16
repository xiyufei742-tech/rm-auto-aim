// 【学习导读｜像素跟随的核心公式】
// 这是“纯计算”函数：给定一组数，返回计划，不开相机、不读串口、不创建线程。
// 输入：target_x（px）、图像宽度（px）、当前 yaw（°）、本轮起始 yaw（°）、斜率（px/°）。
// 输出：能否发送、目标 yaw、像素偏差、原因。main 根据状态决定是否交给发送线程。
// 只把目标移向图像几何中心；图像中心不自动等于枪口落点，也不能推出距离。

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace autoaim {

// 仅用于静止目标的首次无弹 yaw 像素居中；这里不处理 pitch、弹道或串口。
// slope_px_per_deg 必须是这台相机和云台现场实测的 dx / d(yaw)，
// 例如 2026-10-14 手动扫描测得约 +32.35 px/deg。初步内参尚未独立测距验证，
// 像素/角度斜率本身也不能据此推算距离。
// 四种结果：Invalid=输入无效；Hold=已在死区内；Move=可小幅修正；
// Saturated=实际角度越界或已无可用修正余量。只有 Hold/Move 允许提交跟随计划。
enum class PixelFollowStatus { Invalid, Hold, Move, Saturated };

struct PixelFollowPlan {
  PixelFollowStatus status = PixelFollowStatus::Invalid;
  // 只有 status 为 Hold 或 Move 时才能使用 yaw_target_deg。
  // NaN 表示“还没有有效答案”，避免无效输入被误当成目标 0°。
  double yaw_target_deg = std::numeric_limits<double>::quiet_NaN();
  double error_px = std::numeric_limits<double>::quiet_NaN();
  std::string reason;

  bool commandable() const {
    return status == PixelFollowStatus::Hold || status == PixelFollowStatus::Move;
  }
};

inline PixelFollowPlan plan_follow_yaw(double target_x, int frame_width,
                                      double measured_yaw_deg, double base_yaw_deg,
                                      double slope_px_per_deg) {
  // 这些是首次联调的程序边界，不是机械最大行程或比赛规则。
  // 绝对 yaw 在 ±170°；相对本轮基线在 ±2°；每次策略修正 ≤0.15°；死区 ±3 px。
  constexpr double kMaxAbsYawDeg = 170.0;
  constexpr double kMaxFromBaseDeg = 2.0;
  constexpr double kMaxUpdateDeg = 0.15;
  constexpr double kDeadbandPx = 3.0;

  PixelFollowPlan plan;
  if (frame_width < 2 || frame_width > 8192 || !std::isfinite(target_x) ||
      target_x < 0.0 || target_x >= frame_width ||
      !std::isfinite(measured_yaw_deg) || !std::isfinite(base_yaw_deg) ||
      !std::isfinite(slope_px_per_deg)) {
    plan.reason = "图像、角度或斜率不是有效数值";
    return plan;
  }
  if (std::abs(slope_px_per_deg) < 5.0 || std::abs(slope_px_per_deg) > 200.0) {
    plan.reason = "像素与 yaw 的实测斜率超出首次测试范围";
    return plan;
  }
  if (std::abs(measured_yaw_deg) > kMaxAbsYawDeg ||
      std::abs(base_yaw_deg) > kMaxAbsYawDeg) {
    plan.reason = "回传角度或起始角度超出保守绝对边界";
    return plan;
  }

  // 像素编号从 0 开始，所以 1440 宽图像的几何中心是 (1440-1)/2=719.5。
  // error_px>0 说明目标在右侧；<0 在左侧。这里还没有换算成角度。
  plan.error_px = target_x - (frame_width - 1) * 0.5;
  // 取两个允许区间的交集：既不能离起点太远，也不能越过绝对角度边界。
  const double min_yaw = std::max(-kMaxAbsYawDeg, base_yaw_deg - kMaxFromBaseDeg);
  const double max_yaw = std::min(kMaxAbsYawDeg, base_yaw_deg + kMaxFromBaseDeg);
  if (measured_yaw_deg < min_yaw || measured_yaw_deg > max_yaw) {
    plan.status = PixelFollowStatus::Saturated;
    plan.reason = "实际 yaw 已离起始角度超过 2°；停止追踪";
    return plan;
  }
  // 死区让小幅检测抖动不再反复引起修正。Hold 返回当前实测角度，
  // 不会证明目标已达到射击精度；它只是像素控制层的停止条件。
  if (std::abs(plan.error_px) <= kDeadbandPx) {
    plan.status = PixelFollowStatus::Hold;
    plan.yaw_target_deg = measured_yaw_deg;
    plan.reason = "目标在 3 像素死区内，保持当前 yaw";
    return plan;
  }

  // 现场实测斜率决定符号：目标在右且斜率为正时，应减小 yaw。
  // 由局部近似 Δx ≈ slope·Δyaw，令 e+Δx=0，可得 Δyaw=-e/slope。
  // 例：e=+53.379 px、slope=+32.35 px/°，完整修正约 -1.650°。
  // 这一轮先被下一行限为 -0.15°；后续要根据新画面重新计算，不能直接开大步。
  const double full_correction = -plan.error_px / slope_px_per_deg;
  const double one_update = std::clamp(full_correction,
                                       -kMaxUpdateDeg, kMaxUpdateDeg);
  const double proposed = measured_yaw_deg + one_update;
  // clamp 把值夹在上下限间。在限制边缘，实际允许的这一小步可能不足 0.15°。
  // 发送线程还会把本计划转换成每次最多变化 0.05° 的指令。
  const double bounded = std::clamp(proposed, min_yaw, max_yaw);
  if (std::abs(bounded - measured_yaw_deg) < 1e-9) {
    plan.status = PixelFollowStatus::Saturated;
    plan.reason = "目标仍偏离中心，但 yaw 已到达起始角度 ±2° 或绝对 ±170° 边界";
    return plan;
  }

  plan.status = PixelFollowStatus::Move;
  plan.yaw_target_deg = bounded;
  plan.reason = bounded == proposed ? "向图像中心小幅修正 yaw" :
                                      "本次修正受总行程或绝对角度边界限制";
  return plan;
}

}  // namespace autoaim
