// 【学习导读｜单次像素小步】
// 用于先验证“看见右侧目标时，给出的 yaw 符号是否正确”。
// 算法与连续版共用 Δyaw=-像素误差/实测斜率 的思想，但单次限为 ±0.3°。
// pitch_target_deg 原样复制当前 pitch。函数只返回计划，实际发送见 live_pixel_step_safe.cpp。
// 10 月 11 日现场用此思路让目标横坐标向中心移动约 7～8 px；它不负责全程居中。

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace autoaim {

// 只为首次无弹实机联调计算一次小幅 yaw 修正。正负号由现场实测的
// target_x / feedback_yaw 斜率给出；不依赖未标定的镜头内参。
struct PixelStepPlan {
  bool valid = false;
  double yaw_target_deg = 0.0;
  double pitch_target_deg = 0.0;
  double error_px = 0.0;
  std::string reason;
};

inline PixelStepPlan plan_pixel_step(double target_x, int frame_width,
                                     double measured_yaw_deg,
                                     double measured_pitch_deg,
                                     double slope_px_per_deg) {
  PixelStepPlan plan;
  if (frame_width <= 0 || !std::isfinite(target_x) ||
      target_x < 0 || target_x >= frame_width ||
      !std::isfinite(measured_yaw_deg) || !std::isfinite(measured_pitch_deg) ||
      !std::isfinite(slope_px_per_deg)) {
    plan.reason = "图像或角度数值无效";
    return plan;
  }
  if (std::abs(slope_px_per_deg) < 5 || std::abs(slope_px_per_deg) > 200) {
    plan.reason = "像素与 yaw 的实测斜率超出首次测试范围";
    return plan;
  }
  if (std::abs(measured_yaw_deg) > 170 || std::abs(measured_pitch_deg) > 55) {
    plan.reason = "云台靠近首次测试的保守角度边界";
    return plan;
  }

  // 先算目标相对图像几何中心的位置，单位是 px。
  // 后面的除法将 px 除以 px/°，结果才是角度增量 °。
  plan.error_px = target_x - (frame_width - 1) * 0.5;
  // 最多只给出 0.3° 的单次修正，远小于已验证过的 0.5° 手动阶跃。
  const double delta_yaw = std::clamp(-plan.error_px / slope_px_per_deg, -0.3, 0.3);
  // 下位机接收绝对角度，因此要把增量加到当前回传角度上。
  plan.yaw_target_deg = measured_yaw_deg + delta_yaw;
  plan.pitch_target_deg = measured_pitch_deg;
  if (std::abs(plan.yaw_target_deg) > 170) {
    plan.reason = "修正后的 yaw 超出首次测试的保守角度边界";
    return plan;
  }
  plan.valid = true;
  plan.reason = "ok";
  return plan;
}

}  // namespace autoaim
