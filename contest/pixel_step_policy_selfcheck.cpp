// 【学习导读｜单步像素策略自检】
// 将已知像素偏差、角度和斜率送入纯计算函数，检查方向、限幅及无效输入处理。
// 没有相机/云台；它验证公式与边界行为，不测真实装甲板识别质量。

#include "pixel_step_policy.hpp"
#include "protocol.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const char * message) {
  require(std::isfinite(actual) && std::abs(actual - expected) < 5e-6, message);
}

void rejected(double x, int width, double yaw, double pitch, double slope,
              const char * message) {
  const auto plan = autoaim::plan_pixel_step(x, width, yaw, pitch, slope);
  require(!plan.valid, message);
  require(!plan.reason.empty(), "invalid input needs a diagnostic reason");
}

float frame_float(const std::array<std::uint8_t, 18> & frame, std::size_t offset) {
  std::uint32_t bits = 0;
  for (int i = 0; i < 4; ++i) bits |= static_cast<std::uint32_t>(frame[offset + i]) << (8 * i);
  float value = 0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

int main() {
  constexpr int width = 1440;
  constexpr double center = (width - 1) * 0.5;
  constexpr double slope = 32.1;  // 现场静止目标的粗测值，像素/度。
  constexpr double yaw = -38.0;
  constexpr double pitch = 2.3;

  const auto centered = autoaim::plan_pixel_step(center, width, yaw, pitch, slope);
  require(centered.valid, "center target should be valid");
  near(centered.error_px, 0.0, "center error should be zero");
  near(centered.yaw_target_deg, yaw, "center should not change yaw");
  near(centered.pitch_target_deg, pitch, "pixel step must hold pitch");

  const auto right = autoaim::plan_pixel_step(center + 8.0, width, yaw, pitch, slope);
  require(right.valid, "right target should be valid");
  near(right.error_px, 8.0, "right error sign is wrong");
  near(right.yaw_target_deg, yaw - 8.0 / slope,
       "right target must command decreasing yaw");
  near(right.pitch_target_deg, pitch, "right target must hold pitch");

  const auto left = autoaim::plan_pixel_step(center - 8.0, width, yaw, pitch, slope);
  require(left.valid, "left target should be valid");
  near(left.error_px, -8.0, "left error sign is wrong");
  near(left.yaw_target_deg, yaw + 8.0 / slope,
       "left target must command increasing yaw");

  const auto far_right = autoaim::plan_pixel_step(center + 200.0, width, yaw, pitch, slope);
  require(far_right.valid, "far right target should be valid");
  near(far_right.yaw_target_deg, yaw - 0.3, "single yaw step must be capped at -0.3 deg");
  const auto far_left = autoaim::plan_pixel_step(center - 200.0, width, yaw, pitch, slope);
  require(far_left.valid, "far left target should be valid");
  near(far_left.yaw_target_deg, yaw + 0.3, "single yaw step must be capped at +0.3 deg");

  const auto edge_left = autoaim::plan_pixel_step(0.0, width, yaw, pitch, slope);
  const auto edge_right = autoaim::plan_pixel_step(width - 1.0, width, yaw, pitch, slope);
  require(edge_left.valid && edge_right.valid, "image edge coordinates should be valid");
  near(edge_left.yaw_target_deg, yaw + 0.3, "left image edge must be capped");
  near(edge_right.yaw_target_deg, yaw - 0.3, "right image edge must be capped");

  const auto nonfinite = std::numeric_limits<double>::quiet_NaN();
  const auto infinity = std::numeric_limits<double>::infinity();
  rejected(nonfinite, width, yaw, pitch, slope, "NaN target coordinate must be rejected");
  rejected(infinity, width, yaw, pitch, slope, "infinite target coordinate must be rejected");
  rejected(center, width, nonfinite, pitch, slope, "NaN yaw feedback must be rejected");
  rejected(center, width, yaw, infinity, slope, "infinite pitch feedback must be rejected");
  rejected(center, width, yaw, pitch, 0.0, "zero slope must be rejected");
  rejected(center, width, yaw, pitch, nonfinite, "NaN slope must be rejected");
  rejected(center, 0, yaw, pitch, slope, "zero image width must be rejected");
  rejected(-1.0, width, yaw, pitch, slope, "coordinate before image must be rejected");
  rejected(width * 1.0, width, yaw, pitch, slope, "coordinate after image must be rejected");
  rejected(center - 200.0, width, 170.0, pitch, slope,
           "yaw step beyond the conservative boundary must be rejected");
  rejected(center + 200.0, width, -170.0, pitch, slope,
           "negative yaw step beyond the conservative boundary must be rejected");

  // The policy output is passed through the official command encoder only with fire_flag=0.
  const auto command = autoaim::encodeCommand(static_cast<float>(right.yaw_target_deg),
                                               static_cast<float>(right.pitch_target_deg), 0);
  require(command.size() == 18, "command must be 18 bytes");
  require(command[0] == 16 && command[1] == 0 && command[2] == 1 && command[3] == 0 &&
              command[12] == 2 && command[13] == 0,
          "command length and record IDs must match official protocol");
  require(command[14] == 0 && command[15] == 0 && command[16] == 0 && command[17] == 0,
          "fire_flag must be zero");
  near(frame_float(command, 4), right.yaw_target_deg,
       "serialized yaw must match the planned target");
  near(frame_float(command, 8), right.pitch_target_deg,
       "serialized pitch must match the measured pitch");

  std::cout << "pixel step policy: sign, 0.3 deg cap, bounds, nonfinite inputs, no-fire frame OK\n";
}
