#include "pitch_follow_policy.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool value, const char * message) {
  if (!value) throw std::runtime_error(message);
}

int main() {
  using autoaim::PitchFollowStatus;
  using autoaim::plan_follow_pitch;
  constexpr double slope = -32.55;
  const auto below = plan_follow_pitch(570, 1080, 2.3, 2.3, slope);
  require(below.status == PitchFollowStatus::Move &&
          std::abs(below.pitch_target_deg - 2.45) < 1e-9,
          "目标在下方时，应增大 pitch，且每次最多 +0.15°");
  const auto above = plan_follow_pitch(509, 1080, 2.3, 2.3, slope);
  require(above.status == PitchFollowStatus::Move &&
          std::abs(above.pitch_target_deg - 2.15) < 1e-9,
          "目标在上方时，应减小 pitch，且每次最多 -0.15°");
  const auto centered = plan_follow_pitch(539.8, 1080, 2.3, 2.3, slope);
  require(centered.status == PitchFollowStatus::Hold &&
          std::abs(centered.pitch_target_deg - 2.3) < 1e-9,
          "已在死区时应保持当前回传角度");
  const auto bounded = plan_follow_pitch(600, 1080, 3.25, 2.3, slope);
  require(bounded.status == PitchFollowStatus::Move &&
          std::abs(bounded.pitch_target_deg - 3.3) < 1e-9,
          "接近起点 ±1° 边界时应截断增量");
  require(plan_follow_pitch(600, 1080, 3.31, 2.3, slope).status ==
            PitchFollowStatus::Saturated,
          "实际角度越界后必须拒绝继续跟随");
  require(plan_follow_pitch(600, 1080, 2.3, 2.3, 0).status ==
            PitchFollowStatus::Invalid,
          "无效斜率不能产生目标角度");
  require(plan_follow_pitch(std::numeric_limits<double>::quiet_NaN(), 1080,
                            2.3, 2.3, slope).status == PitchFollowStatus::Invalid,
          "无效像素不能产生目标角度");
  std::cout << "PASS: pitch 方向、死区、单次限幅、总行程与无效输入\n";
}
