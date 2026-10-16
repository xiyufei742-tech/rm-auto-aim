// 【学习导读｜后续参考：原项目射击许可判断】
// 此类返回 bool，表示策略是否同意射击；不直接写串口，也不直接驱动机构。
// 当前无弹测试不调用它，所有现场测试报文 fire_flag=0。
// 读 shoot：先检查开关/有效目标，再按距离选择 yaw 容差，最后检查指令与回传。

#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  // 任何一项不满足都返回 false：命令不允许控制、无目标、配置未开启 auto_fire。
  // 这些条件是原策略的一部分，不足以独立构成完整实机发射条件。
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  // 检查新旧 yaw 目标变化是否小、实际 yaw 是否接近上一条目标、瞄准点是否有效。
  // 代码没有对应的 pitch 误差判断，别把它解读为“双轴都瞄准才开火”。
  // std::abs 这里也是普通差值，跨角度边界的处理需另行核查。
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim