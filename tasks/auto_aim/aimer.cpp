// 【学习导读｜后续参考：未来瞄准点与弹道】
// 输入是已经估计运动状态的 Target、图像时刻和弹速；输出 io::Command 的角度(rad)。
// 先补处理/发射延迟，再补弹丸飞行时间，最后把未来瞄准点换成 yaw/pitch。
// 目标位置影响飞行时间，飞行时间又影响目标位置，因此代码迭代求解。
// 本文件已编译并通过离线边界检查，仍未接入现场；不能将其算作实机瞄准成果。

#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr double max_prediction_seconds = std::numeric_limits<int>::max() / 1e6;

bool valid_prediction_time(double seconds)
{
  return std::isfinite(seconds) && seconds >= 0 && seconds < max_prediction_seconds;
}

bool advance_time(Clock::time_point base, double seconds, Clock::time_point & result)
{
  if (!valid_prediction_time(seconds)) return false;
  const auto step = std::chrono::duration_cast<Clock::duration>(
    std::chrono::microseconds(static_cast<int>(seconds * 1e6)));
  if (base.time_since_epoch() > Clock::duration::max() - step) return false;
  result = base + step;
  return true;
}
}  // namespace

Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  auto finite_config = [&](const char * key) {
    const auto value = yaml[key].as<double>();
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string("Aimer config '") + key + "' must be finite");
    }
    return value;
  };
  yaw_offset_ = finite_config("yaw_offset") / 57.3;      // degree to rad
  pitch_offset_ = finite_config("pitch_offset") / 57.3;  // degree to rad
  const auto coming_degrees = finite_config("comming_angle");
  const auto leaving_degrees = finite_config("leaving_angle");
  if (coming_degrees <= 0 || coming_degrees > 180 || leaving_degrees < 0 ||
      leaving_degrees > coming_degrees) {
    throw std::invalid_argument(
      "Aimer config requires 0 < comming_angle <= 180 and 0 <= leaving_angle <= comming_angle");
  }
  comming_angle_ = coming_degrees / 57.3;
  leaving_angle_ = leaving_degrees / 57.3;
  high_speed_delay_time_ = finite_config("high_speed_delay_time");
  low_speed_delay_time_ = finite_config("low_speed_delay_time");
  decision_speed_ = finite_config("decision_speed");
  if (high_speed_delay_time_ < 0 || low_speed_delay_time_ < 0 || decision_speed_ < 0) {
    throw std::invalid_argument("Aimer config delay times and decision_speed must be nonnegative");
  }
  // 即使只写了一侧偏置也检查其有限性；是否启用仍沿用两侧均配置才启用的约定。
  if (yaml["left_yaw_offset"].IsDefined()) finite_config("left_yaw_offset");
  if (yaml["right_yaw_offset"].IsDefined()) finite_config("right_yaw_offset");
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  // debug 仅在本次完整计算成功时发布，失败不能沿用上一帧的有效瞄准点。
  debug_aim_point = {};
  if (targets.empty() || !std::isfinite(bullet_speed) || bullet_speed <= 0) {
    return {false, false, 0, 0};
  }
  auto target = targets.front();
  const auto state = target.ekf_x();
  if (state.size() != 11 || !state.allFinite()) return {false, false, 0, 0};
  double delay_time =
    std::abs(state[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
  if (!valid_prediction_time(delay_time)) return {false, false, 0, 0};
  auto valid_trajectory = [&](const tools::Trajectory & trajectory) {
    return !trajectory.unsolvable && std::isfinite(trajectory.pitch) &&
           trajectory.fly_time > 0 && valid_prediction_time(trajectory.fly_time);
  };

  // 先分别转为浮点 epoch 再相减，避免 time_point::min/max 的整数时长先溢出。
  // 回放时间戳需映射到当前 steady_clock。约35.8分钟仅是旧 int 微秒表示范围，
  // 不是现场数据时效许可；控制层的250/300ms等新鲜度门槛仍独立生效。
  const long double age =
    std::chrono::duration<long double>(Clock::now().time_since_epoch()).count() -
    std::chrono::duration<long double>(timestamp.time_since_epoch()).count();
  if (!std::isfinite(age) || std::abs(age) >= max_prediction_seconds) {
    return {false, false, 0, 0};
  }
  // 考虑 detector/tracker 耗时，固定时序回放仍沿用原来的5ms处理耗时。
  const auto dt = (to_now ? static_cast<double>(age) : 0.005) + delay_time;
  Clock::time_point future;
  if (!advance_time(timestamp, dt, future)) return {false, false, 0, 0};
  target.predict(future);

  // 先从预测目标各块板中选择瞄准点；无合适点则返回 control=false。
  // 空目标、无效瞄准点、弹道不可解都不能继续正常生成角度。
  auto aim_point0 = choose_aim_point(target);
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::hypot(xyz0[0], xyz0[1]);
  if (!std::isfinite(d0) || d0 <= 0) return {false, false, 0, 0};
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (!valid_trajectory(trajectory0)) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  auto final_aim_point = aim_point0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  // 每次用上一轮飞行时间预测未来板位置，再重新算飞行时间。
  // 这是固定点迭代的思路，不是“EKF 再跑十次测量更新”。
  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    Clock::time_point predict_time;
    if (!advance_time(future, prev_fly_time, predict_time)) return {false, false, 0, 0};
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::hypot(xyz.x(), xyz.y());
    if (!std::isfinite(d) || d <= 0) return {false, false, 0, 0};
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (!valid_trajectory(current_traj)) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      return {false, false, 0, 0};
    }
    final_aim_point = aim_point;

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }
  if (!converged) return {false, false, 0, 0};

  // 计算最终角度
  Eigen::Vector3d final_xyz = final_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) return {false, false, 0, 0};
  debug_aim_point = final_aim_point;
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  if (!command.control) return command;
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;
  if (!std::isfinite(command.yaw)) {
    debug_aim_point = {};
    return {false, false, 0, 0};
  }

  return command;
}

// 根据板的朝向、旋转方向与可见区间选点，并通过 lock_id_ 减少两块板之间来回切换。
// 这些区间是原目标几何与策略的假设，需与比赛靶对应。
AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  if (ekf_x.size() != 11 || !ekf_x.allFinite()) return {};
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  if (armor_num == 0) return {};
  for (const auto & xyza : armor_xyza_list) {
    if (!xyza.allFinite()) return {};
  }
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  // 状态 [7] 是角速度 rad/s；[8] 是半径 m，不能用于判断是否快速旋转。
  if (std::abs(ekf_x[7]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim
