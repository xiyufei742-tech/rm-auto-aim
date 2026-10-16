#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
// 有限测试接缝：只检查给定目标几何的选板结果；其他断言均走公开 aim()。
struct AimerTestAccess
{
  static AimPoint choose(Aimer & aimer, const Target & target)
  {
    return aimer.choose_aim_point(target);
  }
};
}  // namespace auto_aim

namespace
{
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

class TemporaryConfigs
{
public:
  TemporaryConfigs()
  {
    auto pattern = (std::filesystem::temp_directory_path() / "srm_aimer_selfcheck_XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    const auto created = ::mkdtemp(name.data());
    if (!created) throw std::runtime_error("cannot create temporary Aimer config directory");
    directory_ = created;
  }

  ~TemporaryConfigs()
  {
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
  }

  std::string write(const std::string & name, double high_delay, double low_delay)
  {
    auto path = directory_ / name;
    std::ofstream stream(path);
    stream << "yaw_offset: 0\npitch_offset: 0\ncomming_angle: 70\nleaving_angle: 10\n"
              "decision_speed: 3\nleft_yaw_offset: 1\nright_yaw_offset: -1\n"
           << "high_speed_delay_time: " << high_delay << '\n'
           << "low_speed_delay_time: " << low_delay << '\n';
    stream.close();
    if (!stream) throw std::runtime_error("cannot write temporary Aimer configuration");
    return path.string();
  }

  std::string write_changed(const std::string & key, const std::string & value)
  {
    auto path = write("invalid.yaml", 0.12, 0.0);
    auto yaml = YAML::LoadFile(path);
    yaml[key] = YAML::Load(value);
    std::ofstream stream(path);
    stream << yaml;
    stream.close();
    if (!stream) throw std::runtime_error("cannot write changed Aimer configuration");
    return path;
  }

private:
  std::filesystem::path directory_;
};

// 用公开预测接口完成一整圈后恢复零相位，消除构造与取时之间的微秒差异。
// 此处只检查预测均值；转一圈带来的协方差变化不会影响本测试的瞄准几何。
std::pair<auto_aim::Target, Clock::time_point> rotating_target(double speed, double radius = 0.2)
{
  auto_aim::Target target(5.0, speed, radius, 0.0);
  auto timestamp = Clock::now();
  target.predict(timestamp);
  if (speed != 0) {
    const auto full_turn = std::copysign(2 * CV_PI, speed);
    target.predict((full_turn - target.ekf_x()[6]) / speed);
  }
  require(std::abs(target.ekf_x()[6]) < 1e-10, "fixture should start at zero phase");
  return {target, timestamp};
}

void require_valid(const io::Command & command, const auto_aim::Aimer & aimer)
{
  require(command.control && !command.shoot, "valid aim must enable control and keep shoot false");
  require(std::isfinite(command.yaw) && std::isfinite(command.pitch), "valid angles must be finite");
  require(aimer.debug_aim_point.valid && aimer.debug_aim_point.xyza.allFinite(),
          "successful aim must publish a finite debug point");
}

void require_invalid(const io::Command & command, const auto_aim::Aimer & aimer)
{
  require(!command.control && !command.shoot && command.yaw == 0 && command.pitch == 0,
          "invalid aim must return a disabled zero command");
  require(!aimer.debug_aim_point.valid && aimer.debug_aim_point.xyza.isZero(0),
          "failed aim must clear the prior debug point");
}
}  // namespace

int main()
{
  try {
    TemporaryConfigs configs;
    auto_aim::Aimer aimer(configs.write("delays.yaml", 0.12, 0.0));
    require(!aimer.debug_aim_point.valid && aimer.debug_aim_point.xyza.isZero(0),
            "new Aimer debug state must be initialized");
    auto [stationary, stationary_time] = rotating_target(0);
    const auto fast = aimer.aim({stationary}, stationary_time, 23.0, false);
    require_valid(fast, aimer);
    const auto slow = aimer.aim({stationary}, stationary_time, 13.0, false);
    require_valid(slow, aimer);
    require(std::abs(slow.pitch) > std::abs(fast.pitch) + 0.05,
            "explicit 13 m/s must not be silently replaced with 23 m/s");

    for (const double speed : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()}) {
      require_valid(aimer.aim({stationary}, stationary_time, 23.0, false), aimer);
      require_invalid(aimer.aim({stationary}, stationary_time, speed, false), aimer);
    }
    require_valid(aimer.aim({stationary}, stationary_time, 23.0, false), aimer);
    require_invalid(aimer.aim({}, stationary_time, 23.0, false), aimer);
    require_valid(aimer.aim({stationary}, stationary_time, 23.0, false), aimer);
    require_invalid(aimer.aim({}, stationary_time, 23.0, io::left_shoot, false), aimer);
    require_invalid(aimer.aim({stationary}, stationary_time, 0.1, false), aimer);

    // 两种时序路径均先拒绝极端时间戳，不能先对整数 duration 做溢出加减。
    for (const bool to_now : {false, true}) {
      for (const auto extreme_time : {Clock::time_point::min(), Clock::time_point::max()}) {
        require_valid(aimer.aim({stationary}, stationary_time, 23.0, false), aimer);
        require_invalid(aimer.aim({stationary}, extreme_time, 23.0, to_now), aimer);
      }
    }
    for (const auto & [key, value] : std::vector<std::pair<std::string, std::string>>{
           {"decision_speed", ".nan"}, {"comming_angle", ".nan"},
           {"high_speed_delay_time", "-1"}, {"leaving_angle", "90"},
           {"left_yaw_offset", ".inf"}}) {
      bool rejected = false;
      try {
        auto_aim::Aimer invalid_config(configs.write_changed(key, value));
      } catch (const std::invalid_argument & error) {
        rejected = std::string(error.what()).find("Aimer config") != std::string::npos;
      }
      require(rejected, "invalid configuration must be clearly rejected: " + key);
    }

    auto [positive, positive_time] = rotating_target(4.0);
    auto [negative, negative_time] = rotating_target(-4.0);
    const auto positive_command = aimer.aim({positive}, positive_time, 23.0, false);
    require_valid(positive_command, aimer);
    const auto negative_command = aimer.aim({negative}, negative_time, 23.0, false);
    require_valid(negative_command, aimer);
    require(positive_command.yaw < 0 && negative_command.yaw > 0 &&
              std::abs(positive_command.yaw + negative_command.yaw) < 1e-8 &&
              std::abs(positive_command.pitch - negative_command.pitch) < 1e-8,
            "opposite rotation directions should use the same speed-dependent delay");

    auto_aim::Aimer geometry(configs.write("geometry.yaml", 0, 0));
    for (const double direction : {-1.0, 1.0}) {
      // 当前板位于正面一侧约23度；高速时另一块位于迎面侧约67度，满足70度窗口。
      auto_aim::Target fast_target(5.0, direction * 3, 0.2, 0);
      fast_target.predict(0.4 / 3);
      fast_target.jumped = true;
      const auto incoming = auto_aim::AimerTestAccess::choose(geometry, fast_target);
      require(incoming.valid && incoming.xyza[3] * direction < -1.0,
              "fast rotation must select the approaching-side armor, independent of radius");
      auto_aim::Target slow_target(5.0, direction, 0.2, 0);
      slow_target.predict(0.4);
      slow_target.jumped = true;
      const auto current = auto_aim::AimerTestAccess::choose(geometry, slow_target);
      require(current.valid && current.xyza[3] * direction > 0.3,
              "slow rotation should retain the only armor within the 60 degree window");
    }

    // 高角速度数值样例让飞行时间迭代振荡；它不代表已验证的实机速度范围。
    auto [oscillating, oscillating_time] = rotating_target(80.0, 0.4);
    require_valid(geometry.aim({stationary}, stationary_time, 23.0, false), geometry);
    require_invalid(geometry.aim({oscillating}, oscillating_time, 23.0, false), geometry);

    std::cout << "PASS: Aimer finite commands, explicit bullet speed, stale debug reset, "
                 "bidirectional delay, angular-speed armor selection, nonconvergence rejection, "
                 "invalid config and extreme timestamps\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
