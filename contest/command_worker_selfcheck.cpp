// 【学习导读｜单步发送线程的虚拟串口自检】
// 通过伪终端模拟串口另一端，检查周期性发出的报文字段和看门狗行为。
// 先临时重命名被包含源码的main，再include现场实现，以便测试同一个CommandWorker。
// 伪终端提供字节收发，不模拟相机、电机惯性、云台PID或真实机械运动。

// Host-only PTY selfcheck of the exact FieldSerial/CommandWorker code used onsite.
// No camera, lower computer, or real /dev/ttyACM* device is opened.
#define main live_pixel_step_safe_program_main
#include "live_pixel_step_safe.cpp"
#undef main

#include <pty.h>

#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const std::string & message) {
  if (!condition) throw std::runtime_error(message);
}

std::uint16_t u16(const std::uint8_t * p) {
  return static_cast<std::uint16_t>(p[0]) |
         (static_cast<std::uint16_t>(p[1]) << 8);
}

std::uint32_t u32(const std::uint8_t * p) {
  return static_cast<std::uint32_t>(p[0]) |
         (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) |
         (static_cast<std::uint32_t>(p[3]) << 24);
}

float f32(const std::uint8_t * p) {
  const std::uint32_t bits = u32(p);
  float value = 0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

struct Command {
  double yaw;
  double pitch;
  std::int32_t fire;
  Clock::time_point received;
};

class PtyReader {
public:
  explicit PtyReader(int master) : master_(master) {}

  Command next(std::chrono::milliseconds timeout = std::chrono::milliseconds(130)) {
    const auto deadline = Clock::now() + timeout;
    while (pending_.size() < 18) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now()).count();
      require(remaining > 0, "等待虚拟串口命令超时");
      pollfd descriptor{master_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining));
      require(ready > 0 && (descriptor.revents & POLLIN),
              "虚拟串口未按期发送命令");
      std::array<std::uint8_t, 256> bytes{};
      const ssize_t count = ::read(master_, bytes.data(), bytes.size());
      require(count > 0, "读取虚拟串口失败");
      pending_.insert(pending_.end(), bytes.begin(), bytes.begin() + count);
    }
    std::array<std::uint8_t, 18> bytes{};
    std::copy_n(pending_.begin(), bytes.size(), bytes.begin());
    pending_.erase(pending_.begin(), pending_.begin() + bytes.size());
    require(u16(bytes.data()) == 16 && u16(bytes.data() + 2) == 1 &&
            u16(bytes.data() + 12) == 2, "串口命令帧头或字段 ID 错误");
    return {f32(bytes.data() + 4), f32(bytes.data() + 8),
            static_cast<std::int32_t>(u32(bytes.data() + 14)), Clock::now()};
  }

private:
  int master_;
  std::vector<std::uint8_t> pending_;
};

void expect_command(const Command & command, double yaw, double pitch) {
  require(command.fire == 0, "发送了非零 fire_flag");
  require(std::abs(command.yaw - yaw) < 0.0001, "yaw 命令值不符");
  require(std::abs(command.pitch - pitch) < 0.0001, "pitch 命令值不符");
}

}  // namespace

int main() {
  constexpr double baseline_yaw = 12.25;
  constexpr double baseline_pitch = -4.75;
  constexpr double step_yaw = baseline_yaw + 0.3;
  int master = -1;
  int slave = -1;
  try {
    char path[128]{};
    require(::openpty(&master, &slave, path, nullptr, nullptr) == 0,
            "无法创建虚拟串口 PTY");
    FieldSerial serial(path, true);
    ::close(slave);
    slave = -1;
    PtyReader reader(master);
    {
      CommandWorker worker(serial, baseline_yaw, baseline_pitch);

      std::vector<Clock::time_point> baseline_times;
      for (int i = 0; i < 6; ++i) {
        worker.beat();
        const auto command = reader.next();
        expect_command(command, baseline_yaw, baseline_pitch);
        baseline_times.push_back(command.received);
      }
      const auto baseline_span = std::chrono::duration_cast<std::chrono::milliseconds>(
        baseline_times.back() - baseline_times.front()).count();
      require(baseline_span >= 150 && baseline_span <= 350,
              "起始角度命令频率与约 20 Hz 不符");

      worker.beat();
      bool rejected_excess = false;
      try {
        worker.set_step(baseline_yaw + 0.31);
      } catch (const std::runtime_error &) {
        rejected_excess = true;
      }
      require(rejected_excess, "未拒绝超过 0.3° 的阶跃");
      worker.set_step(step_yaw);
      for (int i = 0; i < 3; ++i) {
        worker.beat();
        expect_command(reader.next(), step_yaw, baseline_pitch);
      }

      // Simulate the camera/YOLO loop hanging: stop beat(), while the independent
      // serial thread continues. It must return to baseline and latch the fault.
      std::this_thread::sleep_for(std::chrono::milliseconds(380));
      require(worker.tripped(), "主循环超时后看门狗未锁存故障");
      bool rejected_after_stall = false;
      try {
        worker.set_step(step_yaw);
      } catch (const std::runtime_error &) {
        rejected_after_stall = true;
      }
      require(rejected_after_stall, "看门狗故障后仍接受阶跃");

      int consecutive_baseline = 0;
      for (int i = 0; i < 16 && consecutive_baseline < 3; ++i) {
        const auto command = reader.next();
        require(command.fire == 0, "看门狗回退期间 fire_flag 非零");
        require(std::abs(command.pitch - baseline_pitch) < 0.0001,
                "看门狗回退期间 pitch 改变");
        if (std::abs(command.yaw - baseline_yaw) < 0.0001) {
          ++consecutive_baseline;
        } else {
          require(std::abs(command.yaw - step_yaw) < 0.0001,
                  "看门狗回退期间发送未知 yaw 命令");
          require(consecutive_baseline == 0, "回到基线后又发送阶跃命令");
        }
      }
      require(consecutive_baseline == 3, "超时后未持续发送起始角度");
    }
    ::close(master);
    std::cout << "PASS: 虚拟串口约 20 Hz 无弹基线、0.3° 限幅、超时回退与故障锁存\n";
    return 0;
  } catch (const std::exception & error) {
    if (slave >= 0) ::close(slave);
    if (master >= 0) ::close(master);
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
