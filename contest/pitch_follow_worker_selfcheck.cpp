// 【学习导读｜连续发送线程的虚拟串口自检】
// FeedbackEmitter模拟下位机回传，PtyReader读取上位机指令，断言限幅/无弹/回位等行为。
// 复用实际FollowWorker，覆盖正常返回与视觉心跳超时后的锁存处理。
// 这里的模拟回传不是高保真云台动力学仿真；实机仍可能有超调、延迟和机械限制。
// 测试中出现预期的“看门狗中止”文字后总体PASS，可以是故意触发故障的成功用例。

// Host-only PTY selfcheck of the actual FollowWorker used for field testing.
// It never opens a camera, a lower computer, or a real /dev/ttyACM* device.
#define main live_pixel_follow_safe_program_main
#include "live_pixel_pitch_follow_safe.cpp"
#undef main

#include <pty.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {

void require(bool ok, const std::string & message) {
  if (!ok) throw std::runtime_error(message);
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

void put_u16(std::array<std::uint8_t, 30> & bytes, std::size_t offset,
             std::uint16_t value) {
  bytes[offset] = static_cast<std::uint8_t>(value);
  bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

void put_u32(std::array<std::uint8_t, 30> & bytes, std::size_t offset,
             std::uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    bytes[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

void put_f32(std::array<std::uint8_t, 30> & bytes, std::size_t offset, float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  put_u32(bytes, offset, bits);
}

std::array<std::uint8_t, 30> feedback_frame(float yaw, float pitch, int mode) {
  std::array<std::uint8_t, 30> bytes{};
  put_u16(bytes, 0, 28);
  put_u16(bytes, 2, 1);
  put_f32(bytes, 4, yaw);
  put_f32(bytes, 8, pitch);
  put_f32(bytes, 12, 0);
  put_u32(bytes, 16, static_cast<std::uint32_t>(mode));
  put_u32(bytes, 20, 3);
  put_u16(bytes, 24, 2);
  put_f32(bytes, 26, 0);
  return bytes;
}

// The master emulates lower-computer feedback while the slave is opened by
// FieldSerial. In a PTY, writes on the master arrive at the slave, and commands
// written to the slave are read from the master.
class FeedbackEmitter {
public:
  FeedbackEmitter(int master, float yaw, float pitch)
      : master_(master), yaw_(yaw), pitch_(pitch), thread_([this] { run(); }) {}
  FeedbackEmitter(const FeedbackEmitter &) = delete;
  FeedbackEmitter & operator=(const FeedbackEmitter &) = delete;
  ~FeedbackEmitter() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }
  void set_mode(int mode) { mode_.store(mode); }

private:
  void run() {
    while (!stop_.load()) {
      const auto bytes = feedback_frame(yaw_, pitch_, mode_.load());
      std::size_t written = 0;
      while (written < bytes.size() && !stop_.load()) {
        const ssize_t count = ::write(master_, bytes.data() + written,
                                      bytes.size() - written);
        if (count > 0) written += static_cast<std::size_t>(count);
        else if (count < 0 && errno == EINTR) continue;
        else break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  int master_;
  float yaw_;
  float pitch_;
  std::atomic<int> mode_{0};
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

struct Command {
  double yaw = 0;
  double pitch = 0;
  std::int32_t fire = -1;
  Clock::time_point received;
};

class PtyReader {
public:
  explicit PtyReader(int master) : master_(master) {}

  Command next(std::chrono::milliseconds timeout = std::chrono::milliseconds(180)) {
    const auto deadline = Clock::now() + timeout;
    while (pending_.size() < 18) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now()).count();
      require(remaining > 0, "虚拟串口等待命令超时");
      pollfd descriptor{master_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining));
      require(ready > 0 && (descriptor.revents & POLLIN), "虚拟串口未按期发送命令");
      std::array<std::uint8_t, 256> chunk{};
      const ssize_t count = ::read(master_, chunk.data(), chunk.size());
      require(count > 0, "读取虚拟串口命令失败");
      pending_.insert(pending_.end(), chunk.begin(), chunk.begin() + count);
    }
    std::array<std::uint8_t, 18> bytes{};
    std::copy_n(pending_.begin(), bytes.size(), bytes.begin());
    pending_.erase(pending_.begin(), pending_.begin() + bytes.size());
    require(u16(bytes.data()) == 16 && u16(bytes.data() + 2) == 1 &&
            u16(bytes.data() + 12) == 2, "命令帧字段错误");
    return {f32(bytes.data() + 4), f32(bytes.data() + 8),
            static_cast<std::int32_t>(u32(bytes.data() + 14)), Clock::now()};
  }

private:
  int master_;
  std::vector<std::uint8_t> pending_;
};

void check_command(const Command & current, const Command & previous,
                   double baseline_yaw, double baseline_pitch) {
  require(current.fire == 0, "发送了非零 fire_flag");
  require(std::abs(current.yaw - baseline_yaw) < 0.0001, "yaw 命令发生变化");
  require(std::abs(current.pitch - baseline_pitch) <= 1.0001, "pitch 命令超出起点 ±1°");
  require(std::abs(current.pitch - previous.pitch) <= 0.0501,
          "相邻 50 ms 命令变化超过 0.05°");
}

void await_fresh(FieldSerial & serial) {
  const auto deadline = Clock::now() + std::chrono::milliseconds(500);
  while (!serial.latest() && Clock::now() < deadline) {
    serial.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  require(serial.latest().has_value(), "虚拟下位机未提供初始回传");
}

void test_normal_return() {
  constexpr double baseline_yaw = 12.25;
  constexpr double baseline_pitch = -4.75;
  int master = -1;
  int slave = -1;
  char path[128]{};
  require(::openpty(&master, &slave, path, nullptr, nullptr) == 0, "创建 PTY 失败");
  try {
    {
      FieldSerial serial(path, true);
      ::close(slave);
      slave = -1;
      FeedbackEmitter emitter(master, baseline_yaw, baseline_pitch);
      PtyReader reader(master);
      await_fresh(serial);
      FollowWorker worker(serial, baseline_yaw, baseline_pitch);
      Command previous = reader.next();
      require(previous.fire == 0 && std::abs(previous.pitch - baseline_pitch) < 0.0001,
              "初始指令不是无弹基线");
      std::vector<Clock::time_point> baseline_times{previous.received};
      for (int i = 0; i < 5; ++i) {
        worker.beat();
        const Command current = reader.next();
        check_command(current, previous, baseline_yaw, baseline_pitch);
        baseline_times.push_back(current.received);
        previous = current;
      }
      const auto span = std::chrono::duration_cast<std::chrono::milliseconds>(
        baseline_times.back() - baseline_times.front()).count();
      require(span >= 160 && span <= 450, "起始命令频率偏离约 20 Hz");

      emitter.set_mode(1);
      worker.start_follow();
      bool rejected_bound = false;
      try { worker.set_target(baseline_pitch + 1.01); }
      catch (const std::runtime_error &) { rejected_bound = true; }
      require(rejected_bound, "未拒绝超过起点 ±1° 的请求");
      worker.set_target(baseline_pitch + 0.8);
      for (int i = 0; i < 16; ++i) {
        worker.beat();
        const Command current = reader.next();
        check_command(current, previous, baseline_yaw, baseline_pitch);
        previous = current;
      }
      require(previous.pitch > baseline_pitch + 0.4,
              "正常跟随期间 pitch 未按限速朝目标移动");
      require(!worker.tripped(), "正常跟随意外触发故障");

      // A lost target is handled by request_return(): commands keep their
      // 0.05° rate limit until the baseline is reached, then remain there.
      worker.request_return();
      bool rejected_during_return = false;
      try { worker.set_target(baseline_pitch + 0.5); }
      catch (const std::runtime_error &) { rejected_during_return = true; }
      require(rejected_during_return, "回退期间仍接受跟随目标");
      bool saw_baseline = false;
      int baseline_count = 0;
      for (int i = 0; i < 45 && baseline_count < 3; ++i) {
        worker.beat();
        const Command current = reader.next();
        check_command(current, previous, baseline_yaw, baseline_pitch);
        previous = current;
        if (std::abs(current.pitch - baseline_pitch) < 0.0001) {
          saw_baseline = true;
          ++baseline_count;
        } else {
          require(!saw_baseline, "回到基线后再次偏离");
          baseline_count = 0;
        }
      }
      require(baseline_count == 3 && worker.baseline_reached(),
              "目标丢失后的平滑回退未到起点");
    }
    ::close(master);
  } catch (...) {
    if (slave >= 0) ::close(slave);
    if (master >= 0) ::close(master);
    throw;
  }
}

void test_watchdog_return() {
  constexpr double baseline_yaw = -21.5;
  constexpr double baseline_pitch = 3.25;
  int master = -1;
  int slave = -1;
  char path[128]{};
  require(::openpty(&master, &slave, path, nullptr, nullptr) == 0, "创建 PTY 失败");
  try {
    {
      FieldSerial serial(path, true);
      ::close(slave);
      slave = -1;
      FeedbackEmitter emitter(master, baseline_yaw, baseline_pitch);
      PtyReader reader(master);
      await_fresh(serial);
      FollowWorker worker(serial, baseline_yaw, baseline_pitch);
      Command previous = reader.next();
      emitter.set_mode(1);
      worker.start_follow();
      worker.set_target(baseline_pitch - 0.8);
      for (int i = 0; i < 18; ++i) {
        worker.beat();
        const Command current = reader.next();
        check_command(current, previous, baseline_yaw, baseline_pitch);
        previous = current;
      }
      require(previous.pitch < baseline_pitch - 0.4, "看门狗试验前未离开起点");

      // Emulate camera.read()/YOLO hanging while serial feedback continues.
      std::this_thread::sleep_for(std::chrono::milliseconds(380));
      require(worker.tripped(), "主循环卡住超过 300 ms 后没有锁存看门狗故障");
      bool rejected_after_fault = false;
      try { worker.set_target(baseline_pitch - 0.5); }
      catch (const std::runtime_error &) { rejected_after_fault = true; }
      require(rejected_after_fault, "看门狗故障后仍接受新目标");

      int baseline_count = 0;
      bool saw_baseline = false;
      for (int i = 0; i < 50 && baseline_count < 3; ++i) {
        const Command current = reader.next();
        check_command(current, previous, baseline_yaw, baseline_pitch);
        previous = current;
        if (std::abs(current.pitch - baseline_pitch) < 0.0001) {
          saw_baseline = true;
          ++baseline_count;
        } else {
          require(!saw_baseline, "看门狗回正后重新偏离起点");
          baseline_count = 0;
        }
      }
      require(baseline_count == 3 && worker.baseline_reached(),
              "看门狗未按 0.05° 每周期平滑回到起点");
    }
    ::close(master);
  } catch (...) {
    if (slave >= 0) ::close(slave);
    if (master >= 0) ::close(master);
    throw;
  }
}

}  // namespace

int main() {
  try {
    test_normal_return();
    test_watchdog_return();
    std::cout << "PASS: 虚拟串口约 20 Hz、无弹、每周期 ±0.05°、"
                 "总范围 ±1°、目标丢失回正、看门狗锁存与平滑回正\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
