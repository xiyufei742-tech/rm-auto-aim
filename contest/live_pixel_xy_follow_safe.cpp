// 【学习导读｜独立的双轴联调程序】
// 作用：把相机识别的像素中心变成短时、受限、无弹的 yaw/pitch 调整。
// 阅读顺序：先看 main() 状态机 → xy_follow_policy.hpp → FollowWorker → FieldSerial。
// 数据流：Camera::read → YOLO::detect → select_armor → plan_follow_xy
//         → FollowWorker::set_target → FieldSerial::send → 下位机角度回传。
// 当前只用像素误差，不调用 Solver/PnP、Tracker/EKF、Aimer 或 Shooter。
// 本文件角度单位为度；两轴方向均已在现场单轴实验中确认。
// 默认只读；只有显式 --actuate 才写串口。所有写出的 fire_flag 都是 0。
// 这是一段静止靶联调程序，不等于旋转靶预测或自动射击。

#include "protocol.hpp"
#include "xy_follow_policy.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"

namespace {

// 【时间】steady_clock 适合计算经过了多久，不受系统校时影响。
// 后面的超时、图像年龄、回传年龄都在这台上位机的时钟上计算。
using Clock = std::chrono::steady_clock;

// 【启动参数】结构体把命令行选项放在一起。
// 两个斜率分别是现场测得的 dx/dyaw 与 dy/dpitch。
// display 只决定是否画窗口，actuate 才决定是否允许控制云台。
struct Options {
  std::string config_path;
  std::string port;
  int max_frames = 1800;
  bool display = false;
  bool actuate = false;
  double slope_yaw = 0;
  double slope_pitch = 0;
};

void usage(const char * executable) {
  std::cout << "用法：" << executable
            << " --config configs/live_yolo_1011.yaml --port /dev/ttyACM0"
               " --slope-yaw 32.35 --slope-pitch -32.55"
               " [--frames 1800] [--display] [--actuate]\n"
               "默认只读试算，不发送指令。显式 --actuate 才进行最多 8 秒、"
               "两轴发送目标各自距起点不超过 1° 的无弹静止靶测试。\n";
}

// argc 是参数数量，argv 是参数字符串数组；argv[0] 是程序自身路径。
// 两个斜率都由现场扫描测得，不能填成镜头标称焦距。
// 遇到未知参数、缺值、无效数字会抛异常，由 main 最后的 catch 打印原因。
Options parse_args(int argc, char ** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      std::exit(0);
    }
    if (arg == "--display") {
      options.display = true;
      continue;
    }
    if (arg == "--actuate") {
      options.actuate = true;
      continue;
    }
    if (i + 1 >= argc) throw std::runtime_error("缺少参数值：" + arg);
    const std::string value(argv[++i]);
    if (arg == "--config") options.config_path = value;
    else if (arg == "--port") options.port = value;
    else if (arg == "--slope-yaw" || arg == "--slope-pitch") {
      std::size_t consumed = 0;
      const double slope = std::stod(value, &consumed);
      if (consumed != value.size()) throw std::runtime_error(arg + " 含非数字字符");
      if (arg == "--slope-yaw") options.slope_yaw = slope;
      else options.slope_pitch = slope;
    }
    else if (arg == "--frames") {
      std::size_t consumed = 0;
      options.max_frames = std::stoi(value, &consumed);
      if (consumed != value.size() || options.max_frames < 1 || options.max_frames > 10000) {
        throw std::runtime_error("--frames 必须在 1 到 10000 之间");
      }
    } else {
      throw std::runtime_error("未知参数：" + arg);
    }
  }
  if (options.config_path.empty()) throw std::runtime_error("请指定 --config");
  if (options.port.empty()) throw std::runtime_error("请指定 --port");
  if (!std::isfinite(options.slope_yaw) || !std::isfinite(options.slope_pitch) ||
      options.slope_yaw < 5 || options.slope_yaw > 200 ||
      options.slope_pitch > -5 || options.slope_pitch < -200) {
    throw std::runtime_error("请指定实测 --slope-yaw（正）和 --slope-pitch（负），绝对值 5～200 px/°");
  }
  return options;
}

// 【回传及接收时间】value 是解码后的角度等字段，received 是本机收到时刻。
// received 不是下位机测量时刻；age_ms=0 不能证明相机与 IMU 已精确同步。
struct TimedFeedback {
  autoaim::Feedback value;
  Clock::time_point received;
};

// 从任意字节位置开始读时，搜索固定长度和 ID1 前缀，再交由正式协议解析器
// 检查完整 30 字节帧。协议本身无 CRC，仍需检查数值合理性。
// 【串口适配层】把 Linux 文件描述符 fd_ 包装成读取、解析、发送三个操作。
// 协议打包算法在 protocol.cpp；本类另做实际设备读写和起始字节搜索。
// 主线程和发送线程都会读取回传，因此用 mutex 保护共享缓存。
class FieldSerial {
public:
  explicit FieldSerial(const std::string & port, bool writable) : writable_(writable) {
    // O_RDONLY 为只读；O_RDWR 为可读写；O_NONBLOCK 使“暂无数据”不阻塞线程。
    // O_NOCTTY 避免把设备当作控制终端；O_CLOEXEC 防止子进程继承此描述符。
    // 这里通过权限限制只读模式，不只是打印一句“只读”。
    fd_ = ::open(port.c_str(), (writable ? O_RDWR : O_RDONLY) |
                                     O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error("无法打开串口 " + port + ": " + std::strerror(errno));
    if (::tcgetattr(fd_, &saved_) != 0) {
      const std::string message = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("无法获取串口配置：" + message);
    }
    have_saved_ = true;
    termios raw = saved_;
    // raw 模式让二进制字节原样通过，避免终端把换行、回显等规则施加到数据上。
    // 这段代码没有设置一个新的波特率；USB CDC 的实际传输行为由设备实现决定。
    ::cfmakeraw(&raw);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &raw) != 0) {
      const std::string message = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("无法设置串口原始读取模式：" + message);
    }
  }

  // 禁止复制串口对象：两个对象共同持有同一个 fd，析构时可能重复关闭。
  // 下面的析构函数恢复原串口设置并关闭 fd，这是“资源随对象自动释放”。
  FieldSerial(const FieldSerial &) = delete;
  FieldSerial & operator=(const FieldSerial &) = delete;

  ~FieldSerial() {
    if (fd_ >= 0) {
      if (have_saved_) ::tcsetattr(fd_, TCSANOW, &saved_);
      ::close(fd_);
    }
  }

  // 尽量取走此刻已有的数据，最多尝试 32 次，避免无限读取拖住调用者。
  // 一次 read 可能读到半帧、整帧或多帧，所以先放入 buffer_ 再解析。
  // EAGAIN 表示暂时无数据，EINTR 表示被信号打断，两者不等同于设备损坏。
  void poll() {
    std::lock_guard<std::mutex> lock(read_mutex_);
    std::array<std::uint8_t, 4096> chunk{};
    for (int attempts = 0; attempts < 32; ++attempts) {
      const ssize_t count = ::read(fd_, chunk.data(), chunk.size());
      if (count > 0) {
        buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + count);
        parse();
        continue;
      }
      if (count == 0) return;
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) return;
      throw std::runtime_error("读取串口失败：" + std::string(std::strerror(errno)));
    }
  }

  // optional 表示“可能还没有回传”，调用者要先判断是否有值。
  // 返回副本，避免锁释放后调用者继续访问正在被另一线程改写的数据。
  std::optional<TimedFeedback> latest() const {
    std::lock_guard<std::mutex> lock(read_mutex_);
    return latest_;
  }

  // 发的是绝对目标角度，不是“再转多少度”。例如 -52.8° 是目标朝向。
  // 最后一个打包参数固定为 0，代表本测试不请求开火。
  // write 可能只写出部分字节，因此循环补齐整帧，并设 100 ms 写入截止时间。
  void send(double yaw_deg, double pitch_deg) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (!writable_) throw std::runtime_error("只读模式禁止写入串口");
    if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg) ||
        std::abs(yaw_deg) > 170 || std::abs(pitch_deg) > 55) {
      throw std::runtime_error("拒绝发送超出保守边界的云台角度");
    }
    const auto bytes = autoaim::encodeCommand(static_cast<float>(yaw_deg),
                                               static_cast<float>(pitch_deg), 0);
    std::size_t sent = 0;
    const auto deadline = Clock::now() + std::chrono::milliseconds(100);
    while (sent < bytes.size()) {
      const auto count = ::write(fd_, bytes.data() + sent, bytes.size() - sent);
      if (count > 0) {
        sent += static_cast<std::size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) continue;
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        throw std::runtime_error("串口发送失败：" + std::string(std::strerror(errno)));
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now()).count();
      if (remaining <= 0) throw std::runtime_error("串口发送超时");
      pollfd descriptor{fd_, POLLOUT, 0};
      if (::poll(&descriptor, 1, static_cast<int>(remaining)) <= 0) {
        throw std::runtime_error("串口等待可写超时");
      }
    }
  }

private:
  // 合理性筛选只能排除明显异常值，不能代替 CRC。
  // 官方报文没有 CRC；碰巧满足这些范围的损坏数据仍可能通过，不能宣称完全校验。
  static bool plausible(const autoaim::Feedback & feedback) {
    return std::isfinite(feedback.yaw_deg) && std::isfinite(feedback.pitch_deg) &&
           std::isfinite(feedback.roll_deg) && std::isfinite(feedback.bullet_speed_mps) &&
           std::abs(feedback.yaw_deg) <= 720 && std::abs(feedback.pitch_deg) <= 180 &&
           std::abs(feedback.roll_deg) <= 180 && feedback.mode >= 0 && feedback.mode <= 10 &&
           feedback.robot_id >= 0 && feedback.robot_id <= 50 &&
           feedback.bullet_speed_mps >= 0 && feedback.bullet_speed_mps <= 100;
  }

  // 【找帧边界】0x1c 00 是小端的 28，后面的 01 00 是记录 ID1。
  // 它们是固定布局特征，不是协议独立设计的魔数；数据体里也可能碰巧出现。
  // 因此还检查第 24/25 字节的 ID2，并交给协议解析器和数值范围检查。
  void parse() {
    constexpr std::array<std::uint8_t, 4> prefix{{0x1c, 0x00, 0x01, 0x00}};
    while (buffer_.size() >= prefix.size()) {
      const auto start = std::search(buffer_.begin(), buffer_.end(), prefix.begin(), prefix.end());
      if (start == buffer_.end()) {
        // 若没找到完整前缀，仍保留末尾 3 字节：前缀可能跨越两次 read。
        // 例如这次末尾是 1c 00 01，下次开头是 00，拼接后才能识别。
        buffer_.erase(buffer_.begin(), buffer_.end() - 3);
        return;
      }
      buffer_.erase(buffer_.begin(), start);
      if (buffer_.size() < 30) return;
      if (buffer_[24] != 0x02 || buffer_[25] != 0x00) {
        buffer_.erase(buffer_.begin());
        continue;
      }
      try {
        const auto frames = parser_.feed(buffer_.data(), 30);
        if (frames.size() == 1 && plausible(frames.front())) {
          latest_ = TimedFeedback{frames.front(), Clock::now()};
          buffer_.erase(buffer_.begin(), buffer_.begin() + 30);
          continue;
        }
      } catch (const std::exception &) {
        // 起始位置也可能是数据体里的碰巧相同的字节；继续向后找下一帧。
      }
      parser_.clear();
      buffer_.erase(buffer_.begin());
    }
  }

  int fd_ = -1;
  bool writable_ = false;
  mutable std::mutex read_mutex_;
  std::mutex write_mutex_;
  termios saved_{};
  bool have_saved_ = false;
  std::vector<std::uint8_t> buffer_;
  autoaim::FeedbackParser parser_;
  std::optional<TimedFeedback> latest_;
};

  // 【选择目标】只接受蓝色、达到 YAML 最低分数、四角完整且中心在图像内的候选。
// 函数虽会选出较高置信度/更靠近中心的候选，但 main 还要求 candidates==1。
// 多个候选不会直接用于这个首次实机测试。返回指针只在当前 armors 存活期间有效。
const auto_aim::Armor * select_armor(const std::list<auto_aim::Armor> & armors,
                                      const cv::Size & size, double min_confidence,
                                      int & candidates) {
  const auto_aim::Armor * selected = nullptr;
  double best_distance = 0.0;
  candidates = 0;
  const cv::Point2f center((size.width - 1) * 0.5f, (size.height - 1) * 0.5f);
  for (const auto & armor : armors) {
    if (armor.color != auto_aim::Color::blue || armor.confidence < min_confidence ||
        armor.points.size() != 4 || !std::isfinite(armor.center.x) ||
        !std::isfinite(armor.center.y) || armor.center.x < 0 || armor.center.y < 0 ||
        armor.center.x >= size.width || armor.center.y >= size.height) {
      continue;
    }
    ++candidates;
    const double distance = cv::norm(armor.center - center);
    if (!selected || armor.confidence > selected->confidence + 0.001 ||
        (std::abs(armor.confidence - selected->confidence) <= 0.001 &&
         distance < best_distance)) {
      selected = &armor;
      best_distance = distance;
    }
  }
  return selected;
}

// 相机 read() 或 YOLO 可能卡住。独立的 20 Hz 线程负责读取模式回传、
// 限速发送及 300 ms 看门狗。它从不发送开火。
// 【独立发送线程】相机或神经网络可能等待很久，发指令不能完全依赖视觉循环。
// 主线程提交两轴目标；线程根据模式、回传、新鲜度检查发送。
// 请求目标、实际发出的目标和下位机回传角度必须分开看。
class FollowWorker {
public:
  FollowWorker(FieldSerial & serial, double baseline_yaw, double baseline_pitch)
      : serial_(serial), baseline_yaw_(baseline_yaw),
        baseline_pitch_(baseline_pitch), requested_yaw_(baseline_yaw),
        requested_pitch_(baseline_pitch), commanded_yaw_(baseline_yaw),
        commanded_pitch_(baseline_pitch) {
    beat();
    thread_ = std::thread([this] { run(); });
  }

  FollowWorker(const FollowWorker &) = delete;
  FollowWorker & operator=(const FollowWorker &) = delete;

  ~FollowWorker() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }

  // “喂看门狗”：记录视觉循环最近一次正常处理到末尾的时间。
  // 它不传输角度，仅说明视觉主线程还在推进。
  void beat() {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
    heartbeat_ns_.store(ns);
  }

  void start_follow() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_.load() || phase_ != Phase::AwaitVisual) {
      throw std::runtime_error("发送线程未处于可跟随状态");
    }
    phase_ = Phase::Following;
  }

  // 主线程调用此函数更新目标；锁保证发送线程不会读到并发修改的数据。
  // 只能在 Following 阶段更新，出现锁存故障后不允许重新开始跟随。
  void set_target(double yaw, double pitch) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_.load() || phase_ != Phase::Following) {
      throw std::runtime_error("发送线程已停止跟随");
    }
    if (!std::isfinite(yaw) || !std::isfinite(pitch) ||
        std::abs(yaw - baseline_yaw_) > 1.000001 ||
        std::abs(pitch - baseline_pitch_) > 1.000001 ||
        std::abs(yaw) > 170 || std::abs(pitch) > 50) {
      throw std::runtime_error("双轴目标超出起点 ±1° 或绝对角度边界");
    }
    requested_yaw_ = yaw;
    requested_pitch_ = pitch;
  }

  void request_return() {
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = Phase::Returning;
    requested_yaw_ = baseline_yaw_;
    requested_pitch_ = baseline_pitch_;
  }

  bool tripped() const { return fault_.load(); }

  std::string fault_reason() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_reason_;
  }

  // 这里只判断发送出去的目标已回到基线附近，不证明电机实际回位。
  // main 还要检查实际回传与基线的误差小于 0.2°。
  bool baseline_reached() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::abs(commanded_yaw_ - baseline_yaw_) < 0.051 &&
           std::abs(commanded_pitch_ - baseline_pitch_) < 0.051;
  }

  bool manual_seen() const { return manual_seen_.load(); }

  bool io_failed() const { return io_failed_.load(); }

private:
  enum class Phase { AwaitVisual, Following, Returning };

  // latch 是“故障锁存”：出现一次故障就保持故障状态。
  // 目标即使下一帧重新出现，也不会自动恢复本轮跟随；请求退回起始角度。
  void latch(const std::string & reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fault_.exchange(true)) {
      fault_reason_ = reason;
      std::cerr << "跟随中止：" << reason
                << "。若仍在视觉档，请切回手动档。\n";
    }
    phase_ = Phase::Returning;
    requested_yaw_ = baseline_yaw_;
    requested_pitch_ = baseline_pitch_;
  }

  // 线程循环：读回传 → 判断时效/模式/心跳 → 限速形成指令 → 发送 → 等待。
  // 末尾 sleep 50 ms，所以“20 Hz”是名义上限附近的速率，不是硬实时保证。
  void run() {
    while (!stop_.load()) {
      try {
        serial_.poll();
      } catch (const std::exception & error) {
        latch(std::string("读取串口失败：") + error.what());
        io_failed_.store(true);
      }
      const auto feedback = serial_.latest();
      const auto now = Clock::now();
      const bool fresh = feedback && feedback->received <= now &&
                         now - feedback->received <= std::chrono::milliseconds(250);
      if (!fresh) latch("下位机回传超过 250 ms 未更新");

      if (fresh && !io_failed_.load()) {
        if (std::abs(feedback->value.yaw_deg - baseline_yaw_) > 1.2 ||
            std::abs(feedback->value.pitch_deg - baseline_pitch_) > 1.4) {
          latch("云台回传偏离起点超过安全范围");
        }
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch()).count();
        // 300000000 纳秒 = 300 毫秒。主线程卡住时，本线程仍可停止跟随。
        // 另一个 250 ms 条件检查下位机回传；它与视觉心跳检查不同。
        const bool stalled = now_ns - heartbeat_ns_.load() > 300000000LL;
        if (stalled) {
          bool needs_latch = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            needs_latch = phase_ != Phase::Returning;
          }
          if (needs_latch) latch("相机/识别循环超过 300 ms 未更新");
        }

        const int mode = feedback->value.mode;
        Phase phase;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          phase = phase_;
        }
        if (mode == 0 && phase == Phase::Returning) {
          manual_seen_.store(true);
        } else if (mode != 0 && mode != 1) {
          latch("下位机出现未知模式");
        } else if (phase == Phase::Following && mode != 1) {
          latch("视觉档退出");
        }

        double command_yaw = baseline_yaw_;
        double command_pitch = baseline_pitch_;
        bool should_send = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          // 模式 0 只允许等待视觉档时发送起始角度；从视觉档退出后立即停发。
          should_send = (phase_ == Phase::AwaitVisual && (mode == 0 || mode == 1)) ||
                        ((phase_ == Phase::Following || phase_ == Phase::Returning) &&
                         mode == 1);
          if (should_send) {
            const double yaw_target = phase_ == Phase::Following ? requested_yaw_ : baseline_yaw_;
            const double pitch_target = phase_ == Phase::Following ? requested_pitch_ : baseline_pitch_;
            // 【发送限速】相邻发送目标每轴最多变化 0.05°，正常约每 50 ms 更新一次。
            // 它限制的是指令变化速度，不是电机真实速度；真实运动由下位机控制器实现。
            // 与策略层“相对实测角度最多修正 0.15°”是两道不同的约束。
            const double yaw_delta = std::clamp(yaw_target - commanded_yaw_, -0.05, 0.05);
            commanded_yaw_ = std::clamp(commanded_yaw_ + yaw_delta,
                                        baseline_yaw_ - 1.0, baseline_yaw_ + 1.0);
            const double pitch_delta = std::clamp(pitch_target - commanded_pitch_, -0.05, 0.05);
            commanded_pitch_ = std::clamp(commanded_pitch_ + pitch_delta,
                                          baseline_pitch_ - 1.0, baseline_pitch_ + 1.0);
            command_yaw = commanded_yaw_;
            command_pitch = commanded_pitch_;
          }
        }
        if (should_send && !io_failed_.load()) {
          try {
            serial_.send(command_yaw, command_pitch);
          } catch (const std::exception & error) {
            latch(std::string("串口发送失败：") + error.what());
            io_failed_.store(true);
          }
        }
      }
      // 以上一轮发送完成为起点计时；阻塞后不补发密集的“追赶”命令。
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  FieldSerial & serial_;
  const double baseline_yaw_;
  const double baseline_pitch_;
  mutable std::mutex mutex_;
  Phase phase_ = Phase::AwaitVisual;
  double requested_yaw_;
  double requested_pitch_;
  double commanded_yaw_;
  double commanded_pitch_;
  std::string fault_reason_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> fault_{false};
  std::atomic<bool> io_failed_{false};
  std::atomic<bool> manual_seen_{false};
  std::atomic<std::int64_t> heartbeat_ns_{0};
  std::thread thread_;
};

// 手动档确认必须来自连续 3 条不同的、及时的回传，且两轴已回起点附近。
// 这只能减少偶然误码/模式抖动的误判，不能替代独立串口复核。
class ManualExitConfirm {
public:
  bool observe(const std::optional<TimedFeedback> & feedback, Clock::time_point now,
               bool command_at_baseline, double base_yaw, double base_pitch) {
    if (!feedback || feedback->received > now ||
        now - feedback->received >= std::chrono::milliseconds(250)) {
      streak_ = 0;
      return false;
    }
    if (last_received_ && feedback->received <= *last_received_) return false;
    last_received_ = feedback->received;
    const bool valid = feedback->value.mode == 0 && command_at_baseline &&
                       std::abs(feedback->value.yaw_deg - base_yaw) < 0.2 &&
                       std::abs(feedback->value.pitch_deg - base_pitch) < 0.2;
    streak_ = valid ? streak_ + 1 : 0;
    return streak_ >= 3;
  }

private:
  int streak_ = 0;
  std::optional<Clock::time_point> last_received_;
};

}  // namespace

// 【从这里开始逐段读】main 负责整次实验的流程，而非直接驱动电机电流。
// serial/worker 的智能指针放在 try 外，异常处理时仍能访问发送线程并请求回位。
int main(int argc, char ** argv) {
  Options options;
  std::unique_ptr<FieldSerial> serial;
  std::unique_ptr<FollowWorker> worker;
  double baseline_yaw = 0;
  double baseline_pitch = 0;
  try {
    options = parse_args(argc, argv);
    const double min_confidence =
      YAML::LoadFile(options.config_path)["min_confidence"].as<double>();
    if (!std::isfinite(min_confidence) || min_confidence < 0.7 || min_confidence > 1.0) {
      throw std::runtime_error("min_confidence 须在 0.7 到 1.0 之间");
    }
    if (options.actuate) {
      std::cout << "无弹双轴跟随：请确认无弹、云台附近无人、蓝色装甲板静止、"
                   "遥控器在手动档。按回车启动；提示后再切视觉档。\n";
      std::string line;
      if (!std::getline(std::cin, line)) throw std::runtime_error("未收到启动回车");
    }
    tools::Exiter exiter;
    serial = std::make_unique<FieldSerial>(options.port, options.actuate);
    io::Camera camera(options.config_path);
    auto_aim::YOLO yolo(options.config_path, false);

    // 【主流程状态机】
    // Acquire：寻找唯一且稳定的目标；只读模式在此输出计划并结束。
    // WaitVisual：保持原角度，等待人拨开关；HoldVisual：确认视觉档后稳定 0.7 秒。
    // Follow：最多 8 秒反复计算双轴像素修正；Return：限速回起点；WaitManual：等待手动接管。
    // 此 Stage 管实验步骤；FollowWorker::Phase 管发送线程，两者层次不同。
    enum class Stage { Acquire, WaitVisual, HoldVisual, Follow, Return, WaitManual };
    Stage stage = Stage::Acquire;
    auto started = Clock::now();
    auto stage_started = started;
    int stable_frames = 0;
    int visual_streak = 0;
    cv::Point2d last_target{0, 0};
    cv::Point2d anchor_target{0, 0};
    double last_yaw = 0;
    double last_pitch = 0;
    bool have_last_target = false;
    ManualExitConfirm manual_exit;
    std::optional<Clock::time_point> last_visual_feedback_time;
    std::cout << (options.actuate ? "实机模式：20 Hz、fire_flag=0、最多 8 秒双轴跟随" :
                                  "只读模式：仅试算，不发送指令")
              << "；串口=" << options.port
              << "；最低分数=" << min_confidence << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    for (int frame = 0; frame < options.max_frames && !exiter.exit(); ++frame) {
      const auto now = Clock::now();
      if (now - started > std::chrono::seconds(45)) {
        throw std::runtime_error("总测试时间超过 45 秒");
      }
      if (worker && worker->tripped()) {
        throw std::runtime_error("发送线程安全中止：" + worker->fault_reason());
      }

      // 回位阶段只需要有效的角度回传，不再等相机识别。
      // 这样目标已丢失时仍有机会回到实验起点；串口失效时则不能保证回位。
      if (stage == Stage::Return || stage == Stage::WaitManual) {
        const auto feedback = serial->latest();
        if (manual_exit.observe(feedback, now, worker->baseline_reached(),
                                baseline_yaw, baseline_pitch)) {
          std::cout << "已切回手动档；无弹连续跟随测试完成。" << std::endl;
          return 0;
        }
        if (stage == Stage::Return && feedback && feedback->value.mode == 1 &&
            now - feedback->received < std::chrono::milliseconds(250) &&
            worker->baseline_reached() &&
            std::abs(feedback->value.yaw_deg - baseline_yaw) < 0.2 &&
            std::abs(feedback->value.pitch_deg - baseline_pitch) < 0.2) {
          stage = Stage::WaitManual;
          stage_started = now;
          std::cout << "双轴已平滑回到起始角度附近。请拨回手动档；最多等待 10 秒。"
                    << std::endl;
        }
        if (stage == Stage::Return && now - stage_started > std::chrono::seconds(5)) {
          throw std::runtime_error("5 秒内未确认双轴回到起始角度；请手动接管");
        }
        if (stage == Stage::WaitManual && now - stage_started > std::chrono::seconds(10)) {
          throw std::runtime_error("等待切回手动档超时");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      cv::Mat image;
      Clock::time_point timestamp;
      // 取一张 OpenCV 图像及驱动记录的本机时刻；image.cols/rows 是宽/高。
      // 相机 read 可能等待队列，独立 FollowWorker 才能在此时继续监控。
      camera.read(image, timestamp);
      if (image.empty()) throw std::runtime_error("相机图像为空");
      if (worker && worker->tripped()) {
        throw std::runtime_error("发送线程安全中止：" + worker->fault_reason());
      }
      serial->poll();
      // YOLO 输出一组 Armor，包含颜色、数字类别、置信度、四角与像素中心。
      // 本句没有输出目标的米制距离，也没有执行云台控制。
      const auto armors = yolo.detect(image, frame);
      serial->poll();
      const auto measured_at = Clock::now();
      const auto image_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        measured_at - timestamp).count();
      // 在识别完成后检查图像年龄，避免用已过期的画面计算新目标。
      // 这个年龄从驱动在本机取到缓冲区时算起，未包含可验证的全链路硬件延迟。
      if (image_age_ms < -10 || image_age_ms > 250) {
        throw std::runtime_error("图像在识别完成时已超过 250 ms");
      }

      int candidates = 0;
      const auto * armor = select_armor(armors, image.size(), min_confidence, candidates);
      const auto feedback = serial->latest();
      if (frame % 10 == 0) {
        std::cout << "frame=" << frame << " target=("
                  << (armor && candidates == 1 ? armor->center.x : -1) << ","
                  << (armor && candidates == 1 ? armor->center.y : -1) << ")"
                  << " candidates=" << candidates << " image_age_ms=" << image_age_ms;
        if (feedback) {
          const auto feedback_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            measured_at - feedback->received).count();
          std::cout << " yaw=" << feedback->value.yaw_deg
                    << " pitch=" << feedback->value.pitch_deg
                    << " mode=" << feedback->value.mode
                    << " feedback_age_ms=" << feedback_age_ms;
        } else {
          std::cout << " feedback=missing";
        }
        std::cout << std::endl;
      }

      // 绘图使用 image.clone()，避免在供算法使用的原图上直接涂画。
      // 显示缩小为一半，但控制仍使用原始分辨率的像素坐标。
      if (options.display) {
        cv::Mat drawing = image.clone();
        const cv::Point image_center((image.cols - 1) / 2, (image.rows - 1) / 2);
        cv::drawMarker(drawing, image_center, {255, 255, 255}, cv::MARKER_CROSS, 20, 1);
        if (armor && candidates == 1) {
          for (const auto & point : armor->points) cv::circle(drawing, point, 4, {0, 255, 0}, 2);
          cv::circle(drawing, armor->center, 6, {0, 255, 0}, 2);
        }
        cv::resize(drawing, drawing, {}, 0.5, 0.5);
        cv::imshow("safe pixel xy follow", drawing);
        if (cv::waitKey(1) == 'q') throw std::runtime_error("用户按 q 中止");
      }

      if (!feedback) {
        if (measured_at - started > std::chrono::seconds(2)) {
          throw std::runtime_error("两秒内没有收到下位机回传");
        }
        continue;
      }
      const auto feedback_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        measured_at - feedback->received).count();
      if (feedback_age_ms < -10 || feedback_age_ms > 250) {
        throw std::runtime_error("下位机回传超过 250 ms 未更新");
      }
      const double yaw = feedback->value.yaw_deg;
      const double pitch = feedback->value.pitch_deg;
      const int mode = feedback->value.mode;
      // 还没开始控制时，允许继续等目标；进入等待视觉档等后续阶段后，
      // 目标丢失或出现多个候选会中止本轮，而不会自动跳到另一块装甲板。
      if (!armor || candidates != 1) {
        if (stage == Stage::Acquire) {
          stable_frames = 0;
          have_last_target = false;
          continue;
        }
        throw std::runtime_error("蓝色目标丢失或出现多个候选");
      }
      const cv::Point2d target = armor->center;
      if (stage != Stage::Acquire && have_last_target &&
          cv::norm(target - last_target) > 30) {
        throw std::runtime_error("目标位置单帧跳变超过 30 像素");
      }
      last_target = target;
      have_last_target = true;

      // 启动稳定性门槛：同一轮锚点附近连续 5 帧，像素变化小于 5 px，
      // 相邻回传 yaw/pitch 变化都小于 0.1°；这只是联调筛选，不是 EKF 跟踪。
      if (stage == Stage::Acquire) {
        if (options.actuate && mode != 0) {
          throw std::runtime_error("启动时须处于手动档 mode=0");
        }
        if (stable_frames > 0 && cv::norm(target - anchor_target) < 5 &&
            std::abs(yaw - last_yaw) < 0.1 && std::abs(pitch - last_pitch) < 0.1) {
          ++stable_frames;
        } else {
          stable_frames = 1;
          anchor_target = target;
        }
        last_yaw = yaw;
        last_pitch = pitch;
        if (stable_frames < 5) continue;
        const auto plan = autoaim::plan_follow_xy(target.x, target.y, image.cols,
            image.rows, yaw, pitch, yaw, pitch, options.slope_yaw, options.slope_pitch);
        std::cout << "计划：target=(" << target.x << "," << target.y << ")"
                  << " 偏差=(" << plan.yaw.error_px << "," << plan.pitch.error_px
                  << ") px；状态=" << plan.reason << "；yaw " << yaw << "° -> ";
        if (plan.commandable()) std::cout << plan.yaw_target_deg << "°";
        else std::cout << "无可发送目标";
        std::cout << "；pitch " << pitch << "° -> ";
        if (plan.commandable()) std::cout << plan.pitch_target_deg << "°";
        else std::cout << "无可发送目标";
        std::cout << "；fire_flag=0" << std::endl;
        if (!plan.commandable()) throw std::runtime_error("无法规划初始跟随：" + plan.reason);
        // 默认运行到这里就结束：没有创建发送线程，也没有 send。
        if (!options.actuate) {
          std::cout << "只读测试完成，未发送任何云台指令。" << std::endl;
          return 0;
        }
        if (std::abs(plan.yaw.error_px) > 25 ||
            std::abs(plan.pitch.error_px) > 25) {
          throw std::runtime_error("双轴首次实机测试要求目标 x/y 均在画面中心 ±25 px 内");
        }
        baseline_yaw = yaw;
        baseline_pitch = pitch;
        // 先写当前角度，避免切视觉档时沿用下位机以前的目标。
        serial->send(baseline_yaw, baseline_pitch);
        worker = std::make_unique<FollowWorker>(*serial, baseline_yaw, baseline_pitch);
        stage = Stage::WaitVisual;
        stage_started = measured_at;
        std::cout << "开始保持起始角度。请现在拨到视觉档；最多等待 15 秒。"
                  << std::endl;
      } else {
        if (std::abs(yaw - baseline_yaw) > 1.4 ||
            std::abs(pitch - baseline_pitch) > 1.4) {
          throw std::runtime_error("云台偏离起点超过安全范围");
        }
        // 【静止靶假设】目标像素变化应大致来自云台双轴转动。
        // 实测与预期差太多时，可能是靶移动、目标切换或近似失效，程序中止。
        const cv::Point2d expected_target = anchor_target + cv::Point2d(
            options.slope_yaw * (yaw - baseline_yaw),
            options.slope_pitch * (pitch - baseline_pitch));
        if (cv::norm(target - expected_target) > 30) {
          throw std::runtime_error("目标相对环境移动超过 30 像素；仅允许静止装甲板");
        }
        if (stage == Stage::WaitVisual) {
          if (mode != 0 && mode != 1) throw std::runtime_error("意外的云台模式");
          if (measured_at - stage_started > std::chrono::seconds(15)) {
            throw std::runtime_error("等待视觉档超时");
          }
          if (cv::norm(target - anchor_target) > 25) {
            throw std::runtime_error("切档时目标移动超过 25 像素");
          }
          // 只累计新的串口回传时间，避免重复读取同一帧也算“连续 3 次视觉档”。
          if (!last_visual_feedback_time || feedback->received > *last_visual_feedback_time) {
            visual_streak = mode == 1 ? visual_streak + 1 : 0;
            last_visual_feedback_time = feedback->received;
          }
          if (visual_streak >= 3) {
            stage = Stage::HoldVisual;
            stage_started = measured_at;
            std::cout << "视觉档连续收到 3 次新回传，先保持 0.7 秒。" << std::endl;
          }
        } else if (mode != 1) {
          throw std::runtime_error("视觉档退出");
        } else if (stage == Stage::HoldVisual &&
                   measured_at - stage_started >= std::chrono::milliseconds(700)) {
          worker->start_follow();
          stage = Stage::Follow;
          stage_started = measured_at;
          std::cout << "开始 8 秒双轴跟随；目标必须静止，fire_flag=0。"
                    << std::endl;
        }
        if (stage == Stage::Follow) {
          if (measured_at - stage_started >= std::chrono::seconds(8)) {
            worker->request_return();
            stage = Stage::Return;
            stage_started = measured_at;
            std::cout << "跟随结束；正以每 50 ms 不超过 0.05° 回到起始角度。"
                      << std::endl;
          } else {
            const auto plan = autoaim::plan_follow_xy(target.x, target.y, image.cols,
                image.rows, yaw, pitch, baseline_yaw, baseline_pitch,
                options.slope_yaw, options.slope_pitch);
            if (!plan.commandable()) {
              throw std::runtime_error("停止跟随：" + plan.reason);
            }
            // 下一帧会用新的实际回传重新计算；没有积分项或旋转目标预测。
            worker->set_target(plan.yaw_target_deg, plan.pitch_target_deg);
            if (frame % 20 == 0) {
              std::cout << "控制：target=(" << target.x << "," << target.y
                        << ") 偏差=(" << plan.yaw.error_px << ","
                        << plan.pitch.error_px << ") px；回传=(" << yaw << "°,"
                        << pitch << "°) 目标=(" << plan.yaw_target_deg << "°,"
                        << plan.pitch_target_deg << "°)；状态=" << plan.reason
                        << std::endl;
            }
          }
        }
      }
      // 只有本轮处理正常走到这里才更新心跳；异常路径不会冒充成功处理。
      if (worker) worker->beat();
    }
    throw std::runtime_error("帧数用尽或测试被中断");
  } catch (const std::exception & error) {
    // 统一异常出口：先报告中止原因，再在串口/模式允许时请求平滑回起点。
    // 有回位请求不代表已成功回位；要看后续实际回传与手动档确认。
    std::cerr << "无弹跟随中止：" << error.what() << '\n';
    if (worker) {
      worker->request_return();
      // mode=1 时不能在偏角处销毁线程；让发送线程一直平滑退回起点。
      // 如回传/串口失效，自动退回无法验证，只能立即提示人工切手动。
      auto next_prompt = Clock::now();
      const auto return_deadline = next_prompt + std::chrono::seconds(10);
      while (true) {
        const auto now = Clock::now();
        const auto feedback = serial->latest();
        if (worker->io_failed()) {
          std::cerr << "串口读写失败，无法继续自动回正；请立即拨回手动档。\n";
          break;
        }
        if (!feedback || feedback->received > now ||
            now - feedback->received > std::chrono::milliseconds(250)) {
          std::cerr << "回传失效，无法确认回正；请立即拨回手动档。\n";
          break;
        }
        if (feedback->value.mode == 0 || worker->manual_seen()) break;
        if (feedback->value.mode != 1) {
          std::cerr << "出现未知模式；请立即拨回手动档。\n";
          break;
        }
        if (worker->baseline_reached() &&
            std::abs(feedback->value.yaw_deg - baseline_yaw) < 0.2 &&
            std::abs(feedback->value.pitch_deg - baseline_pitch) < 0.2) break;
        if (now >= return_deadline) {
          std::cerr << "10 秒内未确认双轴回到起始角度；请立即拨回手动档。\n";
          break;
        }
        if (now >= next_prompt) {
          std::cerr << "正在限速回到双轴起点；如云台异常，请立即拨回手动档。\n";
          next_prompt = now + std::chrono::seconds(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      std::cerr << "若仍在视觉档，请立即拨回手动档。\n";
    }
    return 1;
  }
}
