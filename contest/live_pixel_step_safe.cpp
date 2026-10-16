// 【学习导读｜第二步：像素决定的一次小角度修正】
// 只读模式只输出计划；--actuate 才建立发送线程，所有命令 fire_flag=0。
// Acquire稳定5帧 → WaitVisual → HoldVisual 0.7s → Step 2s → Return 2s → WaitManual。
// 先读 main()，再读 pixel_step_policy.hpp，最后读 FieldSerial 与 CommandWorker。
// 由启动时选到的目标决定一次 ±0.3° 以内的 yaw 修正，随后回到起点。
// 与 follow 的区别：这里不会每帧重新计算追向图像中心，也没有 follow 的每周期 ±0.05° 平滑层。

#include "protocol.hpp"
#include "pixel_step_policy.hpp"

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

#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
  std::string config_path;
  std::string port;
  std::string video_path;
  int max_frames = 1000;
  bool display = false;
  bool actuate = false;
  double slope_px_per_deg = 0;
};

void usage(const char * executable) {
  std::cout << "用法：" << executable
            << " --config configs/live_yolo_1011.yaml --port /dev/ttyACM0"
               " --slope 32.1 [--frames 1000] [--display] [--actuate]\n"
               "默认只计算一次无弹小幅 yaw 修正，不发送指令。"
               "显式 --actuate 才进行无弹、最多 0.3° 的一次实机修正。\n";
}

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
    else if (arg == "--video") options.video_path = value;
    else if (arg == "--slope") options.slope_px_per_deg = std::stod(value);
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
  if (!std::isfinite(options.slope_px_per_deg) ||
      std::abs(options.slope_px_per_deg) < 5 ||
      std::abs(options.slope_px_per_deg) > 200) {
    throw std::runtime_error("请指定现场实测 --slope，绝对值须在 5 到 200 px/° 之间");
  }
  if (options.actuate && !options.video_path.empty()) {
    throw std::runtime_error("实机修正不能使用录像输入");
  }
  return options;
}

struct TimedFeedback {
  autoaim::Feedback value;
  Clock::time_point received;
};

// 从任意字节位置开始读时，搜索固定长度和 ID1 前缀，再交由正式协议解析器
// 检查完整 30 字节帧。协议本身无 CRC，仍需检查数值合理性。
class FieldSerial {
public:
  explicit FieldSerial(const std::string & port, bool writable) : writable_(writable) {
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

  FieldSerial(const FieldSerial &) = delete;
  FieldSerial & operator=(const FieldSerial &) = delete;

  ~FieldSerial() {
    if (fd_ >= 0) {
      if (have_saved_) ::tcsetattr(fd_, TCSANOW, &saved_);
      ::close(fd_);
    }
  }

  void poll() {
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

  std::optional<TimedFeedback> latest() const { return latest_; }

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
  static bool plausible(const autoaim::Feedback & feedback) {
    return std::isfinite(feedback.yaw_deg) && std::isfinite(feedback.pitch_deg) &&
           std::isfinite(feedback.roll_deg) && std::isfinite(feedback.bullet_speed_mps) &&
           std::abs(feedback.yaw_deg) <= 720 && std::abs(feedback.pitch_deg) <= 180 &&
           std::abs(feedback.roll_deg) <= 180 && feedback.mode >= 0 && feedback.mode <= 10 &&
           feedback.robot_id >= 0 && feedback.robot_id <= 50 &&
           feedback.bullet_speed_mps >= 0 && feedback.bullet_speed_mps <= 100;
  }

  void parse() {
    constexpr std::array<std::uint8_t, 4> prefix{{0x1c, 0x00, 0x01, 0x00}};
    while (buffer_.size() >= prefix.size()) {
      const auto start = std::search(buffer_.begin(), buffer_.end(), prefix.begin(), prefix.end());
      if (start == buffer_.end()) {
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
  std::mutex write_mutex_;
  termios saved_{};
  bool have_saved_ = false;
  std::vector<std::uint8_t> buffer_;
  autoaim::FeedbackParser parser_;
  std::optional<TimedFeedback> latest_;
};

const auto_aim::Armor * select_armor(const std::list<auto_aim::Armor> & armors,
                                      const cv::Size & size, int & candidates) {
  const auto_aim::Armor * selected = nullptr;
  double best_distance = 0.0;
  candidates = 0;
  const cv::Point2f center((size.width - 1) * 0.5f, (size.height - 1) * 0.5f);
  for (const auto & armor : armors) {
    if (armor.color != auto_aim::Color::blue || armor.confidence < 0.8 ||
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

// 相机 read() 可能一直等不到首帧；发指令的线程因此独立于相机循环。
// 主线程超过 300 ms 未完成一轮时，它锁定为起始角度，之后禁止再次阶跃。
// 独立发送线程周期性写入所请求的固定目标，避免视觉处理稍慢就完全停发。
// 这版 worker 的回传/模式判断主要由视觉主流程完成，不能套用 FollowWorker 的全部保障。
class CommandWorker {
public:
  CommandWorker(FieldSerial & serial, double baseline_yaw, double baseline_pitch)
      : serial_(serial), baseline_yaw_(baseline_yaw),
        baseline_pitch_(baseline_pitch), desired_yaw_(baseline_yaw) {
    beat();
    thread_ = std::thread([this] { run(); });
  }

  CommandWorker(const CommandWorker &) = delete;
  CommandWorker & operator=(const CommandWorker &) = delete;

  ~CommandWorker() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }

  void beat() {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
    heartbeat_ns_.store(ns);
  }

  bool tripped() const { return fault_.load(); }

  // 只接收在起始角度 ±0.3° 内的绝对 yaw 目标；不是把每次输入当作累加增量。
  void set_step(double yaw) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (fault_.load()) throw std::runtime_error("看门狗已接管，禁止再次阶跃");
    if (!std::isfinite(yaw) || std::abs(yaw - baseline_yaw_) > 0.300001) {
      throw std::runtime_error("阶跃命令超出起始角度 ±0.3°");
    }
    desired_yaw_ = yaw;
  }

  void set_baseline() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    desired_yaw_ = baseline_yaw_;
  }

private:
  // 以 sleep_until 调度约 50ms 一轮；主流程超过300ms未更新心跳时，锁存故障并改发基线。
  // 恢复相机画面不会自动解除本轮故障。基线指令也不保证失联设备能实际回位。
  void run() {
    auto next = Clock::now();
    while (!stop_.load()) {
      const auto now = Clock::now();
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
      const bool stalled = now_ns - heartbeat_ns_.load() > 300000000LL;
      double yaw_to_send = baseline_yaw_;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (stalled && !fault_.load()) {
          fault_.store(true);
          desired_yaw_ = baseline_yaw_;
          std::cerr << "看门狗：图像循环超过 300 ms 未更新，持续发送起始角度；"
                       "请拨回手动档。\n";
        }
        yaw_to_send = desired_yaw_;
      }
      try {
        serial_.send(yaw_to_send, baseline_pitch_);
      } catch (const std::exception & error) {
        fault_.store(true);
        std::cerr << "看门狗：串口发送失败：" << error.what()
                  << "；请拨回手动档。\n";
        break;
      }
      next += std::chrono::milliseconds(50);
      std::this_thread::sleep_until(next);
      if (Clock::now() > next + std::chrono::milliseconds(100)) next = Clock::now();
    }
  }

  FieldSerial & serial_;
  const double baseline_yaw_;
  const double baseline_pitch_;
  std::mutex state_mutex_;
  double desired_yaw_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> fault_{false};
  std::atomic<std::int64_t> heartbeat_ns_{0};
  std::thread thread_;
};

}  // namespace

// 主线程取图、识别、读反馈并检查稳定性/模式，状态转换时通知 CommandWorker。
// try/catch 处理实验中止，资源对象销毁时等待发送线程结束。
int main(int argc, char ** argv) {
  Options options;
  std::unique_ptr<FieldSerial> serial;
  bool have_baseline = false;
  double baseline_yaw = 0;
  double baseline_pitch = 0;
  try {
    options = parse_args(argc, argv);
    if (options.actuate) {
      std::cout << "首次无弹像素修正：确认无弹、云台附近无人、遥控器在手动档。"
                   "按回车后启动；程序提示时再切视觉档。\n";
      std::string line;
      if (!std::getline(std::cin, line)) throw std::runtime_error("未收到启动回车");
    }
    tools::Exiter exiter;
    serial = std::make_unique<FieldSerial>(options.port, options.actuate);

    std::unique_ptr<io::Camera> camera;
    cv::VideoCapture video;
    if (options.video_path.empty()) {
      camera = std::make_unique<io::Camera>(options.config_path);
    } else {
      video.open(options.video_path);
      if (!video.isOpened()) throw std::runtime_error("无法打开测试视频：" + options.video_path);
    }
    auto_aim::YOLO yolo(options.config_path, false);
    std::unique_ptr<CommandWorker> worker;

    enum class Stage { Acquire, WaitVisual, HoldVisual, Step, Return, WaitManual };
    Stage stage = Stage::Acquire;
    auto stage_started = Clock::now();
    const auto started = stage_started;
    int stable_frames = 0;
    int visual_streak = 0;
    double last_target_x = 0;
    double last_yaw = 0;
    double last_pitch = 0;
    double step_yaw = 0;
    double anchor_target_x = 0;
    bool have_last_target = false;
    std::cout << (options.actuate ? "实机模式：只发 fire_flag=0 的小幅修正" :
                                  "只读模式：仅计算，不发送指令")
              << "；串口=" << options.port << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    for (int frame = 0; frame < options.max_frames && !exiter.exit(); ++frame) {
      if (worker) {
        if (worker->tripped()) throw std::runtime_error("看门狗已接管");
        worker->beat();
      }
      if (Clock::now() - started > std::chrono::seconds(40)) {
        throw std::runtime_error("总测试时间超过 40 秒");
      }
      cv::Mat image;
      Clock::time_point timestamp;
      if (camera) camera->read(image, timestamp);
      else {
        if (!video.read(image)) break;
        timestamp = Clock::now();
      }
      if (image.empty()) throw std::runtime_error("相机图像为空");
      if (worker && worker->tripped()) throw std::runtime_error("看门狗已接管");
      const auto image_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - timestamp).count();
      if (image_age_ms > 250 || image_age_ms < -10) {
        throw std::runtime_error("相机画面超过 250 毫秒未更新");
      }
      serial->poll();
      const auto armors = yolo.detect(image, frame);
      serial->poll();
      const auto now = Clock::now();
      const auto command_image_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - timestamp).count();
      if (command_image_age_ms > 250 || command_image_age_ms < -10) {
        throw std::runtime_error("识别完成时，相机画面已超过 250 毫秒");
      }

      int candidates = 0;
      const auto * armor = select_armor(armors, image.size(), candidates);
      const auto feedback = serial->latest();
      if (frame % 10 == 0) {
        std::cout << "frame=" << frame << " target_x="
                  << (armor && candidates == 1 ? armor->center.x : -1)
                  << " candidates=" << candidates
                  << " image_age_ms=" << command_image_age_ms;
        if (feedback) {
          const auto feedback_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - feedback->received).count();
          std::cout << " yaw=" << feedback->value.yaw_deg
                    << " pitch=" << feedback->value.pitch_deg
                    << " mode=" << feedback->value.mode
                    << " feedback_age_ms=" << feedback_age_ms;
        } else {
          std::cout << " feedback=missing";
        }
        std::cout << std::endl;
      }

      if (options.display) {
        cv::Mat drawing = image.clone();
        const cv::Point image_center((image.cols - 1) / 2, (image.rows - 1) / 2);
        cv::drawMarker(drawing, image_center, {255, 255, 255}, cv::MARKER_CROSS, 20, 1);
        if (armor) {
          for (const auto & point : armor->points) cv::circle(drawing, point, 4, {0, 255, 0}, 2);
          cv::circle(drawing, armor->center, 6, {0, 255, 0}, 2);
        }
        cv::resize(drawing, drawing, {}, 0.5, 0.5);
        cv::imshow("safe pixel yaw step", drawing);
        if (cv::waitKey(1) == 'q') throw std::runtime_error("用户按 q 中止");
      }

      if (!feedback) {
        if (now - started > std::chrono::seconds(2)) {
          throw std::runtime_error("两秒内没有收到下位机回传");
        }
        continue;
      }
      const auto feedback_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - feedback->received).count();
      if (feedback_age_ms > 250 || feedback_age_ms < -10) {
        throw std::runtime_error("下位机回传超过 250 毫秒未更新");
      }
      const double yaw = feedback->value.yaw_deg;
      const double pitch = feedback->value.pitch_deg;
      const int mode = feedback->value.mode;
      if (!armor || candidates != 1) {
        if (stage == Stage::Acquire) {
          stable_frames = 0;
          have_last_target = false;
          continue;
        }
        throw std::runtime_error("蓝色目标丢失或出现多个候选；停止修正");
      }
      const double target_x = armor->center.x;
      if (stage != Stage::Acquire && have_last_target &&
          std::abs(target_x - last_target_x) > 30) {
        throw std::runtime_error("目标突然跳变超过 30 像素；停止修正");
      }
      last_target_x = target_x;
      have_last_target = true;

      if (stage == Stage::Acquire) {
        if (options.actuate && mode != 0) {
          throw std::runtime_error("启动时须处于手动档 mode=0");
        }
        if (stable_frames > 0 && std::abs(target_x - anchor_target_x) < 5 &&
            std::abs(yaw - last_yaw) < 0.1 && std::abs(pitch - last_pitch) < 0.1) {
          ++stable_frames;
        } else {
          stable_frames = 1;
          anchor_target_x = target_x;
        }
        last_yaw = yaw;
        last_pitch = pitch;
        if (stable_frames < 5) continue;
        const auto plan = autoaim::plan_pixel_step(target_x, image.cols, yaw, pitch,
                                                   options.slope_px_per_deg);
        if (!plan.valid) throw std::runtime_error("无法规划像素修正：" + plan.reason);
        std::cout << "计划：target_x=" << target_x << " 中心="
                  << (image.cols - 1) * 0.5 << " 偏差=" << plan.error_px
                  << " px；yaw " << yaw << "° -> " << plan.yaw_target_deg
                  << "°，pitch 保持 " << pitch << "°；fire_flag=0" << std::endl;
        if (!options.actuate) {
          std::cout << "只读测试完成，未发送任何云台指令。" << std::endl;
          return 0;
        }
        baseline_yaw = yaw;
        baseline_pitch = pitch;
        have_baseline = true;
        stage = Stage::WaitVisual;
        stage_started = now;
        // 先写入起始角度，再提示切视觉档，避免沿用下位机上一次目标。
        serial->send(baseline_yaw, baseline_pitch);
        worker = std::make_unique<CommandWorker>(*serial, baseline_yaw, baseline_pitch);
        std::cout << "开始发送起始角度保持。请现在拨到视觉档；最多等待 15 秒。"
                  << std::endl;
      } else {
        if (std::abs(yaw - baseline_yaw) > 0.8 ||
            std::abs(pitch - baseline_pitch) > 0.8) {
          throw std::runtime_error("云台偏离起始角度超过 0.8°；停止修正");
        }
        if (stage == Stage::WaitVisual) {
          if (mode != 0 && mode != 1) throw std::runtime_error("意外的云台模式");
          if (now - stage_started > std::chrono::seconds(15)) {
            throw std::runtime_error("等待视觉档超时");
          }
          if (std::abs(target_x - anchor_target_x) > 25) {
            throw std::runtime_error("等待切档时目标移动超过 25 像素");
          }
          visual_streak = mode == 1 ? visual_streak + 1 : 0;
          if (visual_streak >= 3) {
            stage = Stage::HoldVisual;
            stage_started = now;
            std::cout << "已进入视觉档，先保持起始角度 0.7 秒。" << std::endl;
          }
        } else if (mode != 1) {
          if (stage == Stage::WaitManual && mode == 0) {
            std::cout << "已切回手动档；测试完成。" << std::endl;
            return 0;
          }
          throw std::runtime_error("视觉档退出；停止修正");
        } else if (stage == Stage::HoldVisual &&
                   now - stage_started >= std::chrono::milliseconds(700)) {
          const auto plan = autoaim::plan_pixel_step(target_x, image.cols,
                                                     baseline_yaw, baseline_pitch,
                                                     options.slope_px_per_deg);
          if (!plan.valid) throw std::runtime_error("无法规划像素修正：" + plan.reason);
          step_yaw = plan.yaw_target_deg;
          worker->set_step(step_yaw);
          stage = Stage::Step;
          stage_started = now;
          std::cout << "像素决定单次修正：yaw " << baseline_yaw << "° -> "
                    << step_yaw << "°；持续 2 秒，fire_flag=0。" << std::endl;
        } else if (stage == Stage::Step &&
                   now - stage_started >= std::chrono::seconds(2)) {
          worker->set_baseline();
          stage = Stage::Return;
          stage_started = now;
          std::cout << "回到起始角度，持续 2 秒。" << std::endl;
        } else if (stage == Stage::Return &&
                   now - stage_started >= std::chrono::seconds(2)) {
          stage = Stage::WaitManual;
          stage_started = now;
          std::cout << "请现在拨回手动档；最多等待 10 秒。" << std::endl;
        } else if (stage == Stage::WaitManual &&
                   now - stage_started > std::chrono::seconds(10)) {
          throw std::runtime_error("等待切回手动档超时");
        }
      }

      if (worker) worker->beat();
    }
    throw std::runtime_error("帧数用尽或测试被中断");
  } catch (const std::exception & error) {
    std::cerr << "像素修正测试中止：" << error.what() << '\n';
    if (options.actuate && serial && have_baseline) {
      const auto feedback = serial->latest();
      if (feedback && feedback->value.mode == 1 &&
          Clock::now() - feedback->received < std::chrono::milliseconds(250)) {
        for (int i = 0; i < 4; ++i) {
          try {
            serial->send(baseline_yaw, baseline_pitch);
          } catch (const std::exception &) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
      std::cerr << "若仍在视觉档，请立即拨回手动档。\n";
    }
    return 1;
  }
}
