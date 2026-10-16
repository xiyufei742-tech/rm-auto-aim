// 【学习导读｜现场只读记录】
// Camera 取图 → YOLO 输出蓝色装甲候选与四角 → 读取最新串口回传。
// --session 同时保存未画框 AVI、逐帧 CSV、配置快照和结束状态；
// --phase 只为 static/sweep/rotating 数据注明采集条件，不改变云台。
// 此入口只读串口，不能控制云台；CSV 中的本机接收时刻不是下位机硬件时间戳。
// 阅读顺序：parse_args → ReadOnlyFeedback → SessionWriter → main。

#include "protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace {

using Clock = std::chrono::steady_clock;

// 只读采集也要收尾 AVI/CSV。信号处理器只设置标志，不调用文件或相机 API。
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }

struct StopSignals {
  using Handler = void (*)(int);
  Handler previous_int = std::signal(SIGINT, request_stop);
  Handler previous_term = std::signal(SIGTERM, request_stop);
  ~StopSignals() {
    std::signal(SIGINT, previous_int);
    std::signal(SIGTERM, previous_term);
  }
};

struct Options {
  std::string config_path;
  std::string port;
  std::string video_path;
  std::string session_prefix;
  std::string phase = "static";
  int max_frames = 60;
  double record_fps = 30.0;
  bool display = false;
};

struct TimedFeedback {
  autoaim::Feedback value;
  Clock::time_point received;
};

void usage(const char * executable) {
  std::cout << "用法：" << executable
            << " --config configs/live_yolo.yaml [--port /dev/ttyACM0]"
               " [--video path.avi] [--frames 60] [--display]"
               " [--session output/name] [--phase static|sweep|rotating]"
               " [--record-fps 30]\n"
               "--session 同时保存未标注 AVI 和逐帧 CSV；不会向下位机发送指令。\n";
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
    if (i + 1 >= argc) throw std::runtime_error("缺少参数值：" + arg);
    const std::string value(argv[++i]);
    if (arg == "--config") options.config_path = value;
    else if (arg == "--port") options.port = value;
    else if (arg == "--video") options.video_path = value;
    else if (arg == "--session") options.session_prefix = value;
    else if (arg == "--phase") options.phase = value;
    else if (arg == "--record-fps") {
      std::size_t consumed = 0;
      options.record_fps = std::stod(value, &consumed);
      if (consumed != value.size() || !std::isfinite(options.record_fps) ||
          options.record_fps < 1.0 || options.record_fps > 240.0) {
        throw std::runtime_error("--record-fps 必须在 1 到 240 之间");
      }
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
  if (options.phase != "static" && options.phase != "sweep" &&
      options.phase != "rotating") {
    throw std::runtime_error("--phase 只能是 static、sweep 或 rotating");
  }
  if (!options.session_prefix.empty() && options.session_prefix.back() == '/') {
    throw std::runtime_error("--session 须填写文件名前缀，而不是目录");
  }
  if (!options.session_prefix.empty() && !options.video_path.empty() && !options.port.empty()) {
    throw std::runtime_error("离线视频与当前实时串口不能当作同一时刻的现场记录");
  }
  return options;
}

// 一行 CSV 对应一帧 AVI；同机时钟只能对齐“取帧/收到回传”的软件时间，
// 不能当作相机硬件曝光与下位机 IMU 测量的精确同步。
class SessionWriter {
public:
  explicit SessionWriter(const Options & options) : fps_(options.record_fps) {
    const auto & prefix = options.session_prefix;
    if (prefix.empty()) return;
    avi_path_ = prefix + ".avi";
    csv_path_ = prefix + ".csv";
    config_path_ = prefix + ".config.yaml";
    status_path_ = prefix + ".session.txt";
    for (const auto & path : {avi_path_, csv_path_, config_path_, status_path_}) {
      // dangling symlink 也不应被当作可以覆盖的新文件。
      if (std::filesystem::symlink_status(path).type() != std::filesystem::file_type::not_found) {
        throw std::runtime_error("会话文件已存在，请换一个 --session 名称：" + prefix);
      }
    }
    const auto parent = std::filesystem::path(prefix).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::filesystem::copy_file(options.config_path, config_path_);
    metadata_ = "schema: 1\nsource: " + std::string(options.video_path.empty() ? "camera" : "video") +
                "\nphase: " + options.phase + "\nconfig_snapshot: " +
                std::filesystem::path(config_path_).filename().string() +
                "\nrequested_frames: " + std::to_string(options.max_frames) +
                "\navi_playback_fps: " + std::to_string(fps_) +
                "\nserial_enabled: " + (options.port.empty() ? "false" : "true") +
                "\nraw_detections_meaning: post-NMS YOLO armor results, before probe blue filter\n";
    write_status("recording_or_incomplete");
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_) throw std::runtime_error("无法创建 CSV：" + csv_path_);
    csv_ << "frame,phase,source,width,height,capture_s,processed_s,image_age_ms,"
            "raw_detections,blue_candidates,target_x,target_y,confidence,"
            "x0,y0,x1,y1,x2,y2,x3,y3,"
            "feedback_received_s,feedback_age_ms,yaw_deg,pitch_deg,roll_deg,"
            "mode,robot_id,bullet_speed_mps\n";
    csv_ << std::fixed << std::setprecision(6);
  }

  ~SessionWriter() {
    if (!finished_ && !status_path_.empty()) {
      std::cerr << "会话未完成校验，现有文件只作为部分记录保留：" << status_path_ << '\n';
    }
  }

  bool enabled() const { return csv_.is_open(); }

  void write(int frame, const std::string & phase, const std::string & source,
             const cv::Mat & image, Clock::time_point captured,
             Clock::time_point processed, int raw_detections, int blue_candidates,
             const auto_aim::Armor * selected,
             const std::optional<TimedFeedback> & feedback) {
    if (!enabled()) return;
    if (image.empty()) throw std::runtime_error("不能保存空图像");
    if (image.type() != CV_8UC3) throw std::runtime_error("会话要求 8 位三通道 BGR 图像");
    if (frame != saved_frames_) throw std::runtime_error("会话帧号不连续");
    if (!video_.isOpened()) {
      video_.open(avi_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, image.size());
      if (!video_.isOpened()) throw std::runtime_error("无法创建 AVI：" + avi_path_);
    }
    if (image.size() != frame_size_ && frame_size_.width > 0) {
      throw std::runtime_error("会话中图像尺寸发生变化；停止以防 CSV 与 AVI 错位");
    }
    frame_size_ = image.size();
    if (!first_capture_) first_capture_ = captured;
    video_.write(image);
    csv_ << frame << ',' << phase << ',' << source << ','
         << image.cols << ',' << image.rows << ','
         << std::chrono::duration<double>(captured - *first_capture_).count() << ','
         << std::chrono::duration<double>(processed - *first_capture_).count() << ','
         << std::chrono::duration<double, std::milli>(processed - captured).count() << ','
         << raw_detections << ',' << blue_candidates << ',';
    // 出现多个候选时不强行把其中一个写成“唯一目标”。
    if (selected && blue_candidates == 1 && selected->points.size() == 4) {
      csv_ << selected->center.x << ',' << selected->center.y << ','
           << selected->confidence;
      for (const auto & point : selected->points) csv_ << ',' << point.x << ',' << point.y;
    } else {
      for (int i = 0; i < 11; ++i) {
        if (i > 0) csv_ << ',';
      }
    }
    csv_ << ',';
    if (feedback) {
      csv_ << std::chrono::duration<double>(feedback->received - *first_capture_).count() << ','
           << std::chrono::duration<double, std::milli>(processed - feedback->received).count()
           << ',' << feedback->value.yaw_deg << ',' << feedback->value.pitch_deg << ','
           << feedback->value.roll_deg << ',' << feedback->value.mode << ','
           << feedback->value.robot_id << ',' << feedback->value.bullet_speed_mps;
    } else {
      for (int i = 0; i < 8; ++i) {
        if (i > 0) csv_ << ',';
      }
    }
    csv_ << '\n';
    // 有界刷新：异常退出时尽量少丢 CSV；强杀仍须重新核对可解码帧数。
    if ((saved_frames_ + 1) % 30 == 0) csv_.flush();
    if (!csv_) throw std::runtime_error("CSV 写入失败：" + csv_path_);
    ++saved_frames_;
  }

  void finish() {
    if (status_path_.empty()) return;
    csv_.flush();
    const bool csv_ok = static_cast<bool>(csv_);
    csv_.close();
    video_.release();
    if (!csv_ok || csv_.fail()) throw std::runtime_error("CSV 收尾写入失败：" + csv_path_);
    if (saved_frames_ == 0) throw std::runtime_error("会话没有采到任何帧，未生成有效 AVI");
    std::cout << "录制结束，正在核对 AVI 可解码帧数与 CSV 行数。" << std::endl;
    cv::VideoCapture check(avi_path_);
    if (!check.isOpened()) throw std::runtime_error("保存后的 AVI 无法打开");
    int decoded = 0;
    cv::Mat image;
    while (check.read(image)) {
      if (image.size() != frame_size_) throw std::runtime_error("保存后的视频尺寸不一致");
      ++decoded;
    }
    if (decoded != saved_frames_) {
      throw std::runtime_error("AVI 可解码帧数 " + std::to_string(decoded) +
                               " 与 CSV 行数 " + std::to_string(saved_frames_) + " 不一致");
    }
    write_status("complete_frame_count_verified");
    finished_ = true;
  }

  int saved_frames() const { return saved_frames_; }
  const std::string & avi_path() const { return avi_path_; }
  const std::string & csv_path() const { return csv_path_; }

private:
  void write_status(const std::string & state) const {
    std::ofstream status(status_path_, std::ios::trunc);
    status << "status: " << state << '\n' << metadata_
           << "saved_frames: " << saved_frames_ << '\n';
    status.flush();
    if (!status) throw std::runtime_error("无法写会话状态：" + status_path_);
  }

  double fps_;
  std::string avi_path_;
  std::string csv_path_;
  std::string config_path_;
  std::string status_path_;
  std::string metadata_;
  std::ofstream csv_;
  cv::VideoWriter video_;
  cv::Size frame_size_;
  std::optional<Clock::time_point> first_capture_;
  int saved_frames_ = 0;
  bool finished_ = false;
};

// 串口只有读取权限；从任意字节位置开始读时，搜索固定长度和 ID1 前缀，
// 再交由正式协议解析器检查完整 30 字节帧。协议本身无 CRC，仍需检查数值合理性。
class ReadOnlyFeedback {
public:
  explicit ReadOnlyFeedback(const std::string & port) {
    fd_ = ::open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error("无法只读打开串口 " + port + ": " + std::strerror(errno));
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

  ReadOnlyFeedback(const ReadOnlyFeedback &) = delete;
  ReadOnlyFeedback & operator=(const ReadOnlyFeedback &) = delete;

  ~ReadOnlyFeedback() {
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
  termios saved_{};
  bool have_saved_ = false;
  std::vector<std::uint8_t> buffer_;
  autoaim::FeedbackParser parser_;
  std::optional<TimedFeedback> latest_;
};

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

}  // namespace

int main(int argc, char ** argv) {
  try {
    const auto options = parse_args(argc, argv);
    const double min_confidence =
      YAML::LoadFile(options.config_path)["min_confidence"].as<double>();
    if (!std::isfinite(min_confidence) || min_confidence < 0.7 || min_confidence > 1.0) {
      throw std::runtime_error("min_confidence 须在 0.7 到 1.0 之间");
    }
    StopSignals stop_signals;
    SessionWriter session(options);
    std::unique_ptr<ReadOnlyFeedback> serial;
    if (!options.port.empty()) serial = std::make_unique<ReadOnlyFeedback>(options.port);

    std::unique_ptr<io::Camera> camera;
    cv::VideoCapture video;
    if (options.video_path.empty()) {
      camera = std::make_unique<io::Camera>(options.config_path);
    } else {
      video.open(options.video_path);
      if (!video.isOpened()) throw std::runtime_error("无法打开测试视频：" + options.video_path);
    }
    auto_aim::YOLO yolo(options.config_path, false);

    std::cout << "只读探针启动：不发送云台指令；串口="
              << (serial ? options.port : "未启用")
              << "；最低分数=" << min_confidence << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    for (int frame = 0; frame < options.max_frames && !stop_requested; ++frame) {
      cv::Mat image;
      Clock::time_point timestamp;
      if (camera) camera->read(image, timestamp);
      else {
        if (!video.read(image)) break;
        timestamp = Clock::now();
      }
      if (image.empty()) break;
      if (serial) serial->poll();
      const auto armors = yolo.detect(image, frame);
      if (serial) serial->poll();

      int candidates = 0;
      const auto * armor = select_armor(armors, image.size(), min_confidence, candidates);
      const auto feedback = serial ? serial->latest() : std::nullopt;
      const auto processed = Clock::now();
      session.write(frame, options.phase, camera ? "camera" : "video", image,
                    timestamp, processed, static_cast<int>(armors.size()), candidates,
                    armor, feedback);
      std::cout << "frame=" << frame << " size=" << image.cols << 'x' << image.rows
                << " blue_candidates=" << candidates;
      if (armor) {
        const float cx = (image.cols - 1) * 0.5f;
        const float cy = (image.rows - 1) * 0.5f;
        const float dx = (armor->center.x - cx) / (image.cols * 0.5f);
        const float dy = (armor->center.y - cy) / (image.rows * 0.5f);
        std::cout << " target=(" << armor->center.x << ',' << armor->center.y << ')'
                  << " error_norm=(" << dx << ',' << dy << ')'
                  << " confidence=" << armor->confidence;
      } else {
        std::cout << " target=none";
      }
      if (serial) {
        if (feedback) {
          const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                Clock::now() - feedback->received).count();
          if (age_ms <= 500) {
            std::cout << " feedback=(yaw=" << feedback->value.yaw_deg
                      << " pitch=" << feedback->value.pitch_deg
                      << " mode=" << feedback->value.mode << " age_ms=" << age_ms << ')';
          } else {
            std::cout << " feedback=stale(age_ms=" << age_ms << ')';
          }
        } else {
          std::cout << " feedback=missing";
        }
      }
      std::cout << std::endl;

      if (options.display) {
        cv::Mat drawing = image.clone();
        const cv::Point image_center((image.cols - 1) / 2, (image.rows - 1) / 2);
        cv::drawMarker(drawing, image_center, {255, 255, 255}, cv::MARKER_CROSS, 20, 1);
        cv::putText(drawing, "READ ONLY | blue candidates=" + std::to_string(candidates),
                    {18, 36}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
        if (armor) {
          for (std::size_t index = 0; index < armor->points.size(); ++index) {
            const auto & point = armor->points[index];
            cv::circle(drawing, point, 4, {0, 255, 0}, 2);
            // 原样显示模型给出的点序号，供核对 PnP 物点对应；不按 x/y 重新排序。
            const cv::Point label(cvRound(point.x) + 6, cvRound(point.y) - 6);
            cv::putText(drawing, std::to_string(index), label,
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 0}, 4);
            cv::putText(drawing, std::to_string(index), label,
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
          }
          cv::circle(drawing, armor->center, 6, {0, 255, 0}, 2);
        }
        cv::resize(drawing, drawing, {}, 0.5, 0.5);
        cv::imshow("read-only pixel probe", drawing);
        if (cv::waitKey(1) == 'q') break;
      }
    }
    if (session.enabled()) {
      session.finish();
      std::cout << "会话已保存 " << session.saved_frames() << " 帧："
                << session.avi_path() << " 与 " << session.csv_path() << '\n';
      std::cout << "AVI 使用固定播放帧率；逐帧采集与回传接收时刻以 CSV 为准。"
                << std::endl;
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "只读探针失败：" << error.what() << '\n';
    return 1;
  }
}
