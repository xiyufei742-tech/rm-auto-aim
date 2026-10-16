// 【学习导读｜单独相机预览与录像】
// 从 io::Camera 取得已转换颜色、已按配置旋转的 BGR 图像。
// --record-path 保存未画框 AVI；同名前缀 CSV 记录软件取帧时间。
// 这个程序不调用 YOLO，也不打开比赛串口；标定板录像可用它采集。

#include "io/camera.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }"
  "{record-path    |                     | 保存未标注AVI的路径}"
  "{duration-sec   | 0                   | 录制时长，0表示手动停止}"
  "{record-fps     | 60                  | AVI帧率和采样帧率}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  auto record_path = cli.get<std::string>("record-path");
  auto duration_sec = cli.get<double>("duration-sec");
  auto record_fps = cli.get<double>("record-fps");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }
  if (duration_sec < 0 || record_fps <= 0) {
    throw std::invalid_argument("duration-sec must be >= 0 and record-fps must be > 0");
  }

  cv::VideoWriter video;
  std::ofstream timestamps;
  std::filesystem::path csv_path;
  if (!record_path.empty()) {
    std::filesystem::path video_path(record_path);
    if (video_path.extension() != ".avi") {
      throw std::invalid_argument("record-path must end in .avi");
    }
    if (!video_path.parent_path().empty()) {
      std::filesystem::create_directories(video_path.parent_path());
    }
    csv_path = video_path;
    csv_path.replace_extension(".csv");
    bool avi_exists = std::filesystem::exists(video_path);
    bool csv_exists = std::filesystem::exists(csv_path);
    if (avi_exists || csv_exists) {
      if (avi_exists) {
        std::cerr << "recording already exists (AVI): " << video_path.string() << '\n';
      }
      if (csv_exists) {
        std::cerr << "recording already exists (CSV): " << csv_path.string() << '\n';
      }
      return 2;
    }
  }

  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  std::chrono::steady_clock::time_point first_stamp, last_stamp, last_report_stamp;
  bool started = false;
  int captured_frames = 0;
  int frames_since_report = 0;
  int saved_frames = 0;
  double last_saved_time_s = 0;
  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) {
      tools::logger()->warn("Camera returned an empty frame");
      continue;
    }

    if (!started) {
      first_stamp = timestamp;
      last_stamp = timestamp;
      last_report_stamp = timestamp;
      started = true;
    }
    if (timestamp < last_stamp) {
      tools::logger()->warn("Camera timestamp moved backwards; skipping frame");
      continue;
    }
    ++captured_frames;
    auto elapsed_s = std::chrono::duration<double>(timestamp - first_stamp).count();
    if (duration_sec > 0 && elapsed_s >= duration_sec) break;

    if (captured_frames > 1) ++frames_since_report;
    last_stamp = timestamp;
    auto report_s = std::chrono::duration<double>(timestamp - last_report_stamp).count();
    if (report_s >= 1.0) {
      tools::logger()->info("{:.2f} fps", frames_since_report / report_s);
      frames_since_report = 0;
      last_report_stamp = timestamp;
    }

    if (!record_path.empty()) {
      auto due = elapsed_s >= saved_frames / record_fps;
      if (due) {
        if (!video.isOpened()) {
          video.open(record_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), record_fps, img.size());
          if (!video.isOpened()) throw std::runtime_error("cannot open AVI writer: " + record_path);
          timestamps.open(csv_path);
          if (!timestamps) {
            throw std::runtime_error("cannot open timestamp CSV: " + csv_path.string());
          }
          timestamps << "frame,time_s\n";
          timestamps << std::fixed << std::setprecision(6);
        }
        video.write(img);
        timestamps << saved_frames << ',' << elapsed_s << '\n';
        ++saved_frames;
        last_saved_time_s = elapsed_s;
      }
    }

    if (display) {
      cv::imshow("img", img);
      if (cv::waitKey(1) == 'q') break;
    }
  }

  video.release();
  timestamps.close();
  if (!record_path.empty()) {
    auto effective_fps = saved_frames > 1 && last_saved_time_s > 0
                           ? (saved_frames - 1) / last_saved_time_s
                           : 0.0;
    tools::logger()->info(
      "Saved {} frames ({:.1f}s, {:.1f} effective fps) to {}", saved_frames,
      last_saved_time_s, effective_fps, record_path);
    if (saved_frames > 1 && effective_fps < record_fps * 0.9) {
      tools::logger()->warn(
        "Capture rate was below AVI rate ({:.1f} fps); use the CSV for exact frame times",
        record_fps);
    }
  }
}
