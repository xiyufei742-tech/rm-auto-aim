// 【学习导读｜同济原主流程参考，不是当前实机入口】
// 当前应运行 contest/live_pixel_*；此文件在精简版不编译。
// 原链路：相机 → 同时刻姿态 → YOLO → Tracker(内部 Solver/Target/EKF) → Aimer → CBoard。
// CBoard 是原 CAN 接口，不能直接代替当前 USB CDC 的 protocol.cpp。
// 保留此文件是为了理解完整自瞄各模块如何串起来；默认配置也属于原项目。

#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  // 这些类的构造发生在循环外，避免每帧都重新载入网络、配置和滤波状态。
  // 变量名 detector 这里对应 YOLO 包装器，不是同名的传统 Detector 类。
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    // 原项目从姿态缓存中取接近图像时刻的姿态；t-1ms 是其时间修正。
    // 当前 USB 收到角度的时间并不自动等于拍照时姿态，不能机械照搬这 1ms。
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    // Tracker 内部会调用 Solver 解每块板，再初始化或更新 Target。
    // 所以此处虽未直接写 solver.solve，完整链路仍包含 PnP。
    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    // 原接口发送的是自身定义的 Command；替换通信必须同时适配单位、协议、模式和故障处理。
    // 本循环虽然构造了 Shooter，但没有调用 shooter.shoot，不能说射击判断已经执行。
    cboard.send(command);
  }

  return 0;
}
