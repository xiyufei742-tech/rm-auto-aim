// 【学习导读｜相机加识别的最短入口】
// main：读配置 → 创建 Camera/Detector/YOLO → read → detect → 显示 → 循环。
// 默认用 YOLO，只有 tradition 选项启用时才用传统整图识别。
// 构造中的 true 表示启用识别器调试显示，所以 detect 内部会显示 detection 等窗口。
// 此程序没有串口与控制命令，只检验真实图像能否得到装甲板候选。

#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/live_yolo.yaml | yaml配置文件的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, true);
  auto_aim::YOLO yolo(config_path, true);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;//列表。

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);
    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);//时间差

    // 这里统计 1/检测耗时，并非相机采集到控制输出的全链路 FPS。
    // 取图等待和下面 waitKey(33) 等耗时不在 last～now 的区间内。
    tools::logger()->info("{:.2f} fps", 1 / dt);//保留两位小数

    auto key = cv::waitKey(33);//单位ms
    if (key == 'q') break;
  }

  return 0;
}
//1.读取图像 2.判断图像是否为空 3.选择检测方法 4.获取识别结果armor 5.计算fps,检查退出键 fps是指每秒可以处理的帧数