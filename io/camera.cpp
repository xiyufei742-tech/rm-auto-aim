// 【学习导读｜相机统一入口】
// 调用者只需 Camera(config_path)，然后 read(image, timestamp)。
// 本精简版仅保留现场使用的 HikRobot；真正 SDK 调用在 hikrobot/hikrobot.cpp。
// YAML 中 exposure_ms、gain、vid_pid 配置硬件，rotate_180 配置取图后的软件旋转。

#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  // 从磁盘加载 YAML。config_path 是文件路径，不是相机硬件编号。
  auto yaml = tools::load(config_path);
  rotate_180_ = yaml["rotate_180"] ? yaml["rotate_180"].as<bool>() : false;
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    // 构造 HikRobot 对象会启动其后台线程；unique_ptr 持有对象，离开作用域自动析构。
    // 上层仍通过 CameraBase 接口调用 read，避免每个主程序都直接写 SDK 代码。
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("This copy supports only Hikrobot cameras: " + camera_name);
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  // 两个引用参数是输出：img 接收图像，timestamp 接收本机取帧时刻。
  // 旋转发生在此之后，所有经 io::Camera 取图的识别程序都看到旋转后的画面。
  camera_->read(img, timestamp);
  // 180°旋转同时改变 x、y 朝向；它不会交换红蓝通道。
  // 正式标定应与实际送入算法的图像方向、分辨率保持一致。
  if (rotate_180_ && !img.empty()) cv::rotate(img, img, cv::ROTATE_180);
}

}  // namespace io
