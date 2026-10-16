// 【学习导读｜相机类声明】
// CameraBase 是抽象接口：virtual read(...)=0 表示具体相机必须实现取图。
// Camera 对外提供一致入口，并额外处理图像旋转。
// cv::Mat 保存图像；time_point 保存时刻；参数后的 & 表示用引用把结果交回调用者。

#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
  bool rotate_180_ = false;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
