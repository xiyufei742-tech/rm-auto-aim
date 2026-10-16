// 【学习导读｜海康 SDK 驱动】
// 三方协作：守护线程负责连接/重试；采集线程从 SDK 拿图；主线程通过队列读图。
// SDK 原始缓冲区 → Bayer/RGB/BGR 转换 → OpenCV BGR 图像 → 队列 → Camera::read。
// exposure_ms 在构造时乘 1000 转成 SDK 的微秒。图像颜色修正在 type_map 附近。
// 此处 timestamp 是本机成功获取 SDK 缓冲区后的时刻，并非硬件曝光起始时间。

#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include <cstddef>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(double exposure_ms, double gain, const std::string & vid_pid)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  // [this] 让线程 lambda 可以访问本对象成员。守护线程先尝试启动采集，
  // 若 capturing_ 变为 false，则停止旧资源、重置 USB、重试连接。
  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

// 析构先请求守护线程退出再 join 等它结束，避免对象销毁后线程还访问成员。
// join 不是强杀线程；底层调用卡住时退出仍可能延迟。
HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  // 这里可能阻塞，直到采集线程放入数据。
  // 当前 queue_(1) 使用默认 PopWhenFull=false：队列满时丢弃新来的帧，
  // 保留已排队的那帧，不能把此实现描述成“永远覆盖成最新图”。
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list{};
  // 枚举可用 USB 相机。当前随后打开设备列表第一个相机；vid_pid 用于 USB 重置，
  // 本代码没有根据序列号从多台海康相机中挑选指定相机。
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  // 句柄 handle_ 是 SDK 管理相机连接的标识：枚举 → 创建句柄 → 打开设备 → 开始取流。
  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }
  device_open_ = true;

  // 自动白平衡持续启用；曝光和增益改为手动设值。
  // 设置帧率 150 是请求值，实际取帧率仍受曝光、USB、硬件与处理耗时影响。
  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  MV_CC_SetFrameRate(handle_, 150);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }
  grabbing_ = true;

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw{};
    bool pixel_type_logged = false;
    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      // 向 SDK 借一块图像缓冲区，最长等待 nMsec=100 ms。
      // 读完必须 FreeImageBuffer 归还；需要长期持有的图像要拥有自己的像素存储。
      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      if (!pixel_type_logged) {
        tools::logger()->info("HikRobot pixel type: {:#x}", pixel_type);
        pixel_type_logged = true;
      }
      // OpenCV's legacy two-letter Bayer names use the opposite corner convention.
      // Bayer 原始图每个像素只采一种颜色，要经去马赛克恢复 BGR 三通道。
      // 这里根据 SDK 像素格式选 OpenCV 转换码；历史两字母 Bayer 命名容易对应反。
      // 这正是现场“蓝黄颜色异常”需要检查的驱动映射，而不是调 YOLO 类别。
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerBG2BGR},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerRG2BGR}};
      cv::Mat img;
      auto bayer_conversion = type_map.find(pixel_type);
      const auto expected_pixels =
        static_cast<std::size_t>(frame_info.nWidth) * frame_info.nHeight;
      const auto expected_bytes =
        expected_pixels *
        ((pixel_type == PixelType_Gvsp_BGR8_Packed || pixel_type == PixelType_Gvsp_RGB8_Packed)
           ? 3
           : 1);
      if (expected_pixels == 0 || frame_info.nFrameLen < expected_bytes) {
        tools::logger()->warn(
          "Invalid HikRobot frame size: {} bytes for {}x{}", frame_info.nFrameLen,
          frame_info.nWidth, frame_info.nHeight);
      } else if (bayer_conversion != type_map.end()) {
        cv::Mat raw_bayer(frame_info.nHeight, frame_info.nWidth, CV_8UC1, raw.pBufAddr);
        cv::cvtColor(raw_bayer, img, bayer_conversion->second);
      // 若 SDK 已提供 BGR，就直接 clone；若是 RGB，则交换到 OpenCV 常用的 BGR。
      // clone 防止 SDK 缓冲区归还后，图像对象仍引用被重用的内存。
      } else if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
        img = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC3, raw.pBufAddr).clone();
      } else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat raw_rgb(frame_info.nHeight, frame_info.nWidth, CV_8UC3, raw.pBufAddr);
        cv::cvtColor(raw_rgb, img, cv::COLOR_RGB2BGR);
      } else {
        tools::logger()->warn("Unsupported HikRobot pixel type: {:#x}", pixel_type);
      }

      if (!img.empty()) queue_.push({img, timestamp});

      // 归还借用缓冲区。上面的图像转换/clone 已把有效画面放到自有内存。
      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

// 停止采集并回收线程/句柄。布尔标志记录哪些步骤成功过，供退出和重连时判断。
void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (!handle_) return;
  unsigned int ret;

  if (grabbing_) {
    ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    grabbing_ = false;
  }

  if (device_open_) {
    ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK) tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    device_open_ = false;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
  handle_ = nullptr;
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

// USB 重置是连接故障后的恢复动作，不是目标识别算法。
// 如果两个程序同时占相机，这里的反复重试不能替代结束另一个占用进程。
void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
