// 【学习导读｜YOLO 包装层】
// 主程序调用 YOLO，不直接关心模型版本。本精简副本只允许 yolo_name=yolov5。
// 构造时创建 YOLOV5；detect 把图像转交它并返回 list<Armor>。
// OpenVINO 的模型加载、推理和输出解码在 yolos/yolov5.cpp。

#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/yolov5.hpp"

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  if (yolo_name == "yolov5") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }

  else {
    throw std::runtime_error("This copy supports only yolov5: " + yolo_name);
  }
}

// std::list<Armor> 是本帧检出的装甲板集合，可能为空或有多个。
// frame_count 主要用于显示/调试，不是跟踪器给目标分配的 ID。
std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
