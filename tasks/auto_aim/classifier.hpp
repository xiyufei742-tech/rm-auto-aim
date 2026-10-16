// 【学习导读｜数字分类器接口】
// classify 使用 OpenCV net_；ovclassify 使用 OpenVINO compiled_model_。
// 传入 Armor&，结果写回 name/confidence；不是创建新的几何目标。

#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);

  void classify(Armor & armor);

  void ovclassify(Armor & armor);

private:
  cv::dnn::Net net_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP