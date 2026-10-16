// 【学习导读｜传统识别里的数字分类器】
// 输入已裁出的 armor.pattern，输出 armor.name 与分类置信度。
// classify 走 OpenCV DNN，ovclassify 走 OpenVINO；本文件两条推理路径是不同函数。
// YOLOv5 本身也输出数字类别；不能说每次 YOLO 后都会再调用这个分类网络。
// 当前虽然不走传统识别，构造链 YOLOV5→Detector→Classifier 仍会加载 tiny_resnet。

#include "classifier.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
// 同一 ONNX 分别载入 OpenCV 和 OpenVINO 对象；模型路径由 classify_model 指定。
// 构造加载了模型，不等于该模型在每一帧都执行推理。
Classifier::Classifier(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto model = yaml["classify_model"].as<std::string>();
  net_ = cv::dnn::readNetFromONNX(model);
  auto ovmodel = core_.read_model(model);
  compiled_model_ = core_.compile_model(
    ovmodel, "AUTO", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

void Classifier::classify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

  // 灰度图保持比例缩放，放进32×32黑底；后面把像素转成0～1附近的网络输入。
  // 分类网络输入与 YOLO 的640×640三通道输入不同。
  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(gray, input(roi), {w, h});

  auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());

  net_.setInput(blob);
  cv::Mat outputs = net_.forward();

  // softmax
  // 数值稳定的 softmax：先减去最大值，再指数化并归一化。
  // 最后取最高分的类别；输出分数不等于现场测量得到的总体准确率。
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}

void Classifier::ovclassify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

  // Resize image to 32x32
  // 灰度图保持比例缩放，放进32×32黑底；后面把像素转成0～1附近的网络输入。
  // 分类网络输入与 YOLO 的640×640三通道输入不同。
  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }

  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(gray, input(roi), {w, h});
  // Normalize the input image to [0, 1] range
  input.convertTo(input, CV_32F, 1.0 / 255.0);

  ov::Tensor input_tensor(ov::element::f32, {1, 1, 32, 32}, input.data);

  ov::InferRequest infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat outputs(1, 9, CV_32F, output_tensor.data());

  // Softmax
  // 数值稳定的 softmax：先减去最大值，再指数化并归一化。
  // 最后取最高分的类别；输出分数不等于现场测量得到的总体准确率。
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}

}  // namespace auto_aim