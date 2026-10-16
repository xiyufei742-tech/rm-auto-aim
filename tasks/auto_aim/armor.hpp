// 【学习导读｜装甲板数据结构，连接各个算法模块】
// YOLO 填二维字段：color/name/type/confidence/points/center。
// 后续 Solver 才填 xyz_in_*、ypr_in_*、ypd_in_world；当前像素主程序不使用这些三维字段。
// 像素坐标 px；三维位置 m；ypr 的 yaw/pitch/roll 是 rad；ypd 是 yaw/pitch/distance。
// 枚举 Color 的数值由代码定义，不能直接当作下位机 color/robot_id 字段使用。
// 结构体有多个构造函数，但并非每个构造函数都初始化了所有字段。
// 尤其不要假定 YOLO 输出后的 priority 已有效；接入原 Tracker 前需核查初始化路径。

#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

struct Lightbar
{
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

// Armor 表示一块本帧装甲板；Target 表示跨多帧估计的目标运动模型。
// 两者分别对应“这一帧看见什么”与“目标怎样在动”。
struct Armor
{
  Color color;
  Lightbar left, right;     //used to be const
  // 当前中心采用四角平均。透视明显时，它不一定等于真实板中心的成像位置；
  // 本阶段用它做像素靠近中心的实验，不据此宣称精确瞄准。
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio;         // 长灯条与短灯条的长度之比
  double rectangular_error;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type;
  ArmorName name;
  // YOLO 构造路径未另设优先级；先给所有未配置目标同一最低优先级，
  // 避免 Tracker 排序和 Target 初始化读取未初始化的枚举值。
  ArmorPriority priority = ArmorPriority::fifth;
  int class_id;
  cv::Rect box;
  cv::Mat pattern;
  double confidence;
  bool duplicated;

  Eigen::Vector3d xyz_in_gimbal;  // 单位：m
  Eigen::Vector3d xyz_in_world;   // 单位：m
  Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
  Eigen::Vector3d ypr_in_world;   // 单位：rad
  Eigen::Vector3d ypd_in_world;   // 球坐标系

  double yaw_raw;  // rad

  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
