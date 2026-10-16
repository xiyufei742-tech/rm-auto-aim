// 【学习导读｜后续参考：已编译并离线检查，尚未用于实机发射】
// 简化弹道类声明：unsolvable 表示模型下无解；pitch 为 rad，fly_time 为 s。
// 详见 trajectory.cpp 的物理假设、二次方程与输入边界说明。

#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
struct Trajectory
{
  // 无效输入、没有前向物理解或结果无法用 double 表示时，保持此失败状态。
  // 调用者必须先检查 unsolvable，不能把失败时的 0 当成射击参数。
  bool unsolvable = true;
  double fly_time = 0.0;  // s，失败时也确定初始化
  double pitch = 0.0;     // rad，抬头为正

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m，必须 > 0；此接口不支持 d=0 的垂直射击
  // h 目标相对枪口的竖直高度，单位：m，允许为负
  // 返回两个抛物线解中飞行时间较短的解；这不是实机弹道标定。
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP
