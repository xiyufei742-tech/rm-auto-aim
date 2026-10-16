// 【学习导读｜原开源的内部命令结构】
// control表示是否有有效控制目标，shoot表示是否请求射击，yaw/pitch在原Aimer中用rad。
// 这是内存中的业务结构，不是比赛USB线上的18字节格式，禁止直接发送sizeof(Command)字节。
// 当前contest现场程序使用protocol.cpp独立打包度制角度，fire_flag固定0。

#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

}  // namespace io

#endif  // IO__COMMAND_HPP