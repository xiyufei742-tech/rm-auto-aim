// 【学习导读｜Ctrl+C 退出标志】
// 捕获 SIGINT 后设置退出标志，主循环通过 exiter.exit() 决定是否继续。
// 这不是强制停止所有线程：若主线程正阻塞在相机 read，需要调用返回后才检查标志。
// 此文件也没有注册 SIGTERM；不能把两种信号的处理方式混为一谈。

#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
  std::signal(SIGINT, [](int) { exit_ = true; });
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools