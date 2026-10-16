// 【学习导读｜协议单元自检】
// 使用人工构造字节检查18字节指令和30字节回传，以及半帧/多帧缓存行为。
// require条件不满足就报告失败；通过代表这些已覆盖的编码/解析样例正确。
// 不接真实串口，不能由此证明设备固件已接收或电机已执行命令。

#include "protocol.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() {
    const auto require = [](bool ok, const char* message) {
        if (!ok) throw std::runtime_error(message);
    };
    using autoaim::FeedbackParser;
    const auto tx = autoaim::encodeCommand(1.25f, -0.5f, 0);
    const std::array<std::uint8_t, 18> expected = {
        16, 0, 1, 0, 0, 0, 160, 63, 0, 0, 0, 191, 2, 0, 0, 0, 0, 0
    };
    require(tx == expected, "18-byte TX does not match protocol fixture");

    // 两个完整帧拆成 1、6、其余字节送入，验证半帧和多帧。
    const std::array<std::uint8_t, 30> rx = {
        28, 0, 1, 0, 0, 0, 128, 63, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
        2, 0, 0, 0, 176, 65
    }; // yaw=1°, robot_id=3, bullet_speed=22 m/s
    FeedbackParser parser;
    require(parser.feed(rx.data(), 1).empty(), "one byte should not produce a frame");
    require(parser.pendingBytes() == 1, "partial frame should remain buffered");
    require(parser.feed(rx.data() + 1, 6).empty(), "seven bytes should not produce a frame");
    std::vector<std::uint8_t> tail(rx.begin() + 7, rx.end());
    tail.insert(tail.end(), rx.begin(), rx.end());
    const auto frames = parser.feed(tail.data(), tail.size());
    require(frames.size() == 2, "two complete RX frames expected");
    require(parser.pendingBytes() == 0, "no bytes should remain buffered");
    require(std::abs(frames[0].yaw_deg - 1.0f) < 1e-6f, "yaw decoded incorrectly");
    require(frames[0].robot_id == 3, "robot ID decoded incorrectly");
    require(std::abs(frames[0].bullet_speed_mps - 22.0f) < 1e-6f, "speed decoded incorrectly");

    const std::uint8_t invalid[] = {99, 0};
    bool rejected = false;
    try { parser.feed(invalid, sizeof(invalid)); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected && parser.pendingBytes() == 0, "invalid frame should clear buffer");
    std::cout << "official protocol: tx=18, rx=30, split/multi/invalid OK\n";
}
