#include "protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace {

void printBytes(const std::uint8_t* bytes, std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
        if (i != begin) std::cout << ' ';
        std::cout << std::hex << std::setfill('0') << std::setw(2)
                  << static_cast<unsigned>(bytes[i]);
    }
    std::cout << std::dec << '\n';
}

} // namespace

int main() {
    // 调用比赛项目真正的编码函数。fire_flag=0，不请求开火。
    const auto command = autoaim::encodeCommand(1.25f, -0.5f, 0);
    std::cout << "上位机准备发送：yaw=1.25 度，pitch=-0.5 度，fire_flag=0\n";
    std::cout << "完整发送帧：";
    printBytes(command.data(), 0, command.size());
    std::cout << "  长度(2字节)：";
    printBytes(command.data(), 0, 2);
    std::cout << "  ID 1 + yaw + pitch(10字节)：";
    printBytes(command.data(), 2, 12);
    std::cout << "  ID 2 + fire_flag(6字节)：";
    printBytes(command.data(), 12, 18);

    // 按正式协议构造的一帧示例回传数据，不来自真实比赛设备。
    const std::array<std::uint8_t, 30> feedback_bytes = {
        28, 0, 1, 0, 0, 0, 128, 63, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
        2, 0, 0, 0, 176, 65
    };
    autoaim::FeedbackParser parser;
    const auto first = parser.feed(feedback_bytes.data(), 7);
    std::cout << "\n模拟第一次 read 只收到 7 字节：解析出 " << first.size()
              << " 帧，缓存 " << parser.pendingBytes() << " 字节\n";
    const auto second = parser.feed(feedback_bytes.data() + 7,
                                    feedback_bytes.size() - 7);
    std::cout << "模拟第二次 read 收到余下 23 字节：解析出 " << second.size()
              << " 帧，缓存 " << parser.pendingBytes() << " 字节\n";
    for (const auto& feedback : second) {
        std::cout << "下位机回传：yaw=" << feedback.yaw_deg
                  << " 度，pitch=" << feedback.pitch_deg
                  << " 度，roll=" << feedback.roll_deg
                  << " 度，mode=" << feedback.mode
                  << "，robot_id=" << feedback.robot_id
                  << "，bullet_speed=" << feedback.bullet_speed_mps << " m/s\n";
    }
    std::cout << "本演示只处理内存中的字节，没有打开串口或控制实车。\n";
}
