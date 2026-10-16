// 【学习导读｜报文字节布局，小端编码】
// 发送 18 字节（偏移从 0 算）：
//   0..1 body_len=16；2..3 ID1=1；4..7 yaw(float)；8..11 pitch(float)；
//   12..13 ID2=2；14..17 fire_flag(int32)。body_len 不含自身的两个字节。
// 回传 30 字节：
//   0..1 body_len=28；2..3 ID1=1；4..7 yaw；8..11 pitch；12..15 roll；
//   16..19 mode(int32)；20..23 color/robot_id(int32)；24..25 ID2=2；26..29 弹速(float)。
// ID1、ID2 是同一帧中的两段记录标识，不要求分两次 USB 传输。
// 下位机、USB、操作系统可能把字节合并或拆开，read 的边界不等于协议帧边界。

#include "protocol.hpp"

#include <cstring>
#include <stdexcept>

namespace autoaim {
namespace {

// 小端：低 8 位写在较小偏移。数值 28 写成十六进制字节 1c 00。
// 右移后取 uint8_t，只保留该字节位置的 8 位。
void putU16(std::uint8_t* p, std::uint16_t value) {
    p[0] = static_cast<std::uint8_t>(value);
    p[1] = static_cast<std::uint8_t>(value >> 8);
}

void putU32(std::uint8_t* p, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>(value >> (8 * i));
}

// 保留浮点数的二进制位型，再按小端写出。
// memcpy 不是把 22.8 转成整数 22，而是复制 float 的 32 个表示位。
void putF32(std::uint8_t* p, float value) {
    static_assert(sizeof(float) == 4, "protocol requires float32");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    putU32(p, bits);
}

std::uint16_t getU16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0]) |
           (static_cast<std::uint16_t>(p[1]) << 8);
}

std::uint32_t getU32(const std::uint8_t* p) {
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) value |= static_cast<std::uint32_t>(p[i]) << (8 * i);
    return value;
}

float getF32(const std::uint8_t* p) {
    const std::uint32_t bits = getU32(p);
    float value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

// p 是帧首地址，p+4 意为从第 4 个字节偏移开始读取。
// 解码只保证布局正确；有限数与角度范围还由现场串口层检查。
Feedback parseFrame(const std::uint8_t* p) {
    // 长度字段已由调用者检查。偏移严格按正式文档的 30 字节布局。
    if (getU16(p + 2) != 1 || getU16(p + 24) != 2) {
        throw std::runtime_error("unexpected record ID");
    }
    Feedback feedback;
    feedback.yaw_deg = getF32(p + 4);
    feedback.pitch_deg = getF32(p + 8);
    feedback.roll_deg = getF32(p + 12);
    feedback.mode = static_cast<std::int32_t>(getU32(p + 16));
    feedback.robot_id = static_cast<std::int32_t>(getU32(p + 20));
    feedback.bullet_speed_mps = getF32(p + 26);
    return feedback;
}

} // namespace

// 创建固定大小数组并逐字段填入，不依赖 C++ struct 的内存排布。
// 调用者可传 fire_flag，本项目的无弹现场程序调用时固定传 0。
std::array<std::uint8_t, 18> encodeCommand(float yaw_deg, float pitch_deg,
                                            std::int32_t fire_flag) {
    std::array<std::uint8_t, 18> frame{};
    putU16(frame.data(), 16);
    putU16(frame.data() + 2, 1);
    putF32(frame.data() + 4, yaw_deg);
    putF32(frame.data() + 8, pitch_deg);
    putU16(frame.data() + 12, 2);
    putU32(frame.data() + 14, static_cast<std::uint32_t>(fire_flag));
    return frame;
}

// 例如先收到 10 字节：存入 buffer_，不足 30 就等下一次。
// 再收到 50 字节：共 60 字节，循环解析两帧并返回长度为 2 的 vector。
// 每解析一帧只移除前 30 字节，末尾半帧留待后续输入。
std::vector<Feedback> FeedbackParser::feed(const std::uint8_t* bytes,
                                           std::size_t count) {
    if (count != 0 && bytes == nullptr) throw std::invalid_argument("null data");
    if (count != 0) buffer_.insert(buffer_.end(), bytes, bytes + count);
    std::vector<Feedback> frames;
    while (buffer_.size() >= 2) {
        const auto body_len = getU16(buffer_.data());
        // 若帧首长度不符合协议，清空缓存并报告错误；不能把不明字节硬解释为角度。
        if (body_len != 28) {
            buffer_.clear();
            throw std::runtime_error("invalid feedback body_len; buffer cleared");
        }
        if (buffer_.size() < 30) break;
        try {
            frames.push_back(parseFrame(buffer_.data()));
        } catch (...) {
            buffer_.clear();
            throw;
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + 30);
    }
    return frames;
}

void FeedbackParser::clear() { buffer_.clear(); }
std::size_t FeedbackParser::pendingBytes() const { return buffer_.size(); }

} // namespace autoaim
