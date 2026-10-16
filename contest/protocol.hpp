// 【学习导读｜协议接口声明】
// .hpp 声明“可以调用什么”，对应 .cpp 实现“怎么把字节拼出来”。
// 上位机是运行视觉程序的 NUC；下位机接收目标角度并控制电机，再回传实测状态。
// 这里的角度是度（deg），弹速是米/秒（m/s）；开源 Aimer 内部使用弧度，接入时须转换。
// 本接口只处理报文布局；它不会打开 /dev/ttyACM0，也不负责决定应该瞄准哪里。

#pragma once

#include <array>//固定数组长度
#include <cstddef>
#include <cstdint>
#include <vector>//动态数组

namespace autoaim {

// SRM 校内赛正式 USB CDC 协议。帧内无魔数、CRC 或时间戳。
// 字段顺序见 protocol.cpp 的偏移表。float 是 4 字节，int32_t 是固定 4 字节整数。
// 不要直接把整个结构体内存发出去：编译器可能插入对齐填充字节。
struct Feedback {
    float yaw_deg = 0;
    float pitch_deg = 0;
    float roll_deg = 0;
    std::int32_t mode = 0;
    std::int32_t robot_id = 0; // 协议字段名 color，当前固件语义为机器人 ID
    float bullet_speed_mps = 0;
};//都是初始值设定

// 上位机发送帧固定 18 字节：body_len=16, ID1 yaw/pitch, ID2 fire_flag。
std::array<std::uint8_t, 18> encodeCommand(float yaw_deg, float pitch_deg,
                                            std::int32_t fire_flag);//roll：横滚 yaw 左右转动 pitch：上下转动
//意思就是接收三个控制参数，编码成18字节的数据，供程序发给下位机  encodeCommand就是编码命令

// USB CDC 读取可能返回半帧、多帧；feed 保留未完成帧。
// 本协议无可靠逐字节重同步依据，非法帧时清空缓存。
// 本解析器期望缓存起点就是长度字段；无法保证任意字节位置上的重同步。
// 现场程序的 FieldSerial 在外层搜索固定布局特征，再把候选完整帧交进来。
class FeedbackParser {
public:
    std::vector<Feedback> feed(const std::uint8_t* bytes, std::size_t count);//接收原始字节和字节数量，返回一个装有feedback的动态数组
    //这里是<>返回类型的意思
    void clear();//清除解析器缓存
    std::size_t pendingBytes() const;//返回当前缓存中未解析的字节数

private:
    std::vector<std::uint8_t> buffer_;//缓存区，假设一帧的反馈数据要24字节，第一次传来10,就先存在buffer区域
};

} // namespace autoaim
