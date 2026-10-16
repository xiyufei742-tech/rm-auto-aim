# SRM 校内赛自瞄通信协议

**状态：** 最终协议  
**协议来源：** `control-2026` 的 `infantry_leg&prostrate` 分支  
**参考提交：** `f7b83a58e8dc8af13ae5f328687c2fd7462e8b80`  
**通信介质：** USB CDC 虚拟串口（Linux 端通常表现为 `/dev/ttyACM*`）

本文定义自瞄上位机与下位机之间的应用层报文。下位机固件使用 STM32 USB CDC；USB CDC 不使用固定的波特率、校验位或停止位设置，串口设备的打开参数由 USB 驱动处理。

## 1. 报文总览

协议由一个 2 字节长度字段和若干条数据记录组成：

```text
┌────────────────────┬─────────────────────────────────────────────┐
│ body_len (int16)   │ records                                     │
│ 2 bytes, little-end│ body_len bytes                              │
└────────────────────┴─────────────────────────────────────────────┘
```

- 所有整数和浮点数均采用小端字节序。
- `body_len` 表示后续 `records` 的总字节数，不包含自身的 2 字节。
- 每条记录由 `int16 id` 和该 ID 对应的固定长度数据体组成。
- 当前比赛只使用 ID 1 和 ID 2，并要求每帧按 ID 1、ID 2 的顺序发送。
- 协议没有帧头、帧尾、CRC、时间戳或序列号；长度字段是唯一的逻辑帧边界。
- USB 包边界不等于协议帧边界。上位机读取 USB 设备时必须允许一次读取包含半帧、整帧或多帧。

本比赛使用的帧长度固定如下：

| 方向 | `body_len` | 总帧长 | 记录顺序 |
| --- | ---: | ---: | --- |
| 上位机 → 下位机 | 16 | 18 | ID 1、ID 2 |
| 下位机 → 上位机 | 28 | 30 | ID 1、ID 2 |

USB 全速端点单包上限为 64 字节，因此上述两种完整帧均可放入一个 USB 包；上位机仍应使用流式解析，不能依赖操作系统一次返回一帧。

上位机应维护持久接收缓冲区：先读取 2 字节 `body_len`，待缓冲区达到 `2 + body_len` 字节后再解析完整帧，处理后保留后续字节。由于协议没有魔数和 CRC，遇到非法长度或未知 ID 时无法可靠地在任意字节处重同步，应丢弃当前缓存并等待下一次完整帧。

## 2. 基本数据类型

| 类型 | 长度 | 编码 |
| --- | ---: | --- |
| `int16` | 2 字节 | 有符号、二进制补码、小端 |
| `int32` | 4 字节 | 有符号、二进制补码、小端；对应分支中的 C `int` |
| `float32` | 4 字节 | IEEE-754 单精度、小端；对应分支中的 C `float` |

结构体必须按 1 字节对齐。

## 3. 上位机发送给下位机

上位机发送目标云台姿态和视觉开火请求。每帧必须包含两条记录。

### 3.1 ID 1：云台目标

数据体为 `Gimbal_Receive_s`，长度 8 字节。

| 数据体偏移 | 字段 | 类型 | 单位/含义 |
| ---: | --- | --- | --- |
| 0 | `yaw` | `float32` | 云台 yaw 目标，角度制 |
| 4 | `pitch` | `float32` | 云台 pitch 目标，角度制 |

下位机在视觉控制生效时把这两个值写入云台目标。角度范围受车辆固件的机械限位和控制逻辑约束，协议本身不再重复定义限位。

### 3.2 ID 2：开火请求

数据体为 `Shoot_Receive_s`，长度 4 字节。

| 数据体偏移 | 字段 | 类型 | 含义 |
| ---: | --- | --- | --- |
| 0 | `fire_flag` | `int32` | `0` 表示不请求开火；非零表示请求开火 |

`fire_flag` 只是视觉侧请求。实际发射还必须满足下位机火控状态、摩擦轮状态和人工许可等安全条件；非零值不会绕过这些条件。

### 3.3 上位机发送帧布局

```text
body_len = 16

ID 1 (int16, 1)  + yaw (float32) + pitch (float32)
ID 2 (int16, 2)  + fire_flag (int32)
```

对应的 C 结构体可写为：

```c
#pragma pack(push, 1)
typedef struct {
  float yaw;
  float pitch;
} Gimbal_Receive_s;

typedef struct {
  int32_t fire_flag;
} Shoot_Receive_s;
#pragma pack(pop)
```

## 4. 下位机发送给上位机

下位机发送当前云台姿态、模式/身份字段和裁判系统提供的弹丸初速度。每帧必须包含两条记录。

### 4.1 ID 1：云台状态

数据体为 `Gimbal_Send_s`，长度 20 字节。

| 数据体偏移 | 字段 | 类型 | 单位/含义 |
| ---: | --- | --- | --- |
| 0 | `yaw` | `float32` | 当前 yaw，角度制 |
| 4 | `pitch` | `float32` | 当前 pitch，角度制 |
| 8 | `roll` | `float32` | 当前 roll，角度制 |
| 12 | `mode` | `int32` | 当前分支固定发送 `0`，保留字段 |
| 16 | `color` | `int32` | 当前分支写入机器人 ID；字段名沿用源码 |

`mode` 和 `color` 不应被上位机解释为其他枚举。若后续固件重新定义这两个字段，必须同步更新协议版本。

### 4.2 ID 2：发射状态

数据体为 `Shoot_Send_s`，长度 4 字节。

| 数据体偏移 | 字段 | 类型 | 单位/含义 |
| ---: | --- | --- | --- |
| 0 | `bullet_speed` | `float32` | 裁判系统提供的弹丸初速度，m/s |

在参考实现中，裁判系统数据通过 `VisionSetRefereeData()` 注入视觉通信模块；若尚未注入，模块内部默认值为 `22.0`。

### 4.3 下位机发送帧布局

```text
body_len = 28

ID 1 (int16, 1)  + yaw (float32) + pitch (float32) + roll (float32)
                 + mode (int32) + color (int32)
ID 2 (int16, 2)  + bullet_speed (float32)
```

对应的 C 结构体可写为：

```c
#pragma pack(push, 1)
typedef struct {
  float yaw;
  float pitch;
  float roll;
  int32_t mode;
  int32_t color;
} Gimbal_Send_s;

typedef struct {
  float bullet_speed;
} Shoot_Send_s;
#pragma pack(pop)
```

## 5. 开火与控制约束

1. 上位机将目标角度写入 ID 1，将视觉开火请求写入 ID 2。
2. `fire_flag = 0` 时不得产生视觉开火请求；非零值表示视觉侧允许请求开火。
3. 参考火控逻辑要求人工发射许可与视觉 `fire_flag` 同时满足，才会产生装弹/发射动作。
4. 上位机不得通过发送异常 ID、超长帧或高频重复帧绕过下位机的安全逻辑。
5. 上位机应持续发送完整的 ID 1 + ID 2 帧；省略某条记录时，参考下位机不会自动清零旧字段。

## 6. 与源码的对应关系

| 协议内容 | 参考源码 |
| --- | --- |
| 数据结构、ID 注册 | `Modules/master_machine/master_process.h`、`master_process.c` |
| 长度字段和记录打包/解析 | `Modules/master_machine/srm_protocol.c`、`srm_protocol.h` |
| USB CDC 虚拟串口 | `Bsp/usb/bsp_usb.c`、`Bsp/usb/bsp_usb.h` |
| USB 接收回调和 2048 字节缓存 | `Hardware/stm32-f4/Src/usbd_cdc_if.c`、`Inc/usbd_cdc_if.h` |
| 17 mm 弹丸速度和火控转发 | `UserApp/robot/infantry_wheel_legged_sjtu/comm/comm.c`、`ctrl/ctrl.c` |
