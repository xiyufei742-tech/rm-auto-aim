#!/usr/bin/env python3
# 【学习导读｜现场 Python 串口读取/无弹保持】
# 默认只读，--hold 才允许写入；所有角度是度。两份脚本保留现场阶段文件名。
# 读取路径：receive→FeedbackStream.feed→sample/show；保持路径：hold→stable_target→write_all。
# 无弹保持用于检查协议、绝对角度单位和模式切换会不会造成意外跳转，不做视觉识别。
# 此旧版脚本没有 follow 的完整状态机；模式切换由现场人员按提示进行。

"""SRM 比赛下位机现场串口检查：默认只读，可选无弹保持当前角度。"""

import argparse
from collections import deque
import math
import os
import select
import struct
import sys
import termios
import time
import tty


FRAME_PREFIX = b"\x1c\x00\x01\x00"


# 缓存串口字节并按30字节帧解析；一次读出的长度不要求正好等于一帧。
# recent 保存近来的角度样本，供 stable_target 判断是否已经停稳。
class FeedbackStream:
    def __init__(self):
        self.buffer = bytearray()
        self.latest = None
        self.last_received = 0.0
        self.recent = deque(maxlen=2048)

    def feed(self, data):
        self.buffer.extend(data)
        while True:
            offset = self.buffer.find(FRAME_PREFIX)
            if offset < 0:
                del self.buffer[:-3]
                return
            del self.buffer[:offset]
            if len(self.buffer) < 30:
                return
            if self.buffer[24:26] != b"\x02\x00":
                del self.buffer[0]
                continue
            # < 表示小端，f是4字节float，i是4字节有符号整数；从偏移4开始读。
            # 长度/ID检查和数值范围筛选不是CRC，仍不能证明损坏帧绝不漏过。
            yaw, pitch, roll, mode, robot_id = struct.unpack_from("<fffii", self.buffer, 4)
            speed = struct.unpack_from("<f", self.buffer, 26)[0]
            del self.buffer[:30]
            if not all(math.isfinite(v) for v in (yaw, pitch, roll, speed)):
                continue
            if abs(yaw) > 720 or abs(pitch) > 180 or abs(roll) > 180:
                continue
            self.latest = (yaw, pitch, roll, mode, robot_id, speed)
            self.last_received = time.monotonic()
            self.recent.append((self.last_received, yaw, pitch))


def receive(fd, stream, timeout):
    if select.select([fd], [], [], timeout)[0]:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            return
        if data:
            stream.feed(data)


def sample(fd, stream, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        receive(fd, stream, min(0.05, max(0.0, end - time.monotonic())))
    if stream.latest is None:
        raise RuntimeError("没有收到符合比赛协议的 30 字节回传")
    return stream.latest


def show(label, feedback):
    yaw, pitch, roll, mode, robot_id, speed = feedback
    print(f"{label}: yaw={yaw:.3f}° pitch={pitch:.3f}° roll={roll:.3f}° "
          f"mode={mode} robot_id={robot_id} bullet_speed={speed:.3f}m/s", flush=True)


# 非阻塞串口可能一次只接受部分字节；memoryview 切片保留尚未写完部分。
# 调用完成表示交给操作系统，不等于电机已到目标角度。
def write_all(fd, data):
    remaining = memoryview(data)
    while remaining:
        if not select.select([], [fd], [], 0.5)[1]:
            raise RuntimeError("串口写入超时")
        count = os.write(fd, remaining)
        if count == 0:
            raise RuntimeError("串口没有接受发送数据")
        remaining = remaining[count:]


# 最近0.8s至少5份样本、覆盖至少0.2s，yaw和pitch极差不超过1°。
# 这是启动前的粗稳定门槛，不是控制精度的合格标准。
def stable_target(stream):
    now = time.monotonic()
    samples = [(t, yaw, pitch) for t, yaw, pitch in stream.recent if now - t <= 0.8]
    if len(samples) < 5 or samples[-1][0] - samples[0][0] < 0.2:
        raise RuntimeError("回传帧不足，不能确认当前角度稳定")
    yaws = [yaw for _, yaw, _ in samples]
    pitches = [pitch for _, _, pitch in samples]
    if max(yaws) - min(yaws) > 1 or max(pitches) - min(pitches) > 1:
        raise RuntimeError("云台还在运动，先停稳再试")
    if abs(yaws[-1]) > 180 or abs(pitches[-1]) > 60:
        raise RuntimeError("当前角度超出首次无弹保持测试的保守范围")
    return stream.latest


def hold(fd, stream):
    show("当前", sample(fd, stream, 1.0))
    print("保持当前角度测试：确认无弹，遥控器先保持手动档。回车后开始发送；"
          "看到“发送中”再切视觉档。先切回手动档，再按回车结束。最长 20 秒。", flush=True)
    input()
    termios.tcflush(fd, termios.TCIFLUSH)
    stream.buffer.clear()
    stream.latest = None
    stream.recent.clear()
    sample(fd, stream, 0.8)
    start = stable_target(stream)
    target_yaw, target_pitch = start[:2]
    # 按比赛18字节布局打包，末尾0是fire_flag。yaw/pitch固定取启动时回传值。
    # “保持”不发送0°，而是发送云台此刻已经处于的绝对角度。
    frame = struct.pack("<hhffhi", 16, 1, target_yaw, target_pitch, 2, 0)
    assert len(frame) == 18
    show("保持目标", start)
    print("发送中：fire_flag=0，yaw/pitch 固定为当前回传角度", flush=True)
    end = time.monotonic() + 20.0
    next_send = 0.0
    next_print = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_send:
            write_all(fd, frame)
            next_send = now + 0.05
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        receive(fd, stream, 0.02)
        if time.monotonic() - stream.last_received > 1.0:
            raise RuntimeError("回传中断超过 1 秒；请切回手动档")
        if stream.latest is not None:
            yaw, pitch = stream.latest[:2]
            if abs(yaw - target_yaw) > 3 or abs(pitch - target_pitch) > 3:
                raise RuntimeError("云台偏离保持目标超过 3°；请切回手动档")
            if now >= next_print:
                show("回传", stream.latest)
                next_print = now + 1.0
    print("发送结束。若遥控器仍在视觉档，请立即切回手动档。", flush=True)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", default="/dev/ttyACM0")
    cli.add_argument("--hold", action="store_true", help="显式启用无弹保持当前角度测试")
    args = cli.parse_args()
    # 默认只读打开设备，从文件访问权限上防止误写；显式--hold才可写。
    access = os.O_RDWR if args.hold else os.O_RDONLY
    fd = os.open(args.port, access | os.O_NOCTTY | os.O_NONBLOCK)
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        stream = FeedbackStream()
        if args.hold:
            hold(fd, stream)
        else:
            show("最新回传", sample(fd, stream, 2.0))
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, saved)
        os.close(fd)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, EOFError, KeyboardInterrupt) as error:
        print(f"串口检查失败：{error}", file=sys.stderr)
        print("若遥控器仍在视觉档，请立即切回手动档。", file=sys.stderr)
        sys.exit(1)
