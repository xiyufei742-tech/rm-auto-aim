#!/usr/bin/env python3
# 【学习导读｜固定 +0.5° yaw 阶跃】
# 复用 field_serial_hold_safe 的收包/稳定性/写入函数，程序本身不使用相机。
# 先保持基线 → 人员确认视觉档 → +0.5°持续3s → 基线3s → 人员切回手动。
# 用于验证上位机给出的目标确实被下位机执行，以及单位/符号/量级是否对应。
# 当前 Python 入口会打开可写串口；没有和 C++ 主程序一样的默认只读模式。

"""现场无弹测试：视觉档 yaw 小幅移动 0.5°，随后回到起始角度。"""

import argparse
import math
import os
import select
import struct
import sys
import termios
import time
import tty

from field_serial_hold_safe import FeedbackStream, receive, sample, show, stable_target, write_all


STEP_DEGREES = 0.5
SEND_INTERVAL = 0.05


# 将绝对 yaw/pitch 封装成18字节，fire_flag恒为0；函数本身只生成 bytes。
def command(yaw, pitch):
    frame = struct.pack("<hhffhi", 16, 1, yaw, pitch, 2, 0)
    assert len(frame) == 18
    return frame


# 给定一帧固定目标，在一段时间内反复发它并读反馈，约50ms安排一次发送。
# 统计真实 yaw 最小/最大值，供实验结束后与起始值比较。
def drive(fd, stream, frame, baseline, seconds, label, wait_for_enter=False):
    deadline = time.monotonic() + seconds
    next_send = 0.0
    next_print = 0.0
    yaw_min = math.inf
    yaw_max = -math.inf
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - stream.last_received > 0.3:
            raise RuntimeError("最新回传超过 0.3 秒；立即切回手动档")
        if now >= next_send:
            write_all(fd, frame)
            next_send = now + SEND_INTERVAL
        if wait_for_enter and select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return yaw_min, yaw_max, True
        receive(fd, stream, 0.02)
        if time.monotonic() - stream.last_received > 1.0:
            raise RuntimeError("回传中断超过 1 秒；立即切回手动档")
        if stream.latest is not None:
            yaw, pitch = stream.latest[:2]
            if abs(yaw - baseline[0]) > 3 or abs(pitch - baseline[1]) > 3:
                raise RuntimeError("偏离起始角度超过 3°；立即切回手动档")
            yaw_min = min(yaw_min, yaw)
            yaw_max = max(yaw_max, yaw)
            if now >= next_print:
                show(label, stream.latest)
                next_print = now + 0.5
    return yaw_min, yaw_max, False


def run(fd, stream):
    show("当前", sample(fd, stream, 1.0))
    print("确认无弹、云台附近无人，操作手保持手动档。按回车后开始发送起始角度。", flush=True)
    input()
    termios.tcflush(fd, termios.TCIFLUSH)
    stream.buffer.clear()
    stream.latest = None
    stream.recent.clear()
    sample(fd, stream, 0.8)
    initial = stable_target(stream)
    baseline_yaw, baseline_pitch = initial[:2]
    if abs(baseline_yaw) > 175:
        raise RuntimeError("当前 yaw 太靠近安全边界，不做阶跃测试")
    baseline = command(baseline_yaw, baseline_pitch)
    # 只在基线yaw上加0.5°，pitch原样保持；这里与像素公式没有关系。
    step = command(baseline_yaw + STEP_DEGREES, baseline_pitch)
    show("起始目标", initial)
    print("发送中：fire_flag=0。请切入视觉档，确认云台稳定后按回车；最多等待 20 秒。", flush=True)
    try:
        _, _, confirmed = drive(fd, stream, baseline, initial, 20.0, "基线", True)
        if not confirmed:
            raise RuntimeError("没有收到开始阶跃的回车；立即切回手动档")
        drive(fd, stream, baseline, initial, 0.8, "稳定")
        current = stable_target(stream)
        if abs(current[0] - baseline_yaw) > 1 or abs(current[1] - baseline_pitch) > 1:
            raise RuntimeError("云台没有停在起始位置；立即切回手动档")
        # 回车是人的确认，mode=1是设备回传的确认，两者都要检查。
        # 这一步之前发送保持，不代表此前已经执行了阶跃。
        if current[3] != 1:
            raise RuntimeError("现场回传 mode 不为 1，视觉档未确认；立即切回手动档")
        print(f"阶跃：yaw 目标 +{STEP_DEGREES:.1f}°，持续 3 秒；不要碰云台。", flush=True)
        yaw_min, yaw_max, _ = drive(fd, stream, step, initial, 3.0, "阶跃")
        print("回到起始目标，持续 3 秒。", flush=True)
        drive(fd, stream, baseline, initial, 3.0, "回归")
        print(f"阶跃期间 yaw 相对起始值的范围："
              f"{yaw_min - baseline_yaw:+.3f}° 至 {yaw_max - baseline_yaw:+.3f}°", flush=True)
        print("请先切回手动档，再按回车结束；最多继续发送起始目标 20 秒。", flush=True)
        _, _, confirmed = drive(fd, stream, baseline, initial, 20.0, "等待手动", True)
        if not confirmed:
            raise RuntimeError("等待手动档超时；立即切回手动档")
        print("测试结束。", flush=True)
    # 异常或正常退出都尝试短暂重发基线；回传过旧时停止这一尝试。
    # 这是尽力处理，并非所有设备/通信故障下都保证回位。
    finally:
        # 若中途退出，尽可能用短暂的起始目标覆盖最近的阶跃目标。
        for _ in range(4):
            if time.monotonic() - stream.last_received > 0.3:
                break
            try:
                write_all(fd, baseline)
            except (OSError, RuntimeError):
                break
            time.sleep(SEND_INTERVAL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    args = parser.parse_args()
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        run(fd, FeedbackStream())
    # 外层 finally 恢复串口原设置并关闭文件描述符。
    # 上面的 run 内层 finally 才负责尝试重发基线；两个清理阶段职责不同。
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, saved)
        os.close(fd)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, EOFError, KeyboardInterrupt) as error:
        print(f"测试中止：{error}", file=sys.stderr)
        print("若仍在视觉档，请立即切回手动档。", file=sys.stderr)
        sys.exit(1)
