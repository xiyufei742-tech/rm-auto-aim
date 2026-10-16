#!/usr/bin/env python3
"""无弹 pitch +0.3° 小阶跃：默认只读，显式 --actuate 才发送。"""

import argparse
import math
import os
import struct
import sys
import termios
import time
import tty

from field_serial_hold_safe import FeedbackStream, receive, sample, show, stable_target, write_all


STEP_DEG = 0.3
SEND_PERIOD_S = 0.05


def command(yaw, pitch):
    if not all(math.isfinite(v) for v in (yaw, pitch)) or abs(yaw) > 170 or abs(pitch) > 50:
        raise RuntimeError("拒绝发送超出保守边界的角度")
    frame = struct.pack("<hhffhi", 16, 1, yaw, pitch, 2, 0)
    assert len(frame) == 18
    return frame


def feedback_now(fd, stream, baseline, timeout=0.02):
    receive(fd, stream, timeout)
    if stream.latest is None or time.monotonic() - stream.last_received > 0.25:
        raise RuntimeError("下位机回传超过 250 ms 未更新")
    yaw, pitch, _, mode, _, _ = stream.latest
    if mode not in (0, 1):
        raise RuntimeError("下位机模式不是手动档或视觉档")
    if abs(yaw - baseline[0]) > 0.6 or abs(pitch - baseline[1]) > 0.9:
        raise RuntimeError("云台实际角度偏离起始位置过大")
    return yaw, pitch, mode


def tick(fd, stream, frame, baseline, next_send, *, require_visual=False,
         send_in_manual=False):
    # 先确认回传仍新鲜、模式仍允许，再写下一条指令；切回手动档后不补发旧阶跃。
    yaw, pitch, mode = feedback_now(fd, stream, baseline)
    if require_visual and mode != 1:
        raise RuntimeError("视觉档已退出，停止阶跃")
    now = time.monotonic()
    if now >= next_send and (mode == 1 or send_in_manual):
        write_all(fd, frame)
        next_send = now + SEND_PERIOD_S
    return yaw, pitch, mode, next_send


def hold_until_visual(fd, stream, frame, baseline):
    deadline = time.monotonic() + 15
    next_send = 0.0
    streak = 0
    last_received = 0.0
    while time.monotonic() < deadline:
        _, _, mode, next_send = tick(fd, stream, frame, baseline, next_send,
                                    send_in_manual=True)
        if stream.last_received != last_received:
            streak = streak + 1 if mode == 1 else 0
            last_received = stream.last_received
        if streak >= 3:
            return
    raise RuntimeError("15 秒内未确认连续 3 帧视觉档")


def drive(fd, stream, frame, baseline, seconds, label):
    deadline = time.monotonic() + seconds
    next_send = 0.0
    minimum = math.inf
    maximum = -math.inf
    next_print = 0.0
    while time.monotonic() < deadline:
        _, pitch, _, next_send = tick(fd, stream, frame, baseline, next_send,
                                      require_visual=True)
        minimum = min(minimum, pitch)
        maximum = max(maximum, pitch)
        if time.monotonic() >= next_print:
            show(label, stream.latest)
            next_print = time.monotonic() + 0.5
    return minimum, maximum


def return_to_baseline(fd, stream, frame, baseline, seconds=3):
    deadline = time.monotonic() + seconds
    next_send = 0.0
    stable_since = None
    while time.monotonic() < deadline:
        _, pitch, mode, next_send = tick(fd, stream, frame, baseline, next_send)
        if mode == 0:
            return True
        if abs(pitch - baseline[1]) < 0.15:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= 0.5:
                return False
        else:
            stable_since = None
    raise RuntimeError("3 秒内未确认 pitch 回到起始角度")


def wait_manual(fd, stream, frame, baseline):
    deadline = time.monotonic() + 10
    next_send = 0.0
    while time.monotonic() < deadline:
        _, _, mode, next_send = tick(fd, stream, frame, baseline, next_send)
        if mode == 0:
            return
    raise RuntimeError("10 秒内未切回手动档")


def run(fd, stream, actuate):
    show("当前", sample(fd, stream, 1.0))
    if not actuate:
        print("只读模式：未发送任何云台指令。实机测试须显式加 --actuate。", flush=True)
        return
    if stream.latest[3] != 0:
        raise RuntimeError("启动时必须处于手动档 mode=0")
    print("无弹 pitch +0.3° 阶跃：确认无弹、云台附近无人、遥控器在手动档。"
          "按回车后发送起始角度；看到提示再切视觉档。", flush=True)
    input()
    stream.buffer.clear()
    stream.latest = None
    stream.recent.clear()
    sample(fd, stream, 0.8)
    initial = stable_target(stream)
    if initial[3] != 0:
        raise RuntimeError("开始前已不是手动档")
    baseline = initial[:2]
    baseline_frame = command(*baseline)
    step_frame = command(baseline[0], baseline[1] + STEP_DEG)
    show("起始目标", initial)
    print("开始保持起始角度，fire_flag=0；请现在切视觉档。", flush=True)
    try:
        hold_until_visual(fd, stream, baseline_frame, baseline)
        print("视觉档已确认，先保持 0.7 秒。", flush=True)
        drive(fd, stream, baseline_frame, baseline, 0.7, "稳定")
        print("pitch 目标 +0.3°，持续 2 秒；yaw 保持起始角度。", flush=True)
        minimum, maximum = drive(fd, stream, step_frame, baseline, 2.0, "阶跃")
        print("停止阶跃，回到起始角度。", flush=True)
        manual = return_to_baseline(fd, stream, baseline_frame, baseline)
        print(f"阶跃期间实际 pitch 相对起点：{minimum-baseline[1]:+.3f}°"
              f" 至 {maximum-baseline[1]:+.3f}°", flush=True)
        if not manual:
            print("已确认 pitch 回位；请现在切回手动档。", flush=True)
            wait_manual(fd, stream, baseline_frame, baseline)
        print("无弹 pitch 阶跃测试完成，mode=0。", flush=True)
    finally:
        # 任何异常都优先覆盖旧阶跃命令；若回传失效，不能声称自动回位。
        if stream.latest is not None and stream.latest[3] == 1 and \
                time.monotonic() - stream.last_received <= 0.25:
            try:
                manual = return_to_baseline(fd, stream, baseline_frame, baseline)
                if not manual:
                    print("已尽力回到起始角度；请切回手动档。", flush=True)
                    wait_manual(fd, stream, baseline_frame, baseline)
            except (OSError, RuntimeError):
                print("未能确认自动回位；请立即切回手动档。", file=sys.stderr)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", default="/dev/ttyACM0")
    cli.add_argument("--actuate", action="store_true", help="显式启用无弹小阶跃")
    args = cli.parse_args()
    access = os.O_RDWR if args.actuate else os.O_RDONLY
    fd = os.open(args.port, access | os.O_NOCTTY | os.O_NONBLOCK)
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        run(fd, FeedbackStream(), args.actuate)
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
