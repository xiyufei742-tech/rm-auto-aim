#!/usr/bin/env python3
"""现场环境检查：只读文件/进程信息，不打开相机或串口、不启动云台。"""

import argparse
import datetime
import os
from pathlib import Path
import platform
import shutil
import subprocess


CAMERA_APPS = {
    "srm_live_pixel_probe", "srm_live_pixel_step_safe", "srm_live_pixel_follow_safe",
    "camera_test", "camera_detect_test", "camera_record_test", "MVS", "MVS.sh",
}


def check(project, config, port):
    # 输出三态，不把“检查不到”误报为“确认正常”。
    rows = []

    def add(state, item, detail):
        rows.append((state, item, detail))

    add("信息", "电脑", f"{platform.node()}；{platform.system()} {platform.release()}")
    add("信息", "项目路径", str(project))
    add("通过" if config.is_file() else "缺失", "配置文件", str(config))
    for name in ("camera_test", "srm_live_pixel_probe", "srm_live_pixel_follow_safe"):
        executable = project / "build" / name
        add("通过" if executable.is_file() and os.access(executable, os.X_OK) else "缺失",
            "可执行文件", str(executable))

    if port.exists():
        readable = os.access(port, os.R_OK)
        add("通过" if readable else "需处理", "串口读取权限",
            f"{port}，{'可读' if readable else '当前用户不可读'}；此检查未打开串口")
    else:
        add("缺失", "串口设备", f"{port} 不存在；仅录像时可不使用串口")
    ids = Path("/dev/serial/by-id")
    links = sorted(ids.glob("*")) if ids.exists() else []
    add("信息", "串口固定名称", "; ".join(f"{p.name} -> {p.resolve()}" for p in links) or "未发现")

    lsusb = shutil.which("lsusb")
    if lsusb:
        try:
            usb = subprocess.run([lsusb], capture_output=True, text=True, timeout=5, check=False)
            cameras = [line for line in usb.stdout.splitlines() if "2bdf:" in line.lower()]
            state = "发现" if cameras else "未发现"
            add(state, "海康 USB 相机", "; ".join(cameras) or "lsusb 未显示 2bdf 厂商设备")
            if usb.returncode:
                add("待确认", "USB 枚举", usb.stderr.strip() or f"lsusb 退出码 {usb.returncode}")
        except (OSError, subprocess.TimeoutExpired) as error:
            add("待确认", "USB 枚举", str(error))
    else:
        add("待确认", "USB 枚举", "未安装 lsusb；未尝试安装")

    # 只读取相关程序名，不输出所有进程的参数，避免泄漏不相关信息。
    competing = []
    inaccessible = 0
    for process in Path("/proc").iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            raw = (process / "cmdline").read_bytes().split(b"\0")
            if raw and raw[0]:
                name = Path(os.fsdecode(raw[0])).name
                if name in CAMERA_APPS:
                    competing.append(f"PID {process.name}: {name}")
        except PermissionError:
            inaccessible += 1
        except (FileNotFoundError, ProcessLookupError):
            pass
    add("需处理" if competing else "未发现", "可能占用相机的程序",
        "; ".join(competing) or "可读取的进程中没有发现已知测试程序；不等于已确认相机空闲")
    if inaccessible:
        add("待确认", "进程读取限制", f"有 {inaccessible} 个进程无法读取；不会强行结束任何进程")

    try:
        import cv2
        add("通过", "Python OpenCV", f"{cv2.__version__}（离线标定工具需要）")
    except ImportError:
        add("缺失", "Python OpenCV", "无法 import cv2；C++ 程序是否可运行不由此判断")
    try:
        import yaml
        add("通过", "PyYAML", f"{yaml.__version__}（离线 PnP 读取内参需要）")
    except ImportError:
        add("缺失", "PyYAML", "无法 import yaml；CSV 统计无需此依赖，离线 PnP 需要")
    total, used, free = shutil.disk_usage(project)
    add("信息", "剩余磁盘", f"{free / (1024**3):.2f} GiB；请按实际录像文件大小估算可录时长")
    add("待人工确认", "配置与模型", "确认本次 YAML 的曝光、方向、颜色、模型路径与现场一致；本检查不验证内容")
    add("待人工确认", "机械状态", "本程序没有移动云台，也没有验证模式开关、回位、镜头焦点或角度范围")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--config", type=Path, required=True, help="相对于项目根目录，或绝对路径")
    parser.add_argument("--port", type=Path, default=Path("/dev/ttyACM0"))
    parser.add_argument("--output", type=Path, help="可选保存 Markdown；已存在时拒绝覆盖")
    args = parser.parse_args()
    project = args.project.resolve(strict=True)
    config = args.config if args.config.is_absolute() else project / args.config
    rows = check(project, config, args.port)
    lines = ["# 现场环境检查", "", datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
             "", "本报告只记录环境，不能证明设备通信、识别和控制已经通过。", "",
             "| 状态 | 检查项 | 结果 |", "|---|---|---|"]
    for state, item, detail in rows:
        lines.append("| " + " | ".join(part.replace("|", "\\|").replace("\n", " ")
                                        for part in (state, item, detail)) + " |")
    report = "\n".join(lines) + "\n"
    print(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(report)
        print(f"已保存：{args.output}")


if __name__ == "__main__":
    try:
        main()
    except OSError as error:
        raise SystemExit(f"环境检查失败：{error}")
