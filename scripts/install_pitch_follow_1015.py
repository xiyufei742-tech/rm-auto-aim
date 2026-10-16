#!/usr/bin/env python3
"""安装 10 月 15 日 pitch 单轴联调程序；默认预览，--apply 才修改项目。"""

import argparse
import hashlib
from pathlib import Path
import shutil
import sys
from datetime import datetime


FILES = (
    "contest/live_pixel_pitch_follow_safe.cpp",
    "contest/pitch_follow_policy.hpp",
    "contest/pitch_follow_policy_selfcheck.cpp",
    "contest/pitch_follow_worker_selfcheck.cpp",
)
MARKER = "# BEGIN pitch-follow-1015"
SNIPPET = """

# BEGIN pitch-follow-1015
add_executable(srm_live_pixel_pitch_follow_safe contest/live_pixel_pitch_follow_safe.cpp)
target_link_libraries(srm_live_pixel_pitch_follow_safe PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim srm_protocol)
target_compile_options(srm_live_pixel_pitch_follow_safe PRIVATE -Wall -Wextra)
add_executable(srm_pitch_follow_policy_selfcheck contest/pitch_follow_policy_selfcheck.cpp)
add_test(NAME srm_pitch_follow_policy_selfcheck COMMAND srm_pitch_follow_policy_selfcheck)
add_executable(srm_pitch_follow_worker_selfcheck contest/pitch_follow_worker_selfcheck.cpp)
target_link_libraries(srm_pitch_follow_worker_selfcheck PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim srm_protocol util)
add_test(NAME srm_pitch_follow_worker_selfcheck COMMAND srm_pitch_follow_worker_selfcheck)
# END pitch-follow-1015
"""


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("project", type=Path)
    cli.add_argument("--apply", action="store_true")
    args = cli.parse_args()
    root = Path(__file__).resolve().parent
    project = args.project.resolve()
    cmake = project / "CMakeLists.txt"
    if not cmake.is_file() or not (project / "contest").is_dir():
        raise RuntimeError("目标不是比赛项目根目录（缺少 CMakeLists.txt 或 contest/）")
    cmake_text = cmake.read_text(encoding="utf-8")
    if "add_executable(srm_live_pixel_follow_safe " not in cmake_text:
        raise RuntimeError("目标项目没有已验证的 yaw 跟随入口，请先核对项目路径")
    installed = MARKER in cmake_text
    for relative in FILES:
        source = root / "payload" / relative
        target = project / relative
        if not source.is_file():
            raise RuntimeError(f"安装包缺少文件：{source}")
        if target.exists() and digest(target) != digest(source):
            raise RuntimeError(f"目标已有不同内容，停止以避免覆盖：{target}")
        print(f"{'已相同' if target.exists() else '将新增'}：{relative}，SHA-256={digest(source)}")
    print("CMake 构建入口：", "已存在" if installed else "将追加独立 pitch 目标")
    if not args.apply:
        print("预览结束，未修改项目。核对路径后加 --apply。")
        return
    backup = project / "output" / ("pitch_follow_backup_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    if not installed:
        backup.mkdir(parents=True, exist_ok=False)
        shutil.copy2(cmake, backup / "CMakeLists.txt")
    for relative in FILES:
        source = root / "payload" / relative
        target = project / relative
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            if digest(target) != digest(source):
                raise RuntimeError(f"复制后校验失败：{target}")
    if not installed:
        with cmake.open("a", encoding="utf-8") as stream:
            stream.write(SNIPPET)
    print("安装完成；未修改 yaw 程序、现场 YAML 或下位机。")
    if not installed:
        print(f"CMake 原文件备份：{backup / 'CMakeLists.txt'}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError) as error:
        print(f"安装失败：{error}", file=sys.stderr)
        sys.exit(1)
