#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

case "${1:-帮助}" in
  构建)
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j 4
    ;;
  自检)
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j 4
    ctest --test-dir build --output-on-failure
    ;;
  帮助|-h|--help)
    cat <<'HELP'
自瞄项目（个人电脑 / NUC 均可构建）：
  ./运行.sh 构建     编译相机、识别、比赛串口、无弹跟随及后续算法库
  ./运行.sh 自检     编译并运行不接实机的 14 项 C++ 自检

实机只读与无弹跟随命令请看“开始这里.md”。
HELP
    ;;
  *)
    echo "未知选项：$1；运行 ./运行.sh 帮助 查看说明。" >&2
    exit 2
    ;;
esac
