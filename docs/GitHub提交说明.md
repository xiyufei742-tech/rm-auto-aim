# GitHub 提交说明

## 这份副本的目标

本目录用于提交可阅读、可构建、能说明实测过程的自瞄工程。原目录没有被修改。提交版聚焦比赛实际使用的相机、YOLO、USB CDC、无弹单轴/双轴控制、标定和分析工具，同时保留完整自瞄后续要继续接入的 `Solver/Tracker/Target/Aimer/Shooter` 源码。

## 已包含

- 当前 C++/Python 源码、CMake 与运行脚本。
- YOLOv5、数字分类模型和当前预览配置。
- 比赛规则、通信协议、开源来源和许可证。
- 10 月 9、11、14、15 日调试日记。
- 中期评定架构、五分钟讲稿、问答和专项实测说明。
- 小型统计报告、双轴控制关键日志、旋转候选关联 CSV、最新标定摘要。
- 14 项 C++ 自检与 13 项 Python 回归检查。

## 未包含

- `build/`、缓存、core dump 和临时日志。
- 现场原始 AVI、完整逐帧 CSV、输出目录和传输压缩包。
- 本机 OpenVINO 运行包。
- 旧版、重复、已被最终代码替代的临时文件。

这些内容已写入 `.gitignore`。现场原始数据继续单独归档，避免代码仓库膨胀；报告中的实测结论必须保留原始数据才能进一步复核。

## 上传前检查

```bash
git status --short
git ls-files | grep -E '\.(avi|mp4|tar|tar\.gz|zip)$' && echo "发现不应提交的大文件"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure
python3 tests/field_tools_selfcheck.py
python3 tests/validate_board_distance_selfcheck.py
```

本机 2026-10-15 的干净副本验证结果为：完整编译成功，14/14 项 C++ 自检通过，11 项现场数据工具检查和 2 项独立测距检查通过。该结果仍不等于实车、旋转预测或命中验证通过。
