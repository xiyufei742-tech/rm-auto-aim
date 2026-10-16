# RoboMaster 校内赛自瞄项目

面向校内赛自瞄赛道的个人工程。项目以同济大学 SuperPower 战队开源的
[`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25) 为基础，保留海康相机与
YOLOv5 装甲板识别模块，并完成比赛 USB CDC 协议、现场只读探针、无弹像素闭环、录像分析、
相机标定和只读旋转候选关联等适配。

本仓库只提交源码、配置、模型、测试、调试日记和小型证据摘要。现场原始 AVI、完整 CSV、
编译产物和本机 OpenVINO 运行包不进入 Git；它们由现场数据清单单独归档。

## 当前成果

```text
海康相机 → YOLOv5 蓝色装甲板与四角 → 像素 x/y 误差
        → 受限 yaw/pitch 指令 → USB CDC → 下位机与云台回传
```

- 完成比赛相机取图、180° 方向修正、Bayer/BGR 颜色修正和现场曝光调整。
- 完成正式 USB CDC 协议的回传解析与命令编码；所有现有实机测试均为 `fire_flag=0`。
- 静止靶扫描得到 `dx/dyaw≈+32.35 px/°`、`dy/dpitch≈−32.552 px/°`。
- 静止靶首次双轴无弹闭环将像素误差从 `(+16.3,−19.8)` 缩小到 `(+2.9,−1.8) px`，
  随后平滑回位并由独立回传确认手动档、弹速 0。
- 旋转靶 900 帧录像中出现 272 帧多候选；已完成只读跨帧候选关联原型，尚未用于云台控制。
- 本车相机棋盘标定候选结果使用 35 个视图、覆盖 3×3 区域中的 8 区，重投影 RMS 为
  `0.2864 px`。固定棋盘直线实测 `1.970 m`，PnP 中位结果 `1.9750 m`；该结果不等于
  5 m 装甲板、外参、弹道或命中精度验证。

尚未完成可靠的旋转目标身份关联、三维运动跟踪、预测、相机到云台/枪口外参、弹道补偿、
实弹发射和命中率统计。

## 目录结构

```text
assets/             YOLOv5 与数字分类模型
calibration/        棋盘视频标定、独立距离验证和标定板记录
configs/            本地预览配置（现场配置另随原始数据归档）
contest/            比赛协议、只读探针、无弹控制、分析工具和自检
docs/               技术说明、调试日记、中期材料、规则与协议
evidence/           可直接审查的小型报告、控制日志和关联 CSV
io/                 海康相机接口及原开源通信类型参考
scripts/            现场工具安装与打包脚本
tasks/auto_aim/     装甲板、YOLO、PnP、跟踪、预测和开火判断源码
tests/              相机入口和 Python 回归检查
tools/              图像、日志、数学、EKF 和弹道共用工具
```

详细入口见[开始这里](开始这里.md)，代码顺序见[代码阅读指南](代码阅读指南.md)，每日过程见
[调试日记](docs/调试日记/README.md)，中期讲解见[中期评定](docs/中期评定/README.md)。

## 构建环境

主要依赖：CMake 3.16+、C++17、OpenCV、fmt、Eigen3、spdlog、yaml-cpp、OpenSSL、
libusb、OpenVINO 2024.x，以及 x86_64 海康相机 SDK。仓库保留当前相机 SDK 头文件和
`libMvCameraControl.so`；OpenVINO 运行包不提交，需要系统安装或通过 `OpenVINO_DIR` 指定。

车载环境如安装在 `/opt/intel/openvino_2024`，先执行对应 `setupvars.sh`。随后：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure
python3 tests/field_tools_selfcheck.py
python3 tests/validate_board_distance_selfcheck.py
```

也可以使用：

```bash
./运行.sh 构建
./运行.sh 自检
```

自检只验证协议、策略、虚拟串口和合成几何数据，不等于实机识别、云台控制或命中通过。

## 当前实机入口

- `srm_live_pixel_probe`：相机、YOLO 和串口回传的只读记录。
- `srm_live_pixel_step_safe`：单次受限 yaw 小步测试。
- `srm_live_pixel_follow_safe`：静止靶受限 yaw 跟随。
- `srm_live_pixel_pitch_follow_safe`：静止靶受限 pitch 跟随。
- `srm_live_pixel_xy_follow_safe`：静止靶受限双轴跟随。

所有控制入口默认只读；实机写串口必须显式启用对应参数，并继续遵守无弹、人员远离云台、
手动档启动、随时可切回手动档等现场条件。

## 开源来源与证据边界

原项目说明保存在[同济开源项目原 README](docs/同济开源项目原README.md)，原 MIT
[`LICENSE`](LICENSE) 保留。本仓库将“开源代码已有功能”“离线自检通过”和“本车实机观察”
分开记录；源码存在或编译通过不代表已经完成比赛验证。

