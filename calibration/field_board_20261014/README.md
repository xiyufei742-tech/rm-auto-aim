# 2026-10-14 现场标定板记录

- 类型：平面棋盘格，纵向摆放。
- 黑白方格：横向 9 格、纵向 12 格。
- **标定参数：横向内角点 8，纵向内角点 11，相邻内角点间距约 30 mm。**
- 量尺依据：`board_ruler.jpg` 中相邻两条竖向格线分别对齐尺子约 0 cm、3 cm、6 cm；请以实物再核对一次，避免镜头视差或尺子未贴平带来误差。
- `board_full.jpg` 上以 OpenCV `findChessboardCornersSB(..., (8, 11))` 找到 88 个角点；这仅检查了板型和点数。
- 原始照片：`board_full.jpg`、`board_ruler.jpg`。手机照片不用于计算比赛相机内参。

拿到比赛相机之后，保持实际比赛的分辨率、镜头焦距与对焦状态，录到至少 20 个不同位置、距离和倾角的清晰完整棋盘姿态。板不要弯曲。再用 `calibration/calibrate_from_video.py` 对录像执行标定；输出内参需用未参与标定的图像和已知距离目标验证后才可用于 PnP。

命令模板（把视频路径和新结果目录改成实际路径）：

```bash
python3 calibration/calibrate_from_video.py output/board_20261014.avi \
  --board chessboard --cols 8 --rows 11 --spacing-mm 30 \
  --stride 15 --max-views 30 --min-views 12 \
  --output-dir output/calibration_board_20261014
```
