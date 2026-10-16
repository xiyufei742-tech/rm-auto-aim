#!/usr/bin/env python3
"""从本机 AVI 离线估计相机内参；不打开相机、串口或 CAN，不改现有配置。

例子（9×6、25 mm 仅为命令格式示例，必须换成实物尺寸）：
  python3 calibration/calibrate_from_video.py output/board.avi \
    --board chessboard --cols 9 --rows 6 --spacing-mm 25 \
    --output-dir output/calibration_01

需要 Python 3、numpy、opencv-python（或系统 python3-opencv）。
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:
    raise SystemExit("缺少 Python OpenCV/numpy，请先确认车载环境中的 python3 可以 import cv2, numpy。") from exc


def positive_int(value):
    value = int(value)
    if value <= 0:
        raise argparse.ArgumentTypeError("必须为正整数")
    return value


def positive_float(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("必须为有限正数")
    return value


def arguments():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", type=Path, help="已录制的视频文件，保持原分辨率、方向和裁剪")
    parser.add_argument("--board", required=True, choices=("chessboard", "circles"))
    parser.add_argument("--cols", type=positive_int, required=True, help="每行内角点数 / 对称圆阵列圆心数")
    parser.add_argument("--rows", type=positive_int, required=True, help="内角点行数 / 对称圆阵列圆心行数")
    parser.add_argument("--spacing-mm", type=positive_float, required=True,
                        help="棋盘小格边长 / 相邻圆心间距，毫米；必须实测")
    parser.add_argument("--stride", type=positive_int, default=10, help="每隔多少帧尝试一次，默认 10")
    parser.add_argument("--max-views", type=positive_int, default=40, help="最多接受多少幅不同图像，默认 40")
    parser.add_argument("--min-views", type=positive_int, default=12, help="至少接受多少幅不同图像，默认 12，最小 6")
    parser.add_argument("--output-dir", type=Path, required=True, help="必须是尚不存在的目录")
    args = parser.parse_args()
    if min(args.cols, args.rows) < 3:
        parser.error("cols 和 rows 均至少为 3；不能填黑白格总数代替内角点数")
    if args.min_views < 6 or args.max_views < args.min_views:
        parser.error("要求 6 <= min-views <= max-views")
    if not args.video.is_file():
        parser.error("输入视频文件不存在")
    if args.output_dir.exists() or args.output_dir.is_symlink():
        parser.error("输出目录已存在；请换一个新名字，以免覆盖先前数据")
    return args


def detect_points(gray, args):
    """棋盘使用亚像素角点；圆阵列由 OpenCV 的 blob 中心检测器定位。"""
    pattern = (args.cols, args.rows)
    if args.board == "circles":
        # 对称圆阵列，默认黑圆白底；不支持非对称圆阵列/圆心间距交错的板。
        return cv2.findCirclesGrid(gray, pattern, flags=cv2.CALIB_CB_SYMMETRIC_GRID)
    # 没有纹理的帧直接跳过，避免反复搜索一张黑图。
    if float(np.std(gray)) < 3.0:
        return False, None
    found, points = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if found:
        points = cv2.cornerSubPix(gray, points, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    elif hasattr(cv2, "findChessboardCornersSB"):
        # SB 已返回亚像素角点，不再使用传统 cornerSubPix 反复细化。
        found, points = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    return found, points


def nearest_view_distance(points, previous):
    """像素域近重复筛除，不把拍摄帧数冒充几何独立视角数。

    对棋盘/对称圆板可能出现的 180° 索引反序同时比较。
    这里只检测图像中的相似布局，不能证明标定板真的发生了三维倾斜。
    """
    if not previous:
        return math.inf
    flat = points.reshape(-1, 2)
    return min(float(np.sqrt(np.mean(np.sum((flat - order) ** 2, axis=1))))
               for old in previous for order in (old.reshape(-1, 2), old.reshape(-1, 2)[::-1]))


def write_report(args, stats, image_size, accepted, failure=None, result=None):
    counts = stats["counts"]
    board_meaning = "内角点（黑白格交界的交叉点）" if args.board == "chessboard" else "对称阵列的圆心（黑圆白底）"
    lines = ["# 视频相机内参标定报告", "", f"- 状态：{'未生成内参：' + failure if failure else '完成离线数值求解，待实物复核'}",
             f"- 输入：`{args.video.resolve()}`", f"- OpenCV：{cv2.__version__}",
             f"- 板类型：`{args.board}`；{args.cols} 列 × {args.rows} 行；计数对象是{board_meaning}。",
             f"- 相邻点间距：{args.spacing_mm:g} mm。棋盘是小格边长，圆板是圆心间距。",
             f"- 抽帧间隔：{args.stride}；接受上限：{args.max_views}；最低要求：{args.min_views}。",
             f"- 已解码：{stats['decoded']} 帧；已抽查：{counts['sampled']} 帧；成功检测：{counts['detected']} 帧。",
             f"- 未检测到完整板：{counts['not_found']}；近重复剔除：{counts['duplicate']}；接受：{len(accepted)}。",
             f"- 扫描结束原因：{stats['stop_reason']}。", ""]
    if image_size:
        width, height = image_size
        lines += [f"- 图像尺寸：{width} × {height} px。", f"- 近重复阈值：{stats['duplicate_threshold_px']:.2f} px（全体点对应距离 RMS，约图像对角线的 1%）。"]
    if accepted:
        centers = np.array([item["center"] for item in accepted])
        all_points = np.concatenate([item["points"].reshape(-1, 2) for item in accepted])
        areas = [item["area_fraction"] for item in accepted]
        cells = {(min(2, int(x * 3)), min(2, int(y * 3))) for x, y in centers}
        lines += [f"- 板中心覆盖 3 × 3 分区中的 {len(cells)} 区。",
                  f"- 板中心归一化范围：x={centers[:, 0].min():.3f}～{centers[:, 0].max():.3f}，y={centers[:, 1].min():.3f}～{centers[:, 1].max():.3f}。",
                  f"- 所有角点像素覆盖范围：x={all_points[:, 0].min():.1f}～{all_points[:, 0].max():.1f}，y={all_points[:, 1].min():.1f}～{all_points[:, 1].max():.1f}。",
                  f"- 板上检测点凸包占画面比例：{min(areas):.1%}～{max(areas):.1%}。",
                  "- 上述统计只描述图像覆盖。请检查 accepted/：需要明显的左右倾斜、上下倾斜、远近和画面边缘位置变化，单纯平移不能保证标定可用。", ""]
    if result:
        lines += ["## 求解结果", "", f"- 全部接受点的重投影 RMS：**{result['rms']:.4f} px**。",
                  f"- 每幅图重投影 RMS：中位数 {result['median']:.4f} px，最大 {result['maximum']:.4f} px。",
                  "- 完整参数：camera_intrinsics.yaml；逐幅误差和姿态：views.csv。",
                  "- 畸变顺序：k1, k2, p1, p2, k3；使用普通针孔模型，不适用于鱼眼镜头。", ""]
        if result["warnings"]:
            lines += ["### 需要复查", ""] + [f"- {warning}" for warning in result["warnings"]] + [""]
    lines += ["## 使用边界与下一步", "",
              "1. 逐张检查 accepted/ 中角点是否落在正确交叉点/圆心上。错误点、模糊板、弯曲板、尺寸填错会影响结果。",
              "2. 这是参与求解图像的训练重投影误差；小误差不代表 5 m 测距准确，也不是独立验证。另录一段未参与标定的视频，结合已知距离和实际板尺寸验证 PnP。",
              "3. 标定只得到这组相机、镜头和当前对焦状态的内参与畸变。没有求相机到云台/枪口的外参，没有验证弹道。",
              "4. 使用时须保持分辨率、ROI、缩放和 rotate_180 设置与本视频相同。更换镜头、改变对焦或裁剪后不能直接沿用。",
              "5. 本工具不会把结果自动写进现场配置；人工确认数据与验证结果后，再复制相应参数。旧配置中的外参不能因此视为已验证。",
              "6. 圆阵列必须是对称圆板。棋盘 cols/rows 是内角点数，例如 10 × 7 个黑白格只有 9 × 6 个内角点。",
              "7. observations.csv 记录抽查帧的处理结果；source_time_s 来自视频标称 FPS，仅供定位视频帧，不能替代采集时钟。", ""]
    (args.output_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def calibrate(args, image_size, accepted):
    # 米制物点使输出 tvec 的单位为米；内参和重投影误差始终以像素为单位。
    grid = np.zeros((args.cols * args.rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    grid *= args.spacing_mm / 1000.0
    object_points = [grid.copy() for _ in accepted]
    image_points = [item["points"].astype(np.float32) for item in accepted]
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None, flags=0,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    distortion = distortion.reshape(-1)
    if (not math.isfinite(rms) or not np.all(np.isfinite(matrix)) or
            not np.all(np.isfinite(distortion)) or len(distortion) != 5 or min(matrix[0, 0], matrix[1, 1]) <= 0):
        raise ValueError("求解得到无效参数，请增加倾斜与覆盖变化并检查标定板尺寸/角点")
    errors = []
    with (args.output_dir / "views.csv").open("x", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "source_time_s", "reprojection_rms_px", "center_x_norm", "center_y_norm",
                         "area_fraction", "rvec_x", "rvec_y", "rvec_z", "tvec_x_m", "tvec_y_m", "tvec_z_m"])
        for view, rvec, tvec in zip(accepted, rvecs, tvecs):
            projected, _ = cv2.projectPoints(grid, rvec, tvec, matrix, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - view["points"].reshape(-1, 2)) ** 2, axis=1))))
            errors.append(error)
            writer.writerow([view["frame"], view["time"], error, *view["center"], view["area_fraction"], *rvec.ravel(), *tvec.ravel()])
    width, height = image_size
    warnings = []
    if rms > 1.0:
        warnings.append("总体 RMS 超过 1 px；检查模糊、角点位置、标定板平整程度，并补拍。1 px 不是通用通过标准。")
    if not (0 <= matrix[0, 2] < width and 0 <= matrix[1, 2] < height):
        warnings.append("主点落在图像范围外，当前解不适合直接用于现场。")
    if not (0.5 < matrix[0, 0] / matrix[1, 1] < 2.0):
        warnings.append("fx/fy 比例异常；请重点检查行列数及角点匹配。")
    if not (0.2 * max(image_size) < min(matrix[0, 0], matrix[1, 1]) and
            max(matrix[0, 0], matrix[1, 1]) < 10 * max(image_size)):
        warnings.append("估计焦距明显偏离常见范围；可能是视角不足，也可能是特殊镜头，需单独核实。")
    yaml = ["# 离线求解结果，待实物验证；不要直接覆盖整个现场配置。",
            f"image_width: {width}", f"image_height: {height}",
            "camera_matrix: [" + ", ".join(f"{n:.12g}" for n in matrix.ravel()) + "]",
            "distort_coeffs: [" + ", ".join(f"{n:.12g}" for n in distortion) + "]",
            f"board_type: {args.board}", f"pattern_cols: {args.cols}", f"pattern_rows: {args.rows}",
            f"center_distance_mm: {args.spacing_mm:.12g}", f"accepted_views: {len(accepted)}",
            f"reprojection_rms_px: {rms:.12g}",
            "source_video: " + json.dumps(str(args.video.resolve()), ensure_ascii=False),
            "# 畸变顺序为 k1, k2, p1, p2, k3。tvec 在 views.csv 中，单位米。", ""]
    (args.output_dir / "camera_intrinsics.yaml").write_text("\n".join(yaml), encoding="utf-8")
    return {"rms": rms, "median": float(np.median(errors)), "maximum": max(errors), "warnings": warnings}


def run(args):
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise ValueError("无法打开输入视频，请检查视频格式和路径")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "accepted").mkdir()
    stats = {"decoded": 0, "stop_reason": "视频读取结束（EOF 或解码停止）", "duplicate_threshold_px": 0.0,
             "counts": {"sampled": 0, "detected": 0, "not_found": 0, "duplicate": 0}}
    accepted, image_size = [], None
    failure, result = None, None
    try:
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        with (args.output_dir / "observations.csv").open("x", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["frame", "source_time_s", "status", "nearest_view_rms_px"])
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                index = stats["decoded"]
                stats["decoded"] += 1
                size = (frame.shape[1], frame.shape[0])
                if image_size is None:
                    image_size = size
                    stats["duplicate_threshold_px"] = max(3.0, math.hypot(*size) * 0.01)
                if size != image_size:
                    raise ValueError("视频中途分辨率发生变化，不能混在同一次内参求解中")
                if index % args.stride:
                    continue
                stats["counts"]["sampled"] += 1
                source_time = index / fps if math.isfinite(fps) and fps > 0 else ""
                found, points = detect_points(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), args)
                if not found or points is None or len(points) != args.cols * args.rows or not np.all(np.isfinite(points)):
                    stats["counts"]["not_found"] += 1
                    writer.writerow([index, source_time, "not_found", ""])
                    continue
                stats["counts"]["detected"] += 1
                distance = nearest_view_distance(points, [item["points"] for item in accepted])
                if distance < stats["duplicate_threshold_px"]:
                    stats["counts"]["duplicate"] += 1
                    writer.writerow([index, source_time, "near_duplicate", distance])
                    continue
                preview = frame.copy()
                cv2.drawChessboardCorners(preview, (args.cols, args.rows), points, True)
                cv2.putText(preview, f"frame {index}", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                if not cv2.imwrite(str(args.output_dir / "accepted" / f"frame_{index:06d}.jpg"), preview):
                    raise OSError("保存角点预览图失败，请检查磁盘空间")
                center = points.reshape(-1, 2).mean(axis=0) / np.array(size)
                area = cv2.contourArea(cv2.convexHull(points.astype(np.float32))) / (size[0] * size[1])
                accepted.append({"frame": index, "time": source_time, "points": points,
                                 "center": center, "area_fraction": area})
                writer.writerow([index, source_time, "accepted", distance if math.isfinite(distance) else ""])
                print(f"接受 {len(accepted):02d}/{args.max_views}：frame={index}", flush=True)
                if len(accepted) >= args.max_views:
                    stats["stop_reason"] = "已达到 max-views 上限，未继续扫描余下视频"
                    break
        if len(accepted) < args.min_views:
            raise ValueError(f"仅接受 {len(accepted)} 幅不同图像，低于最低要求 {args.min_views}；请补拍清晰、不同倾角和位置的标定板")
        result = calibrate(args, image_size, accepted)
    except (ValueError, OSError, cv2.error) as exc:
        failure = str(exc)
        stats["stop_reason"] = "处理未完成，详见状态"
        raise
    except KeyboardInterrupt:
        failure = "用户中断，未完成本次求解"
        stats["stop_reason"] = "用户中断"
        raise
    finally:
        capture.release()
        write_report(args, stats, image_size, accepted, failure, result)
    print(f"离线求解完成：{len(accepted)} 幅图像，重投影 RMS={result['rms']:.4f} px。")
    print(f"请先检查 {args.output_dir / 'report.md'} 与 accepted/；这不是实物测距或外参验证。")


def main():
    try:
        run(arguments())
    except (ValueError, OSError, cv2.error) as exc:
        print(f"标定未完成：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("已中断；保留已写入的预览图/记录，请下次使用新的输出目录。", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
