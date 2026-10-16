#!/usr/bin/env python3
r"""用独立棋盘录像核验已标定相机的 PnP 距离；只读输入，不连接设备。

例子（已知距离必须实测，5.0 仅为示例）：
  python3 calibration/validate_board_distance.py output/board_check.avi \
    --intrinsics output/calibration_result/camera_intrinsics.yaml \
    --cols 8 --rows 11 --spacing-mm 30 --known-distance-m 5.0 \
    --output-dir output/board_distance_check

已知距离指相机光心到棋盘「内角点网格几何中心」的直线距离。录像必须是
未参与内参拟合的新录像，且相机分辨率、方向、ROI、镜头和对焦状态相同。
只生成新目录中的报告和逐帧 CSV；不改原视频、内参或控制配置。
需要 Python 3、numpy、opencv-python（或系统 python3-opencv）。
"""

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:
    raise SystemExit("缺少 Python OpenCV/numpy，请确认车载 python3 可以 import cv2, numpy。") from exc


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("必须为正整数")
    return number


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须为有限正数")
    return number


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", type=Path, help="新录制、未参与标定的棋盘视频")
    parser.add_argument("--intrinsics", type=Path, required=True,
                        help="calibrate_from_video.py 输出的 camera_intrinsics.yaml")
    parser.add_argument("--cols", type=positive_int, required=True, help="棋盘每行内角点数")
    parser.add_argument("--rows", type=positive_int, required=True, help="棋盘内角点行数")
    parser.add_argument("--spacing-mm", type=positive_float, required=True, help="实测棋盘小格边长，毫米")
    parser.add_argument("--known-distance-m", type=positive_float, required=True,
                        help="相机光心到内角点网格几何中心的实测直线距离，米")
    parser.add_argument("--stride", type=positive_int, default=15, help="每隔多少帧检测一次，默认 15")
    parser.add_argument("--max-views", type=positive_int, default=30, help="最多分析多少幅完整棋盘，默认 30")
    parser.add_argument("--min-views", type=positive_int, default=5, help="至少需要多少幅有效图像，默认 5")
    parser.add_argument("--max-rms-px", type=positive_float, default=1.5,
                        help="逐帧 PnP 内点重投影 RMS 上限，默认 1.5 px；这不是测距通过标准")
    parser.add_argument("--output-dir", type=Path, required=True, help="必须是尚不存在的新结果目录")
    args = parser.parse_args(argv)
    if min(args.cols, args.rows) < 3:
        parser.error("cols 和 rows 均至少为 3，且必须填内角点数，不是黑白格数")
    if args.max_views < args.min_views:
        parser.error("要求 min-views <= max-views")
    if not args.video.is_file():
        parser.error("输入视频文件不存在")
    if not args.intrinsics.is_file():
        parser.error("内参文件不存在")
    if args.output_dir.exists() or args.output_dir.is_symlink():
        parser.error("输出目录已存在；请换一个新名字，避免覆盖此前结果")
    return args


def load_intrinsics(path):
    """读取本项目标定脚本输出的简单 YAML，不依赖 PyYAML。"""
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        if key in fields:
            raise ValueError(f"内参文件中 {key} 重复")
        fields[key] = value.strip()
    required = ("image_width", "image_height", "camera_matrix", "distort_coeffs", "source_video")
    missing = [key for key in required if key not in fields]
    if missing:
        raise ValueError("内参文件缺少字段：" + ", ".join(missing))
    try:
        width, height = int(fields["image_width"]), int(fields["image_height"])
        flat_matrix = json.loads(fields["camera_matrix"])
        flat_distortion = json.loads(fields["distort_coeffs"])
        source_video = json.loads(fields["source_video"])
        matrix = np.asarray(flat_matrix, dtype=np.float64)
        distortion = np.asarray(flat_distortion, dtype=np.float64)
    except (ValueError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("内参文件的尺寸、矩阵、畸变或源视频字段无法解析") from exc
    if (width <= 0 or height <= 0 or matrix.shape != (9,) or distortion.shape != (5,) or
            not isinstance(source_video, str) or not source_video):
        raise ValueError("内参文件格式无效；应由 calibrate_from_video.py 生成")
    matrix = matrix.reshape(3, 3)
    if not (np.all(np.isfinite(matrix)) and np.all(np.isfinite(distortion)) and
            matrix[0, 0] > 0 and matrix[1, 1] > 0 and
            np.allclose(matrix[0, 1], 0, atol=1e-8) and
            np.allclose(matrix[1, 0], 0, atol=1e-8) and
            np.allclose(matrix[2], (0, 0, 1), atol=1e-8) and
            0 <= matrix[0, 2] < width and 0 <= matrix[1, 2] < height):
        raise ValueError("相机内参无效：检查焦距、主点、矩阵结构和非有限数")
    source_path = Path(source_video)
    if not source_path.is_absolute():
        source_path = path.parent / source_path
    return (width, height), matrix, distortion, source_path


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.digest()


def check_independent_video(video, source):
    """拒绝相同路径、硬链接、符号链接及可访问的同字节副本。"""
    if video.resolve() == source.resolve() or (source.is_file() and video.samefile(source)):
        raise ValueError("验证视频就是内参标定源视频；请另录一段独立录像")
    if not source.is_file():
        return "内参原视频在本机不可访问；只能检查路径，无法排除复制改名"
    if video.stat().st_size != source.stat().st_size:
        return "与内参原视频文件大小不同"
    if sha256_file(video) == sha256_file(source):
        raise ValueError("验证视频与内参标定源视频字节相同；复制改名不是独立验证")
    return "与内参原视频 SHA-256 不同"


def object_grid(cols, rows, spacing_mm):
    """以所有内角点的几何中心为原点，单位为米。"""
    grid = np.zeros((cols * rows, 3), dtype=np.float64)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    grid[:, 0] -= (cols - 1) / 2.0
    grid[:, 1] -= (rows - 1) / 2.0
    grid *= spacing_mm / 1000.0
    return grid


def detect_points(gray, cols, rows):
    if float(np.std(gray)) < 3.0:
        return None
    pattern = (cols, rows)
    found, points = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if found:
        points = cv2.cornerSubPix(gray, points, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    elif hasattr(cv2, "findChessboardCornersSB"):
        found, points = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found or points is None or len(points) != cols * rows or not np.all(np.isfinite(points)):
        return None
    return points.reshape(-1, 2).astype(np.float64)


def estimate_pose(grid, image_points, matrix, distortion):
    """RANSAC 剔除偶发错点；中心物点为原点，故 tvec 就是相机系中心位置。"""
    if image_points.shape != (len(grid), 2) or not np.all(np.isfinite(image_points)):
        return None
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        grid, image_points, matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE,
        iterationsCount=200, reprojectionError=2.0, confidence=0.99)
    if not ok or inliers is None:
        return None
    indices = inliers.reshape(-1)
    if len(indices) < max(12, math.ceil(0.9 * len(grid))):
        return None
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(grid[indices], image_points[indices],
                                         matrix, distortion, rvec, tvec)
    else:
        ok, rvec, tvec = cv2.solvePnP(grid[indices], image_points[indices], matrix, distortion,
                                      rvec, tvec, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
    translation = tvec.reshape(3)
    rotation, _ = cv2.Rodrigues(rvec)
    camera_grid = (rotation @ grid.T).T + translation
    if (not np.all(np.isfinite(translation)) or not np.all(np.isfinite(camera_grid)) or
            np.any(camera_grid[:, 2] <= 0)):
        return None
    projected, _ = cv2.projectPoints(grid, rvec, tvec, matrix, distortion)
    squared = np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1)
    inlier_rms = float(np.sqrt(np.mean(squared[indices])))
    all_rms = float(np.sqrt(np.mean(squared)))
    distance = float(np.linalg.norm(translation))
    if not all(math.isfinite(v) for v in (inlier_rms, all_rms, distance)) or distance <= 0:
        return None
    return {"x_m": float(translation[0]), "y_m": float(translation[1]), "z_m": float(translation[2]),
            "distance_m": distance, "rms_inlier_px": inlier_rms, "rms_all_px": all_rms,
            "inliers": len(indices), "corners": len(grid)}


def analyze_video(args, image_size, matrix, distortion):
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise ValueError("无法打开输入视频，请检查路径和解码器")
    grid = object_grid(args.cols, args.rows, args.spacing_mm)
    stats = {"decoded": 0, "sampled": 0, "not_found": 0, "pnp_failed": 0, "high_rms": 0}
    views = []
    try:
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            index = stats["decoded"]
            stats["decoded"] += 1
            if (frame.shape[1], frame.shape[0]) != image_size:
                raise ValueError(f"frame={index} 分辨率不符：视频为 {frame.shape[1]}×{frame.shape[0]}，内参为 {image_size[0]}×{image_size[1]}")
            if index % args.stride or len(views) >= args.max_views:
                continue
            stats["sampled"] += 1
            points = detect_points(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), args.cols, args.rows)
            if points is None:
                stats["not_found"] += 1
                continue
            try:
                pose = estimate_pose(grid, points, matrix, distortion)
            except cv2.error:
                pose = None
            if pose is None:
                stats["pnp_failed"] += 1
                continue
            if pose["rms_inlier_px"] > args.max_rms_px:
                stats["high_rms"] += 1
                continue
            pose["frame"] = index
            pose["source_time_s"] = index / fps if math.isfinite(fps) and fps > 0 else ""
            pose["error_m"] = pose["distance_m"] - args.known_distance_m
            pose["error_pct"] = 100.0 * pose["error_m"] / args.known_distance_m
            views.append(pose)
    finally:
        capture.release()
    if stats["decoded"] == 0:
        raise ValueError("视频没有可解码帧")
    if len(views) < args.min_views:
        raise ValueError(f"仅有 {len(views)} 幅有效棋盘图像，低于 min-views={args.min_views}；"
                         f"抽查 {stats['sampled']} 帧，未找到完整板 {stats['not_found']}，PnP 失败 {stats['pnp_failed']}，"
                         f"重投影误差过大 {stats['high_rms']}。请拍清晰、完整且足够大的棋盘")
    return stats, views


def write_results(args, image_size, source_check, stats, views):
    """仅在成功求得足够图像后创建新结果目录；输入始终只读。"""
    args.output_dir.mkdir(parents=True, exist_ok=False)
    names = ("frame", "source_time_s", "distance_m", "error_m", "error_pct", "x_m", "y_m", "z_m",
             "rms_inlier_px", "rms_all_px", "inliers", "corners")
    with (args.output_dir / "views.csv").open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=names)
        writer.writeheader()
        writer.writerows(views)
    distances = np.asarray([row["distance_m"] for row in views])
    errors = distances - args.known_distance_m
    rms_values = np.asarray([row["rms_inlier_px"] for row in views])
    depths = np.asarray([row["z_m"] for row in views])
    lines = ["# 独立棋盘已知距离验证报告", "", "- 状态：完成离线 PnP 与实测距离对照；需要人工核实量距基准与图像质量。",
             f"- 验证视频：`{args.video.resolve()}`",
             f"- 内参文件：`{args.intrinsics.resolve()}`",
             f"- 与标定源视频的独立性检查：{source_check}。",
             f"- 分辨率：{image_size[0]} × {image_size[1]} px；OpenCV：{cv2.__version__}。",
             f"- 棋盘：{args.cols} 列 × {args.rows} 行内角点；实测小格边长 {args.spacing_mm:g} mm。",
             f"- 实测距离：**{args.known_distance_m:.4f} m**，相机光心到内角点网格几何中心的直线距离。",
             f"- 已解码 {stats['decoded']} 帧；抽查 {stats['sampled']} 帧；完整板未检出 {stats['not_found']}，"
             f"PnP/内点不足 {stats['pnp_failed']}，内点 RMS>{args.max_rms_px:g} px {stats['high_rms']}；有效 {len(views)} 帧。",
             "", "## 距离结果", "",
             f"- PnP 直线距离：中位数 **{np.median(distances):.4f} m**，范围 {distances.min():.4f}～{distances.max():.4f} m。",
             f"- 相对实测值：中位偏差 **{np.median(errors):+.4f} m ({100*np.median(errors)/args.known_distance_m:+.2f}%)**；"
             f"单帧偏差范围 {errors.min():+.4f}～{errors.max():+.4f} m。",
             f"- 相机前向 z 深度：中位数 {np.median(depths):.4f} m；直线距离使用 sqrt(x²+y²+z²)，不是 z。",
             f"- 内点重投影 RMS：中位数 {np.median(rms_values):.3f} px，最大 {rms_values.max():.3f} px。",
             "- 逐帧位置、距离、偏差、重投影误差和内点数见 `views.csv`。", "", "## 解释边界", "",
             "1. 必须实测光心到内角点网格中心；量到镜头前沿或板外框时，基准偏差会直接进入结果。",
             "2. 独立性检查可发现同一路径、硬链接或字节相同副本；不能证明不同录像的拍摄条件正确。",
             "3. 此结果只核验棋盘尺度与相机坐标系下的距离，不验证装甲板角点模型、相机到云台外参、弹道或命中。",
             "4. 重投影误差小只表示像素点与模型投影一致；请同时检查尺寸、倾角、清晰度和实测量距。",
             "5. `source_time_s` 由视频标称 FPS 推算，仅用于定位帧，不是采集时钟。", ""]
    with (args.output_dir / "report.md").open("x", encoding="utf-8") as handle:
        handle.write("\n".join(lines))
    return float(np.median(distances)), float(np.median(errors))


def main(argv=None):
    try:
        args = arguments(argv)
        image_size, matrix, distortion, source = load_intrinsics(args.intrinsics)
        source_check = check_independent_video(args.video, source)
        stats, views = analyze_video(args, image_size, matrix, distortion)
        median, error = write_results(args, image_size, source_check, stats, views)
    except (ValueError, OSError, cv2.error) as exc:
        print(f"验证未完成：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("已中断；原视频和内参未修改。", file=sys.stderr)
        return 130
    print(f"独立棋盘测距：{len(views)} 帧有效，实测 {args.known_distance_m:.4f} m，"
          f"PnP 中位 {median:.4f} m，偏差 {error:+.4f} m ({100*error/args.known_distance_m:+.2f}%)。")
    print(f"逐帧结果：{args.output_dir / 'views.csv'}；报告：{args.output_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
