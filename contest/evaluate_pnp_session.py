#!/usr/bin/env python3
"""用现场 CSV 四角点和已标定内参做离线平面 PnP 诊断，不生成控制指令。

依赖：Python 3、numpy、OpenCV、PyYAML，以及同目录 analyze_field_session.py。
width/height 必须是检测四角点所对应矩形的实际尺寸，没有默认装甲板尺寸。
"""

import argparse
from collections import Counter
import csv
import math
from pathlib import Path
from statistics import median
import sys

try:
    import cv2
    import numpy as np
    import yaml
except ImportError as exc:
    raise SystemExit("缺少依赖：本工具需要 numpy、Python OpenCV 和 PyYAML（import yaml）。") from exc

from analyze_field_session import load_rows


def positive_float(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return value


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="srm_live_pixel_probe --session 生成的 CSV")
    parser.add_argument("--intrinsics", type=Path, required=True, help="包含 image_width/height 的 camera_intrinsics.yaml")
    parser.add_argument("--width-mm", type=positive_float, required=True, help="检测点 0→1 / 3→2 对应的实物边长，毫米")
    parser.add_argument("--height-mm", type=positive_float, required=True, help="检测点 0→3 / 1→2 对应的实物边长，毫米")
    parser.add_argument("--known-distance-m", type=positive_float,
                        help="可选：相机光心到矩形中心的实测欧氏距离，米；不是沿相机 z 轴的深度")
    parser.add_argument("--output-dir", type=Path, required=True, help="尚不存在的目录，避免覆盖")
    args = parser.parse_args()
    if args.output_dir.exists() or args.output_dir.is_symlink():
        parser.error("输出目录已存在；请换一个新名字，避免覆盖以前结果")
    return args


def read_intrinsics(path, rows):
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError("内参 YAML 必须是包含 camera_matrix/distort_coeffs 的映射")
    for key, expected in (("image_width", int(rows[0]["width"])), ("image_height", int(rows[0]["height"]))):
        if type(data.get(key)) is not int or data[key] != expected:
            raise ValueError(f"内参 {key}={data.get(key)!r} 与 CSV 的 {expected} 不匹配；不能直接混用分辨率/缩放")
    for key, count in (("camera_matrix", 9), ("distort_coeffs", 5)):
        value = data.get(key)
        if not isinstance(value, list) or len(value) != count or any(isinstance(item, (list, dict, bool)) for item in value):
            raise ValueError(f"{key} 必须是包含 {count} 个数字的平铺数组")
    try:
        matrix = np.asarray(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(data["distort_coeffs"], dtype=np.float64)
    except (TypeError, ValueError):
        raise ValueError("内参或畸变包含非数字值") from None
    if not (np.all(np.isfinite(matrix)) and np.all(np.isfinite(distortion))):
        raise ValueError("内参或畸变存在 NaN/Infinity")
    if (min(matrix[0, 0], matrix[1, 1]) <= 0 or
            not np.allclose(matrix[2], [0, 0, 1], rtol=0, atol=1e-12) or
            abs(matrix[0, 1]) > 1e-12 or abs(matrix[1, 0]) > 1e-12):
        raise ValueError("camera_matrix 必须符合普通 OpenCV 针孔内参 [fx,0,cx;0,fy,cy;0,0,1]，fx/fy 为正")
    return matrix, distortion


def solve_frame(row, object_points, matrix, distortion):
    candidates = int(row["blue_candidates"])
    if candidates != 1:
        return ("no_target" if candidates == 0 else "multiple_targets"), [], 0
    # 沿用 armor.cpp：left.top, right.top, right.bottom, left.bottom。
    # 神经网络构造函数直接接收模型点序；solver.cpp 也沿用这个对应关系。
    # 不能根据本帧 x/y 排序：斜板或旋转时重排会改变物点身份。
    image_points = np.array([[float(row[f"x{i}"]), float(row[f"y{i}"])] for i in range(4)], dtype=np.float64)
    with np.errstate(over="ignore"):
        contour = image_points.astype(np.float32).reshape(-1, 1, 2)
    if not np.all(np.isfinite(contour)):
        return "degenerate_or_wrong_corner_order", [], 0
    edges = np.roll(image_points, -1, axis=0) - image_points
    if (not cv2.isContourConvex(contour) or abs(cv2.contourArea(contour)) < 4.0 or
            float(np.min(np.linalg.norm(edges, axis=1))) < 1.0):
        return "degenerate_or_wrong_corner_order", [], 0
    try:
        answer = cv2.solvePnPGeneric(object_points, image_points, matrix, distortion, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return "ippe_solver_error", [], 0
    if not answer[0]:
        return "ippe_no_solution", [], 0
    solutions = []
    rejected = 0
    for rvec, tvec in zip(answer[1], answer[2]):
        rvec, tvec = rvec.reshape(3), tvec.reshape(3)
        if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
            rejected += 1
            continue
        rotation, _ = cv2.Rodrigues(rvec)
        in_camera = (rotation @ object_points.T).T + tvec
        if not np.all(in_camera[:, 2] > 1e-6):
            rejected += 1
            continue
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, matrix, distortion)
        if not np.all(np.isfinite(projected)):
            rejected += 1
            continue
        # 真正的二维欧氏距离 RMS；不用把 cv2 返回的每坐标误差误当同一定义。
        with np.errstate(over="ignore", invalid="ignore"):
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
            distance = float(np.linalg.norm(tvec))
        if not (math.isfinite(error) and math.isfinite(distance)):
            rejected += 1
            continue
        solutions.append({"rvec": rvec, "tvec": tvec, "rms": error, "range": distance})
    solutions.sort(key=lambda item: item["rms"])
    return ("ok" if solutions else "no_positive_finite_solution"), solutions, rejected


def report(args, rows, stats, best, ambiguity, rejected):
    counts = "、".join(f"{key}={value}" for key, value in sorted(stats.items()))
    lines = ["# 离线平面 PnP 诊断", "",
             f"- 状态：{'完成几何计算，未验证实机精度' if best else '失败：没有可用的前方有限解'}。",
             f"- CSV：`{args.csv_path.resolve()}`", f"- 内参：`{args.intrinsics.resolve()}`",
             f"- 分辨率：{rows[0]['width']} × {rows[0]['height']}；已与内参 YAML 严格核对。",
             f"- 本次矩形尺寸：**{args.width_mm:g} × {args.height_mm:g} mm**，用户输入，程序未验证实物。",
             f"- 输入帧数：{len(rows)}；有解帧数：{len(best)}；状态计数：{counts}。",
             f"- 排除非有限或任一角点位于相机后方的候选解：{rejected} 个。", "",
             "## 角点、尺寸与坐标约定", "",
             "- 点序必须是 0 左上、1 右上、2 右下、3 左下，沿用 armor.cpp 的装甲板角点对应关系。先在编号角点画面中确认，再使用本报告。程序不会按 x/y 重新排序，也不能仅从凸四边形证明物点身份正确。",
             "- width 是 0–1/3–2 的实物边长，height 是 0–3/1–2 的实物边长。如果检测的是灯条端点，就测端点矩形；不能把装甲板外壳尺寸直接套进来。",
             "- 物体原点为矩形几何中心，物体点位于 XY 平面：0=(-w/2,-h/2,0)，1=(w/2,-h/2,0)，2=(w/2,h/2,0)，3=(-w/2,h/2,0)。物体 x 向右、y 向下、z 按右手法则。",
             "- 输出平移位于 OpenCV 相机坐标：x 向右、y 向下、z 向前，单位米。range_m=sqrt(tx²+ty²+tz²)，z_m 只是光轴方向深度。",
             "- rvec 是物体坐标到相机坐标的 Rodrigues 旋转向量（弧度），不是云台 yaw/pitch。这里的物体坐标轴与原 solver.cpp 的轴选取不同，不能直接复制它的欧拉角解释。", ""]
    if best:
        ranges = [item["range"] for item in best]
        rms_values = [item["rms"] for item in best]
        lines += ["## 最低重投影误差候选的统计", "",
                  f"- range：中位数 {median(ranges):.5f} m，范围 {min(ranges):.5f}～{max(ranges):.5f} m。",
                  f"- 四点重投影 RMS：中位数 {median(rms_values):.4f} px，最大 {max(rms_values):.4f} px。",
                  "- candidate_rank=1 只表示该帧误差较小，不能视为真值姿态，也没有做帧间跟踪或自动拒绝高误差帧。", ""]
        if args.known_distance_m is not None:
            relative = [(distance - args.known_distance_m) / args.known_distance_m * 100 for distance in ranges]
            absolute = [abs(value) for value in relative]
            lines += [f"- 输入距离真值：{args.known_distance_m:g} m（**相机光心到矩形中心的欧氏距离**）。",
                      f"- 有符号相对误差中位数：{median(relative):+.3f}%；绝对相对误差中位数：{median(absolute):.3f}%，最大 {max(absolute):.3f}%。",
                      "- 只有这段数据中相机与矩形中心的实际欧氏距离保持该真值时，上述距离误差才有意义。测到外壳、地面水平距离或枪口距离都不能直接当作光心距离。", ""]
    lines += ["## 平面双解", "",
              "- 求解采用 solvePnPGeneric + SOLVEPNP_IPPE，保留所有通过前方/有限数检查的候选。poses.csv 中同一帧可占两行；candidate_rank 按本工具计算的四点 RMS 排序。",
              f"- 保留两个候选的帧数：{len(ambiguity)}。"]
    if ambiguity:
        error_gaps = [item[0] for item in ambiguity]
        range_gaps = [item[1] for item in ambiguity]
        lines += [f"- 两解 RMS 差：中位数 {median(error_gaps):.4f} px，最小 {min(error_gaps):.4f} px。",
                  f"- 两解距离差绝对值：中位数 {median(range_gaps):.6f} m，最大 {max(range_gaps):.6f} m。",
                  f"- 两解 RMS 差小于 0.5 px 的帧数：{sum(value < 0.5 for value in error_gaps)}（提示用的经验阈值，不是准确性判据）。"]
    lines += ["- 近正面、小目标或远距离时，两种姿态可能重投影都很好。距离接近也不代表姿态相同；不要把最低误差候选当作已经消除了平面歧义。", "",
              "## 使用边界", "",
              "1. 本工具只读取 CSV 和 YAML；不打开相机/串口，不计算或发送云台指令，不读取实机外参。",
              "2. 小重投影误差不能证明尺寸、内参、点序正确。同比例填错矩形尺寸时，距离也会同比例错，而重投影误差仍可接近零。",
              "3. YAML 与 CSV 分辨率相同仍不证明 ROI、rotate_180、缩放、镜头焦距和对焦一致，这些必须现场核对。",
              "4. 没有标定相机到云台/枪口的外参，没有进行弹道、时延或运动预测验证。不要把相机坐标的位姿直接作为云台目标角度。",
              "5. no_target / multiple_targets 为没有唯一目标；degenerate_or_wrong_corner_order 表示非凸、自交或面积/边长过小。这些帧在 poses.csv 中留状态、不填假位姿。",
              "6. poses.csv 每个输入帧至少一行：有双解时两行。统计有解帧数只计一次，避免误把双解数量当成检测帧数。", ""]
    (args.output_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def run(args):
    rows = load_rows(args.csv_path)
    matrix, distortion = read_intrinsics(args.intrinsics, rows)
    width, height = args.width_mm / 1000, args.height_mm / 1000
    object_points = np.array([[-width / 2, -height / 2, 0], [width / 2, -height / 2, 0],
                              [width / 2, height / 2, 0], [-width / 2, height / 2, 0]], dtype=np.float64)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    fields = ["frame", "capture_s", "status", "candidate_rank", "valid_candidates", "reprojection_rms_px",
              "rvec_x_rad", "rvec_y_rad", "rvec_z_rad", "x_m", "y_m", "z_m", "range_m",
              "known_distance_m", "range_error_m", "relative_error_percent",
              "two_solution_rms_gap_px", "two_solution_range_gap_m"]
    stats, best, ambiguity = Counter(), [], []
    rejected = 0
    with (args.output_dir / "poses.csv").open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            status, solutions, dropped = solve_frame(row, object_points, matrix, distortion)
            stats[status] += 1
            rejected += dropped
            base = {"frame": row["frame"], "capture_s": row["capture_s"], "status": status,
                    "valid_candidates": len(solutions)}
            if not solutions:
                writer.writerow(base)
                continue
            best.append(solutions[0])
            if len(solutions) >= 2:
                error_gap = solutions[1]["rms"] - solutions[0]["rms"]
                range_gap = abs(solutions[1]["range"] - solutions[0]["range"])
                ambiguity.append((error_gap, range_gap))
                base.update(two_solution_rms_gap_px=error_gap, two_solution_range_gap_m=range_gap)
            for index, solution in enumerate(solutions, start=1):
                result = dict(base, candidate_rank=index, reprojection_rms_px=solution["rms"], range_m=solution["range"])
                result.update(zip(("rvec_x_rad", "rvec_y_rad", "rvec_z_rad"), solution["rvec"]))
                result.update(zip(("x_m", "y_m", "z_m"), solution["tvec"]))
                if args.known_distance_m is not None:
                    delta = solution["range"] - args.known_distance_m
                    result.update(known_distance_m=args.known_distance_m, range_error_m=delta,
                                  relative_error_percent=delta / args.known_distance_m * 100)
                writer.writerow(result)
    report(args, rows, stats, best, ambiguity, rejected)
    print(f"PnP 离线诊断：{len(rows)} 帧中 {len(best)} 帧有解，{len(ambiguity)} 帧保留双解。")
    print(f"报告：{args.output_dir / 'report.md'}；这不代表实机测距/瞄准已通过。")
    if not best:
        raise ValueError("所有帧均无可用解；已保存 poses.csv 和失败报告")


def main():
    try:
        run(arguments())
    except (ValueError, OSError, yaml.YAMLError, cv2.error) as exc:
        print(f"PnP 诊断未通过：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("已中断；输出可能不完整，请下次使用新目录。", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
