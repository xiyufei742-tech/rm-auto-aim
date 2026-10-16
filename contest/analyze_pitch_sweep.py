#!/usr/bin/env python3
"""Fit a local image-y / gimbal-pitch slope from a read-only field CSV.

This program reads a completed session; it never opens the camera or serial port.
The two timestamps are software timestamps on the NUC, not hardware sync.
"""

import argparse
import math
from pathlib import Path

from analyze_field_session import load_rows, number


def fit_pitch(rows):
    if rows[0]["phase"] != "sweep" or rows[0]["source"] != "camera":
        raise ValueError("需要现场相机的 sweep 会话 CSV")
    pairs = []
    excluded = {"候选不唯一": 0, "缺少回传": 0, "图像或回传过期": 0,
                "软件时刻相差超过 100 ms": 0, "非手动档": 0}
    for row in rows:
        if int(row["blue_candidates"]) != 1:
            excluded["候选不唯一"] += 1
            continue
        if row["feedback_received_s"] == "":
            excluded["缺少回传"] += 1
            continue
        if number(row, "image_age_ms") > 250 or number(row, "feedback_age_ms") > 250:
            excluded["图像或回传过期"] += 1
            continue
        if abs(number(row, "feedback_received_s") - number(row, "capture_s")) > 0.1:
            excluded["软件时刻相差超过 100 ms"] += 1
            continue
        if int(row["mode"]) != 0:
            excluded["非手动档"] += 1
            continue
        pairs.append((number(row, "pitch_deg"), number(row, "target_y"),
                      number(row, "yaw_deg")))
    if len(pairs) < 20:
        raise ValueError("有效配对少于 20 帧，不能拟合")
    pitch = [pair[0] for pair in pairs]
    y = [pair[1] for pair in pairs]
    yaw = [pair[2] for pair in pairs]
    pitch_span = max(pitch) - min(pitch)
    yaw_span = max(yaw) - min(yaw)
    if pitch_span < 1.0:
        raise ValueError("pitch 扫描不到 1°，不能拟合")
    if yaw_span > 0.5:
        raise ValueError("yaw 同时变化超过 0.5°，本工具拒绝把混合扫描当 pitch 斜率")
    mean_pitch = sum(pitch) / len(pitch)
    mean_y = sum(y) / len(y)
    denominator = sum((value - mean_pitch) ** 2 for value in pitch)
    if denominator < 1e-9:
        raise ValueError("pitch 方差不足")
    slope = sum((p - mean_pitch) * (pixel_y - mean_y)
                for p, pixel_y in zip(pitch, y)) / denominator
    intercept = mean_y - slope * mean_pitch
    residual = sum((pixel_y - (slope * p + intercept)) ** 2
                   for p, pixel_y in zip(pitch, y))
    total = sum((pixel_y - mean_y) ** 2 for pixel_y in y)
    r2 = 1.0 - residual / total if total > 1e-9 else math.nan
    return {
        "pairs": len(pairs), "excluded": excluded, "pitch_span": pitch_span,
        "yaw_span": yaw_span, "y_span": max(y) - min(y), "slope": slope,
        "r2": r2,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="只读探针生成的 pitch sweep CSV")
    parser.add_argument("--output", type=Path, help="另存 Markdown 报告；默认只打印")
    args = parser.parse_args()
    rows = load_rows(args.csv)
    result = fit_pitch(rows)
    excluded = "、".join(f"{key} {value} 帧" for key, value in result["excluded"].items())
    report = "\n".join([
        "# Pitch 手动扫描：局部像素斜率", "",
        f"- 输入：`{args.csv.name}`，{len(rows)} 帧；有效配对 {result['pairs']} 帧。",
        f"- 筛除：{excluded}。",
        f"- pitch 跨度：{result['pitch_span']:.3f}°；yaw 同时变化："
        f"{result['yaw_span']:.3f}°；目标纵坐标跨度：{result['y_span']:.2f} px。",
        f"- 一元线性拟合：**dy/dpitch = {result['slope']:+.3f} px/°**，"
        f"R² = {result['r2']:.4f}。", "",
        "这是当前静止靶、安装与视场附近的经验映射；图像采集时刻和回传接收时刻"
        "只是同机软件时钟配对，并非传感器硬件同步。R² 是线性拟合度，"
        "不是检测准确率、测距精度或命中率。",
        "",
    ])
    print(report)
    if args.output:
        output = args.output
        protected = [args.csv, args.csv.with_suffix(".avi"),
                     args.csv.with_suffix(".config.yaml"),
                     args.csv.with_suffix(".session.txt")]
        if output.is_symlink() or any(output.resolve() == path.resolve() for path in protected):
            raise ValueError("报告路径不能覆盖原始数据或其链接")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(report, encoding="utf-8")
        print(f"已保存：{output}")


if __name__ == "__main__":
    main()
