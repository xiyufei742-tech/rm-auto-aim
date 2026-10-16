#!/usr/bin/env python3
"""Summarize a read-only field session CSV; no camera or serial access."""

import argparse
import csv
import math
import os
from pathlib import Path
from statistics import median
import tempfile


TARGET_FIELDS = ("target_x", "target_y", "confidence", "x0", "y0", "x1", "y1",
                 "x2", "y2", "x3", "y3")
FEEDBACK_FIELDS = ("feedback_received_s", "feedback_age_ms", "yaw_deg", "pitch_deg",
                   "roll_deg", "mode", "robot_id", "bullet_speed_mps")
REQUIRED = {
    "frame", "phase", "source", "width", "height", "capture_s", "processed_s",
    "image_age_ms", "raw_detections", "blue_candidates",
    *TARGET_FIELDS, *FEEDBACK_FIELDS,
}
# CSV 时间戳保留到微秒，两个时间戳相减最多引入约 0.001 ms 舍入误差。
TIME_TOLERANCE_MS = 0.003


def number(row, key):
    value = row[key]
    if value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"frame={row['frame']} 的 {key} 不是数字") from None
    if not math.isfinite(result):
        raise ValueError(f"frame={row['frame']} 的 {key} 不是有限数")
    return result


def required_number(row, key):
    result = number(row, key)
    if result is None:
        raise ValueError(f"frame={row['frame']} 的 {key} 不能为空")
    return result


def integer(row, key, minimum=None, maximum=None):
    value = row[key]
    digits = value[1:] if value.startswith(("+", "-")) else value
    if not digits or any(char not in "0123456789" for char in digits):
        raise ValueError(f"frame={row['frame']} 的 {key} 必须是整数")
    result = int(value)
    if ((minimum is not None and result < minimum) or
            (maximum is not None and result > maximum)):
        raise ValueError(f"frame={row['frame']} 的 {key} 超出范围")
    return result


def percentile(values, percent):
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * percent / 100.0
    low, high = math.floor(index), math.ceil(index)
    return values[low] + (values[high] - values[low]) * (index - low)


def display(value, digits=2):
    return "无数据" if value is None else f"{value:.{digits}f}"


def runs(rows, predicate):
    result = []
    start = None
    for index, row in enumerate(rows):
        if predicate(row):
            if start is None:
                start = index
        elif start is not None:
            result.append((start, index - 1))
            start = None
    if start is not None:
        result.append((start, len(rows) - 1))
    return sorted(result, key=lambda segment: segment[1] - segment[0] + 1, reverse=True)


def sweep_samples(rows):
    """Only pair fresh software timestamps; these are not hardware-synced samples."""
    pairs = []
    excluded = {"非相机来源": 0, "候选不唯一或缺失": 0, "缺少回传": 0,
                "图像年龄超过 250 ms": 0, "回传年龄超过 250 ms": 0,
                "回传接收与图像采集相差超过 100 ms": 0}
    for row in rows:
        if row["source"] != "camera":
            excluded["非相机来源"] += 1
            continue
        if int(row["blue_candidates"]) != 1:
            excluded["候选不唯一或缺失"] += 1
            continue
        if row["feedback_received_s"] == "":
            excluded["缺少回传"] += 1
            continue
        if number(row, "image_age_ms") > 250:
            excluded["图像年龄超过 250 ms"] += 1
            continue
        if number(row, "feedback_age_ms") > 250:
            excluded["回传年龄超过 250 ms"] += 1
            continue
        delta_ms = abs(number(row, "feedback_received_s") -
                       number(row, "capture_s")) * 1000
        if delta_ms > 100 + 1e-9:
            excluded["回传接收与图像采集相差超过 100 ms"] += 1
            continue
        x, yaw = number(row, "target_x"), number(row, "yaw_deg")
        pairs.append((yaw, x))
    return pairs, excluded


def sweep_fit(rows):
    """Rough local dx/dyaw fit for a stationary target during manual sweep."""
    pairs, _ = sweep_samples(rows)
    if len(pairs) < 20:
        return None, "有效成对样本少于 20 帧"
    yaw_span = max(yaw for yaw, _ in pairs) - min(yaw for yaw, _ in pairs)
    if yaw_span < 1.0:
        return None, "有效 yaw 扫动不足 1°"
    mean_yaw = sum(yaw for yaw, _ in pairs) / len(pairs)
    mean_x = sum(x for _, x in pairs) / len(pairs)
    xx = sum((yaw - mean_yaw) ** 2 for yaw, _ in pairs)
    if xx < 1e-9:
        return None, "角度变化不足"
    slope = sum((yaw - mean_yaw) * (x - mean_x) for yaw, x in pairs) / xx
    intercept = mean_x - slope * mean_yaw
    residual = sum((x - (slope * yaw + intercept)) ** 2 for yaw, x in pairs)
    total = sum((x - mean_x) ** 2 for _, x in pairs)
    r2 = 1 - residual / total if total > 1e-9 else None
    return (len(pairs), yaw_span, slope, r2), None


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not REQUIRED.issubset(reader.fieldnames):
            missing = sorted(REQUIRED - set(reader.fieldnames or []))
            raise ValueError("CSV 缺少字段：" + ", ".join(missing))
        if len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise ValueError("CSV 有重复字段名")
        rows = list(reader)
    if not rows:
        raise ValueError("CSV 没有帧数据")
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise ValueError("CSV 某行列数不一致")
    expected_phase = rows[0]["phase"]
    expected_source = rows[0]["source"]
    if expected_phase not in {"static", "sweep", "rotating"}:
        raise ValueError("phase 必须是 static、sweep 或 rotating")
    if expected_source not in {"camera", "video"}:
        raise ValueError("source 必须是 camera 或 video")
    width, height = (integer(rows[0], key, 2) for key in ("width", "height"))
    previous_time = None
    previous_processed = None
    previous_feedback_time = None
    for index, row in enumerate(rows):
        if integer(row, "frame", 0) != index:
            raise ValueError(f"frame 编号应从 0 连续，检查第 {index} 行")
        if row["phase"] != expected_phase or row["source"] != expected_source:
            raise ValueError("同一会话混入不同 phase/source")
        if integer(row, "width", 2) != width or integer(row, "height", 2) != height:
            raise ValueError("会话中分辨率改变")
        captured = required_number(row, "capture_s")
        processed = required_number(row, "processed_s")
        image_age = required_number(row, "image_age_ms")
        if (captured < 0 or (index == 0 and captured != 0) or
                (previous_time is not None and captured < previous_time)):
            raise ValueError(f"frame={index} 的采集时间须从 0 开始且不递减")
        if (processed < captured or image_age < 0 or
                (previous_processed is not None and processed < previous_processed)):
            raise ValueError(f"frame={index} 的处理时间或图像年龄无效")
        if abs((processed - captured) * 1000 - image_age) > TIME_TOLERANCE_MS:
            raise ValueError(f"frame={index} 的图像年龄与采集/处理时间不一致")
        previous_time = captured
        previous_processed = processed
        raw_count = integer(row, "raw_detections", 0)
        candidate_count = integer(row, "blue_candidates", 0, raw_count)
        if candidate_count == 1:
            target = {key: required_number(row, key) for key in TARGET_FIELDS}
            if not (0 <= target["target_x"] < width and 0 <= target["target_y"] < height):
                raise ValueError(f"frame={index} 的目标中心超出图像范围")
            if not 0 <= target["confidence"] <= 1:
                raise ValueError(f"frame={index} 的 confidence 必须在 0～1 之间")
            # 角点必须有限；部分出画的装甲板允许角点位于图外，中心仍须在图内。
        elif any(row[key] != "" for key in TARGET_FIELDS):
            raise ValueError(f"frame={index} 的候选不唯一，目标字段应全部留空")
        feedback_present = [row[key] != "" for key in FEEDBACK_FIELDS]
        if any(feedback_present):
            if not all(feedback_present):
                raise ValueError(f"frame={index} 的回传字段必须全部为空或全部填写")
            if expected_source == "video":
                raise ValueError(f"frame={index} 的离线视频不能配入当前实时回传")
            feedback = {key: required_number(row, key) for key in FEEDBACK_FIELDS}
            received, age = feedback["feedback_received_s"], feedback["feedback_age_ms"]
            if (age < 0 or received > processed or
                    (previous_feedback_time is not None and received < previous_feedback_time)):
                raise ValueError(f"frame={index} 的回传时间或年龄无效")
            if abs((processed - received) * 1000 - age) > TIME_TOLERANCE_MS:
                raise ValueError(f"frame={index} 的回传年龄与接收/处理时间不一致")
            previous_feedback_time = received
            if (abs(feedback["yaw_deg"]) > 720 or abs(feedback["pitch_deg"]) > 180 or
                    abs(feedback["roll_deg"]) > 180 or
                    not 0 <= feedback["bullet_speed_mps"] <= 100):
                raise ValueError(f"frame={index} 的回传角度或弹速超出记录器范围")
            integer(row, "mode", 0, 10)
            integer(row, "robot_id", 0, 50)
    return rows


def session_status(path, row_count):
    """Read the recorder's historical completion marker; do not certify the AVI now."""
    sidecar = path.with_suffix(".session.txt")
    if not sidecar.exists():
        return "未找到同名 .session.txt：未核对 AVI 完整性，以下仅分析当前 CSV。"
    metadata = {}
    for line in sidecar.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if separator and key.strip() in {"status", "saved_frames"}:
            key = key.strip()
            if key in metadata:
                return "部分记录：会话状态文件有重复字段，无法确认采集是否正常完成。"
            metadata[key] = value.strip()
    saved = metadata.get("saved_frames", "")
    if not saved or any(char not in "0123456789" for char in saved):
        return "部分记录：会话状态文件缺少有效 saved_frames，无法核对 CSV 行数。"
    saved_count = int(saved)
    if metadata.get("status") != "complete_frame_count_verified":
        return (f"部分记录：记录器未标记正常完成；状态文件记录 {saved_count} 帧，"
                f"当前 CSV 有 {row_count} 帧。AVI 需另行检查。")
    if saved_count != row_count:
        return (f"部分记录：状态文件称保存 {saved_count} 帧，当前 CSV 有 {row_count} 帧，"
                "帧数不一致；不能沿用采集完成时的核对结果。")
    suffix = "当前同名 AVI 缺失。" if not path.with_suffix(".avi").is_file() else ""
    return (f"记录器标记采集完成时已完整解码 AVI 并核对 {saved_count} 帧，"
            "与当前 CSV 行数一致。分析器本次没有重新解码 AVI，"
            "该标记不能证明文件之后未损坏。" + suffix)


def summarize(path, rows):
    phase, source = rows[0]["phase"], rows[0]["source"]
    width = int(rows[0]["width"])
    n = len(rows)
    unique = [row for row in rows if int(row["blue_candidates"]) == 1]
    no_target = [row for row in rows if int(row["blue_candidates"]) == 0]
    multiple = [row for row in rows if int(row["blue_candidates"]) > 1]
    confidence = [number(row, "confidence") for row in unique]
    confidence = [value for value in confidence if value is not None]
    ages = [number(row, "image_age_ms") for row in rows]
    feedback_ages = [number(row, "feedback_age_ms") for row in rows]
    feedback_ages = [age for age in feedback_ages if age is not None]
    fresh_feedback = [age for age in feedback_ages if 0 <= age <= 250]
    capture_span = number(rows[-1], "capture_s") - number(rows[0], "capture_s")
    observed_fps = (n - 1) / capture_span if capture_span > 0 and n > 1 else None
    center_x = (width - 1) / 2
    errors = [number(row, "target_x") - center_x for row in unique]
    centered = runs(rows, lambda row: int(row["blue_candidates"]) == 1 and
                    abs(number(row, "target_x") - center_x) <= 3)
    misses = runs(rows, lambda row: int(row["blue_candidates"]) != 1)
    modes = {}
    for row in rows:
        age = number(row, "feedback_age_ms")
        if age is not None and 0 <= age <= 250:
            mode = str(integer(row, "mode", 0, 10))
            modes[mode] = modes.get(mode, 0) + 1
    jumps = []
    for first, second in zip(rows, rows[1:]):
        if int(first["blue_candidates"]) == int(second["blue_candidates"]) == 1:
            delta = abs(number(second, "target_x") - number(first, "target_x"))
            if delta > 30:
                jumps.append((int(second["frame"]), delta))

    time_label = "采集时刻跨度" if source == "camera" else "离线解码/处理时刻跨度"
    age_label = "图像处理年龄" if source == "camera" else "离线单帧处理耗时"
    lines = [
        "# 现场会话分析", "",
        f"- 数据：`{path.name}`；阶段：`{phase}`；来源：`{source}`。",
        f"- 共 {n} 帧，图像宽 {width} px，{time_label} {capture_span:.2f} s。",
        "- 会话状态：" + session_status(path, n),
        "- 这份报告统计程序输出，不包含人工标注真值；候选覆盖率不能称为检测准确率或命中率。",
        "- `raw_detections` 是 YOLO 接口返回、已经过 NMS 与类别/分数过滤的装甲板数；随后筛选得到 `blue_candidates`。它不是模型原始输出框数。",
        "", "## 关键数值", "",
        "| 指标 | 结果 |", "|---|---:|",
        f"| 唯一蓝色候选 | {len(unique)}/{n}（{len(unique)/n:.1%}） |",
        f"| 没有合格蓝色候选 | {len(no_target)}/{n}（{len(no_target)/n:.1%}） |",
        f"| 多个合格蓝色候选 | {len(multiple)}/{n}（{len(multiple)/n:.1%}） |",
        f"| 模型分数中位数 / 最低值 | {display(median(confidence) if confidence else None, 3)} / {display(min(confidence) if confidence else None, 3)} |",
        f"| {age_label} P50 / P95 / 最大值 | {display(percentile(ages,50))} / {display(percentile(ages,95))} / {display(max(ages))} ms |",
        f"| 回传年龄 P50 / P95 / 最大值 | {display(percentile(feedback_ages,50))} / {display(percentile(feedback_ages,95))} / {display(max(feedback_ages) if feedback_ages else None)} ms |",
        f"| 新鲜回传（0～250 ms） | {len(fresh_feedback)}/{n} 帧 |",
        f"| 有新鲜回传的图像帧数（按 mode 分组） | {', '.join(f'{key}:{value}' for key,value in sorted(modes.items())) or '无新鲜回传'} |",
        f"| 横向偏差范围 | {display(min(errors) if errors else None)} ～ {display(max(errors) if errors else None)} px |",
        f"| 单帧横向变化 >30 px | {len(jumps)} 次 |",
    ]
    if source == "camera":
        lines.append(f"| 同机取帧时刻推算的处理率 | {display(observed_fps)} 帧/s |")
    else:
        lines.extend(["", "离线视频按程序解码速度处理；上述时间跨度不是原视频真实拍摄时序。"])

    lines.extend(["", "## 时间段定位", ""])
    if misses:
        lines.append("最长的候选不唯一/缺失区间（按帧数排序）：")
        lines.append("")
        for start, end in misses[:5]:
            lines.append(f"- frame {start}～{end}，{end-start+1} 帧；采集时刻 {rows[start]['capture_s']}～{rows[end]['capture_s']} s。")
    else:
        lines.append("所有帧均只有一个合格蓝色候选。")
    lines.extend(["", f"进入画面中心 ±3 px 的帧数：{sum(end-start+1 for start,end in centered)}；最长连续 {centered[0][1]-centered[0][0]+1 if centered else 0} 帧。"])
    if jumps:
        lines.extend(["", "较大的单帧横向变化（可能是转动、目标切换或识别异常，需看原视频）：", ""])
        for frame, delta in jumps[:10]:
            lines.append(f"- frame {frame}：变化 {delta:.1f} px。")

    if phase == "sweep":
        pairs, excluded = sweep_samples(rows)
        result, reason = sweep_fit(rows)
        lines.extend(["", "## 手动 yaw 扫描的局部斜率", ""])
        lines.append(f"总计 {n} 帧，保留 {len(pairs)} 帧，排除 {sum(excluded.values())} 帧；每帧按以下顺序计入首个不满足的条件：")
        lines.append("")
        for label, count in excluded.items():
            lines.append(f"- {label}：{count} 帧。")
        lines.extend(["", "参与拟合须为唯一蓝色候选，图像与回传年龄均不超过 250 ms，回传接收与图像采集时刻的绝对差不超过 100 ms。这只是软件时间配对筛选，不能证明曝光与姿态测量同步。", ""])
        if reason:
            lines.append("暂不计算：" + reason + "。")
        else:
            count, yaw_span, slope, r2 = result
            lines.append(f"用 {count} 帧、yaw 跨度 {yaw_span:.2f}° 作一元线性拟合，得到 **dx/dyaw≈{slope:+.2f} px/°**，R²={display(r2,3)}。")
            lines.append("仅适用于静止靶、固定相机与本次视场附近；回传接收时间并非姿态采样时间，扫描速度过快会引入时间错位。")
            lines.append("和上次 +32.1 px/° 对照，差异大时先查镜头/安装/转动中的目标或识别跳变，不直接改控制参数。")
    elif phase == "rotating":
        lines.extend(["", "## 旋转靶判读", "", "把最长缺失区间的帧号带到同名 AVI 逐帧查看：板是否背对相机、灯条拖影、角点跳到别的亮斑。", "候选数覆盖率不能表示旋转靶的真实召回率；需人工标注‘可见且应检出’的帧后才能计算。"])
    else:
        lines.extend(["", "## 静止靶判读", "", "查看是否能连续稳定检出以及横向偏差是否逐步接近 0。若做了无弹跟随，必须结合独立的实际回传确认回位和 mode。"])

    lines.extend(["", "## 使用界限", "", "- AVI 使用固定播放帧率；真实软件取帧与回传接收时刻记录在 CSV。", "- `feedback_age_ms` 是本机收到回传后的年龄，不是相机曝光到下位机采样的总延迟。", "- mode 统计按图像帧计数，同一条回传可能对应多帧图像；它不是独立回传包数量。", "- 本分析程序只读 CSV；它不发送控制或开火指令，也没有检查 AVI 是否完整或与 CSV 等帧数。", ""])
    return "\n".join(lines)


def write_report(source, destination, report):
    """Protect the raw pair, including aliases, then replace only the report entry."""
    if destination.suffix.lower() != ".md":
        raise ValueError("报告输出须使用 .md 后缀，以免覆盖 CSV/AVI 数据")
    protected = {source}
    for base in (source, source.resolve()):
        protected.update(base.with_suffix(suffix) for suffix in
                         (".avi", ".config.yaml", ".session.txt"))
    for path in protected:
        if (destination.resolve() == path.resolve() or
                (destination.exists() and path.exists() and destination.samefile(path))):
            raise ValueError("报告路径指向原始会话文件或其链接，拒绝覆盖：" + str(destination))
    if destination.is_symlink() and destination.resolve().suffix.lower() in {".csv", ".avi"}:
        raise ValueError("报告路径是指向 CSV/AVI 的链接，拒绝覆盖：" + str(destination))
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        # 同目录临时文件 + 原子替换不会通过输出文件的硬链接修改原始数据。
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=destination.parent,
                                         prefix="." + destination.name + ".", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(report)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="srm_live_pixel_probe --session 生成的 CSV")
    parser.add_argument("--output", type=Path, help=".md 报告路径，默认与 CSV 同目录且后缀为 .report.md")
    args = parser.parse_args()
    rows = load_rows(args.csv)
    report = summarize(args.csv, rows)
    destination = args.output or args.csv.with_suffix(".report.md")
    write_report(args.csv, destination, report)
    print(f"已分析 {len(rows)} 帧：{destination}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, csv.Error) as error:
        raise SystemExit(f"分析失败：{error}")
