#!/usr/bin/env python3
"""把现场 CSV 中已有的检测结果画到同名 AVI；不推理、不打开设备。"""

import argparse
import math
import os
from pathlib import Path
import tempfile

try:
    import cv2
except ImportError:
    raise SystemExit("回放标注需要 Python OpenCV（cv2）；请先完成环境检查。") from None

from analyze_field_session import load_rows, number


def text_line(image, text, origin, color=(255, 255, 255), size=0.55):
    """OpenCV 内置字体使用英文标签，避免中文变成问号。"""
    width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 1)[0][0]
    if width > image.shape[1] - 20:
        size *= max(1, image.shape[1] - 20) / width
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                size, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                size, color, 1, cv2.LINE_AA)


def annotate(image, row, output_size, playback_fps):
    """按已经校验的 CSV 绘制；多个候选时不会选择或捏造一个框。"""
    width, height = image.shape[1], image.shape[0]
    result = cv2.resize(image, output_size, interpolation=cv2.INTER_AREA)
    sx, sy = output_size[0] / width, output_size[1] / height

    def point(x, y):
        scaled = (x * sx, y * sy)
        if any(not math.isfinite(value) or abs(value) > (1 << 30) for value in scaled):
            raise ValueError(f"frame={row['frame']} 的角点过大，无法可靠绘制")
        return tuple(round(value) for value in scaled)

    center = point((width - 1) / 2, (height - 1) / 2)
    cv2.drawMarker(result, center, (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
    candidates = int(row["blue_candidates"])
    if candidates == 1:
        corners = [point(number(row, f"x{i}"), number(row, f"y{i}")) for i in range(4)]
        # 图外角点保留其连线的可见部分；不把图外点伪装成边缘上的角点。
        for index in range(4):
            first, second = corners[index], corners[(index + 1) % 4]
            visible, first, second = cv2.clipLine((0, 0, *output_size), first, second)
            if visible:
                cv2.line(result, first, second, (0, 255, 0), 2, cv2.LINE_AA)
        for index, corner in enumerate(corners):
            if 0 <= corner[0] < output_size[0] and 0 <= corner[1] < output_size[1]:
                cv2.circle(result, corner, 4, (0, 255, 0), 1, cv2.LINE_AA)
                text_line(result, str(index), (corner[0] + 6, corner[1] - 5), (0, 255, 0))
        target = point(number(row, "target_x"), number(row, "target_y"))
        cv2.circle(result, target, 5, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.line(result, center, target, (0, 200, 255), 1, cv2.LINE_AA)

    age = number(row, "feedback_age_ms")
    if age is None:
        feedback = "feedback=MISSING  mode=--  yaw=--  feedbackAge=--"
        feedback_color = (0, 180, 255)
    else:
        fresh = age <= 250
        feedback = (f"feedback={'FRESH' if fresh else 'STALE'}  mode={row['mode']}  "
                    f"yaw={number(row, 'yaw_deg'):+.3f} deg  feedbackAge={age:.1f} ms")
        feedback_color = (150, 255, 150) if fresh else (0, 180, 255)
    target_label = "UNIQUE" if candidates == 1 else "MISSING" if candidates == 0 else "MULTIPLE / no unique box"
    capture_label = "capture_s" if row["source"] == "camera" else "capture_s (decode clock)"
    lines = [
        (f"frame={row['frame']}  {capture_label}={number(row, 'capture_s'):.3f}  phase={row['phase']}", (255, 255, 255)),
        (f"blue candidates={candidates}  target={target_label}  imageAge={number(row, 'image_age_ms'):.1f} ms", (255, 255, 255)),
        (feedback, feedback_color),
        (f"Fixed playback {playback_fps:.3f} FPS; CSV time is software time, not exposure timing", (210, 210, 210)),
    ]
    panel_height = min(output_size[1], 104)
    result[:panel_height] = (result[:panel_height] * 0.3).astype(result.dtype)
    for index, (text, color) in enumerate(lines):
        text_line(result, text, (10, 21 + index * 24), color)
    return result


def render(csv_path, destination, scale):
    if not math.isfinite(scale) or not 0 < scale <= 4:
        raise ValueError("--scale 必须大于 0 且不超过 4")
    if destination.suffix.lower() != ".avi":
        raise ValueError("--output-video 必须使用 .avi 后缀")
    source_video = csv_path.with_suffix(".avi")
    # lexists 连悬空软链接也拒绝；已有硬链接同样是已有输出，绝不覆盖。
    if os.path.lexists(destination):
        raise ValueError("输出已存在（或是链接），请换一个新文件名：" + str(destination))
    if destination.resolve() in {csv_path.resolve(), source_video.resolve()}:
        raise ValueError("输出不能是原始 CSV 或 AVI")
    rows = load_rows(csv_path)
    if not source_video.is_file():
        raise ValueError("未找到与 CSV 同名的 AVI：" + str(source_video))
    width, height = int(rows[0]["width"]), int(rows[0]["height"])
    # MJPG 编码偶数宽高，避免后端静默截去末行/末列。
    output_size = tuple(int(round(size * scale)) // 2 * 2 for size in (width, height))
    if min(output_size) < 2:
        raise ValueError("缩放后的宽高须至少为 2 像素，请增大 --scale")
    capture = cv2.VideoCapture(str(source_video))
    writer = cv2.VideoWriter()
    verification = None
    temporary = None
    try:
        if not capture.isOpened():
            raise ValueError("无法解码输入 AVI：" + str(source_video))
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("输入 AVI 没有有效播放帧率，无法沿用原播放帧率")
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(prefix=".field_render_", suffix=".avi",
                                         dir=destination.parent, delete=False) as stream:
            temporary = Path(stream.name)
        writer.open(str(temporary), cv2.VideoWriter_fourcc(*"MJPG"), fps, output_size)
        if not writer.isOpened():
            raise ValueError("无法创建临时标注 AVI")
        for index, row in enumerate(rows):
            ok, image = capture.read()
            if not ok or image is None:
                raise ValueError(f"输入 AVI 只有 {index} 个可解码帧，CSV 有 {len(rows)} 行；停止输出")
            if image.shape[:2] != (height, width):
                raise ValueError(f"输入 AVI 第 {index} 帧尺寸与 CSV 不一致；停止输出")
            writer.write(annotate(image, row, output_size, fps))
        extra, _ = capture.read()
        if extra:
            raise ValueError(f"输入 AVI 超过 CSV 的 {len(rows)} 帧；停止输出")
        capture.release()
        writer.release()

        verification = cv2.VideoCapture(str(temporary))
        if not verification.isOpened():
            raise ValueError("临时标注 AVI 收尾后无法打开")
        decoded = 0
        while True:
            ok, image = verification.read()
            if not ok:
                break
            if image.shape[:2] != (output_size[1], output_size[0]):
                raise ValueError("临时标注 AVI 解码尺寸不一致")
            decoded += 1
        verification.release()
        if decoded != len(rows):
            raise ValueError(f"临时标注 AVI 仅可解码 {decoded} 帧，预期 {len(rows)} 帧；停止输出")
        # 创建新的目录项而不替换旧文件，即使运行中出现同名输出也会失败。
        os.link(temporary, destination)
        temporary.unlink()
        temporary = None
        print(f"标注完成并核对 {decoded} 帧：{destination}")
        print(f"输出尺寸 {output_size[0]}×{output_size[1]}，沿用输入固定播放帧率 {fps:.3f} FPS。")
        print("固定播放帧率不代表实际采集时序；逐帧软件时间以 CSV 为准。未运行模型推理或访问设备。")
        print("画面标记：绿色为唯一候选及角点 0～3，白色十字为图像中心；FRESH/STALE/MISSING 表示回传新鲜/过期/缺失。")
    finally:
        capture.release()
        writer.release()
        if verification is not None:
            verification.release()
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="现场记录 CSV，读取同名 AVI")
    parser.add_argument("--output-video", type=Path, required=True, help="新的标注 AVI 路径，已有文件不覆盖")
    parser.add_argument("--scale", type=float, default=0.75, help="图像缩放比例，默认 0.75；宽高取偶数")
    args = parser.parse_args()
    render(args.csv, args.output_video, args.scale)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, cv2.error) as error:
        raise SystemExit(f"回放标注失败：{error}")
    except KeyboardInterrupt:
        raise SystemExit("已取消标注；本次临时视频已清理，原始输入保留。") from None
