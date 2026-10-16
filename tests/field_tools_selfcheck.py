#!/usr/bin/env python3
"""现场离线工具的可复现回归检查：不连接真实设备，不读取现场录像。

运行：python3 tests/field_tools_selfcheck.py
依赖：Python 3、numpy、OpenCV、PyYAML。
所有 CSV、内参和报告都放入临时目录，测试结束自动清理。
这些检查证明软件对合成输入的行为，不代表相机、串口或云台通过实测。
"""

import argparse
from contextlib import redirect_stdout
import csv
import io
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest

try:
    import cv2
    import numpy as np
    import yaml
except ImportError as exc:
    raise SystemExit("自检需要 numpy、Python OpenCV、PyYAML，请先确认环境中可以导入这些包。") from exc

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "contest"))
sys.path.insert(0, str(ROOT / "calibration"))
import analyze_field_session as analysis
import calibrate_from_video as calibration
import evaluate_pnp_session as pnp


# 记录器的公开 29 列格式。独立写出契约，避免测试跟着解析器一起改变列名。
FIELDS = [
    "frame", "phase", "source", "width", "height", "capture_s", "processed_s", "image_age_ms",
    "raw_detections", "blue_candidates", "target_x", "target_y", "confidence",
    "x0", "y0", "x1", "y1", "x2", "y2", "x3", "y3",
    "feedback_received_s", "feedback_age_ms", "yaw_deg", "pitch_deg", "roll_deg", "mode",
    "robot_id", "bullet_speed_mps",
]
TARGET_KEYS = ["target_x", "target_y", "confidence", "x0", "y0", "x1", "y1", "x2", "y2", "x3", "y3"]


def make_row(index=0, *, camera=False, candidates=1):
    row = {key: "" for key in FIELDS}
    captured = index * 0.1
    row.update(frame=index, phase="static", source="camera" if camera else "video", width=960, height=720,
               capture_s=captured, processed_s=captured + 0.02, image_age_ms=20,
               raw_detections=candidates, blue_candidates=candidates)
    if candidates == 1:
        row.update(target_x=479.5, target_y=359.5, confidence=0.9,
                   x0=449.5, y0=344.5, x1=509.5, y1=344.5,
                   x2=509.5, y2=374.5, x3=449.5, y3=374.5)
    if camera:
        row.update(feedback_received_s=captured + 0.015, feedback_age_ms=5,
                   yaw_deg=index * 0.1, pitch_deg=0, roll_deg=0, mode=0, robot_id=3, bullet_speed_mps=0)
    return row


class TemporaryFiles(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="selfaim_offline_check_")
        self.folder = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def csv_file(self, name, rows):
        path = self.folder / name
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        return path


class SessionParsingChecks(TemporaryFiles):
    def test_complete_29_column_session(self):
        """完整数据应保留像素与角度，不把两种单位混在一起。"""
        source = self.csv_file("session.csv", [make_row(i, camera=True) for i in range(3)])
        rows = analysis.load_rows(source)
        self.assertEqual(len(FIELDS), 29)
        self.assertEqual(len(rows), 3)
        self.assertEqual(float(rows[2]["yaw_deg"]), 0.2)
        self.assertEqual(float(rows[2]["target_x"]), 479.5)

    def test_missing_and_multiple_targets_do_not_create_fake_centers(self):
        """无目标/多个候选时允许留空，但不能夹带上次的旧中心。"""
        source = self.csv_file("session.csv", [make_row(0, candidates=0), make_row(1, candidates=2)])
        rows = analysis.load_rows(source)
        self.assertEqual([row["target_x"] for row in rows], ["", ""])
        bad = make_row(candidates=2)
        bad["target_x"] = 479.5
        with self.assertRaisesRegex(ValueError, "候选不唯一"):
            analysis.load_rows(self.csv_file("stale_target.csv", [bad]))

    def test_partial_target_nan_and_inconsistent_timestamp_rejected(self):
        """缺列值、无穷数和相互矛盾的时钟不能继续算出漂亮报告。"""
        for key, value in (("x2", ""), ("confidence", "nan"), ("image_age_ms", 999)):
            with self.subTest(field=key):
                bad = make_row()
                bad[key] = value
                with self.assertRaises(ValueError):
                    analysis.load_rows(self.csv_file("bad.csv", [bad]))

    def test_sweep_filters_old_and_misaligned_feedback(self):
        """角度包虽存在，太旧或与图像差太远时也不能参与像素/角度斜率拟合。"""
        # 单独检查每种情况，使各记录的时钟自身保持合法。
        pairs, _ = analysis.sweep_samples(analysis.load_rows(self.csv_file("fresh.csv", [make_row(camera=True)])))
        self.assertEqual(len(pairs), 1)
        cases = [dict(make_row(camera=True), processed_s=0.3, image_age_ms=300,
                      feedback_received_s=0.295, feedback_age_ms=5),
                 dict(make_row(camera=True), feedback_received_s=-0.3, feedback_age_ms=320),
                 dict(make_row(camera=True), processed_s=0.2, image_age_ms=200,
                      feedback_received_s=0.195, feedback_age_ms=5)]
        for index, case in enumerate(cases):
            with self.subTest(case=index):
                validated = analysis.load_rows(self.csv_file(f"old_{index}.csv", [case]))
                pairs, excluded = analysis.sweep_samples(validated)
                self.assertEqual(pairs, [])
                self.assertEqual(sum(excluded.values()), 1)

    def test_report_cannot_overwrite_raw_data_or_its_links(self):
        """报告路径填错或指向原始数据链接时，CSV 必须原封不动。"""
        source = self.csv_file("session.csv", [make_row()])
        original = source.read_bytes()
        link = self.folder / "linked.md"
        link.symlink_to(source)
        hard_link = self.folder / "hard_link.md"
        os.link(source, hard_link)
        for target in (source, link, hard_link):
            with self.subTest(target=target.name):
                with self.assertRaises(ValueError):
                    analysis.write_report(source, target, "不应写入")
                self.assertEqual(source.read_bytes(), original)
        report = self.folder / "report.md"
        analysis.write_report(source, report, "正常报告")
        self.assertEqual(report.read_text(), "正常报告")


class PoseChecks(TemporaryFiles):
    def setUp(self):
        super().setUp()
        self.matrix = np.array([[950.0, 0, 479.5], [0, 960.0, 359.5], [0, 0, 1.0]])
        self.distortion = np.array([-0.08, 0.02, 0.001, -0.002, 0.003])
        self.translation = np.array([0.2, 0.1, 2.4])
        self.object_points = np.array([[-0.08, -0.032, 0], [0.08, -0.032, 0],
                                       [0.08, 0.032, 0], [-0.08, 0.032, 0]])
        self.intrinsics = self.folder / "intrinsics.yaml"
        self.parameters = dict(image_width=960, image_height=720, camera_matrix=self.matrix.ravel().tolist(),
                               distort_coeffs=self.distortion.tolist())
        self.intrinsics.write_text(yaml.safe_dump(self.parameters))

    def projected_row(self, index=0):
        rotation = np.array([0.2 + index * 0.01, 0.35 + index * 0.004, -0.1 + index * 0.005])
        image, _ = cv2.projectPoints(self.object_points, rotation, self.translation, self.matrix, self.distortion)
        image = image.reshape(-1, 2)
        row = make_row(index)
        row.update(target_x=image[:, 0].mean(), target_y=image[:, 1].mean())
        for point, (x, y) in enumerate(image):
            row[f"x{point}"], row[f"y{point}"] = x, y
        return row

    def test_distorted_synthetic_pose_recovers_translation_and_keeps_two_solutions(self):
        """已知相机+畸变+姿态投影出四角，再求回应恢复距离，同时保留平面双解。"""
        rows = analysis.load_rows(self.csv_file("pose.csv", [self.projected_row(i) for i in range(4)]))
        matrix, distortion = pnp.read_intrinsics(self.intrinsics, rows)
        for row in rows:
            status, solutions, _ = pnp.solve_frame(row, self.object_points, matrix, distortion)
            self.assertEqual(status, "ok")
            self.assertEqual(len(solutions), 2)
            np.testing.assert_allclose(solutions[0]["tvec"], self.translation, atol=1e-7, rtol=0)
            self.assertLess(solutions[0]["rms"], 1e-6)
            self.assertLessEqual(solutions[0]["rms"], solutions[1]["rms"])

    def test_wrong_physical_scale_doubles_range_without_large_pixel_error(self):
        """尺寸填错可能仍有很小重投影误差；距离必须跟着错误尺寸同比例变化。"""
        row = self.projected_row()
        _, solutions, _ = pnp.solve_frame(row, self.object_points * 2, self.matrix, self.distortion)
        self.assertAlmostEqual(solutions[0]["range"], 2 * np.linalg.norm(self.translation), places=7)
        self.assertLess(solutions[0]["rms"], 1e-6)

    def test_degenerate_and_missing_target_do_not_output_a_pose(self):
        """四点共线、没有目标时，绝不能填入零距离等假位姿。"""
        row = self.projected_row()
        for index in range(4):
            row[f"x{index}"], row[f"y{index}"] = 450 + index * 10, 360
        for candidate in (row, make_row(candidates=0), make_row(candidates=2)):
            status, solutions, _ = pnp.solve_frame(candidate, self.object_points, self.matrix, self.distortion)
            self.assertNotEqual(status, "ok")
            self.assertEqual(solutions, [])

    def test_resolution_mismatch_is_rejected(self):
        """不同分辨率的内参不能直接混用。"""
        rows = analysis.load_rows(self.csv_file("pose.csv", [self.projected_row()]))
        self.parameters["image_width"] = 1440
        self.intrinsics.write_text(yaml.safe_dump(self.parameters))
        with self.assertRaisesRegex(ValueError, "不匹配"):
            pnp.read_intrinsics(self.intrinsics, rows)

    def test_all_failed_session_has_failure_report_and_no_fake_pose(self):
        """全段无目标须宣告失败，仍留下原因报告供现场排查。"""
        source = self.csv_file("missing.csv", [make_row(candidates=0)])
        out = self.folder / "result"
        args = argparse.Namespace(csv_path=source, intrinsics=self.intrinsics, width_mm=160,
                                  height_mm=64, known_distance_m=None, output_dir=out)
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "所有帧均无可用解"):
            pnp.run(args)
        with (out / "poses.csv").open() as handle:
            poses = list(csv.DictReader(handle))
        self.assertEqual(poses[0]["status"], "no_target")
        self.assertEqual(poses[0]["range_m"], "")
        self.assertIn("失败", (out / "report.md").read_text())


class CalibrationChecks(TemporaryFiles):
    def test_projected_grid_calibration_and_rms_definition(self):
        """直接生成多姿态网格投影：验证数值求解及 RMS，不假称测过相机或角点检测。"""
        size = (960, 720)
        truth = np.array([[950.0, 0, 479.5], [0, 960.0, 359.5], [0, 0, 1.0]])
        distortion = np.array([-0.06, 0.01, 0.001, -0.001, 0.0])
        grid = np.zeros((9 * 6, 3), np.float32)
        grid[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.025
        random = np.random.default_rng(20261013)
        views = []
        for index in range(18):
            rvec = random.uniform([-0.5, -0.5, -0.2], [0.5, 0.5, 0.2])
            rotation, _ = cv2.Rodrigues(rvec)
            center = random.uniform([-0.1, -0.07, 0.48], [0.1, 0.07, 0.75])
            tvec = center - rotation @ np.array([0.1, 0.0625, 0])
            image, _ = cv2.projectPoints(grid, rvec, tvec, truth, distortion)
            # 小量固定种子的像素噪声，使 RMS 检验不是“全部恒等于零”的空测试。
            image += random.normal(0, 0.03, image.shape).astype(np.float32)
            views.append(dict(frame=index, time=index * 0.1, points=image,
                              center=image.reshape(-1, 2).mean(axis=0) / size,
                              area_fraction=cv2.contourArea(cv2.convexHull(image)) / math.prod(size)))
        args = argparse.Namespace(cols=9, rows=6, spacing_mm=25, board="chessboard",
                                  video=self.folder / "synthetic_points_only.avi", output_dir=self.folder)
        result = calibration.calibrate(args, size, views)
        parameters = yaml.safe_load((self.folder / "camera_intrinsics.yaml").read_text())
        self.assertEqual(len(parameters["camera_matrix"]), 9)
        self.assertEqual(len(parameters["distort_coeffs"]), 5)
        estimated = np.array(parameters["camera_matrix"]).reshape(3, 3)
        self.assertLess(abs(estimated[0, 0] / truth[0, 0] - 1), 0.01)
        self.assertLess(abs(estimated[1, 1] / truth[1, 1] - 1), 0.01)
        with (self.folder / "views.csv").open() as handle:
            saved = list(csv.DictReader(handle))
        squared_errors = []
        for actual, pose in zip(views, saved):
            rvec = np.array([float(pose[key]) for key in ("rvec_x", "rvec_y", "rvec_z")])
            tvec = np.array([float(pose[key]) for key in ("tvec_x_m", "tvec_y_m", "tvec_z_m")])
            predicted, _ = cv2.projectPoints(grid, rvec, tvec, estimated, np.array(parameters["distort_coeffs"]))
            residual = actual["points"].reshape(-1, 2) - predicted.reshape(-1, 2)
            squared = (residual ** 2).sum(axis=1)
            squared_errors.extend(squared)
            self.assertAlmostEqual(math.sqrt(float(squared.mean())), float(pose["reprojection_rms_px"]), places=4)
        self.assertEqual(len(saved), len(views))
        self.assertAlmostEqual(math.sqrt(float(np.mean(squared_errors))), result["rms"], places=4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
