#!/usr/bin/env python3
"""独立棋盘测距工具的合成数据自检；不连接相机或控制设备。

运行：python3 tests/validate_board_distance_selfcheck.py
"""

import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "calibration"))
import validate_board_distance as checker


class BoardDistanceChecks(unittest.TestCase):
    def test_distorted_off_axis_board_recovers_euclidean_distance_with_outlier(self):
        """偏轴/倾斜的 5 m 棋盘，距离必须取中心向量范数并抗一个错点。"""
        matrix = np.array([[1799.5, 0, 738.9], [0, 1799.3, 536.2], [0, 0, 1]], dtype=float)
        distortion = np.array([-0.079, 0.19, -0.0004, 0.0007, -0.159], dtype=float)
        grid = checker.object_grid(8, 11, 30)
        rvec = np.array([0.2, -0.15, 0.05], dtype=float)
        center = np.array([0.25, -0.16, 4.92], dtype=float)
        points, _ = cv2.projectPoints(grid, rvec, center, matrix, distortion)
        points = points.reshape(-1, 2)
        points += np.random.default_rng(42).normal(0, 0.12, points.shape)
        points[3] += (20, -12)
        result = checker.estimate_pose(grid, points, matrix, distortion)
        self.assertIsNotNone(result)
        self.assertLess(abs(result["distance_m"] - np.linalg.norm(center)), 0.01)
        self.assertGreater(result["distance_m"] - result["z_m"], 0.005)
        self.assertLess(result["inliers"], len(grid))
        self.assertGreater(result["rms_all_px"], result["rms_inlier_px"])

    def test_same_video_copy_rejected_and_bad_intrinsics_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source = folder / "calibration.avi"
            copy = folder / "renamed.avi"
            source.write_bytes(b"calibration-video-content")
            copy.write_bytes(source.read_bytes())
            with self.assertRaisesRegex(ValueError, "字节相同"):
                checker.check_independent_video(copy, source)
            copy.write_bytes(b"new independent video content")
            self.assertIn("不同", checker.check_independent_video(copy, source))

            fields = {"image_width": 1440, "image_height": 1080,
                      "camera_matrix": [1799, 0, 739, 0, 1799, 536, 0, 0, 1],
                      "distort_coeffs": [-0.08, 0.19, 0, 0, -0.15],
                      "source_video": str(source)}
            intrinsics = folder / "intrinsics.yaml"
            intrinsics.write_text("\n".join(f"{key}: {json.dumps(value)}" for key, value in fields.items()))
            size, _, _, _ = checker.load_intrinsics(intrinsics)
            self.assertEqual(size, (1440, 1080))
            fields["camera_matrix"][0] = float("nan")
            intrinsics.write_text("\n".join(f"{key}: {json.dumps(value)}" for key, value in fields.items()))
            with self.assertRaisesRegex(ValueError, "内参无效"):
                checker.load_intrinsics(intrinsics)


if __name__ == "__main__":
    unittest.main()
