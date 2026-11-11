"""Offline RGB-D localization tests; synthetic frames only, no hardware access."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from evoarm_rsi.localize import locate_saved_trial, unproject_depth


class FakeVideo:
    def __init__(self, frame):
        self.frame = frame
        self.open = True
        self.position = 0

    def isOpened(self):
        return self.open

    def set(self, key, value):
        self.position = value

    def read(self):
        return True, self.frame.copy()

    def release(self):
        self.open = False


class RGBDLocalizationTests(unittest.TestCase):
    def test_unproject_requires_real_intrinsics(self):
        self.assertEqual(
            unproject_depth(420, 300, 500, {
                "fx": 400, "fy": 400, "cx": 320, "cy": 240,
            }),
            [125.0, 75.0, 500.0],
        )
        with self.assertRaises(ValueError):
            unproject_depth(400, 300, 500, {"fx": 0, "fy": 400, "cx": 320, "cy": 240})

    def _sample(self, root):
        root = Path(root)
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        # Yellow circle at x=160, y=110, radius=12 pixels.
        cv2.circle(frame, (160, 110), 12, (0, 255, 255), -1)
        depth = np.full((240, 320), 600, dtype=np.uint16)
        cv2.circle(depth, (160, 110), 7, 560, -1)
        (root/"manifest.json").write_text(json.dumps({
            "trial_id": "rgbd-test", "simulated": False
        }))
        (root/"frames.jsonl").write_text("\n".join([
            json.dumps({"host_monotonic_ns": 1_000_000_000 + n*100_000_000})
            for n in range(4)
        ])+"\n")
        (root/"rgb.mp4").write_bytes(b"test mock video")
        np.savez_compressed(
            root/"depth_samples.npz",
            depth_mm=np.stack([depth,depth]),
            monotonic_ns=np.array([1_000_000_000,1_200_000_000],dtype=np.int64),
        )
        return frame

    def test_saved_trial_detects_yellow_depth_and_does_not_invent_robot_xyz(self):
        with tempfile.TemporaryDirectory() as d:
            frame = self._sample(d)
            grid = np.zeros((6,9,2),dtype=np.float32)
            # Pixel -> board-grid homography (unit scale)
            H = np.eye(3,dtype=np.float64)
            with (patch("cv2.VideoCapture",return_value=FakeVideo(frame)),
                  patch("control_ui.vision.find_chessboard",return_value=(grid,H))):
                report = locate_saved_trial(d)
            self.assertEqual(report["ball_frames_detected"],2)
            self.assertEqual(report["ball_frames_with_valid_depth"],2)
            self.assertEqual(report["ball_frames_with_live_board_xy"],2)
            self.assertEqual(report["robot_target_xyz_mm"],None)
            self.assertFalse(report["motion_authorized"])
            self.assertFalse(report["camera_intrinsics_verified"])
            self.assertAlmostEqual(report["median_ball_surface_depth_mm"],560,delta=1)
            obs=report["observations"][0]
            self.assertAlmostEqual(obs["apparent_depth_difference_mm"],40,delta=1)
            self.assertIsNone(obs["ball_camera_surface_xyz_mm"])
            self.assertTrue((Path(d)/"rgbd_localization.json").exists())

    def test_intrinsics_need_verified_color_alignment(self):
        with tempfile.TemporaryDirectory() as d:
            frame=self._sample(d)
            path=Path(d)/"intrinsics.json"
            base={"fx":400,"fy":400,"cx":160,"cy":120,"aligned_to":"depth",
                  "verified":True}
            path.write_text(json.dumps(base))
            with self.assertRaisesRegex(ValueError,"COLOR-aligned"):
                locate_saved_trial(d,intrinsics_file=path)
            base["aligned_to"]="color"
            base["verified"]=False
            path.write_text(json.dumps(base))
            with self.assertRaisesRegex(ValueError,"not marked verified"):
                locate_saved_trial(d,intrinsics_file=path)
            base["verified"]=True
            path.write_text(json.dumps(base))
            with (patch("cv2.VideoCapture",return_value=FakeVideo(frame)),
                  patch("control_ui.vision.find_chessboard",return_value=(None,None))):
                report=locate_saved_trial(d,intrinsics_file=path)
            self.assertTrue(report["camera_intrinsics_verified"])
            self.assertIsNotNone(report["observations"][0]["ball_camera_surface_xyz_mm"])
            self.assertIsNone(report["robot_target_xyz_mm"])

    def test_unmatched_depth_and_rgb_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            self._sample(d)
            data=np.load(Path(d)/"depth_samples.npz")
            depth=data["depth_mm"].copy()
            data.close()
            np.savez_compressed(
                Path(d)/"depth_samples.npz",depth_mm=depth,
                monotonic_ns=np.array([5_000_000_000,5_100_000_000],dtype=np.int64)
            )
            with self.assertRaisesRegex(ValueError,"no depth sample aligned"):
                locate_saved_trial(d)


if __name__=="__main__":
    unittest.main()
