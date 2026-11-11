"""The observed 15fps/20fps timing mismatch, with synthetic video only."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from evoarm_rsi.video_timing import correct_video_timing
from evoarm_rsi.camera import CameraFrame, TrialVideoRecorder


class VideoTimingTests(unittest.TestCase):
    def fixture(self, folder):
        path = Path(folder)
        writer = cv2.VideoWriter(str(path/"rgb.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 20, (80, 60))
        self.assertTrue(writer.isOpened())
        for i in range(16):
            writer.write(np.full((60, 80, 3), i*10, dtype=np.uint8))
        writer.release()
        (path/"frames.jsonl").write_text("\n".join(json.dumps({
            "host_monotonic_ns": round(1e9+i*1e9/15)}) for i in range(16)))
        return path

    def test_measured_15fps_playback_keeps_frame_count_and_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self.fixture(tmp)
            original = (path/"rgb.mp4").read_bytes()
            result = correct_video_timing(path)
            cap = cv2.VideoCapture(result["output_file"])
            self.assertEqual(cap.get(cv2.CAP_PROP_FRAME_COUNT), 16)
            self.assertAlmostEqual(cap.get(cv2.CAP_PROP_FPS), 15, places=2)
            cap.release()
            self.assertEqual((path/"rgb.mp4").read_bytes(), original)
            self.assertTrue(result["per_frame_timestamps_authoritative"])
            self.assertAlmostEqual(result["capture_span_s"], 1)

    def test_bad_times_and_overwrite_are_rejected_without_changing_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self.fixture(tmp)
            with self.assertRaisesRegex(ValueError, "separate"):
                correct_video_timing(path, output=path/"rgb.mp4")
            (path/"frames.jsonl").write_text('{"host_monotonic_ns":2}\n{"host_monotonic_ns":1}')
            with self.assertRaisesRegex(ValueError, "increasing"):
                correct_video_timing(path)

    def test_frame_count_mismatch_leaves_no_corrected_artifact(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self.fixture(tmp)
            lines = (path/"frames.jsonl").read_text().splitlines()
            (path/"frames.jsonl").write_text("\n".join(lines[:-1]))
            with self.assertRaisesRegex(ValueError, "count"):
                correct_video_timing(path)
            self.assertFalse((path/"rgb-realtime.mp4").exists())
            self.assertTrue((path/"rgb.mp4").exists())

    def test_new_capture_corrects_native_15fps_and_keeps_original(self):
        clock = {"time": 0.0}

        class Source:
            def read(self):
                clock["time"] += 1/15
                return CameraFrame(rgb_bgr=np.full((60, 80, 3), 100, dtype=np.uint8),
                    monotonic_ns=round(clock["time"]*1e9), camera_name="synthetic")

        with tempfile.TemporaryDirectory() as tmp:
            result = TrialVideoRecorder(Source(), fps=20).capture(
                tmp, .5, clock=lambda: clock["time"])
            self.assertTrue(result["video_timing"]["corrected"])
            self.assertTrue((Path(tmp)/"rgb-capture-original.mp4").is_file())
            cap = cv2.VideoCapture(str(Path(tmp)/"rgb.mp4"))
            self.assertAlmostEqual(cap.get(cv2.CAP_PROP_FPS), 15, places=2)
            self.assertEqual(cap.get(cv2.CAP_PROP_FRAME_COUNT), result["frames"])
            cap.release()
