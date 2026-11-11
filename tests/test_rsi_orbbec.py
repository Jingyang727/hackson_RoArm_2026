"""Gemini startup regression tests: no physical device or SDK necessary."""
import unittest
from unittest.mock import patch

import numpy as np

from evoarm_rsi.orbbec import Gemini335LProvider


class Color:
    pass


class Depth:
    def get_width(self):
        return 4

    def get_height(self):
        return 3

    def get_data(self):
        return np.full((3, 4), 100, dtype=np.uint16).tobytes()

    def get_depth_scale(self):
        return 1.0


class Frames:
    def __init__(self, color=True, depth=True):
        self.color = Color() if color else None
        self.depth = Depth() if depth else None

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


class Aligned:
    def __init__(self, result):
        self.result = result

    def as_frame_set(self):
        return self.result


class FakePipeline:
    def __init__(self, frames):
        self.frames = list(frames)
        self.calls = 0

    def wait_for_frames(self, timeout_ms):
        self.calls += 1
        return self.frames.pop(0) if self.frames else None


class FakeAligner:
    def __init__(self, results):
        self.results = list(results)
        self.calls = 0

    def process(self, frames):
        self.calls += 1
        return self.results.pop(0) if self.results else None


class GeminiWarmupTests(unittest.TestCase):
    def test_skips_missing_pair_and_none_alignment_then_recovers(self):
        pipeline = FakePipeline([
            None, Frames(color=True, depth=False),
            Frames(), Frames(), Frames(),
        ])
        aligner = FakeAligner([
            None, Aligned(None), Aligned(Frames()),
        ])
        camera = Gemini335LProvider(warmup_timeout_s=2)
        camera.pipeline = pipeline
        camera.aligner = aligner
        with patch.object(Gemini335LProvider, "_bgr",
                          return_value=np.zeros((3, 4, 3), dtype=np.uint8)):
            frame = camera.read()
        self.assertEqual(pipeline.calls, 5)
        self.assertEqual(aligner.calls, 3)
        self.assertEqual(camera.dropped_frames, 4)
        self.assertEqual(frame.depth_mm.shape, (3, 4))
        self.assertEqual(int(frame.depth_mm[0, 0]), 100)
        self.assertEqual(frame.camera_name, "orbbec_gemini_335l")

    def test_repeated_empty_alignment_fails_with_bounded_error(self):
        camera = Gemini335LProvider(warmup_timeout_s=0.01)
        camera.pipeline = FakePipeline([Frames()] * 260)
        camera.aligner = FakeAligner([None] * 260)
        with self.assertRaisesRegex(RuntimeError, "warmup timed out"):
            camera.read()
        self.assertGreater(camera.dropped_frames, 0)
        self.assertLessEqual(camera.dropped_frames, 240)

    def test_misaligned_depth_fails_instead_of_silently_saving(self):
        camera = Gemini335LProvider()
        camera.pipeline = FakePipeline([Frames()])
        camera.aligner = FakeAligner([Aligned(Frames())])
        with patch.object(Gemini335LProvider, "_bgr",
                          return_value=np.zeros((2, 4, 3), dtype=np.uint8)):
            with self.assertRaisesRegex(RuntimeError, "mismatch"):
                camera.read()

    def test_sdk_intrinsics_saved_as_unverified_candidate_only(self):
        class Intrinsic:
            fx = 690.0
            fy = 691.0
            cx = 160.0
            cy = 120.0
            width = 4
            height = 3

        class Param:
            rgb_intrinsic = Intrinsic()

        pipe = FakePipeline([Frames()])
        pipe.get_camera_param = lambda: Param()
        provider = Gemini335LProvider()
        provider.pipeline = pipe
        provider.aligner = FakeAligner([Aligned(Frames())])
        with patch.object(Gemini335LProvider, "_bgr",
                          return_value=np.zeros((3, 4, 3), dtype=np.uint8)):
            provider.read()
        self.assertIsNotNone(provider.color_intrinsics)
        self.assertFalse(provider.color_intrinsics["verified"])
        self.assertEqual(provider.color_intrinsics["width"], 4)
        self.assertEqual(provider.color_intrinsics["fx"], 690.0)

    def test_invalid_sdk_intrinsics_never_marked_verified(self):
        class BadIntrinsic:
            fx = 0.0
            fy = 690.0
            cx = 160.0
            cy = 120.0
            width = 4
            height = 3

        class BadParam:
            rgb_intrinsic = BadIntrinsic()

        pipe = FakePipeline([Frames()])
        pipe.get_camera_param = lambda: BadParam()
        provider = Gemini335LProvider()
        provider.pipeline = pipe
        with patch.object(Gemini335LProvider, "_bgr",
                          return_value=np.zeros((3, 4, 3), dtype=np.uint8)):
            provider.read()
        self.assertIsNone(provider.color_intrinsics)
        self.assertEqual(provider.intrinsics_error, "ValueError")

    def test_recording_clock_starts_after_warmup_and_depth_spans_window(self):
        import tempfile
        from pathlib import Path
        from evoarm_rsi.camera import CameraFrame, TrialVideoRecorder

        state = {"t": 0.0, "calls": 0}

        class FakeSource:
            def read(self):
                state["calls"] += 1
                # Simulate a three-second startup before the first frame.
                state["t"] += 3.0 if state["calls"] == 1 else 0.1
                return CameraFrame(
                    rgb_bgr=np.zeros((3, 4, 3), dtype=np.uint8),
                    depth_mm=np.ones((3, 4), dtype=np.uint16),
                    monotonic_ns=int(state["t"] * 1e9),
                    camera_name="mock_rgbd",
                )

        with tempfile.TemporaryDirectory() as temp:
            with patch("cv2.VideoWriter") as writer:
                writer.return_value.isOpened.return_value = True
                result = TrialVideoRecorder(
                    FakeSource(), fps=10, max_depth_frames=3
                ).capture(Path(temp), 1.0, clock=lambda: state["t"])
                writer.return_value.release.assert_called_once()
            self.assertGreaterEqual(result["frames"], 7)
            self.assertEqual(result["depth_samples"], 3)
            self.assertGreaterEqual(state["t"], 4.0)
            with np.load(Path(temp) / "depth_samples.npz") as depth:
                stamps = depth["monotonic_ns"]
                self.assertGreater(float(stamps[-1] - stamps[0]) / 1e9, 0.5)

    def test_unstarted_pipeline_fails_immediately(self):
        with self.assertRaisesRegex(RuntimeError, "not started"):
            Gemini335LProvider().read()


if __name__ == "__main__":
    unittest.main()
