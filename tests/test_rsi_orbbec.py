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

    def test_unstarted_pipeline_fails_immediately(self):
        with self.assertRaisesRegex(RuntimeError, "not started"):
            Gemini335LProvider().read()


if __name__ == "__main__":
    unittest.main()
