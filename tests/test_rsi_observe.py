"""Hardware-free tests of first REAL observation trial orchestration."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from evoarm_rsi.core import TrialStore
from evoarm_rsi.observe import record_observation


class FakeRGBDSource:
    def __init__(self, fail_start=False):
        self.started = False
        self.stopped = False
        self.fail_start = fail_start

    def start(self):
        if self.fail_start:
            raise RuntimeError("uvc_open failed -3")
        self.started = True

    def stop(self):
        self.stopped = True


class ObservationTrialTests(unittest.TestCase):
    def test_real_observation_persists_metadata_without_robot_action(self):
        with tempfile.TemporaryDirectory() as temp:
            store = TrialStore(temp)
            fake = FakeRGBDSource()
            with patch("evoarm_rsi.observe.TrialVideoRecorder") as rec:
                rec.return_value.capture.return_value = {
                    "rgb_file": "rgb.mp4", "frames": 190,
                    "depth_samples": 32, "camera": "orbbec_gemini_335l"
                }
                trial = record_observation(
                    camera="gemini", seconds=10, store=store, source=fake
                )
                rec.assert_called_once_with(fake, fps=20)
                rec.return_value.capture.assert_called_once_with(
                    Path(temp) / trial["trial_id"], 10
                )
            self.assertTrue(fake.started)
            self.assertTrue(fake.stopped)
            manifest = store.read(trial["trial_id"])
            self.assertEqual(manifest["status"], "recorded")
            self.assertFalse(manifest["robot_motion_executed"])
            self.assertFalse(manifest["agent_called"])
            self.assertFalse(manifest["simulated"])
            self.assertEqual(store.read(trial["trial_id"], "observation.json")
                             ["capture"]["frames"], 190)

    def test_camera_failure_is_preserved_in_manifest(self):
        with tempfile.TemporaryDirectory() as temp:
            store = TrialStore(temp)
            fake = FakeRGBDSource(fail_start=True)
            with self.assertRaisesRegex(RuntimeError, "uvc_open"):
                record_observation(store=store, source=fake, seconds=2)
            ids = [item.name for item in Path(temp).iterdir()]
            self.assertEqual(len(ids), 1)
            self.assertEqual(store.read(ids[0])["status"], "failed")
            self.assertFalse(fake.started)

    def test_recorder_failure_still_releases_camera(self):
        with tempfile.TemporaryDirectory() as temp:
            fake = FakeRGBDSource()
            with patch("evoarm_rsi.observe.TrialVideoRecorder") as rec:
                rec.return_value.capture.side_effect = RuntimeError("RGB encoder error")
                with self.assertRaisesRegex(RuntimeError, "encoder"):
                    record_observation(
                        store=TrialStore(temp), source=fake, seconds=3
                    )
            self.assertTrue(fake.stopped)

    def test_duration_is_bounded_before_camera_start(self):
        fake = FakeRGBDSource()
        with self.assertRaisesRegex(ValueError, "duration"):
            record_observation(seconds=30, source=fake)
        self.assertFalse(fake.started)


if __name__ == "__main__":
    unittest.main()
