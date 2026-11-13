"""Read-only telemetry + RGB-D trial regression tests; no robot required."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from evoarm_rsi.core import TrialStore
from evoarm_rsi.robot_telemetry import ReadOnlyRobotSampler
from evoarm_rsi.observe import record_observation


class FakeResponse:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read(self, maximum):
        return json.dumps({"x": 12.5, "y": 0, "z": -30.0,
                           "tit": 0.5, "r": 0.0}).encode()


class FakeCamera:
    def __init__(self):
        self.stopped = False

    def start(self):
        pass

    def stop(self):
        self.stopped = True


class RSITelemetryTests(unittest.TestCase):
    def test_rejects_nonlocal_and_non_status_urls(self):
        for url in ("https://example.com/api/status",
                    "http://8.8.8.8:8000/api/status",
                    "http://127.0.0.1:8765/api/unlock-once",
                    "http://127.0.0.1:8765/api/status?x=1"):
            with self.subTest(url=url):
                with self.assertRaises(ValueError):
                    ReadOnlyRobotSampler(url)

    def test_readonly_sampler_records_timestamped_real_feedback(self):
        with tempfile.TemporaryDirectory() as temp:
            store = TrialStore(temp)
            store.create("abcd", {"status": "created"})
            sampler = ReadOnlyRobotSampler(
                "http://127.0.0.1:8765/api/status", interval_s=0.1
            )
            with patch("urllib.request.OpenerDirector.open", return_value=FakeResponse()):
                sampler.start(store, "abcd")
                import time
                time.sleep(0.13)
                result = sampler.stop()
            self.assertGreaterEqual(result["real_samples"], 1)
            self.assertTrue(result["read_only"])
            records = [
                json.loads(row) for row in
                (Path(temp)/"abcd"/"trajectory.jsonl").read_text().splitlines()
            ]
            self.assertGreaterEqual(len(records), 1)
            self.assertEqual(records[0]["source"], "real_roarm_readonly_api")
            self.assertLessEqual(
                records[0]["sample_start_monotonic_ns"],
                records[0]["sample_end_monotonic_ns"],
            )
            self.assertEqual(records[0]["robot_status"]["x"], 12.5)

    def test_camera_trial_optional_telemetry_is_recorded_without_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            store = TrialStore(temp)
            fake_camera = FakeCamera()
            with (patch("evoarm_rsi.observe.TrialVideoRecorder") as recorder,
                  patch("evoarm_rsi.robot_telemetry.ReadOnlyRobotSampler") as sampler):
                recorder.return_value.capture.return_value = {
                    "rgb_file": "rgb.mp4", "frames": 148,
                    "depth_samples": 32, "camera": "orbbec_gemini_335l"
                }
                sampler.return_value.stop.return_value = {
                    "real_samples": 3, "poll_errors": 0,
                    "first_error_type": None, "read_only": True,
                }
                result = record_observation(
                    camera="gemini", seconds=10, source=fake_camera,
                    store=store,
                    robot_status_url="http://127.0.0.1:8765/api/status",
                )
                sampler.return_value.start.assert_called_once()
                sampler.return_value.stop.assert_called_once()
            manifest = store.read(result["trial_id"])
            self.assertTrue(fake_camera.stopped)
            self.assertTrue(manifest["robot_feedback_recorded"])
            self.assertFalse(manifest["robot_motion_executed"])
            self.assertFalse(manifest["agent_called"])

    def test_telemetry_failure_does_not_fake_robot_feedback(self):
        with tempfile.TemporaryDirectory() as temp:
            store = TrialStore(temp)
            with (patch("evoarm_rsi.observe.TrialVideoRecorder") as recorder,
                  patch("evoarm_rsi.robot_telemetry.ReadOnlyRobotSampler") as sampler):
                recorder.return_value.capture.return_value = {
                    "rgb_file": "rgb.mp4", "frames": 100,
                    "depth_samples": 10, "camera": "orbbec_gemini_335l"
                }
                sampler.return_value.stop.return_value = {
                    "real_samples": 0, "poll_errors": 5,
                    "first_error_type": "HTTPError", "read_only": True,
                }
                out = record_observation(
                    camera="gemini", source=FakeCamera(), store=store,
                    robot_status_url="http://127.0.0.1:8765/api/status",
                )
            self.assertFalse(store.read(out["trial_id"])["robot_feedback_recorded"])


if __name__ == "__main__":
    unittest.main()
