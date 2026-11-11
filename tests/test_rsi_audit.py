"""Audit assertions for RSI experiments; no hardware/LLM."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from evoarm_rsi.audit import audit_trial


class RSIAuditTests(unittest.TestCase):
    def test_camera_only_is_not_reported_as_grasp_success(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            (folder/"manifest.json").write_text(json.dumps({
                "trial_id": "a", "camera_type": "gemini",
                "kind": "real_camera_observation_only",
                "simulated": False, "robot_feedback_recorded": False,
            }))
            with (folder/"frames.jsonl").open("w") as out:
                for i in range(20):
                    out.write(json.dumps({
                        "host_monotonic_ns": 123 + i * 100_000_000,
                        "device_timestamp_ms": None,
                    }) + "\n")
            np.savez_compressed(
                folder/"depth_samples.npz",
                depth_mm=np.ones((3, 2, 2), dtype=np.uint16),
                monotonic_ns=np.array([0, 1_000_000_000, 1_900_000_000]),
            )
            result = audit_trial(folder)
            self.assertEqual(result["rgb_frames"], 20)
            self.assertEqual(result["depth_samples"], 3)
            self.assertTrue(result["rgb_timestamps_strictly_increasing"])
            self.assertFalse(result["grasp_outcome_observed"])
            self.assertEqual(result["robot_feedback_samples"], 0)
            self.assertTrue(any("cannot evaluate grasp" in w
                                for w in result["warnings"]))

    def test_real_roarm_telemetry_counts_only_live_status(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            (folder/"manifest.json").write_text(json.dumps({
                "trial_id": "b", "camera_type": "gemini",
                "kind": "real_camera_observation_only", "simulated": False,
                "robot_feedback_recorded": True,
            }))
            (folder/"frames.jsonl").write_text(
                "\n".join(json.dumps({
                    "host_monotonic_ns": i * 200_000_000,
                    "device_timestamp_ms": 100+i,
                }) for i in range(10))+"\n"
            )
            (folder/"trajectory.jsonl").write_text(
                "\n".join(json.dumps({
                    "source": source,
                    "sample_end_monotonic_ns": i * 500_000_000,
                }) for i, source in enumerate((
                    "simulator_not_real_feedback",
                    "real_roarm_readonly_api",
                    "real_roarm_readonly_api",
                )))+"\n"
            )
            result = audit_trial(folder)
            self.assertEqual(result["robot_feedback_samples"], 2)
            self.assertTrue(result["rgb_device_timestamps_available"])
            self.assertFalse(result["grasp_outcome_observed"])


if __name__ == "__main__":
    unittest.main()
