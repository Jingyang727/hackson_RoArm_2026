"""Hardware-free RSI regression tests."""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from evoarm_rsi.core import TrialPolicy, TrialRunner, TrialStore, MockRobot
from evoarm_rsi.gateway import CompanyModelGateway
from evoarm_rsi.camera import CameraFrame, OrbbecRGBDAdapter


class RSICoreTests(unittest.TestCase):
    def test_mock_trial_writes_evidence_and_returns_home(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot = MockRobot()
            store = TrialStore(tmp)
            runner = TrialRunner(store=store, robot=robot)
            outcome = runner.run_once(TrialPolicy(), action_window_s=10)
            trial_id = outcome["trial_id"]
            path = Path(tmp) / trial_id
            self.assertTrue(robot.is_home)
            self.assertTrue((path / "manifest.json").exists())
            self.assertTrue((path / "commands.jsonl").exists())
            self.assertTrue((path / "trajectory.jsonl").exists())
            self.assertTrue((path / "observation.json").exists())
            self.assertTrue((path / "evaluation.json").exists())
            self.assertEqual(store.read(trial_id)["status"], "completed")
            self.assertTrue(store.read(trial_id)["simulated"])
            self.assertFalse(outcome["evaluation"]["success"])
            self.assertEqual(len((path/"commands.jsonl").read_text().splitlines()), 4)

    def test_model_proposal_changes_only_next_policy(self):
        with tempfile.TemporaryDirectory() as tmp:
            original = TrialPolicy()
            runner = TrialRunner(store=TrialStore(tmp))
            outcome = runner.run_once(
                original, action_window_s=10,
                propose_next=lambda policy, result: {"offset_x_mm": 2.0},
            )
            self.assertEqual(original.offset_x_mm, 0)
            self.assertEqual(outcome["next_policy"]["offset_x_mm"], 2.0)
            self.assertEqual(outcome["next_policy"]["revision"], 1)
            self.assertTrue((Path(tmp)/outcome["trial_id"]/"reflection.json").exists())

    def test_invalid_model_proposal_is_recorded_not_sent_to_robot(self):
        with tempfile.TemporaryDirectory() as tmp:
            robot = MockRobot()
            runner = TrialRunner(store=TrialStore(tmp), robot=robot)
            with self.assertRaisesRegex(ValueError, "unrecognized"):
                runner.run_once(propose_next=lambda p, e: {"joint_1_deg": 300})
            self.assertTrue(robot.is_home)
            paths = list(Path(tmp).iterdir())
            self.assertEqual(len(paths), 1)
            self.assertEqual(json.loads((paths[0]/"manifest.json").read_text())["status"],
                             "failed")

    def test_fault_does_not_blindly_command_home(self):
        class FaultRobot(MockRobot):
            def run_grasp(self, policy, max_seconds, on_feedback):
                self.fault = True
                self.is_home = False
                return {"status": "fault"}
        with tempfile.TemporaryDirectory() as tmp:
            robot = FaultRobot()
            with self.assertRaisesRegex(RuntimeError, "fault"):
                TrialRunner(store=TrialStore(tmp), robot=robot).run_once()
            self.assertFalse(robot.is_home)
            self.assertEqual(len(list(Path(tmp).iterdir())), 1)

    def test_hardware_adapter_is_rejected_even_when_instantiated(self):
        class RealArm:
            hardware = True
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError, "rejects hardware"):
                TrialRunner(store=TrialStore(tmp), robot=RealArm()).run_once()
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_window_and_policy_bounds(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = TrialRunner(store=TrialStore(tmp))
            with self.assertRaisesRegex(ValueError, "10 seconds"):
                runner.run_once(action_window_s=11)
        with self.assertRaises(ValueError):
            TrialPolicy(offset_x_mm=9)
        with self.assertRaises(ValueError):
            TrialPolicy().updated({"offset_y_mm": 30})
        with self.assertRaises(ValueError):
            TrialPolicy().updated({"anything": 1})

    def test_orbbec_rgbd_adapter_contract(self):
        class FakeProvider:
            def __init__(self):
                self.started = False
                self.stopped = False

            def start(self):
                self.started = True

            def read(self):
                return CameraFrame(rgb_bgr=object(), depth_mm=object(),
                                   monotonic_ns=123, camera_name="gemini335l")

            def stop(self):
                self.stopped = True

        provider = FakeProvider()
        source = OrbbecRGBDAdapter(provider)
        source.start()
        frame = source.read()
        source.stop()
        self.assertTrue(provider.started)
        self.assertTrue(provider.stopped)
        self.assertEqual(frame.monotonic_ns, 123)
        self.assertEqual(frame.camera_name, "gemini335l")
        with self.assertRaisesRegex(RuntimeError, "not configured"):
            OrbbecRGBDAdapter().start()

    def test_company_gateway_requires_explicit_env_no_public_fallback(self):
        with patch.dict(os.environ, {}, clear=True):
            gateway = CompanyModelGateway()
            self.assertFalse(gateway.configured())
            with self.assertRaisesRegex(RuntimeError, "not configured"):
                gateway.propose_next({}, {})


if __name__ == "__main__":
    unittest.main()
