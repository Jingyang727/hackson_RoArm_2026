"""Task-level regression tests: RGB-D, Grok outcomes, history and 10s budget.

All media are synthetic. No robot, company service or Langfuse is accessed.
"""
import contextlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock

import cv2
import numpy as np

from evoarm_rsi.gateway import CompanyModelGateway
from evoarm_rsi.task_agent import (
    doctor, inspect_scene, review_task, timing_preflight, validate_motion_report, validate_verdict,
)
from evoarm_rsi.task_scene import object_depth, validate_scene, load_rgbd
from evoarm_rsi.task_memory import TaskMemory, remote_history


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.folder = self.root / "recording"
        self.folder.mkdir()
        self.trial = "a" * 32
        self.stamps = [1_000_000_000 + i*500_000_000 for i in range(21)]
        (self.folder / "manifest.json").write_text(json.dumps({
            "trial_id": self.trial, "simulated": False, "status": "recorded"}))
        (self.folder / "frames.jsonl").write_text("\n".join(json.dumps({
            "host_monotonic_ns": stamp, "frame_index": i}) for i, stamp in enumerate(self.stamps)))
        writer = cv2.VideoWriter(str(self.folder/"rgb.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 2, (80, 60))
        self.assertTrue(writer.isOpened())
        for i in range(21):
            image = np.zeros((60, 80, 3), dtype=np.uint8)
            cv2.circle(image, (20+i, 30), 5, (0, 255, 255), -1)
            writer.write(image)
        writer.release()
        np.savez_compressed(self.folder/"depth_samples.npz",
                            depth_mm=np.full((21, 60, 80), 500, dtype=np.uint16),
                            monotonic_ns=np.asarray(self.stamps))
        self.report = {"kind": "pick_place_trial_v1", "simulated": False,
                       "capture_trial_id": self.trial, "setup_id": "rig-v1",
                       "grasp_attempted": True, "placement_attempted": True,
                       "motion_error": None, "policy": {"offset_x_mm": 0},
                       "events": [{"stage": name, "host_monotonic_ns": self.stamps[i]}
                                  for name, i in (("start", 0), ("close", 4), ("lift", 8),
                                                  ("release", 14), ("finish", 18))]}
        self.memory = TaskMemory(self.root/"memory")

    def test_dynamic_funnel_detection_and_target_depth_reach_grok(self):
        gateway = Mock()
        gateway.complete_json.return_value = {
            "ball": {"bbox_norm": [.1, .2, .4, .8], "confidence": .9},
            "funnel_mouth": {"bbox_norm": [.5, .1, .9, .6], "confidence": .8}}
        result = inspect_scene(self.folder, setup_id="rig-v1", memory=self.memory, gateway=gateway)
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["robot_targets_available"])
        self.assertEqual(result["objects"]["funnel_mouth"]["depth"]["surface_depth_mm"], 500)
        content = gateway.complete_json.call_args.args[1]
        self.assertIn("spatial_depth", json.loads(content[0]["text"]))
        self.assertIn("can move", gateway.complete_json.call_args.args[0])

    def test_zero_depth_black_object_stays_unknown(self):
        out = object_depth(np.zeros((60, 80)), [.1, .1, .8, .8])
        self.assertEqual(out["status"], "unknown")
        self.assertIsNone(out["surface_depth_mm"])
        self.assertIsNone(out["robot_target_xyz_mm"])

    def test_camera_surface_requires_matching_verified_intrinsics(self):
        depth = np.full((60, 80), 500)
        intr = {"fx": 100, "fy": 100, "cx": 40, "cy": 30, "width": 80,
                "height": 60, "aligned_to": "color", "verified": False}
        self.assertIsNone(object_depth(depth, [.2, .2, .5, .5], intrinsics=intr)[
            "camera_surface_xyz_candidate_mm"])
        intr["verified"] = True
        self.assertIsNotNone(object_depth(depth, [.2, .2, .5, .5], intrinsics=intr)[
            "camera_surface_xyz_candidate_mm"])

    def test_success_and_prior_result_are_given_to_next_grok_call(self):
        gateway = Mock()
        gateway.complete_json.return_value = {"pickup_observed": True, "ball_entered_funnel": True,
                                             "evidence": ["ball moves with gripper", "ball visible inside funnel"],
                                             "candidate": None}
        first = review_task(self.folder, self.report, setup_id="rig-v1", memory=self.memory, gateway=gateway)
        self.assertTrue(first["goal_met"])
        self.assertEqual(first["execution_elapsed_s"], 9)
        self.assertEqual(first["history_trials_used"], 0)
        self.assertFalse(first["candidate_applied"])
        inspect_scene(self.folder, setup_id="rig-v1", memory=self.memory, gateway=Mock(
            complete_json=Mock(return_value={"ball": None, "funnel_mouth": None})))
        context = self.memory.context("rig-v1", use_remote=False)
        self.assertEqual(context["prior_trials"][0]["trial_id"], self.trial)
        self.assertEqual(self.memory.context("rig-v1", exclude_trial_id=self.trial)["prior_trials"], [])
        # A new capture receives the previous trial, while replaying the
        # same trial cannot seed its own verdict into the reviewer.
        (self.folder/"manifest.json").write_text(json.dumps({
            "trial_id": "b"*32, "simulated": False, "status": "recorded"}))
        second_gateway = Mock()
        second_gateway.complete_json.return_value = {"ball": None, "funnel_mouth": None}
        inspect_scene(self.folder, setup_id="rig-v1", memory=self.memory, gateway=second_gateway)
        sent = json.loads(second_gateway.complete_json.call_args.args[1][0]["text"])
        self.assertTrue(sent["history"]["prior_trials"][0]["ball_entered_funnel"])
        self.assertEqual(self.memory.context("different-rig")["prior_trials"], [])

    def test_elapsed_is_measured_not_inferred_from_10s_video(self):
        verdict = validate_verdict({"pickup_observed": True, "ball_entered_funnel": True,
                                    "evidence": [], "candidate": None}, self.report, 12)
        self.assertTrue(verdict["visual_task_success"])
        self.assertFalse(verdict["within_10s"])
        self.assertFalse(verdict["goal_met"])
        self.assertEqual(verdict["failure_type"], "time_budget")
        self.assertEqual(timing_preflight([{}]*14)["status"], "impossible_with_current_spacing")
        self.assertFalse(timing_preflight([{}]*5)["verified_within_budget"])

    def test_command_ack_and_unattempted_place_do_not_prove_task_success(self):
        report = {**self.report, "placement_attempted": False}
        verdict = validate_verdict({"pickup_observed": True, "ball_entered_funnel": True,
                                    "evidence": [], "candidate": None}, report, 7)
        self.assertIsNone(verdict["ball_entered_funnel"])
        self.assertFalse(verdict["goal_met"])

    def test_actual_fault_discards_model_adjustment_and_no_motor_retry(self):
        verdict = validate_verdict({"pickup_observed": False, "ball_entered_funnel": False,
            "evidence": [], "candidate": {"parameter": "offset_x_mm", "delta": 4, "reason": "left"}},
            {**self.report, "motion_error": "cartesian_stalled"}, 8)
        self.assertIsNone(verdict["candidate"])
        self.assertEqual(verdict["failure_type"], "motion_error")

    def test_invalid_motor_or_speed_proposals_rejected(self):
        with self.assertRaises(ValueError):
            validate_verdict({"pickup_observed": False, "ball_entered_funnel": False,
                "evidence": [], "candidate": {"parameter": "speed", "delta": 1, "reason": "faster"}},
                self.report, 8)
        with self.assertRaises(ValueError):
            validate_scene({"ball": {"bbox_norm": [0, 0, 2, 1], "confidence": .9}})

    def test_mismatched_capture_and_nonmonotonic_events_rejected(self):
        with self.assertRaises(ValueError):
            validate_motion_report(self.report, "b"*32, "rig-v1")
        bad = {**self.report, "events": list(reversed(self.report["events"]))}
        with self.assertRaises(ValueError):
            validate_motion_report(bad, self.trial, "rig-v1")

    def test_missing_video_coverage_cannot_be_success(self):
        self.report["events"][-1]["host_monotonic_ns"] = self.stamps[-1] + 500_000_000
        gateway = Mock(complete_json=Mock(return_value={"pickup_observed": True,
            "ball_entered_funnel": True, "evidence": ["looks done"], "candidate": None}))
        out = review_task(self.folder, self.report, setup_id="rig-v1", memory=self.memory, gateway=gateway)
        self.assertFalse(out["goal_met"])
        self.assertFalse(out["all_events_within_video"])
        self.assertEqual(out["failure_type"], "unknown")

    def test_sparse_misaligned_depth_rejected(self):
        np.savez(self.folder/"depth_samples.npz", depth_mm=np.ones((1, 60, 80)),
                 monotonic_ns=[self.stamps[-1] + 1_000_000_000])
        with self.assertRaisesRegex(ValueError, "far apart"):
            load_rgbd(self.folder, 0)

    def test_company_runtime_uses_grok_and_never_reasoning_content(self):
        env = {"EVOARM_COMPANY_LLM_TOKEN": "synthetic-token"}
        response = Mock()
        response.read.return_value = json.dumps({"choices": [{"message": {
            "content": '{"ball":null,"funnel_mouth":null}',
            "reasoning_content": '{"motor":"move"}'}}]}).encode()
        with patch.dict(os.environ, env), patch("evoarm_rsi.gateway.request.urlopen",
                                               return_value=contextlib.nullcontext(response)) as call:
            gateway = CompanyModelGateway(base_url="https://approved.example/v1", model="grok-test")
            result = gateway.complete_json("scene", [])
            self.assertIsNone(result["ball"])
            self.assertNotIn("motor", result)
            payload = json.loads(call.call_args.args[0].data)
            self.assertEqual(payload["model"], "grok-test")
            with self.assertRaises(ValueError):
                CompanyModelGateway(base_url="https://approved.example/v1", model="other").complete_json("", [])

    def test_doctor_does_not_poll_busy_arm_or_change_state(self):
        with patch("pathlib.Path.home", return_value=self.root), patch(
            "evoarm_rsi.real_probe._controller_get", return_value={
                "connected": True, "pending": {"kind": "cartesian"}}) as read, patch(
            "evoarm_rsi.camera_service.client_request", return_value={"robot_control": False}) as camera:
            out = doctor("http://127.0.0.1:8765")
        self.assertFalse(out["robot_moved"])
        self.assertNotIn("status", out)
        self.assertEqual(read.call_args.args[1], "/api/health")
        read.assert_called_once()
        self.assertEqual(camera.call_args.args[1:3], ("GET", "/health"))


class MemoryTests(unittest.TestCase):
    def test_langfuse_output_retrieved_into_context_and_outage_uses_local(self):
        item = {"trial_id": "trial1", "setup_id": "rig1", "pickup_observed": True,
                "ball_entered_funnel": False, "within_10s": True,
                "failure_type": "missed_funnel", "policy": {}, "created_at_unix": 1}
        body = {"data": [{"metadata": {"setup_id": "rig1"}, "output": json.dumps(item)},
                         {"metadata": {"setup_id": "other"}, "output": json.dumps(item)}]}
        response = Mock(read=Mock(return_value=json.dumps(body).encode()))
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {
            "EVOARM_LANGFUSE_URL": "https://lf.example", "LANGFUSE_PUBLIC_KEY": "test-pk",
            "LANGFUSE_SECRET_KEY": "test-sk"}), patch("evoarm_rsi.task_memory.request.urlopen",
                return_value=contextlib.nullcontext(response)) as call:
            memory = TaskMemory(tmp)
            context = memory.context("rig1")
            self.assertEqual(len(context["prior_trials"]), 1)
            self.assertEqual(context["langfuse"], "retrieved")
            self.assertIn("/api/public/v2/observations?", call.call_args.args[0].full_url)
            memory.remember(item)
            with patch("evoarm_rsi.task_memory.remote_history", side_effect=TimeoutError):
                self.assertEqual(len(memory.context("rig1")["prior_trials"]), 1)

    def test_export_is_explicit_supported_sdk_and_not_read_visibility_claim(self):
        with tempfile.TemporaryDirectory() as tmp:
            memory = TaskMemory(tmp)
            memory.remember({"trial_id": "t1", "setup_id": "s1", "failure_type": "unknown"})
            client = Mock()
            client.start_as_current_observation.return_value = contextlib.nullcontext(Mock())
            out = memory.publish("t1", client=client)
            self.assertTrue(out["submitted"])
            self.assertFalse(out["read_visibility_confirmed"])
            client.flush.assert_called_once()


if __name__ == "__main__":
    unittest.main()
