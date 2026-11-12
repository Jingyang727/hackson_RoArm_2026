"""Real geometry and read-only acquisition contracts; no physical hardware."""
import asyncio
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
import io

import numpy as np

from control_ui.robot_calibration import CalibrationError
from evoarm_rsi import bench_calibration as bench
from tests.test_serial_transport_errors import api_request
from evoarm.serial_arm import ArmCommunicationError


def feedback(xyz=(300., 50., -90.), **changes):
    result = {"T": 1051, **dict(zip("xyz", xyz)), "tit": .2, "b": 0., "s": 0.,
              "e": math.pi/2, "t": .2, "r": 0., "g": 3.,
              **{key: 1 for key in bench.TORQUE_KEYS}}
    return {**result, **changes}


def sample(xyz=(300., 50., -90.), **changes):
    return {"kind": bench.SAMPLE_KIND, "samples": [
        {"host_monotonic_ns": 1_000_000_000+i*350_000_000,
         "robot_status": feedback(xyz, **changes)} for i in range(3)]}


def make_session(path):
    reference = {"square_mm": 24., "source_trial_id": "empty-board-v2",
                 "corners_px": [[float(c*20), float(r*20)] for r in range(6) for c in range(9)]}
    content = {"kind": bench.SESSION_KIND, "setup_id": "bench-layout-v2", "session_id": "new-session",
               "reference": reference, "reference_sha256": bench._digest(reference),
               "points": copy.deepcopy(bench.POINTS), "records": {}, "history": []}
    path.write_text(json.dumps(content))
    return content


class StationaryAcquisition(unittest.TestCase):
    class Arm:
        def __init__(self):
            self.lock = threading.RLock()
            self.reads = 0
            self.fault = None
            self.pending = None

        def health(self):
            assert self.lock._is_owned(), "sampling must exclude concurrent controller mutations"
            return {"connected": True, "teach_active": False, "fault": self.fault, "pending": self.pending}

        def status(self):
            assert self.lock._is_owned()
            self.reads += 1
            return feedback()

    def test_lock_held_for_entire_sample_and_raw_feedback_preserved(self):
        arm = self.Arm()
        def delay(_):
            self.assertTrue(arm.lock._is_owned())
        with patch.object(bench.time, "monotonic_ns", side_effect=[1_000_000_000, 1_350_000_000, 1_700_000_000]):
            result = bench.stationary_sample(arm, sleep_fn=delay)
        self.assertEqual(arm.reads, 3)
        self.assertFalse(result["motion_commands_sent"])
        self.assertEqual(result["robot_xyz_mm"], [300., 50., -90.])
        self.assertEqual(result["samples"][0]["robot_status"], feedback())

    def test_fault_or_pending_prevents_even_reading(self):
        for key in ("pending", "fault"):
            arm = self.Arm()
            setattr(arm, key, {"reason": "unverified_previous_motion"})
            with self.assertRaises(CalibrationError):
                bench.stationary_sample(arm)
            self.assertEqual(arm.reads, 0)

    def test_fault_during_status_prevents_capture(self):
        arm = self.Arm()
        def status():
            arm.fault = {"reason": "serial_failure"}
            return feedback()
        arm.status = status
        with self.assertRaises(CalibrationError):
            bench.stationary_sample(arm)

    def test_drift_torque_release_and_fake_timestamps_rejected(self):
        drift = sample()
        drift["samples"][-1]["robot_status"]["z"] += 3
        joint_drift = sample()
        joint_drift["samples"][-1]["robot_status"]["s"] += math.radians(1)
        fast = sample()
        fast["samples"][-1]["host_monotonic_ns"] -= 200_000_000
        for value in (drift, joint_drift, fast, sample(torswitchS=0)):
            with self.assertRaises(CalibrationError):
                bench.validate_sample(value)


class SessionAcquisition(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/"session.json"
        make_session(self.path)

    def test_requires_physical_alignment_without_network_or_writes(self):
        before = self.path.read_bytes()
        with patch.object(bench, "_controller_get") as get:
            with self.assertRaises(CalibrationError):
                bench.capture_point(self.path, "P1")
        get.assert_not_called()
        self.assertEqual(before, self.path.read_bytes())

    def test_get_only_three_raw_samples_resume_and_no_legacy_file(self):
        with patch.object(bench, "_controller_get", return_value=sample()) as get:
            result = bench.capture_point(self.path, "P1", aligned=True)
        get.assert_called_once_with("http://127.0.0.1:8765", "/api/calibration/stationary-sample")
        self.assertEqual(result["next_point"], "P2")
        _, restored = bench.load_session(self.path)
        self.assertEqual(restored["records"]["P1"]["board_mm"], [24., 24.])
        self.assertEqual(len(restored["records"]["P1"]["raw_feedback"]["samples"]), 3)
        self.assertEqual(len(list(self.path.parent.glob("session-samples/*.json"))), 1)
        self.assertFalse((self.path.parent/"robot_xy_calibration.json").exists())

    def test_rerecord_requires_replace_and_preserves_both_attempts(self):
        with patch.object(bench, "_controller_get", return_value=sample()):
            bench.capture_point(self.path, "P1", aligned=True)
            with self.assertRaises(CalibrationError):
                bench.capture_point(self.path, "P1", aligned=True)
            bench.capture_point(self.path, "P1", aligned=True, replace=True)
        _, session = bench.load_session(self.path)
        self.assertEqual(len(session["history"]), 1)
        self.assertEqual(len(list(self.path.parent.glob("session-samples/*.json"))), 2)

    def test_tool_attitude_change_rejected_without_destroying_prior_point(self):
        with patch.object(bench, "_controller_get", return_value=sample()):
            bench.capture_point(self.path, "P1", aligned=True)
        before = self.path.read_bytes()
        with patch.object(bench, "_controller_get", return_value=sample(tit=.6)):
            with self.assertRaises(CalibrationError):
                bench.capture_point(self.path, "P2", aligned=True)
        self.assertEqual(before, self.path.read_bytes())

    def test_changed_reference_rejected_before_network(self):
        raw = json.loads(self.path.read_text())
        raw["reference"]["square_mm"] = 30
        self.path.write_text(json.dumps(raw))
        with patch.object(bench, "_controller_get") as get:
            with self.assertRaises(CalibrationError):
                bench.capture_point(self.path, "P1", aligned=True)
        get.assert_not_called()

    def test_existing_session_not_overwritten_by_init(self):
        before = self.path.read_bytes()
        with self.assertRaises(CalibrationError):
            bench.start_session("nonexistent", setup_id="bench-layout-v2", square_mm=24, output=self.path)
        self.assertEqual(before, self.path.read_bytes())

    def test_parallel_capture_is_rejected_without_losing_session(self):
        before = self.path.read_bytes()
        with bench._session_lock(self.path), patch.object(bench, "_controller_get") as get:
            with self.assertRaises(CalibrationError):
                bench.capture_point(self.path, "P1", aligned=True)
        get.assert_not_called()
        self.assertEqual(before, self.path.read_bytes())

    def test_old_service_404_produces_actionable_message_without_saving_point(self):
        err = HTTPError("http://127.0.0.1:8765/api/calibration/stationary-sample", 404,
                        "Not Found", {}, io.BytesIO(b'{"detail":"Not Found"}'))
        with patch.object(bench, "_controller_get", side_effect=err):
            with self.assertRaisesRegex(CalibrationError, "8765"):
                bench.capture_point(self.path, "P1", aligned=True)
        self.assertEqual(bench.session_status(self.path)["next_point"], "P1")


class SamplingAPI(unittest.TestCase):
    def test_get_route_returns_sampling_data(self):
        with patch.object(bench, "stationary_sample", return_value=sample()) as acquire:
            start, body = asyncio.run(api_request("/api/calibration/stationary-sample", {}, method="GET"))
        self.assertEqual(start["status"], 200)
        self.assertEqual(body["kind"], bench.SAMPLE_KIND)
        acquire.assert_called_once()

    def test_unstable_sample_is_json_409(self):
        with patch.object(bench, "stationary_sample", side_effect=CalibrationError("not still")):
            start, body = asyncio.run(api_request("/api/calibration/stationary-sample", {}, method="GET"))
        self.assertEqual(start["status"], 409)
        self.assertEqual(body["detail"], "not still")

    def test_transport_failure_is_json_409(self):
        with patch.object(bench, "stationary_sample", side_effect=ArmCommunicationError("device gone")):
            start, body = asyncio.run(api_request("/api/calibration/stationary-sample", {}, method="GET"))
        self.assertEqual(start["status"], 409)
        self.assertEqual(body["error_code"], "serial_communication_error")


class PlaneGeometry(unittest.TestCase):
    def points(self):
        # Image-down board axes become a proper 3D rotation with -Z normal.
        yaw, tilt = .7, .04
        rz = np.asarray([[math.cos(yaw), -math.sin(yaw), 0],
                         [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.]])
        rx = np.asarray([[1., 0, 0], [0, math.cos(tilt), -math.sin(tilt)],
                         [0, math.sin(tilt), math.cos(tilt)]])
        rotation = rz @ rx @ np.diag([1., -1., -1.])
        offset = np.asarray([350., 50., -90.])
        records = {}
        for key, grid in bench.POINTS.items():
            board = [v*24. for v in grid]
            records[key] = {"id": key, "board_mm": board,
                            "robot_xyz_mm": (rotation @ np.asarray(board+[0.])+offset).tolist()}
        return records, rotation, offset

    def test_recovers_metric_3d_plane_including_negative_table_height(self):
        points, rotation, offset = self.points()
        fit = bench.fit_points(points)
        np.testing.assert_allclose(fit["board_to_robot_rotation_3x3"], rotation, atol=1e-10)
        np.testing.assert_allclose(fit["board_to_robot_translation_mm"], offset, atol=1e-10)
        self.assertTrue(fit["planar_fit_validated"])
        self.assertFalse(fit["motion_authorized"])
        self.assertFalse(fit["camera_to_robot_verified"])
        plane = fit["table_plane_robot"]
        self.assertGreater(plane["normal_up"][2], 0)
        for row in points.values():
            self.assertAlmostEqual(np.dot(plane["normal_up"], row["robot_xyz_mm"])+plane["d_mm"], 0)

    def test_bad_heldout_point_fails_without_affecting_training_fit(self):
        points, _, _ = self.points()
        baseline = bench.fit_points(points)
        points["V1"]["robot_xyz_mm"][0] += 30
        result = bench.fit_points(points)
        self.assertFalse(result["planar_fit_validated"])
        self.assertAlmostEqual(result["validation_error_mm"], 30)
        self.assertEqual(result["board_to_robot_rotation_3x3"], baseline["board_to_robot_rotation_3x3"])
        self.assertLess(result["train_rms_mm"], 1e-10)

    def test_wrong_scale_and_missing_point_cannot_pass(self):
        points, _, _ = self.points()
        for record in points.values():
            record["board_mm"] = [v*2 for v in record["board_mm"]]
        self.assertFalse(bench.fit_points(points)["planar_fit_validated"])
        del points["V1"]
        with self.assertRaises(CalibrationError):
            bench.fit_points(points)


if __name__ == "__main__":
    unittest.main()
