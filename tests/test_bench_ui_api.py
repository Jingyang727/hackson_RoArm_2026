"""The browser's new sampling routes read feedback without changing torque."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from control_ui.service import ArmService
from tests.test_robot_calibration import FakeArm
from tests.test_bench_calibration import make_session
from tests.test_serial_transport_errors import api_request
from evoarm_rsi import bench_calibration as bench


class BenchUIApi(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/"session.json"
        data = make_session(self.path)
        data["reference"]["image_size"] = [1280, 720]
        data["reference_sha256"] = bench._digest(data["reference"])
        self.path.write_text(json.dumps(data))
        self.service = ArmService()
        self.service.arm = FakeArm()
        for target, kwargs in (("control_ui.bench_ui.session_path", {"return_value": self.path}),
                               ("control_ui.main.service", {"new": self.service})):
            patcher = patch(target, **kwargs)
            patcher.start()
            self.addCleanup(patcher.stop)

    def request(self, path, value=None, method="POST"):
        return asyncio.run(api_request(path, value or {}, method=method))

    def test_status_does_not_connect_or_sample_serial(self):
        with patch.object(self.service, "status", side_effect=AssertionError("unexpected sample")):
            start, body = self.request("/api/bench/status", method="GET")
        self.assertEqual(start["status"], 200)
        self.assertEqual(body["next_point"], "P1")
        self.assertEqual(self.service.arm.commands, [])

    def test_support_and_alignment_declaration_required_before_recording(self):
        before = self.path.read_bytes()
        start, _ = self.request("/api/bench/capture", {"point_id": "P1", "aligned_and_supported": False})
        self.assertEqual(start["status"], 409)
        self.assertEqual(before, self.path.read_bytes())
        self.assertEqual(self.service.arm.commands, [])

    def test_mixed_torque_cannot_be_captured(self):
        original = self.service.arm.status
        def mixed():
            raw = original()
            raw["torswitchS"] = 0
            return raw
        self.service.arm.status = mixed
        start, _ = self.request("/api/bench/capture", {"point_id": "P1", "aligned_and_supported": True})
        self.assertEqual(start["status"], 409)
        self.assertEqual(bench.session_status(self.path)["next_point"], "P1")

    def test_five_supported_manual_points_fit_without_ever_relocking(self):
        self.service.arm.torque = 0
        for key, grid in bench.POINTS.items():
            self.service.arm.xyz = [300-grid[1]*24., 80-grid[0]*24., -90.]
            start, body = self.request("/api/bench/capture", {"point_id": key, "aligned_and_supported": True})
            self.assertEqual(start["status"], 200, body)
        self.assertTrue(body["fit"]["planar_fit_validated"])
        self.assertEqual(self.service.arm.torque, 0)
        self.assertEqual(self.service.arm.commands, [])
        self.assertEqual(len(list(self.path.parent.glob("candidate-*.json"))), 1)
        self.assertEqual(len(list(self.path.parent.glob("session-samples/*.json"))), 5)
        _, restored = bench.load_session(self.path)
        self.assertTrue(all(not r["torque_locked"] and r["operator_support_required"] for r in restored["records"].values()))
        start, state = self.request("/api/bench/status", method="GET")
        self.assertEqual(sum(p["captured"] for p in state["points"]), 5)
        self.assertTrue(state["fit"]["planar_fit_validated"])
