"""First motion from a synthetic contact pose; no serial, SDK or model calls."""
import json
import math
from pathlib import Path
import tempfile
import threading
import os
import time
import asyncio
import unittest
from unittest.mock import patch

from evoarm_rsi import bench_start as start
from evoarm_rsi.bench_calibration import POINTS
from evoarm_rsi.roarm_geometry import forward
from tests.test_bench_calibration import make_session


def fixture(path):
    # Synthetic near-vertical pose. An unchanged shoulder outside the narrower
    # manual-training interval is not a target to chase or automatically reset.
    status = {"T": 1051, **{key: math.radians(value) for key, value in
              dict(b=0, s=53, e=62, t=55, r=0, g=179).items()},
              **{"torswitch"+key: 1 for key in "BSETRG"}}
    pose = forward(status)
    status.update(zip("xyz", pose["xyz_mm"]))
    status["tit"] = pose["pitch_rad"]
    session = make_session(path)
    session["records"] = {key: {"id": key, "board_mm": [grid[0]*24., grid[1]*24.],
        "robot_xyz_mm": [200+grid[0]*24., -80+grid[1]*24., status["z"]],
        "tool_pose_rad": {"pitch": status["tit"], "roll": status["r"], "gripper": status["g"]}}
        for key, grid in POINTS.items()}
    path.write_text(json.dumps(session))
    return session, status


def high_pose(status, wrist=80.):
    result = {**status, "s": math.radians(32), "e": math.radians(58), "t": math.radians(wrist)}
    pose = forward(result)
    result.update(zip("xyz", pose["xyz_mm"]))
    result["tit"] = pose["pitch_rad"]
    return result


class LiftGeometry(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/"session.json"
        self.session, self.status = fixture(self.path)

    def test_predicted_path_rises_without_moving_other_joints_or_using_teaching(self):
        plan = start.plan_lift(self.session, self.status)
        self.assertEqual((plan["joint"], plan["delta_deg"]), (3, -5))
        self.assertGreater(plan["predicted_lift_mm"], 8)
        heights = [p["height_mm"] for p in plan["predicted_path"]]
        self.assertTrue(all(b >= a for a, b in zip(heights, heights[1:])))
        self.assertFalse(plan["grasp_attempted"])
        self.assertFalse(plan["taught_positions_used"])
        self.assertEqual(self.status["s"], math.radians(53))

    def test_changed_tool_model_or_open_gripper_rejected_without_forcing_a_contact_pose(self):
        for change in ({"x": self.status["x"]+10}, {"g": 2.},
                       {"tit": .5}, {"torswitchS": 0}):
            with self.assertRaises(ValueError):
                start.plan_lift(self.session, {**self.status, **change})
        for record in self.session["records"].values():
            record["robot_xyz_mm"][2] -= 30
        self.assertEqual(start.plan_start(self.session, self.status)["start_mode"],"low_pose_lift")

    def test_high_flat_tool_does_not_need_to_reproduce_calibration_pitch(self):
        status = high_pose(self.status, wrist=0)
        plan = start.plan_start(self.session, status)
        self.assertEqual(plan["start_mode"], "already_clear")
        self.assertFalse(plan["motion_required"])
        self.assertFalse(plan["full_pick_place_ready"])

    def test_high_vertical_pose_gets_inward_wrist_recovery_without_lowering_limits(self):
        from control_ui.safety import JOINTS, plan_to_angle, UnsafeMotion
        status = high_pose(self.status)
        plan = start.plan_start(self.session, status)
        self.assertEqual(plan["start_mode"], "high_wrist_recovery")
        self.assertEqual(plan["target_deg"], 55.)
        self.assertTrue(all(p["height_mm"] >= 100 for p in plan["predicted_path"]))
        self.assertEqual(JOINTS[4][2:], (-60.,60.))
        with self.assertRaises(UnsafeMotion): plan_to_angle(status,4,55.)

    def test_recovery_rejects_other_outside_joints_and_vendor_limit_violations(self):
        for status in (high_pose(self.status,95), {**high_pose(self.status),"b":math.radians(80)}):
            pose=forward(status);status.update(zip("xyz",pose["xyz_mm"]));status["tit"]=pose["pitch_rad"]
            with self.assertRaises(ValueError): start.plan_start(self.session,status)

    def test_failed_heldout_calibration_is_not_motion_authority(self):
        self.session["records"]["V1"]["robot_xyz_mm"][0] += 30
        with self.assertRaisesRegex(ValueError, "held-out"):
            start.plan_lift(self.session, self.status)


class LiftLifecycle(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.home = Path(self.temp.name)
        self.path = self.home/".evoarm/rsi/calibrations/bench-layout-v2/session.json"
        self.path.parent.mkdir(parents=True)
        self.session, self.status = fixture(self.path)
        self.calls = []; self.command = False; self.armed = True
        self.done = threading.Event(); self.warming = threading.Event()
        self.request_id = None
        self.fail_command = False; self.no_progress = False; self.wrong_record = False
        self.last_joint = 3
        self.health = lambda: {"connected": True, "motion_enabled": True, "armed": self.armed,
            "teach_active": False, "torque_status": [1]*6, "pending": None, "fault": None,
            "last_command": {"status": "feedback_confirmed", "joint": self.last_joint,
                             "to_deg": math.degrees(self.status["e" if self.last_joint==3 else "t"]),
                             "bench_camera_request_id": self.request_id}}

    def get(self, base, endpoint):
        if endpoint == "/api/status":
            return dict(self.status)
        if endpoint == "/api/health":
            if self.command: self.done.set()
            return self.health()
        if endpoint == "/api/bench/motion/capabilities": return {"high_wrist_recovery":True}
        raise AssertionError(endpoint)

    def post(self, base, endpoint, payload):
        self.calls.append((endpoint, payload))
        if endpoint == "/api/disarm":
            self.armed = False; return {"armed": False}
        self.assertTrue(self.warming.is_set(), "motion must follow this camera's start")
        if endpoint == "/api/step":
            self.assertEqual(payload, {"joint": 3, "delta": -5})
            field="e";target=self.status[field]-math.radians(5)
        else:
            self.assertEqual(endpoint,"/api/bench/motion/start-once")
            self.assertEqual(payload["camera_request_id"],self.request_id)
            self.last_joint=4;field="t";target=math.radians(55)
        self.command = True
        if self.fail_command:
            self.done.set()
            raise OSError("response lost after a possible physical write")
        if not self.no_progress:
            self.status[field] = target
            pose = forward(self.status)
            self.status.update(zip("xyz", pose["xyz_mm"])); self.status["tit"] = pose["pitch_rad"]
        return {"joint": self.last_joint, "to_deg": math.degrees(self.status[field])}

    def camera(self, home, method, endpoint, payload=None):
        if endpoint == "/health":
            return {"camera_service": "ready", "robot_control": False,
                    "recording": self.warming.is_set() and not self.done.is_set(),
                    "recording_request_ids": True, "recording_request_id": "other" if self.wrong_record else self.request_id,
                    "phase": "recording" if self.warming.is_set() else "idle",
                    "recording_started_monotonic_ns": self.capture_start if self.warming.is_set() else None}
        self.assertEqual((method, endpoint), ("POST", "/record"))
        self.assertEqual(payload["seconds"], 10)
        self.request_id = payload["request_id"]
        self.capture_start = start.time.monotonic_ns()
        self.warming.set()
        self.done.wait(.2)  # synthetic recording, not ten seconds of real hardware
        folder = self.home/".evoarm/rsi/experiments"/("a"*32)
        folder.mkdir(parents=True, exist_ok=True)
        (folder/"manifest.json").write_text(json.dumps({"trial_id":folder.name,"simulated":False,"status":"recorded"}))
        (folder/"frames.jsonl").write_text("\n".join(json.dumps({"host_monotonic_ns":self.capture_start+i*1_000_000_000}) for i in range(11)))
        (folder/"rgb.mp4").write_bytes(b"synthetic placeholder; no physical camera")
        (folder/"depth_samples.npz").write_bytes(b"synthetic placeholder")
        return {"trial_id": "a"*32, "request_id": self.request_id}

    def run_lift(self, execute=True):
        with (patch.object(start, "_controller_get", side_effect=self.get),
              patch.object(start, "_controller_post", side_effect=self.post),
              patch.object(start, "client_request", side_effect=self.camera)):
            return start.run(setup_id="bench-layout-v2", home=self.home, execute=execute)

    def test_readonly_never_records_or_sends_a_command(self):
        report = self.run_lift(False)
        self.assertFalse(report["robot_moved"])
        self.assertEqual(self.calls, [])
        self.assertFalse(self.warming.is_set())

    def test_rejected_prediction_returns_observed_state_without_motion_or_recording(self):
        self.status["x"]+=20
        report=self.run_lift()
        self.assertTrue(report["blocked"])
        self.assertFalse(report["command_attempted"])
        self.assertEqual(report["observed_status"]["x"],self.status["x"])
        self.assertEqual(self.calls,[])
        self.assertFalse(self.warming.is_set())

    def test_real_execution_contract_records_before_motion_and_disarms_without_torque(self):
        report = self.run_lift()
        self.assertTrue(report["motion_feedback_verified"])
        self.assertTrue(report["movement_covered_by_recording"])
        manifest=json.loads((self.home/".evoarm/rsi/experiments"/("a"*32)/"manifest.json").read_text())
        self.assertTrue(manifest["robot_motion_executed"])
        self.assertFalse(manifest["grasp_attempted"])
        self.assertGreater(report["measured_lift_mm"], 8)
        self.assertFalse(report["grasp_attempted"])
        self.assertEqual([x[0] for x in self.calls], ["/api/step", "/api/disarm"])
        self.assertTrue((self.path.parent/"first-motion/lift-attempt.json").is_file())
        self.assertTrue((self.home/".evoarm/rsi/experiments"/("a"*32)/"pregrasp-lift.json").is_file())
        # Even if someone re-arms later, the preparation cannot replay a
        # physical command when a response may have been lost.
        self.armed = True
        before = len(self.calls)
        with self.assertRaisesRegex(RuntimeError,"receipt"): self.run_lift()
        self.assertEqual(len(self.calls), before)

    def test_uncertain_post_is_never_retried_or_reversed(self):
        self.fail_command = True
        report = self.run_lift()
        self.assertFalse(report["motion_feedback_verified"])
        self.assertIn("response lost", report["motion_error"])
        self.assertEqual([x[0] for x in self.calls], ["/api/step", "/api/disarm"])
        self.armed = True
        with self.assertRaisesRegex(RuntimeError, "receipt"): self.run_lift()
        self.assertEqual(len(self.calls), 2)

    def test_acknowledgement_without_actual_tip_lift_is_not_success(self):
        self.no_progress = True
        report = self.run_lift()
        self.assertTrue(report["command_acknowledged"])
        self.assertFalse(report["motion_feedback_verified"])
        self.assertIn("did not verify", report["motion_error"])

    def test_another_recording_request_is_not_permission_to_move(self):
        self.wrong_record = True
        report = self.run_lift()
        self.assertFalse(report["command_attempted"])
        self.assertEqual([c[0] for c in self.calls], ["/api/disarm"])
        self.assertFalse((self.path.parent/"first-motion/lift-attempt.json").exists())

    def test_already_high_execute_does_not_unlock_record_or_send_a_motion(self):
        self.status=high_pose(self.status,wrist=0)
        self.armed=False
        report=self.run_lift()
        self.assertTrue(report["start_clearance_ready"])
        self.assertFalse(report["motion_required"])
        self.assertFalse(report["robot_moved"])
        self.assertEqual(self.calls,[])
        self.assertFalse(self.warming.is_set())

    def test_high_wrist_recovery_records_and_verifies_j4_not_another_lift(self):
        self.status=high_pose(self.status)
        report=self.run_lift()
        self.assertEqual(report["start_mode"],"high_wrist_recovery")
        self.assertTrue(report["motion_feedback_verified"],report)
        self.assertTrue(report["start_clearance_ready"],report)
        self.assertLessEqual(math.degrees(report["final_status"]["t"]),60.)
        self.assertEqual([c[0] for c in self.calls],["/api/bench/motion/start-once","/api/disarm"])


class EndToEndSimulatedLift(unittest.TestCase):
    def test_existing_armservice_and_real_camera_http_recording_contract(self):
        self._exercise()

    def test_high_wrist_recovery_through_guarded_api_and_ten_second_recording(self):
        self._exercise(recovery=True)

    def _exercise(self, recovery=False):
        """Real ASGI, UNIX HTTP and MP4/depth writing; both devices synthetic."""
        from control_ui import main
        from control_ui.service import ArmService
        from evoarm_rsi.camera_hub import CameraHub
        from evoarm_rsi.camera_service import CameraServer
        from tests.test_camera_hub import SyntheticSource
        from tests.test_serial_transport_errors import api_request
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp); path=home/".evoarm/rsi/calibrations/bench-layout-v2/session.json"
            path.parent.mkdir(parents=True)
            _, status=fixture(path)
            if recovery:status=high_pose(status)
            class Arm:
                def __init__(self): self.commands=[]
                def status(self): return dict(status)
                def send(self, payload):
                    self.commands.append(payload)
                    joint=4 if recovery else 3
                    if payload["T"]!=121 or payload["joint"]!=joint:
                        raise AssertionError("unexpected motor command")
                    status["t" if recovery else "e"]=math.radians(payload["angle"])
                    pose=forward(status);status.update(zip("xyz",pose["xyz_mm"]))
                    status["tit"]=pose["pitch_rad"]
            service=ArmService();service.arm=Arm();service.armed_until=time.monotonic()+60
            server=CameraServer(home/"camera.sock",os.getuid(),os.getgid(),home,
                                hub=CameraHub(SyntheticSource))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def get(base, endpoint):
                response,body=asyncio.run(api_request(endpoint,{},method="GET"))
                if response["status"]!=200: raise RuntimeError(body)
                return body
            def post(base, endpoint, payload):
                response,body=asyncio.run(api_request(endpoint,payload))
                if response["status"]!=200: raise RuntimeError(body)
                return body
            try:
                with (patch.dict(os.environ,{"EVOARM_ENABLE_MOTION":"1","EVOARM_SETUP_ID":"bench-layout-v2"}),
                      patch.object(main,"service",service),
                      patch("control_ui.bench_ui.session_path",return_value=path),
                      patch.object(start,"_controller_get",side_effect=get),
                      patch.object(start,"_controller_post",side_effect=post),
                      patch("evoarm_rsi.camera_service.socket_path",return_value=home/"camera.sock")):
                    report=start.run(setup_id="bench-layout-v2",home=home,execute=True)
                self.assertTrue(report["motion_feedback_verified"],report)
                self.assertEqual(report["start_mode"],"high_wrist_recovery" if recovery else "low_pose_lift")
                if recovery:self.assertTrue(report["start_clearance_ready"],report)
                self.assertTrue(report["movement_covered_by_recording"],report)
                self.assertNotIn("camera_error",report)
                self.assertGreater(report["recording_observed_span_s"],9.)
                self.assertEqual(len(service.arm.commands),1)
                self.assertEqual(service.armed_until,0)
                self.assertIsNone(service.pending)
                trial=home/".evoarm/rsi/experiments"/report["capture_trial_id"]
                manifest=json.loads((trial/"manifest.json").read_text())
                self.assertEqual(manifest["camera_request_id"],report["camera_request_id"])
                self.assertTrue(manifest["robot_motion_executed"])
                self.assertFalse(manifest["grasp_attempted"])
            finally:
                server.shutdown();server.server_close();thread.join(timeout=2)
