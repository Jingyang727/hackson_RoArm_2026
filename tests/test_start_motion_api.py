"""Measured high-wrist recovery through the existing sole serial owner."""
import asyncio
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

from control_ui import main
from control_ui.service import ArmService
from evoarm_rsi.roarm_geometry import forward
from tests.test_bench_start import fixture, high_pose
from tests.test_serial_transport_errors import api_request


class StartMotionAPI(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/"session.json"
        self.session,status=fixture(self.path)
        self.status=high_pose(status)
        owner=self
        class Arm:
            fail=False
            def __init__(self):self.commands=[]
            def status(self):return dict(owner.status)
            def send(self,payload):
                self.commands.append(payload)
                if self.fail:raise OSError("synthetic lost write acknowledgement")
        self.service=ArmService();self.service.arm=Arm();self.service.armed_until=time.monotonic()+60
        self.nonce=uuid.uuid4().hex
        self.camera={"robot_control":False,"recording":True,"phase":"recording",
                     "recording_request_id":self.nonce,"recording_started_monotonic_ns":time.monotonic_ns()}
        for target,kwargs in (("control_ui.main.service",{"new":self.service}),
                              ("control_ui.bench_ui.session_path",{"return_value":self.path}),
                              ("control_ui.start_motion.client_request",{"return_value":self.camera})):
            p=patch(target,**kwargs);p.start();self.addCleanup(p.stop)
        p=patch.dict(os.environ,{"EVOARM_ENABLE_MOTION":"1","EVOARM_SETUP_ID":"bench-layout-v2"})
        p.start();self.addCleanup(p.stop)

    def payload(self):
        return {"reference_sha256":self.session["reference_sha256"],
                "expected_joint_rad":[self.status[k] for k in "bsetrg"],"camera_request_id":self.nonce}

    def request(self,payload=None):
        return asyncio.run(api_request("/api/bench/motion/start-once",payload or self.payload()))

    def test_requires_real_lease_and_does_not_modify_global_limits(self):
        from control_ui.safety import JOINTS, plan_to_angle, UnsafeMotion
        self.service.armed_until=0
        response,_=self.request()
        self.assertEqual(response["status"],409)
        self.assertEqual(self.service.arm.commands,[])
        self.assertEqual(JOINTS[4][2:],(-60.,60.))
        with self.assertRaises(UnsafeMotion):plan_to_angle(self.status,4,55.)

    def test_only_one_server_computed_inward_target_and_nonce_is_consumed(self):
        response,body=self.request()
        self.assertEqual(response["status"],200,body)
        self.assertEqual(self.service.arm.commands,[{"T":121,"joint":4,"angle":55.,"spd":15,"acc":20}])
        self.assertEqual(self.service.last_command["bench_camera_request_id"],self.nonce)
        again,_=self.request()
        self.assertEqual(again["status"],409)
        self.assertEqual(len(self.service.arm.commands),1)

    def test_other_recording_stale_recording_or_changed_reference_cannot_move(self):
        for change in ({"recording_request_id":"b"*32},
                       {"recording_started_monotonic_ns":time.monotonic_ns()-3_000_000_000}):
            with patch("control_ui.start_motion.client_request",return_value={**self.camera,**change}):
                response,_=self.request()
            self.assertEqual(response["status"],409)
        response,_=self.request({**self.payload(),"reference_sha256":"0"*64})
        self.assertEqual(response["status"],409)
        self.assertEqual(self.service.arm.commands,[])

    def test_arbitrary_motor_targets_cannot_be_supplied(self):
        response,_=self.request({**self.payload(),"target_deg":90})
        self.assertEqual(response["status"],422)
        self.assertEqual(self.service.arm.commands,[])

    def test_pose_changed_after_prediction_is_rejected(self):
        expected=self.payload()
        self.status["t"]-=math.radians(2)
        pose=forward(self.status);self.status.update(zip("xyz",pose["xyz_mm"]));self.status["tit"]=pose["pitch_rad"]
        response,_=self.request(expected)
        self.assertEqual(response["status"],409)
        self.assertEqual(self.service.arm.commands,[])

    def test_uncertain_write_latches_fault_and_cannot_replay(self):
        self.service.arm.fail=True
        response,_=self.request()
        self.assertEqual(response["status"],409)
        self.assertEqual(self.service.fault["code"],"motion_transmission_unverified")
        self.assertEqual(self.service.armed_until,0)
        response,_=self.request()
        self.assertEqual(response["status"],409)
        self.assertEqual(len(self.service.arm.commands),1)
