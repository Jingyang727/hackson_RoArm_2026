"""Fake motor tests for Gemini-v2 first full supervised grasp session."""
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from control_ui.pick_execution import PickExecutor,PickExecutionError
from control_ui.service import ArmService
from tests.test_rsi_grasp_candidate import POSES,Q


class FakeArm:
    def __init__(self):
        self.commands=[]
        self.xyz=[362.171,-1.111,-34.895]
    def status(self,timeout=None):
        return {
            "T":1051,"x":self.xyz[0],"y":self.xyz[1],"z":self.xyz[2],
            "tit":0.31,"r":0.02,"g":2.587826,
            "b":0.,"s":0.,"e":math.pi/2,"t":0.,
            **{"torswitch"+j:1 for j in "BSETRG"}
        }
    def send(self,cmd):
        self.commands.append(cmd)


class RSIV2BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.home=Path(self.temp.name)
        self.trial="a"*32
        root=self.home/".evoarm"/"rsi"
        exp=root/"experiments"/self.trial
        analysis=root/"analysis"/self.trial
        exp.mkdir(parents=True)
        analysis.mkdir(parents=True)
        (exp/"manifest.json").write_text(json.dumps({
            "simulated":False,"trial_id":self.trial
        }))
        (analysis/"ball_qualification.json").write_text(json.dumps({
            **Q,"trial_id":self.trial
        }))
        (self.home/".evoarm"/"pick_poses_v2.json").write_text(
            json.dumps(POSES))
        self.service=ArmService()
        self.arm=FakeArm()
        self.service.arm=self.arm
        self.service.armed_until=time.monotonic()+300
        self.executor=PickExecutor()

    def test_valid_recent_gemini_generates_actual_9_step_execution_session(self):
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1",
                 "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"1"})):
            session=self.executor.start_rsi_grasp(
                self.trial,self.service,
                "已检查当前Gemini黄球位置、夹爪高度和运动区域"
            )
            self.assertGreaterEqual(session["total"],8)
            self.assertEqual(session["next_step"]["stage"],"hover_ball")
            self.assertEqual(self.arm.commands,[])
            # This endpoint never automatically sends the full grasp.
            self.assertFalse(session["auto_execute"])

    def test_stale_camera_or_unapproved_mode_rejected_before_motion(self):
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1",
                 "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"0"})):
            with self.assertRaisesRegex(PickExecutionError,"ENABLE_RSI_GRASP"):
                self.executor.start_rsi_grasp(
                    self.trial,self.service,
                    "已检查当前Gemini黄球位置、夹爪高度和运动区域"
                )
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1",
                 "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"1"})):
            report=self.home/".evoarm"/"rsi"/"analysis"/self.trial/"ball_qualification.json"
            old=time.time()-400
            os.utime(report,(old,old))
            with self.assertRaisesRegex(PickExecutionError,"older than 5"):
                self.executor.start_rsi_grasp(
                    self.trial,self.service,
                    "已检查当前Gemini黄球位置、夹爪高度和运动区域"
                )
        self.assertEqual(self.arm.commands,[])

    def test_must_be_near_recorded_hover_or_no_motion(self):
        self.arm.xyz=[360,-5,182]
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1",
                 "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"1"})):
            with self.assertRaisesRegex(PickExecutionError,"taught safe Hover"):
                self.executor.start_rsi_grasp(
                    self.trial,self.service,
                    "已检查当前Gemini黄球位置、夹爪高度和运动区域"
                )
        self.assertEqual(self.arm.commands,[])


if __name__=="__main__":
    unittest.main()
