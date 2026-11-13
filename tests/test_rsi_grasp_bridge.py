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
        self.xyz=[360.339,-4.975,182.572]
    def status(self,timeout=None):
        angles=getattr(self,"angles",{1:0.,2:0.,3:90.,4:0.,5:0.,6:180.})
        return {
            "T":1051,"x":self.xyz[0],"y":self.xyz[1],"z":self.xyz[2],
            "tit":0.31,"r":0.02,"g":2.587826,
            "b":math.radians(angles[1]),"s":math.radians(angles[2]),
            "e":math.radians(angles[3]),"t":math.radians(angles[4]),
            "r":0.02,
            "g":math.radians(angles[6]),
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
            self.assertEqual(session["next_step"]["stage"],"current_high_align")
            self.assertEqual(self.arm.commands,[])
            # This endpoint never automatically sends the full grasp.
            self.assertFalse(session["auto_execute"])

    def test_one_time_operator_batch_still_uses_existing_step_limits(self):
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{
                  "EVOARM_ENABLE_PICK":"1","EVOARM_ENABLE_MOTION":"1",
                  "EVOARM_ENABLE_RSI_GRASP":"1","EVOARM_ENABLE_RSI_BATCH":"1",
              })):
            session=self.executor.start_rsi_grasp(
                self.trial,self.service,
                "已检查当前Gemini黄球位置、夹爪高度和运动区域"
            )
            self.assertTrue(session["auto_execute"])
            token=session["session_id"]
            first=self.executor.step(
                token,0,"已批准一次RSI抓取试次自动分段执行",self.service
            )
            self.assertEqual(first["stage"],"current_high_align")
            self.assertEqual(len(self.arm.commands),1)
            with self.assertRaises(PickExecutionError):
                self.executor.step(
                    token,1,"已批准一次RSI抓取试次自动分段执行",self.service
                )
            self.assertEqual(len(self.arm.commands),1)
            self.executor.cancel(self.service)
            with self.assertRaises(PickExecutionError):
                self.executor.step(
                    token,1,"已批准一次RSI抓取试次自动分段执行",self.service
                )

    def test_near_factory_joint_offsets_are_accepted_not_precise_equality(self):
        # J1's 3.69 deg and J2 gravity sag of -4.04 deg should NOT
        # invalidate the entire grasp. The actual Cartesian start drives
        # the local transit plan.
        self.arm.xyz=[360.339,-4.975,204.0]
        self.arm.angles={1:3.69,2:-4.04,3:90.,4:0.,5:0.,6:180.}
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1",
                  "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"1"})):
            session=self.executor.start_rsi_grasp(
                self.trial,self.service,
                "已检查当前Gemini黄球位置、夹爪高度和运动区域"
            )
        self.assertEqual(session["next_step"]["stage"],"current_high_align")
        self.assertEqual(self.arm.commands,[])

    def test_one_trial_operator_permission_avoids_restart(self):
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"0",
                  "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"0",
                  "EVOARM_ENABLE_RSI_BATCH":"0"})):
            self.service.status()
            approval=self.executor.enable_rsi_once(
                self.service,"已在现场确认本次RSI抓取、运动范围和可断电"
            )
            self.assertFalse(approval["restart_required"])
            session=self.executor.start_rsi_grasp(
                self.trial,self.service,
                "已检查当前Gemini黄球位置、夹爪高度和运动区域"
            )
            self.assertTrue(session["auto_execute"])
            self.assertEqual(self.executor.rsi_runtime_until,0)
            # The old legacy pick startup flags are still OFF, but
            # a one-time operator-authorized RSI session may be stepped.
            token=session["session_id"]
            self.executor.step(
                token,0,"已批准一次RSI抓取试次自动分段执行",self.service
            )
            self.assertEqual(len(self.arm.commands),1)

    def test_readonly_runner_never_connects_to_camera(self):
        from evoarm_rsi.grasp_run import run_grasp_attempt
        from unittest.mock import patch
        with (patch("evoarm_rsi.grasp_run._controller_get",
                    side_effect=[{"connected":True,"fault":None,"pending":None,
                                  "armed":False,"torque_status":[1]*6},
                                 {"x":362.17,"y":-1.11,"z":-34.89}]),
              patch("evoarm_rsi.grasp_run.client_request") as camera):
            output=run_grasp_attempt(self.trial,execute=False)
        self.assertFalse(output["robot_moved"])
        camera.assert_not_called()

    def test_stale_camera_or_unapproved_mode_rejected_before_motion(self):
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1",
                 "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"0"})):
            with self.assertRaisesRegex(PickExecutionError,"one-trial operator permission"):
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

    def test_not_at_official_pose_no_motion(self):
        self.arm.xyz=[360,-5,100]
        self.arm.angles={1:20.,2:0.,3:90.,4:0.,5:0.,6:180.}
        with (patch("pathlib.Path.home",return_value=self.home),
              patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1",
                 "EVOARM_ENABLE_MOTION":"1","EVOARM_ENABLE_RSI_GRASP":"1"})):
            with self.assertRaisesRegex(PickExecutionError,"too far from official initial"):
                self.executor.start_rsi_grasp(
                    self.trial,self.service,
                    "已检查当前Gemini黄球位置、夹爪高度和运动区域"
                )
        self.assertEqual(self.arm.commands,[])


if __name__=="__main__":
    unittest.main()
