"""Supervised T104/T106 execution with fake feedback; never drive real hardware."""
import math
import os
import unittest
from unittest.mock import patch

from control_ui.pick_execution import PickExecutor, PickExecutionError
from control_ui.service import ArmService


class SimulatedArm:
    def __init__(self):
        self.commands = []
        self.current = {"x": 190.0, "y": 0.0, "z": 165.0, "tit": 0.0,
                        "b": 0.0, "s": 0.0, "e": math.pi/2,
                        "t": 0.0, "r": 0.0, "g": 1.5}

    def status(self, timeout=None):
        return {"T":1051, **self.current,
                **{"torswitch"+i:1 for i in "BSETRG"}}

    def send(self, command):
        self.commands.append(command.copy())
        if command["T"]==104:
            self.current.update({key:command[key] for key in "xyz"})
            self.current["tit"]=command["t"];self.current["r"]=command["r"]
            self.current["g"]=command["g"]
        elif command["T"]==106:
            self.current["g"]=command["cmd"]


def plan():
    return {
      "ball_robot_xy_mm":[200.0,0.0],
      "waypoints":[
        {"stage":"hover_ball","kind":"cartesian",
         "xyz_mm":[200.0,0.0,165.0],"t":0.0,"r":0.0,"g":1.5,"spd":0.12},
        {"stage":"open_before_pick","kind":"gripper",
         "g":1.5,"spd":100,"acc":10},
        {"stage":"descend_pick_part1","kind":"cartesian",
         "xyz_mm":[200.0,0.0,145.0],"t":0.0,"r":0.0,"g":1.5,"spd":0.12},
      ],
    }


class SupervisedPickTests(unittest.TestCase):
    def service(self):
        svc=ArmService()
        svc.arm=SimulatedArm()
        svc.unlock_once("已检查设备并清空运动区域")
        return svc

    def test_opt_in_is_required(self):
        with patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"0","EVOARM_ENABLE_MOTION":"1"}),\
             patch("control_ui.pick_execution.build_from_saved_files",return_value=plan()):
            svc=self.service();exe=PickExecutor()
            with self.assertRaises(PickExecutionError):
                exe.start([100,50],[1920,1080],[[0,0]]*54,svc,
                          "已核对标定、示教参数、桌面与工作空间")
            self.assertEqual(svc.arm.commands,[])

    def test_one_step_at_a_time_with_verified_feedback(self):
        with patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1","EVOARM_ENABLE_MOTION":"1"}),\
             patch("control_ui.pick_execution.build_from_saved_files",return_value=plan()):
            svc=self.service();exe=PickExecutor()
            result=exe.start([100,50],[1920,1080],[[0,0]]*54,svc,
                              "已核对标定、示教参数、桌面与工作空间")
            token=result["session_id"]
            out=exe.step(token,0,"已目视确认本步的运动空间与位置",svc)
            self.assertEqual(out["command"]["T"],104)
            self.assertIsNotNone(svc.pending)
            with self.assertRaises(PickExecutionError):
                exe.step(token,1,"已目视确认本步的运动空间与位置",svc)
            svc.status()
            self.assertEqual(svc.last_command["status"],"feedback_confirmed")
            svc.last_motion_at -= 1.0
            noop=exe.step(token,1,"已目视确认本步的运动空间与位置",svc)
            self.assertFalse(noop["sent"])  # T104 already set the gripper to the open angle.
            self.assertTrue(noop["verified"])
            self.assertEqual(svc.last_command["status"],"feedback_confirmed")
            self.assertIsNone(svc.pending)
            svc.last_motion_at -= 1.0
            out=exe.step(token,2,"已目视确认本步的运动空间与位置",svc)
            self.assertEqual(out["command"]["T"],104)
            self.assertEqual(out["command"]["z"],145)
            svc.status()
            self.assertTrue(exe.state(svc)["completed"])
            self.assertEqual(len(svc.arm.commands),2)

    def test_cannot_replay_old_index(self):
        with patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1","EVOARM_ENABLE_MOTION":"1"}),\
             patch("control_ui.pick_execution.build_from_saved_files",return_value=plan()):
            svc=self.service();exe=PickExecutor()
            token=exe.start([100,50],[1920,1080],[[0,0]]*54,svc,
                           "已核对标定、示教参数、桌面与工作空间")["session_id"]
            exe.step(token,0,"已目视确认本步的运动空间与位置",svc)
            svc.status()
            with self.assertRaises(PickExecutionError):
                exe.step(token,0,"已目视确认本步的运动空间与位置",svc)
            self.assertEqual(len(svc.arm.commands),1)

    def test_cannot_start_from_low_or_far_initial_pose(self):
        with patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1","EVOARM_ENABLE_MOTION":"1"}),\
             patch("control_ui.pick_execution.build_from_saved_files",return_value=plan()):
            svc=self.service();exe=PickExecutor()
            token=exe.start([100,50],[1920,1080],[[0,0]]*54,svc,
                           "已核对标定、示教参数、桌面与工作空间")["session_id"]
            svc.arm.current["z"]=110
            with self.assertRaises(PickExecutionError):
                exe.step(token,0,"已目视确认本步的运动空间与位置",svc)
            svc.arm.current["z"]=165;svc.arm.current["x"]=20
            with self.assertRaises(PickExecutionError):
                exe.step(token,0,"已目视确认本步的运动空间与位置",svc)
            self.assertEqual(svc.arm.commands,[])

    def test_diagnose_reports_pose_mismatch_without_motion(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_PICK":"1","EVOARM_ENABLE_MOTION":"1"}), \
             patch("control_ui.pick_execution.build_from_saved_files",return_value=plan()):
            svc=self.service()
            svc.arm.current["tit"]=math.radians(25)
            exe=PickExecutor()
            result=exe.start([100,50],[1920,1080],[[0,0]]*54,svc,
                             "已核对标定、示教参数、桌面与工作空间")
            diagnostic=exe.diagnose(svc)
            self.assertFalse(diagnostic["ready"])
            self.assertAlmostEqual(diagnostic["pitch_delta_deg"],25.0)
            self.assertEqual(svc.arm.commands,[])
            with self.assertRaisesRegex(PickExecutionError,"当前Pitch"):
                exe.step(result["session_id"],0,
                         "已目视确认本步的运动空间与位置",svc)
            self.assertEqual(svc.arm.commands,[])

    def test_roll_wraparound_is_not_a_large_orientation_difference(self):
        p=plan()
        p["waypoints"][0]["r"]=math.radians(-179)
        with patch.dict(os.environ, {"EVOARM_ENABLE_PICK":"1","EVOARM_ENABLE_MOTION":"1"}), \
             patch("control_ui.pick_execution.build_from_saved_files",return_value=p):
            svc=self.service()
            svc.arm.current["r"]=math.radians(179)
            exe=PickExecutor()
            result=exe.start([100,50],[1920,1080],[[0,0]]*54,svc,
                             "已核对标定、示教参数、桌面与工作空间")
            diagnostic=exe.diagnose(svc)
            self.assertAlmostEqual(diagnostic["roll_delta_deg"],-2.0)
            self.assertTrue(diagnostic["ready"])
            # Only a simulated arm is used here; this is not a hardware approval.
            exe.step(result["session_id"],0,
                     "已目视确认本步的运动空间与位置",svc)
            self.assertEqual(len(svc.arm.commands),1)

    def test_cancel_prevents_further_motor_commands(self):
        with patch.dict(os.environ,{"EVOARM_ENABLE_PICK":"1","EVOARM_ENABLE_MOTION":"1"}),\
             patch("control_ui.pick_execution.build_from_saved_files",return_value=plan()):
            svc=self.service();exe=PickExecutor()
            token=exe.start([100,50],[1920,1080],[[0,0]]*54,svc,
                           "已核对标定、示教参数、桌面与工作空间")["session_id"]
            exe.cancel(svc)
            with self.assertRaises(PickExecutionError):
                exe.step(token,0,"已目视确认本步的运动空间与位置",svc)
            self.assertEqual(svc.arm.commands,[])


if __name__=="__main__":
    unittest.main()
