"""Fast teaching regression tests: fake hardware only, no serial robot actuation."""
import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from control_ui.service import ArmService
from control_ui.fast_teach import capture_pose, read_poses
from control_ui.safety import UnsafeMotion


class FakeArm:
    def __init__(self):
        self.commands = []
        self.raw = {"T":1051, "x":220., "y":0., "z":140., "tit":0.,
                    "b":0., "s":0., "e":math.pi/2, "t":0., "r":0., "g":2.0}
        self.raw.update({"torswitch"+k:1 for k in "BSETRG"})

    def status(self, timeout=None):
        return dict(self.raw)

    def send(self, command):
        self.commands.append(command.copy())


class FastTeachingTests(unittest.TestCase):
    def setup_arm(self):
        svc=ArmService()
        svc.arm=FakeArm()
        svc.unlock_once("已检查设备并清空运动区域")
        return svc

    def test_one_absolute_target_sends_one_bounded_joint_command(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION":"1"}):
            svc=self.setup_arm()
            result=svc.teach_move_to(4,20.0)
            self.assertEqual(result["status"],"sent_not_verified")
            self.assertEqual(len(svc.arm.commands),1)
            self.assertEqual(svc.arm.commands[0]["T"],121)
            self.assertEqual(svc.arm.commands[0]["joint"],4)
            self.assertEqual(svc.arm.commands[0]["angle"],20.0)
            with self.assertRaises(UnsafeMotion):
                svc.teach_move_to(4,25.0)  # pending, no second command
            self.assertEqual(len(svc.arm.commands),1)

    def test_excessive_delta_and_joint_limit_send_nothing(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION":"1"}):
            svc=self.setup_arm()
            for goal in (45.0, 999.0):
                with self.assertRaises(UnsafeMotion):
                    svc.teach_move_to(4,goal)
            self.assertEqual(svc.arm.commands,[])

    def test_gripper_uses_t106(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION":"1"}):
            svc=self.setup_arm()
            target=math.degrees(2.0)+10
            svc.teach_move_to(6,target)
            self.assertEqual(svc.arm.commands[0]["T"],106)
            self.assertAlmostEqual(svc.arm.commands[0]["cmd"],math.radians(target),4)

    def test_saved_snapshots_do_not_modify_xy_or_profile(self):
        with tempfile.TemporaryDirectory() as home:
            with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION":"1","HOME":home}):
                with patch("control_ui.fast_teach.Path.home",return_value=Path(home)):
                    svc=self.setup_arm()
                    xy=Path(home)/".evoarm"/"robot_xy_calibration.json"
                    profile=Path(home)/".evoarm"/"pick_profile.json"
                    xy.parent.mkdir(parents=True)
                    xy.write_text("xy untouched")
                    profile.write_text("profile untouched")
                    capture_pose(svc,"grasp",sleep_fn=lambda _:None)
                    capture_pose(svc,"gripper_open",sleep_fn=lambda _:None)
                    result=read_poses()
                    self.assertIn("grasp",result["poses"])
                    self.assertNotIn("xyz_mm",result["poses"]["gripper_open"])
                    self.assertEqual(xy.read_text(),"xy untouched")
                    self.assertEqual(profile.read_text(),"profile untouched")
                    self.assertEqual(svc.arm.commands,[])

    def test_capture_fails_when_arm_feedback_is_moving(self):
        with tempfile.TemporaryDirectory() as home:
            with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION":"1"}):
                with patch("control_ui.fast_teach.Path.home",return_value=Path(home)):
                    svc=self.setup_arm()
                    state=[0]
                    def status():
                        state[0]+=1
                        data=svc.arm.status()
                        data["x"] += 8*state[0]
                        return data
                    with patch.object(svc,"status",side_effect=status):
                        with self.assertRaises(UnsafeMotion):
                            capture_pose(svc,"grasp",sleep_fn=lambda _:None)


if __name__=="__main__":
    unittest.main()
