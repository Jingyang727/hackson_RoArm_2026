"""No real motor hardware: 10-degree adaptive feedback experiment tests."""
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from evoarm_rsi.control_calibration import (
    run_control_experiments,_direction,
)


class VirtualController:
    def __init__(self, response=.85, fault_after_send=False):
        self.positions={1:0.0,2:0.0,3:90.0,4:0.0,5:0.0,6:180.0}
        self.commands=[]
        self.ratio=response
        self.fault_after_send=fault_after_send
        self.fault=None
    def get(self,base,endpoint):
        if endpoint=="/api/health":
            return {
                "connected":True,"motion_enabled":True,"armed":True,
                "teach_active":False,"fault":self.fault,"pending":None,
                "allowed_joints":[1,2,3,4,5,6],
                "last_command":{"status":"feedback_confirmed"},
            }
        if endpoint=="/api/status":
            xyz=[350.0+self.positions[1],-5.0+self.positions[2],
                 185.0+self.positions[3]-90.0]
            return {
                "T":1051,"b":math.radians(self.positions[1]),
                "s":math.radians(self.positions[2]),
                "e":math.radians(self.positions[3]),
                "t":math.radians(self.positions[4]),
                "r":math.radians(self.positions[5]),
                "g":math.radians(self.positions[6]),
                "x":xyz[0],"y":xyz[1],"z":xyz[2],
                **{"torswitch"+c:1 for c in "BSETRG"},
            }
        raise AssertionError("unexpected endpoint "+endpoint)
    def move(self,base,joint,target):
        self.commands.append((joint,target))
        self.positions[joint]+=self.ratio*(target-self.positions[joint])
        if self.fault_after_send:
            self.fault={"code":"motion_stalled_or_offset"}
        return {"status":"sent_not_verified","joint":joint}


class CalibrationTests(unittest.TestCase):
    def test_ten_degrees_not_one_or_two(self):
        with self.assertRaisesRegex(ValueError,"5° or 10°"):
            run_control_experiments(degrees=1)
        with self.assertRaisesRegex(ValueError,"5° or 10°"):
            run_control_experiments(degrees=3)
        self.assertEqual(_direction(174.0,1,10,1),-1)
        self.assertEqual(_direction(74.0,1,10,1),1)

    def test_preview_does_not_send_motor_commands(self):
        robot=VirtualController()
        with patch("evoarm_rsi.control_calibration._controller_get",
                   side_effect=robot.get),patch(
                   "evoarm_rsi.control_calibration._controller_absolute",
                   side_effect=robot.move):
            result=run_control_experiments(execute=False)
        self.assertEqual(robot.commands,[])
        self.assertEqual(result["status"],"read_only_preview")
        self.assertEqual(result["degrees"],10)
        self.assertEqual([r["joint"] for r in
                          result["planned_initial_movements"]],[1,2,3])

    def test_real_feedback_drives_return_not_blind_reverse(self):
        robot=VirtualController(response=.85)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/"run.json"
            with (patch("evoarm_rsi.control_calibration._controller_get",
                        side_effect=robot.get),
                  patch("evoarm_rsi.control_calibration._controller_absolute",
                        side_effect=robot.move),
                  patch("evoarm_rsi.control_calibration.time.sleep")):
                result=run_control_experiments(
                    joints=(1,),cycles=1,execute=True,consent=True,
                    output_file=out)
            self.assertEqual(result["status"],"completed")
            self.assertEqual(result["completed_actions"],2)
            self.assertEqual(len(robot.commands),4)
            self.assertEqual(robot.commands[0],(1,10.0))
            self.assertEqual(robot.commands[1],(1,0.0))
            self.assertAlmostEqual(result["actions"][0]["observed_forward_deg"],8.5)
            self.assertEqual(result["actions"][0]["forward_error_deg"],1.5)
            self.assertTrue(out.exists())

    def test_joint_two_and_three_not_j1_only(self):
        robot=VirtualController(response=1.0)
        with tempfile.TemporaryDirectory() as tmp:
            with (patch("evoarm_rsi.control_calibration._controller_get",
                        side_effect=robot.get),
                  patch("evoarm_rsi.control_calibration._controller_absolute",
                        side_effect=robot.move),
                  patch("evoarm_rsi.control_calibration.time.sleep")):
                result=run_control_experiments(
                    joints=(1,2,3),execute=True,consent=True,
                    output_file=Path(tmp)/"results.json")
        self.assertEqual(result["status"],"completed")
        self.assertEqual(result["completed_actions"],6)
        self.assertEqual(len(robot.commands),12)
        self.assertEqual({j for j,_ in robot.commands},{1,2,3})

    def test_real_fault_stops_batch_without_factory_reset_or_retry(self):
        robot=VirtualController(fault_after_send=True)
        with tempfile.TemporaryDirectory() as tmp:
            with (patch("evoarm_rsi.control_calibration._controller_get",
                        side_effect=robot.get),
                  patch("evoarm_rsi.control_calibration._controller_absolute",
                        side_effect=robot.move),
                  patch("evoarm_rsi.control_calibration.time.sleep")):
                result=run_control_experiments(
                    joints=(1,2,3),execute=True,consent=True,
                    output_file=Path(tmp)/"fault.json")
        self.assertEqual(result["status"],"stopped")
        self.assertEqual(len(robot.commands),1)
        self.assertFalse(result["factory_reset_sent"])
        self.assertEqual(result["completed_actions"],0)
        self.assertIn("fault",result["stop_reason"])


if __name__=="__main__":
    unittest.main()
