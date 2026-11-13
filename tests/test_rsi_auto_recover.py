"""Real-feedback-only RSI Cartesian auto recovery. No hardware actuations in tests."""
import math
import time
import unittest
from unittest.mock import patch

from control_ui.service import ArmService, CART_MIN_STALL_ELAPSED_S
from control_ui.safety import UnsafeMotion
from control_ui.pick_execution import PickExecutor,PickExecutionError


class FakeArm:
    def __init__(self):
        self.commands=[]
        self.j1=0.0
        self.xyz=[360.0,-2.2,186.2]
    def status(self,*a,**k):
        return {
            "T":1051,
            "b":math.radians(self.j1),"s":0.0,"e":math.pi/2,
            "t":0.0,"r":0.0,"g":math.pi,
            "x":self.xyz[0],"y":self.xyz[1],"z":self.xyz[2],
            **{"torswitch"+v:1 for v in "BSETRG"},
        }
    def send(self,*a,**k):
        self.commands.append((a,k))
        raise AssertionError("software fault recovery must never command motors")


class RSIAutoRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.svc=ArmService()
        self.arm=FakeArm()
        self.svc.arm=self.arm
        self.svc.last_torque_status=[1]*6
        self.svc.armed_until=time.monotonic()+150
        self.session="session-123"
        self.svc.last_command={
            "pick_session_id":self.session,
            "pick_step":0,
            "status":"unverified_fault",
        }
        self.svc.fault={
            "code":"cartesian_stalled",
            "message":"XYZ stable but 20mm away",
            "error_mm":20.35,
        }

    def test_automatic_stall_recovery_is_read_only_and_disarms(self):
        with patch("control_ui.service.time.sleep",return_value=None):
            result=self.svc.recover_rsi_cartesian_stall(self.session)
        self.assertTrue(result["software_fault_cleared"])
        self.assertFalse(result["motor_command_sent"])
        self.assertFalse(result["motion_reauthorized"])
        self.assertAlmostEqual(result["actual_xyz_mm"][2],186.2)
        self.assertIsNone(self.svc.fault)
        self.assertIsNone(self.svc.pending)
        self.assertEqual(self.svc.armed_until,0)
        self.assertEqual(self.arm.commands,[])

    def test_other_session_or_non_cartesian_fault_not_auto_cleared(self):
        with self.assertRaisesRegex(UnsafeMotion,"不属于本次"):
            self.svc.recover_rsi_cartesian_stall("other-session")
        self.assertIsNotNone(self.svc.fault)
        self.svc.fault["code"]="feedback_timeout"
        with self.assertRaisesRegex(UnsafeMotion,"仅支持"):
            self.svc.recover_rsi_cartesian_stall(self.session)
        self.assertEqual(self.arm.commands,[])

    def test_joint_or_xyz_drift_keeps_fault_latched(self):
        state={"reads":0}
        old=self.arm.status
        def changing(*args,**kwargs):
            state["reads"]+=1
            feedback=old(*args,**kwargs)
            feedback["z"]=186.2+state["reads"]*4.0
            return feedback
        self.arm.status=changing
        with patch("control_ui.service.time.sleep",return_value=None):
            with self.assertRaisesRegex(UnsafeMotion,"仍有运动"):
                self.svc.recover_rsi_cartesian_stall(self.session)
        self.assertIsNotNone(self.svc.fault)
        self.assertEqual(self.arm.commands,[])

    def test_cartesian_plateau_before_eight_seconds_not_stalled(self):
        self.svc.fault=None
        self.svc.pending={
            "kind":"cartesian",
            "xyz_start":[360.0,-11.2,204.4],
            "xyz_target":[360.9,-11.2,204.4],
            "since":time.monotonic()-3.0,
            "stable_intervals":3
        }
        with patch("control_ui.service.observe_stability",return_value=True):
            self.svc.status()
        self.assertIsNone(self.svc.fault)
        self.assertIsNotNone(self.svc.pending)
        self.svc.pending["since"]=time.monotonic()-(CART_MIN_STALL_ELAPSED_S+0.5)
        with patch("control_ui.service.observe_stability",return_value=True):
            self.svc.status()
        self.assertEqual(self.svc.fault["code"],"cartesian_stalled")
        self.assertEqual(self.arm.commands,[])

    def test_pick_session_recovery_cancels_remaining_movements(self):
        executor=PickExecutor()
        executor.session_id=self.session
        executor.plan={
            "kind":"rsi_v2_gemini_grasp_and_lift_candidate",
            "waypoints":[{"stage":"hover_ball","kind":"cartesian"}],
        }
        executor.rsi_batch_approved=True
        executor.rsi_batch_until=time.monotonic()+60
        with patch("control_ui.service.time.sleep",return_value=None):
            result=executor.recover_rsi_stall(self.session,self.svc)
        self.assertTrue(result["session_cancelled"])
        self.assertFalse(result["may_resume"])
        self.assertTrue(executor.cancelled)
        self.assertEqual(self.arm.commands,[])
        with self.assertRaises(PickExecutionError):
            executor.recover_rsi_stall(self.session,self.svc)


if __name__=="__main__":
    unittest.main()
