"""Regression tests for previously invisible 'pending forever' jog lock.

All tests run with fake arm responses; no actual servo commands or USB hardware.
"""
import math
import os
import unittest
from unittest.mock import patch

from control_ui.service import (
    ArmService, JOG_CONFIRM_TIMEOUT_S, RECOVERY_SAMPLE_GAP_S,
)
from control_ui.safety import UnsafeMotion


class FakeArm:
    def __init__(self, move_on_send=False):
        self.move_on_send = move_on_send
        self.commands = []
        self.angle = [0.0, 0.0, math.pi / 2, 0.0, 0.0, math.radians(175)]
        self.next_status_delta = 0.0

    def status(self, timeout=None):
        result = {
            "T": 1051,
            **dict(zip("bsetrg", self.angle)),
            **{f"torswitch{k}": 1 for k in "BSETRG"},
        }
        if self.next_status_delta:
            self.angle[0] += math.radians(self.next_status_delta)
        return result

    def send(self, cmd):
        self.commands.append(cmd.copy())
        if self.move_on_send:
            if cmd["T"] == 121:
                self.angle[cmd["joint"] - 1] = math.radians(cmd["angle"])
            elif cmd["T"] == 106:
                self.angle[5] = cmd["cmd"]


class MotionStateTests(unittest.TestCase):
    def make(self, move_on_send=False):
        service = ArmService()
        service.arm = FakeArm(move_on_send)
        service.unlock_once("已检查设备并清空运动区域")
        return service

    def test_three_successive_jogs_do_not_appear_stuck(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}), \
             patch("control_ui.service.MIN_GAP_SECONDS", 0.0):
            svc = self.make(move_on_send=True)
            for step_number in range(1, 4):
                result = svc.step(1, 5)
                self.assertEqual(result["to_deg"], step_number * 5)
                self.assertIsNotNone(svc.pending)
                svc.status()
                self.assertIsNone(svc.pending)
                self.assertIsNone(svc.fault)
            self.assertEqual(len(svc.arm.commands), 3)
            self.assertEqual(svc.last_command["status"], "feedback_confirmed")

    def test_failure_becomes_visible_fault_not_infinite_pending(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}), \
             patch("control_ui.service.MIN_GAP_SECONDS", 0.0):
            svc = self.make(move_on_send=False)
            svc.step(1, 5)
            svc.pending["since"] -= JOG_CONFIRM_TIMEOUT_S + 1
            svc.status()
            self.assertIsNone(svc.pending)
            self.assertEqual(svc.fault["code"], "motion_not_confirmed")
            self.assertAlmostEqual(svc.fault["actual_deg"], 0)
            self.assertAlmostEqual(svc.fault["expected_deg"], 5)
            self.assertFalse(svc.health()["armed"])
            with self.assertRaises(UnsafeMotion):
                svc.step(1, 5)
            self.assertEqual(len(svc.arm.commands), 1)

    def test_stable_recovery_requires_manual_acknowledgement_and_reunlock(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}), \
             patch("control_ui.service.time.sleep", return_value=None), \
             patch("control_ui.service.MIN_GAP_SECONDS", 0.0):
            svc = self.make()
            svc.step(1, 5)
            svc.pending["since"] -= JOG_CONFIRM_TIMEOUT_S + 1
            svc.status()
            with self.assertRaises(UnsafeMotion):
                svc.recover_fault("not confirmed")
            recovered = svc.recover_fault("已检查机械臂静止且运动区域安全")
            self.assertTrue(recovered["recovered"])
            self.assertIsNone(svc.fault)
            self.assertFalse(svc.health()["armed"])
            self.assertEqual(len(svc.arm.commands), 1)  # Recovery never moves arm.
            with self.assertRaises(UnsafeMotion):
                svc.step(1, 5)
            svc.unlock_once("已检查设备并清空运动区域")
            svc.step(1, 5)
            self.assertEqual(len(svc.arm.commands), 2)

    def test_recovery_refuses_motion_detected_during_samples(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}), \
             patch("control_ui.service.time.sleep", return_value=None):
            svc = self.make()
            svc.fault = {"code": "motion_not_confirmed", "message": "test"}
            svc.arm.next_status_delta = 2
            with self.assertRaises(UnsafeMotion):
                svc.recover_fault("已检查机械臂静止且运动区域安全")
            self.assertIsNotNone(svc.fault)
            self.assertEqual(svc.arm.commands, [])

    def test_disconnect_does_not_erase_unresolved_fault(self):
        svc = ArmService()
        svc.fault = {"code": "motion_not_confirmed", "message": "需要检查"}
        svc.disconnect()
        self.assertEqual(svc.health()["fault"]["code"], "motion_not_confirmed")

    def test_missing_serial_feedback_latches_fault(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}), \
             patch("control_ui.service.MIN_GAP_SECONDS", 0.0):
            svc = self.make()
            svc.step(1, 5)
            def fail(*args, **kwargs):
                raise TimeoutError("Serial timed out")
            svc.arm.status = fail
            with self.assertRaises(TimeoutError):
                svc.status()
            self.assertEqual(svc.fault["code"], "feedback_timeout")
            self.assertIsNone(svc.pending)
            self.assertEqual(len(svc.arm.commands), 1)


if __name__ == "__main__":
    unittest.main()
