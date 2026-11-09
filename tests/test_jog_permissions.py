"""Regression tests for explicit manual-motion permissions (no robot hardware)."""
import math
import os
import unittest
from unittest.mock import patch

from control_ui.service import ArmService
from control_ui.safety import UnsafeMotion


class FakeArm:
    def __init__(self):
        self.commands = []

    def status(self):
        return {"T": 1051, "b": 0.0, "s": 0.0, "e": math.pi / 2,
                "t": 0.0, "r": 0.0, "g": math.radians(175),
                "torswitchG": 1, "torswitchB": 1}

    def send(self, payload):
        self.commands.append(payload)


class MotionPermissionTests(unittest.TestCase):
    def test_readonly_by_default(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "0"}, clear=False):
            app = ArmService()
            app.arm = FakeArm()
            self.assertFalse(app.health()["motion_enabled"])
            with self.assertRaises(UnsafeMotion):
                app.unlock_once("已检查设备并清空运动区域")

    def test_only_gripper_is_allowed_initially(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "6"}):
            app = ArmService()
            app.arm = FakeArm()
            self.assertEqual(app.health()["allowed_joints"], [6])
            app.unlock_once("已检查设备并清空运动区域")
            with self.assertRaises(UnsafeMotion):
                app.step(1, 1)
            self.assertEqual(app.arm.commands, [])

    def test_one_gripper_step_consumes_authorization(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "6"}):
            app = ArmService()
            app.arm = FakeArm()
            app.unlock_once("已检查设备并清空运动区域")
            result = app.step(6, -1)
            self.assertEqual(result["joint"], 6)
            self.assertEqual(app.arm.commands[0]["T"], 121)
            self.assertEqual(app.arm.commands[0]["joint"], 6)
            self.assertEqual(app.arm.commands[0]["spd"], 3)
            with self.assertRaises(UnsafeMotion):
                app.step(6, -1)

    def test_invalid_allowlist_fails_closed(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "bogus"}):
            app = ArmService()
            app.arm = FakeArm()
            self.assertEqual(app.health()["allowed_joints"], [])
            app.unlock_once("已检查设备并清空运动区域")
            with self.assertRaises(UnsafeMotion):
                app.step(6, -1)

    def test_decline_authorization(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "6"}):
            app = ArmService()
            app.arm = FakeArm()
            with self.assertRaises(UnsafeMotion):
                app.unlock_once("不确认")


if __name__ == "__main__":
    unittest.main()
