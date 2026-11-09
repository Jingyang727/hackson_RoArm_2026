"""Service tests with mocked serial arm (no hardware or movement)."""
import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from control_ui.service import ArmService
from control_ui.safety import UnsafeMotion


class FakeArm:
    def __init__(self):
        self.commands = []
        self.feedback = {
            "T": 1051, "b": 0.0, "s": 0.0, "e": math.pi / 2,
            "t": 0.0, "r": 0.0, "g": math.radians(175),
            **{f"torswitch{k}": 1 for k in "BSETRG"},
        }

    def status(self, timeout=None):
        return self.feedback.copy()

    def send(self, payload):
        self.commands.append(payload.copy())


class ServiceSafetyTests(unittest.TestCase):
    def test_readonly_is_default(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "0"}):
            service = ArmService()
            service.arm = FakeArm()
            self.assertFalse(service.health()["motion_enabled"])
            with self.assertRaises(UnsafeMotion):
                service.unlock_once("已检查设备并清空运动区域")

    def test_six_joints_available_after_explicit_opt_in(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "1,2,3,4,5,6"}):
            service = ArmService()
            service.arm = FakeArm()
            self.assertEqual(service.health()["allowed_joints"], [1, 2, 3, 4, 5, 6])
            service.unlock_once("已检查设备并清空运动区域")
            result = service.step(1, 5)
            self.assertEqual(result["to_deg"], 5.0)
            self.assertEqual(service.arm.commands[0]["T"], 121)
            self.assertEqual(service.arm.commands[0]["spd"], 15)
            self.assertIsNotNone(service.health()["pending"])
            with self.assertRaises(UnsafeMotion):
                service.step(2, 5)

    def test_joint_allowlist(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "6"}):
            service = ArmService()
            service.arm = FakeArm()
            service.unlock_once("已检查设备并清空运动区域")
            with self.assertRaises(UnsafeMotion):
                service.step(1, 1)
            self.assertEqual(service.arm.commands, [])

    def test_invalid_allowlist_fails_closed(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "invalid"}):
            service = ArmService()
            service.arm = FakeArm()
            service.unlock_once("已检查设备并清空运动区域")
            self.assertEqual(service.health()["allowed_joints"], [])
            with self.assertRaises(UnsafeMotion):
                service.step(6, -1)

    def test_disarm_blocks_new_moves(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}):
            service = ArmService()
            service.arm = FakeArm()
            service.unlock_once("已检查设备并清空运动区域")
            service.disarm()
            with self.assertRaises(UnsafeMotion):
                service.step(1, 3)

    def test_gripper_api_recovers_from_180_point_9_without_overclosing(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "6"}):
            service = ArmService()
            service.arm = FakeArm()
            service.arm.feedback["g"] = math.radians(180.9)
            service.unlock_once("已检查设备并清空运动区域")
            result = service.gripper_step(-15)
            self.assertAlmostEqual(result["to_deg"], 165.9, places=1)
            self.assertEqual(service.arm.commands[0]["T"], 106)
            self.assertIn("cmd", service.arm.commands[0])
            self.assertAlmostEqual(service.arm.commands[0]["cmd"], math.radians(165.9), places=4)
            self.assertEqual(len(service.arm.commands), 1)

    def test_gripper_api_refuses_further_overclose(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1", "EVOARM_ALLOWED_JOINTS": "6"}):
            service = ArmService()
            service.arm = FakeArm()
            service.arm.feedback["g"] = math.radians(180.9)
            service.unlock_once("已检查设备并清空运动区域")
            with self.assertRaises(UnsafeMotion):
                service.gripper_step(15)
            self.assertEqual(service.arm.commands, [])

    def test_official_factory_reset_sends_T100_and_waits_for_feedback(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1",
                                      "EVOARM_ALLOWED_JOINTS": "1,2,3,4,5,6"}):
            service = ArmService()
            service.arm = FakeArm()
            service.arm.feedback["b"] = math.radians(30)
            service.unlock_once("已检查设备并清空运动区域")
            response = service.factory_reset()
            self.assertEqual(response["status"], "sent_not_verified")
            self.assertEqual(service.arm.commands, [{"T": 100}])
            self.assertEqual(service.health()["pending"]["kind"], "factory")
            with self.assertRaises(UnsafeMotion):
                service.step(1, 1)
            service.arm.feedback.update({
                "b": 0.0, "s": 0.0, "e": math.pi / 2,
                "t": 0.0, "r": 0.0, "g": math.pi,
            })
            service.status()
            self.assertIsNone(service.health()["pending"])

    def test_official_reset_requires_all_joints_authorized(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1",
                                      "EVOARM_ALLOWED_JOINTS": "6"}):
            service = ArmService()
            service.arm = FakeArm()
            service.unlock_once("已检查设备并清空运动区域")
            with self.assertRaises(UnsafeMotion):
                service.factory_reset()
            self.assertEqual(service.arm.commands, [])

    def test_official_reset_refused_when_not_unlocked(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1",
                                      "EVOARM_ALLOWED_JOINTS": "1,2,3,4,5,6"}):
            service = ArmService()
            service.arm = FakeArm()
            with self.assertRaises(UnsafeMotion):
                service.factory_reset()
            self.assertEqual(service.arm.commands, [])

    def test_save_and_recall_user_home_in_single_bounded_step(self):
        with tempfile.TemporaryDirectory() as td:
            with patch("control_ui.home_pose.pose_file", return_value=Path(td) / "home.json"):
                with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}):
                    service = ArmService()
                    service.arm = FakeArm()
                    service.home_save()
                    service.arm.feedback["b"] = math.radians(30)
                    preview = service.home_info("saved")
                    first = preview["rows"][0]
                    self.assertAlmostEqual(first["difference_deg"], -30.0, places=2)
                    service.unlock_once("已检查设备并清空运动区域")
                    result = service.home_step("saved", 1)
                    self.assertAlmostEqual(result["to_deg"], 25.0, places=2)
                    self.assertEqual(service.arm.commands[0]["joint"], 1)
                    self.assertEqual(service.arm.commands[0]["spd"], 15)


if __name__ == "__main__":
    unittest.main()
