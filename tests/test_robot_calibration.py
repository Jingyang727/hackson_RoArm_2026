"""Read-only robot XY calibration and opt-in torque teaching: mocked hardware tests."""
import json
import math
import os
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from control_ui.robot_calibration import RobotCalibration, CalibrationError, fit_board_to_robot
from control_ui.service import ArmService
from control_ui.safety import UnsafeMotion

SIZE = 24.0
TARGETS = [("P1", 1, 1), ("P2", 7, 1), ("P3", 1, 4), ("P4", 7, 4), ("V1", 4, 2)]


def mapped_xy(col, row):
    board = np.array([col * SIZE, row * SIZE], dtype=float)
    return (np.array([[0, -1], [-1, 0]]) @ board + [310, 75]).tolist()


class FakeArm:
    def __init__(self):
        self.lock = threading.RLock()
        self.xyz = [300., 50., 85.]
        self.commands = []
        self.torque = 1
        self.connected = True
        self.pending = None
        self.fault = None

    def health(self):
        return {"connected": self.connected, "pending": self.pending, "fault": self.fault}

    def status(self, timeout=None):
        return {"T": 1051, **dict(zip("xyz", self.xyz)), "tit": 0., "b": 0., "s": 0.,
                "e": math.pi / 2, "t": 0., "r": 0., "g": math.pi,
                **{"torswitch" + k: self.torque for k in "BSETRG"}}

    def send(self, command):
        self.commands.append(command)
        if command.get("T") == 210:
            self.torque = command["cmd"]


class XYMathTests(unittest.TestCase):
    def setUp(self):
        # Never touch the user's actual ~/.evoarm/ files in automated tests.
        folder=tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        output=Path(folder.name)
        one=patch("control_ui.robot_calibration.session_path",return_value=output/"session.json")
        two=patch("control_ui.robot_calibration.default_path",return_value=output/"xy.json")
        one.start();two.start()
        self.addCleanup(one.stop);self.addCleanup(two.stop)

    def records(self):
        return [{"id": n, "board_mm": [x * SIZE, y * SIZE],
                 "robot_xyz_mm": [*mapped_xy(x, y), 85]} for n, x, y in TARGETS]

    def test_four_point_rigid_fit_with_reflection_and_holdout(self):
        result = fit_board_to_robot(self.records())
        self.assertTrue(result["validated"])
        self.assertAlmostEqual(result["train_rms_mm"], 0)
        self.assertAlmostEqual(result["validation_error_mm"], 0)
        self.assertEqual(result["determinant"], -1)
        self.assertFalse(result["auto_move_enabled"])
        self.assertFalse(result["tcp_calibrated"])

    def test_independent_validation_detects_large_error(self):
        rec = self.records()
        rec[-1]["robot_xyz_mm"][0] += 30
        result = fit_board_to_robot(rec)
        self.assertFalse(result["validated"])
        self.assertGreater(result["validation_error_mm"], 10)

    def test_rejects_collinear_points(self):
        records = [{"id": "P"+str(i+1), "board_mm": [i*SIZE, 0],
                    "robot_xyz_mm": [i*SIZE, 0, 70]} for i in range(4)]
        with self.assertRaises(CalibrationError):
            fit_board_to_robot(records)

    def test_capture_five_points_and_save_no_actuation(self):
        cal, fake = RobotCalibration(), FakeArm()
        corners=[[100+c*40,100+r*40] for r in range(6) for c in range(9)]
        token = cal.start(SIZE, [1920, 1080],corners)["session_id"]
        with tempfile.TemporaryDirectory() as folder:
            with patch("control_ui.robot_calibration.default_path", return_value=Path(folder)/"xy.json"):
                for n, x, y in TARGETS:
                    fake.xyz = [*mapped_xy(x, y), 85]
                    result = cal.capture(token, n, x, y, fake, sleep_fn=lambda _: None)
                self.assertEqual(len(result["records"]), 5)
                self.assertTrue(result["ready"])
                saved = cal.save(token)
                self.assertTrue(saved["saved"])
                saved_data=json.loads(Path(saved["path"]).read_text())
                self.assertTrue(saved_data["fit"]["validated"])
                self.assertEqual(len(saved_data["corners_px"]),54)
                self.assertEqual(fake.commands, [])

    def test_capture_moving_or_duplicate_refused(self):
        cal, fake = RobotCalibration(), FakeArm()
        token = cal.start(SIZE, [1920, 1080])["session_id"]
        fake.pending = {"joint": 1}
        with self.assertRaises(CalibrationError):
            cal.capture(token, "P1", 1, 1, fake, sleep_fn=lambda _: None)
        fake.pending = None
        cal.capture(token, "P1", 1, 1, fake, sleep_fn=lambda _: None)
        with self.assertRaises(CalibrationError):
            cal.capture(token, "P2", 1, 1, fake, sleep_fn=lambda _: None)

    def test_rejects_misaligned_wrist_pose(self):
        cal,fake=RobotCalibration(),FakeArm()
        token=cal.start(SIZE,[1920,1080])["session_id"]
        fake.xyz=[*mapped_xy(1,1),85]
        cal.capture(token,"P1",1,1,fake,sleep_fn=lambda _:None)
        base=fake.status
        def changed(timeout=None):
            data=base(timeout);data["tit"]=math.radians(15);return data
        fake.status=changed
        fake.xyz=[*mapped_xy(7,1),85]
        with self.assertRaises(CalibrationError):
            cal.capture(token,"P2",7,1,fake,sleep_fn=lambda _:None)

    def test_session_recovers_after_restart_without_losing_p1_p2(self):
        cal, arm=RobotCalibration(),FakeArm()
        corners=[[100+c*40,100+r*40] for r in range(6) for c in range(9)]
        token=cal.start(SIZE,[1920,1080],corners)["session_id"]
        for name,col,row in TARGETS[:2]:
            arm.xyz=[*mapped_xy(col,row),85]
            cal.capture(token,name,col,row,arm,sleep_fn=lambda _:None)
        restored=RobotCalibration()
        state=restored.summary()
        self.assertEqual(state["session_id"],token)
        self.assertEqual([r["id"] for r in state["records"]],["P1","P2"])
        self.assertFalse(state["saved"])
        self.assertTrue(state["corners_saved"])
        self.assertTrue(Path(state["restored_from"]).exists())
        self.assertEqual(arm.commands,[])

    def test_fifth_point_autosaves_and_restarts_with_validated_final(self):
        cal, arm=RobotCalibration(),FakeArm()
        corners=[[100+c*40,100+r*40] for r in range(6) for c in range(9)]
        token=cal.start(SIZE,[1920,1080],corners)["session_id"]
        for name,col,row in TARGETS:
            arm.xyz=[*mapped_xy(col,row),85]
            result=cal.capture(token,name,col,row,arm,sleep_fn=lambda _:None)
        self.assertTrue(result["ready"])
        self.assertTrue(result["saved"])
        output=Path(result["saved_path"])
        self.assertTrue(output.exists())
        self.assertEqual(output.stat().st_mode & 0o777,0o600)
        recovered=RobotCalibration().summary()
        self.assertTrue(recovered["ready"])
        self.assertTrue(recovered["saved"])
        self.assertEqual(len(recovered["records"]),5)
        self.assertEqual(len(json.loads(output.read_text())["corners_px"]),54)
        self.assertEqual(arm.commands,[])

    def test_new_session_does_not_overwrite_previous_final_calibration(self):
        from control_ui.robot_calibration import default_path
        cal,arm=RobotCalibration(),FakeArm()
        corners=[[100+c*40,100+r*40] for r in range(6) for c in range(9)]
        token=cal.start(SIZE,[1920,1080],corners)["session_id"]
        for name,col,row in TARGETS:
            arm.xyz=[*mapped_xy(col,row),85]
            cal.capture(token,name,col,row,arm,sleep_fn=lambda _:None)
        old_bytes=default_path().read_bytes()
        cal.start(SIZE,[1920,1080],corners)
        self.assertFalse(cal.summary()["saved"])
        self.assertEqual(default_path().read_bytes(),old_bytes)

    def test_not_ready_without_validation(self):
        cal, fake = RobotCalibration(), FakeArm()
        token = cal.start(SIZE, [1920, 1080])["session_id"]
        for n, x, y in TARGETS[:4]:
            fake.xyz = [*mapped_xy(x, y), 85]
            cal.capture(token, n, x, y, fake, sleep_fn=lambda _: None)
        self.assertFalse(cal.summary()["ready"])
        with self.assertRaises(CalibrationError):
            cal.save(token)


class TeachModeTests(unittest.TestCase):
    def test_default_disabled(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_TEACH": "0"}):
            svc = ArmService()
            svc.arm = FakeArm()
            with self.assertRaises(UnsafeMotion):
                svc.set_torque(False, "已托住机械臂并确认释放全部关节")
            self.assertEqual(svc.arm.commands, [])

    def test_first_t105_timeout_then_success_no_second_t210(self):
        class TransientArm(FakeArm):
            def __init__(self):
                super().__init__()
                self.reads_after_command = 0
            def status(self, timeout=None):
                if self.commands:
                    self.reads_after_command += 1
                    if self.reads_after_command == 1:
                        raise RuntimeError("状态反馈 T=1051 超时")
                return super().status(timeout)

        with patch.dict(os.environ, {"EVOARM_ENABLE_TEACH": "1"}), \
             patch("control_ui.service.time.sleep", return_value=None):
            svc = ArmService()
            arm = TransientArm()
            svc.arm = arm
            result = svc.set_torque(False, "已托住机械臂并确认释放全部关节")
            self.assertTrue(result["verified"])
            self.assertEqual(result["verification_attempts"], 2)
            self.assertEqual(arm.commands, [{"T": 210, "cmd": 0}])
            self.assertEqual(svc.health()["torque_status"], [0] * 6)
            self.assertTrue(svc.health()["teach_active"])
            self.assertTrue(svc.health()["torque_last_result"]["verified"])

    def test_three_read_timeouts_not_a_false_failure_or_extra_send(self):
        class UnreadableArm(FakeArm):
            def status(self, timeout=None):
                if self.commands:
                    raise RuntimeError("状态反馈 T=1051 超时")
                return super().status(timeout)

        with patch.dict(os.environ, {"EVOARM_ENABLE_TEACH": "1", "EVOARM_ENABLE_MOTION": "1"}), \
             patch("control_ui.service.time.sleep", return_value=None):
            svc = ArmService()
            arm = UnreadableArm()
            svc.arm = arm
            result = svc.set_torque(False, "已托住机械臂并确认释放全部关节")
            self.assertFalse(result["verified"])
            self.assertIsNone(result["torque_enabled"])
            self.assertEqual(result["verification_attempts"], 3)
            self.assertEqual(arm.commands, [{"T": 210, "cmd": 0}])
            self.assertTrue(svc.teach_active)
            self.assertIsNone(svc.health()["torque_status"])
            with self.assertRaises(UnsafeMotion):
                svc.unlock_once("已检查设备并清空运动区域")

    def test_relock_after_unverified_release_no_double_send(self):
        class TimedReleaseArm(FakeArm):
            def __init__(self):
                super().__init__()
                self.reads_failing = True
            def status(self, timeout=None):
                if self.commands and self.reads_failing:
                    raise RuntimeError("状态反馈超时")
                return super().status(timeout)

        with patch.dict(os.environ, {"EVOARM_ENABLE_TEACH": "1"}), \
             patch("control_ui.service.time.sleep", return_value=None):
            svc = ArmService()
            arm = TimedReleaseArm()
            svc.arm = arm
            off = svc.set_torque(False, "已托住机械臂并确认释放全部关节")
            self.assertFalse(off["verified"])
            arm.reads_failing = False
            self.assertEqual(arm.status()["torswitchS"], 0)
            on = svc.set_torque(True, "已托住机械臂并确认重新锁定")
            self.assertTrue(on["verified"])
            self.assertEqual(arm.commands, [{"T": 210, "cmd": 0}, {"T": 210, "cmd": 1}])
            self.assertFalse(svc.teach_active)

    def test_already_locked_relock_is_read_only_and_clears_teach_lock(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_TEACH": "1"}):
            svc = ArmService()
            arm = FakeArm()
            svc.arm = arm
            svc.teach_active = True
            result = svc.set_torque(True, "已托住机械臂并确认重新锁定")
            self.assertTrue(result["verified"])
            self.assertFalse(result["command_sent"])
            self.assertEqual(arm.commands, [])
            self.assertFalse(svc.teach_active)

    def test_explicit_release_blocks_jog_and_factory_reset(self):
        with patch.dict(os.environ, {"EVOARM_ENABLE_TEACH": "1", "EVOARM_ENABLE_MOTION": "1"}):
            svc = ArmService()
            fake = FakeArm()
            svc.arm = fake
            released = svc.set_torque(False, "已托住机械臂并确认释放全部关节")
            self.assertTrue(released["verified"])
            self.assertEqual(fake.commands[-1], {"T": 210, "cmd": 0})
            self.assertTrue(svc.teach_active)
            with self.assertRaises(UnsafeMotion):
                svc.unlock_once("已检查设备并清空运动区域")
            with self.assertRaises(UnsafeMotion):
                svc.step(1, 1)
            with self.assertRaises(UnsafeMotion):
                svc.factory_reset()
            locked = svc.set_torque(True, "已托住机械臂并确认重新锁定")
            self.assertTrue(locked["verified"])
            self.assertEqual(fake.commands[-1], {"T": 210, "cmd": 1})
            self.assertFalse(svc.teach_active)
            self.assertFalse(svc.health()["armed"])


if __name__ == "__main__":
    unittest.main()
