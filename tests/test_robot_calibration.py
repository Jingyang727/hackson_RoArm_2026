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
        token = cal.start(SIZE, [1920, 1080])["session_id"]
        with tempfile.TemporaryDirectory() as folder:
            with patch("control_ui.robot_calibration.default_path", return_value=Path(folder)/"xy.json"):
                for n, x, y in TARGETS:
                    fake.xyz = [*mapped_xy(x, y), 85]
                    result = cal.capture(token, n, x, y, fake, sleep_fn=lambda _: None)
                self.assertEqual(len(result["records"]), 5)
                self.assertTrue(result["ready"])
                saved = cal.save(token)
                self.assertTrue(saved["saved"])
                self.assertTrue(json.loads(Path(saved["path"]).read_text())["fit"]["validated"])
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
