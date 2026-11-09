"""Tests for local home pose storage; never connect to a real robot."""
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from control_ui.home_pose import save_pose, load_pose
from control_ui.safety import FACTORY_DEGREES, checked_home_pose, plan_home_step, UnsafeMotion


class HomePoseTests(unittest.TestCase):
    def test_save_and_restore_user_pose(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "home.json"
            with patch("control_ui.home_pose.pose_file", return_value=path):
                data = save_pose(FACTORY_DEGREES)
                self.assertTrue(path.exists())
                restored = load_pose()
                self.assertEqual(restored["degrees"][3], 90)
                self.assertEqual(restored["degrees"][6], 180)
                self.assertIn("created_at", data)

    def test_absent_file_means_no_home(self):
        with tempfile.TemporaryDirectory() as td:
            with patch("control_ui.home_pose.pose_file", return_value=Path(td) / "no-file"):
                self.assertIsNone(load_pose())

    def test_bad_file_is_refused(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "home.json"
            path.write_text("{bad json")
            with patch("control_ui.home_pose.pose_file", return_value=path):
                with self.assertRaises(UnsafeMotion):
                    load_pose()

    def test_cannot_save_pose_outside_training_range(self):
        status = {"T": 1051, "b": math.radians(150), "s": 0, "e": math.pi / 2,
                  "t": 0, "r": 0, "g": math.pi}
        with self.assertRaises(UnsafeMotion):
            checked_home_pose(status)


if __name__ == "__main__":
    unittest.main()
