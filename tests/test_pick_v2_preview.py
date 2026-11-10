"""V2 taught-pose pick preview: no serial ports or motor commands required."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from control_ui.pick_plan import (
    PickPlanningError, build_from_saved_files, preview_v2_poses,
)


class PickV2PreviewTests(unittest.TestCase):
    def setUp(self):
        self.corners = [
            [float(100 + c * 25), float(80 + r * 25)]
            for r in range(6) for c in range(9)
        ]
        self.cal = {
            "fit": {
                "validated": True,
                "matrix_2x2": [[1., 0.], [0., 1.]],
                "offset_mm": [300., -50.],
            },
            "square_mm": 24.,
            "image_size": [640, 480],
            "corners_px": self.corners,
            "records": [
                {"id": "P1", "board_mm": [0, 0]},
                {"id": "P2", "board_mm": [192, 0]},
                {"id": "P3", "board_mm": [192, 120]},
                {"id": "P4", "board_mm": [0, 120]},
            ],
        }
        self.poses = {
            "home": {"xyz_mm": [360.339, -4.975, 182.572],
                     "pitch_rad": 0.116583, "roll_rad": 0.003068},
            "hover": {"xyz_mm": [362.171, -1.111, -34.895],
                      "pitch_rad": 0.753185, "roll_rad": 0.003068},
            "grasp": {"xyz_mm": [361.05, -1.108, -86.787],
                      "pitch_rad": 0.880505, "roll_rad": 0.003068},
            "place": {"xyz_mm": [266.348, 263.099, -32.417],
                      "pitch_rad": 0.724039, "roll_rad": 0.003068},
            "gripper_open": {"gripper_rad": 2.587826},
            "gripper_close": {"gripper_rad": 2.805651},
        }

    def test_v2_target_uses_camera_xy_and_taught_z_without_motor_approval(self):
        plan = preview_v2_poses(
            [96, 48], self.cal, self.poses,
            image_size=[640, 480], corners_px=self.corners,
        )
        self.assertEqual(plan["source"], "pick_poses_v2.json")
        self.assertEqual(plan["ball_robot_xy_mm"], [396., -2.])
        self.assertEqual(plan["place_robot_xy_mm"], [266.35, 263.1])
        self.assertFalse(plan["execution_eligible"])
        self.assertTrue(plan["preview_only"])
        self.assertTrue(all(w["preview_only"] for w in plan["waypoints"]))
        hover = next(w for w in plan["waypoints"]
                     if w["stage"] == "hover_ball_candidate")
        grasp = next(w for w in plan["waypoints"]
                     if w["stage"] == "grasp_ball_candidate")
        self.assertEqual(hover["xyz_mm"], [396., -2., -34.9])
        self.assertEqual(grasp["xyz_mm"], [396., -2., -86.79])
        self.assertAlmostEqual(grasp["t"], 0.88051, places=5)
        self.assertTrue(any("转运净空" in w for w in plan["reachability_warnings"]))

    def test_v2_preview_moves_xy_with_ball_but_not_z(self):
        a = preview_v2_poses([96, 48], self.cal, self.poses,
                             image_size=[640, 480], corners_px=self.corners)
        b = preview_v2_poses([120, 72], self.cal, self.poses,
                             image_size=[640, 480], corners_px=self.corners)
        self.assertEqual(b["ball_robot_xy_mm"][0] - a["ball_robot_xy_mm"][0], 24.)
        self.assertEqual(b["ball_robot_xy_mm"][1] - a["ball_robot_xy_mm"][1], 24.)
        self.assertEqual(a["waypoints"][1]["xyz_mm"][2],
                         b["waypoints"][1]["xyz_mm"][2])

    def test_v2_missing_stage_does_not_use_stale_profile(self):
        data = dict(self.poses)
        del data["grasp"]
        with self.assertRaisesRegex(PickPlanningError, "grasp"):
            preview_v2_poses([96, 48], self.cal, data,
                             image_size=[640, 480], corners_px=self.corners)

    def test_saved_v2_never_falls_back_to_legacy_motor_planner(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            pose_file = root / "pick_poses_v2.json"
            pose_file.write_text('{"version": 2}', encoding="utf-8")
            with (
                patch("control_ui.robot_calibration.session_path",
                      return_value=root / "absent-session"),
                patch("control_ui.fast_teach.pose_path", return_value=pose_file),
                patch("control_ui.fast_teach.read_poses",
                      return_value={"version": 2, "poses": self.poses}),
                patch("control_ui.pick_plan._read_file", return_value=self.cal),
            ):
                read_only = build_from_saved_files(
                    [96, 48], image_size=[640, 480],
                    corners_px=self.corners, preview_only=True,
                )
                self.assertEqual(read_only["source"], "pick_poses_v2.json")
                with self.assertRaisesRegex(PickPlanningError, "拒绝运动"):
                    build_from_saved_files(
                        [96, 48], image_size=[640, 480],
                        corners_px=self.corners, preview_only=False,
                    )


if __name__ == "__main__":
    unittest.main()
