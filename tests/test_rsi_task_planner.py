"""Scene-derived plans without any task-position teaching or hardware."""
import unittest

from evoarm_rsi.task_planner import plan_from_world


class DynamicPlannerTests(unittest.TestCase):
    def setUp(self):
        self.world = {"setup_id": "layout-v2", "calibration_id": "cal-v2",
            "host_monotonic_ns": 1_000_000_000, "gripper_empty": True,
            "current_tcp_robot_mm": [200, 0, 160],
            "current_gripper_rad": 3.,
            "ball": {"stable": True, "center_robot_mm": [250, 0, 15], "radius_mm": 15},
            "funnel": {"stable": True, "mouth_center_robot_mm": [300, 80, 80],
                       "mouth_radius_mm": 50, "measurement": "calibrated_mouth_center"}}
        self.geometry = {"setup_id": "layout-v2", "calibration_id": "cal-v2",
            "frame": "robot_base_tcp", "verified": True, "table_z_mm": 0,
            "workspace_xy_mm": [[100, -150], [450, -150], [450, 200], [100, 200]],
            "tcp_z_range_mm": [10, 300], "tool_radius_mm": 20,
            "max_scene_age_s": 2,
            "release_clearance_mm": 15, "transit_clearance_mm": 30,
            "gripper_open_rad": 1.5, "gripper_close_rad": 2.8,
            "pitch_rad": 0, "roll_rad": 0}

    def plan(self):
        return plan_from_world(self.world, self.geometry, now_ns=1_500_000_000)

    def test_lift_preserves_the_measured_grip_before_opening_at_clearance(self):
        plan = self.plan()
        self.assertEqual(plan["skills"][0]["gripper_rad"], 3.)
        self.assertEqual(plan["skills"][1]["stage"], "open_before_approach")
        self.assertEqual(plan["skills"][1]["gripper_rad"], 1.5)
        del self.world["current_gripper_rad"]
        with self.assertRaisesRegex(ValueError,"current gripper"):
            self.plan()

    def test_ball_and_funnel_targets_follow_current_scene_not_taught_poses(self):
        first = self.plan()
        self.assertFalse(first["taught_positions_used"])
        self.assertFalse(first["motion_authorized"])
        self.assertFalse(first["execution_eligible"])
        self.assertIn("release", [s["stage"] for s in first["skills"]])
        self.world["ball"]["center_robot_mm"] = [210, 20, 15]
        self.world["funnel"]["mouth_center_robot_mm"] = [350, 100, 90]
        second = self.plan()
        self.assertNotEqual(first["ball_target_mm"], second["ball_target_mm"])
        self.assertNotEqual(first["funnel_target_mm"], second["funnel_target_mm"])
        self.assertEqual(second["skills"][-1]["tcp_xyz_mm"][:2], [350, 100])
        self.assertFalse(second["timing"]["verified_within_budget"])

    def test_layout_change_or_stale_scene_cannot_reuse_targets(self):
        self.world["setup_id"] = "old-layout"
        with self.assertRaisesRegex(ValueError, "versions"):
            self.plan()
        self.world["setup_id"] = "layout-v2"
        self.world["host_monotonic_ns"] = -2_000_000_000
        with self.assertRaisesRegex(ValueError, "fresh"):
            self.plan()

    def test_replanning_from_retreat_does_not_ratchet_travel_height_up(self):
        first = self.plan()
        self.world["current_tcp_robot_mm"] = first["skills"][-1]["tcp_xyz_mm"]
        second = self.plan()
        self.assertEqual(first["skills"][-1]["tcp_xyz_mm"][2],
                         second["skills"][-1]["tcp_xyz_mm"][2])

    def test_funnel_interior_is_not_treated_as_release_plane(self):
        self.world["funnel"]["measurement"] = "interior_depth"
        with self.assertRaisesRegex(ValueError, "interior"):
            self.plan()

    def test_missing_calibration_and_out_of_workspace_targets_rejected(self):
        self.geometry["verified"] = False
        with self.assertRaisesRegex(ValueError, "verified"):
            self.plan()
        self.geometry["verified"] = True
        self.world["funnel"]["mouth_center_robot_mm"] = [800, 0, 80]
        with self.assertRaisesRegex(ValueError, "workspace"):
            self.plan()

    def test_clearances_and_aperture_size_are_checked(self):
        self.geometry["tcp_z_range_mm"][1] = 100
        with self.assertRaisesRegex(ValueError, "height"):
            self.plan()
        self.geometry["tcp_z_range_mm"][1] = 300
        self.world["funnel"]["mouth_radius_mm"] = 10
        with self.assertRaisesRegex(ValueError, "fit"):
            self.plan()


if __name__ == "__main__":
    unittest.main()
