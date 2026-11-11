"""False-yellow rejection and robot XY preview, no camera or motor."""
import unittest

from evoarm_rsi.qualify import qualify_trial


CAL = {
    "square_mm": 24,
    "fit": {
        "validated": True,
        "validation_error_mm": 3.94,
        "matrix_2x2": [[0, -1], [-1, 0]],
        "offset_mm": [408, 90],
    },
}


class QualifyTests(unittest.TestCase):
    def test_rejects_yellow_outside_board_then_qualifies_last_stable_ball(self):
        observations = []
        for i in range(7):
            observations.append({
                "rgb_frame_index": i,
                "ball_board_xy_mm": [348, 597],
                "ball_center_px": [1154, 129],
                "ball_surface_depth_mm": 560,
                "apparent_depth_difference_mm": 2,
            })
        for i in range(12):
            observations.append({
                "rgb_frame_index": 70+i,
                "ball_board_xy_mm": [101.2+(i%3)*0.1, 45.9],
                "ball_center_px": [522.9, 342.2],
                "ball_surface_depth_mm": 533,
                "apparent_depth_difference_mm": 32,
            })
        result = qualify_trial(
            {"read_only": True, "motion_authorized": False,
             "trial_id": "x", "observations": observations},
            CAL,
            {"fx": 609.18, "fy": 609.08, "cx": 638.26, "cy": 361.9,
             "aligned_to": "color", "verified": False},
        )
        self.assertEqual(result["eligible_inside_board_with_depth"], 12)
        self.assertEqual(result["rejected_counts"]["outside_inner_corner_board"], 7)
        self.assertTrue(result["stable_ball"])
        self.assertAlmostEqual(result["robot_xy_preview_mm"][0], 362.1,delta=0.3)
        self.assertFalse(result["motion_authorized"])
        self.assertIsNone(result["robot_target_xyz_mm"])
        self.assertFalse(result["intrinsics_verified"])

    def test_no_valid_board_ball_does_not_invent_robot_target(self):
        observations = [{
            "ball_board_xy_mm": [400, 600],
            "ball_surface_depth_mm": 533,
            "apparent_depth_difference_mm": 40,
        }]
        result = qualify_trial({
            "read_only": True, "motion_authorized": False,
            "observations": observations,
        }, CAL)
        self.assertFalse(result["stable_ball"])
        self.assertIsNone(result["robot_xy_preview_mm"])

    def test_rejects_unvalidated_planar_calibration(self):
        with self.assertRaisesRegex(ValueError, "validated"):
            qualify_trial(
                {"read_only": True, "motion_authorized": False, "observations": []},
                {"fit": {"validated": False}},
            )

    def test_rejects_motion_authorized_input(self):
        with self.assertRaisesRegex(ValueError, "read-only"):
            qualify_trial(
                {"read_only": False, "motion_authorized": True, "observations": []},
                CAL,
            )


if __name__ == "__main__":
    unittest.main()
