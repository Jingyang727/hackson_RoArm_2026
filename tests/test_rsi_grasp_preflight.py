"""Hardware-free checks for real grasp preflight, no motor commands."""
import unittest

from evoarm_rsi.grasp_preflight import inspect_grasp_preflight

CAL={"fit":{"validated":True,"validation_error_mm":3.94,"z_calibrated":False,"tcp_calibrated":False}}
POSES={"poses":{
    "home":{"xyz_mm":[360.339,-4.975,182.572]},
    "hover":{"xyz_mm":[362.171,-1.111,-34.895]},
    "grasp":{"xyz_mm":[361.05,-1.108,-86.787]},
    "place":{"xyz_mm":[266.348,263.099,-32.417]},
    "gripper_open":{"gripper_rad":2.587826},
    "gripper_close":{"gripper_rad":2.805651},
}}
Q={
    "trial_id":"real_trial",
    "read_only":True,
    "motion_authorized":False,
    "stable_ball":True,
    "robot_xy_preview_mm":[360.676,-11.034],
    "ball_board_xy_mm":[101.2,45.89],
    "robot_target_xyz_mm":None,
}


class GraspPreflightTests(unittest.TestCase):
    def test_taught_grasp_offset_is_reported_but_no_motion_granted(self):
        r=inspect_grasp_preflight(Q,CAL,POSES)
        self.assertAlmostEqual(r["xy_comparisons"]["grasp_to_ball_xy_mm"],9.933,delta=.01)
        self.assertFalse(r["motion_authorized"])
        self.assertIsNone(r["robot_target_xyz_mm"])
        self.assertFalse(r["readiness"]["ball_center_height_and_tcp_verified"])
        self.assertFalse(r["readiness"]["robot_motion_allowed"])

    def test_rejects_unstable_target(self):
        with self.assertRaisesRegex(ValueError,"no stable"):
            inspect_grasp_preflight({**Q,"stable_ball":False},CAL,POSES)

    def test_rejects_fabricated_motion_target(self):
        with self.assertRaisesRegex(ValueError,"unexpected robot motion target"):
            inspect_grasp_preflight({**Q,"robot_target_xyz_mm":[1,2,3]},CAL,POSES)

    def test_rejects_unvalidated_xy_mapping(self):
        with self.assertRaisesRegex(ValueError,"not validated"):
            inspect_grasp_preflight(Q,{"fit":{"validated":False}},POSES)

    def test_rejects_unexpected_gripper_direction(self):
        changed={"poses":{**POSES["poses"],"gripper_open":{"gripper_rad":3.1}}}
        with self.assertRaisesRegex(ValueError,"gripper direction"):
            inspect_grasp_preflight(Q,CAL,changed)


if __name__=="__main__":
    unittest.main()
