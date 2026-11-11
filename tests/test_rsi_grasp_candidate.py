"""Hardware-free v2 Gemini grasp-and-lift attempt planning tests."""
import copy
import unittest
from evoarm_rsi.grasp_candidate import build_grasp_candidate,GraspCandidateError

Q={
    "trial_id":"real","read_only":True,"motion_authorized":False,
    "stable_ball":True,"latest_observation_eligible":True,
    "ball_board_xy_mm":[101.2,45.89],
    "robot_xy_preview_mm":[360.68,-11.03],
}
POSES={"poses":{
    "home":{"xyz_mm":[360.339,-4.975,182.572]},
    "hover":{"xyz_mm":[362.171,-1.111,-34.895],"pitch_rad":0.31,"roll_rad":0.02},
    "grasp":{"xyz_mm":[361.05,-1.108,-86.787],"pitch_rad":0.31,"roll_rad":0.02},
    "place":{"xyz_mm":[266.348,263.099,-32.417]},
    "gripper_open":{"gripper_rad":2.587826},
    "gripper_close":{"gripper_rad":2.805651},
}}


class GraspCandidateTests(unittest.TestCase):
    def test_generates_a_real_grasp_lift_sequence_not_just_j1(self):
        plan=build_grasp_candidate(Q,POSES)
        names=[row["stage"] for row in plan["waypoints"]]
        self.assertEqual(names[0],"hover_ball")
        self.assertIn("close_ball",names)
        self.assertIn("lift_ball",names)
        self.assertGreater(len(names),7)
        self.assertEqual(plan["ball_robot_xy_mm"],[360.68,-11.03])
        self.assertLessEqual(plan["taught_grasp_xy_gap_mm"],11)
        self.assertIs(plan["motion_authorized"],False)
        self.assertIs(plan["auto_execute"],False)
        previous=None
        for row in plan["waypoints"]:
            if row["kind"]!="cartesian":continue
            z=row["xyz_mm"][2]
            if previous is not None:
                self.assertLessEqual(abs(z-previous),20.01)
            previous=z

    def test_rejects_stale_ball_and_incomplete_teaching(self):
        with self.assertRaisesRegex(GraspCandidateError,"stable"):
            build_grasp_candidate({**Q,"latest_observation_eligible":False},POSES)
        with self.assertRaisesRegex(GraspCandidateError,"six measured"):
            build_grasp_candidate(Q,{"poses":{"grasp":POSES["poses"]["grasp"]}})

    def test_rejects_out_of_range_xy_and_unsafe_height(self):
        with self.assertRaisesRegex(GraspCandidateError,"20mm"):
            build_grasp_candidate({**Q,"robot_xy_preview_mm":[400,-90]},POSES)
        shifted=copy.deepcopy(POSES)
        shifted["poses"]["hover"]["xyz_mm"][2]=-85
        with self.assertRaisesRegex(GraspCandidateError,"vertical"):
            build_grasp_candidate(Q,shifted)


if __name__=="__main__":
    unittest.main()
