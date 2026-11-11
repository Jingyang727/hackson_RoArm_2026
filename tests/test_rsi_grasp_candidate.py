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
    "home":{"xyz_mm":[360.339,-4.975,182.572],"pitch_rad":0.31,"roll_rad":0.02},
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

    def test_live_home_generates_bounded_high_then_vertical_transition(self):
        start={"x":360.339,"y":-4.975,"z":182.572,
               "tit":0.31,"r":0.02}
        plan=build_grasp_candidate(Q,POSES,start_status=start)
        self.assertEqual(plan["start_mode"],"live_home_transit")
        self.assertGreater(plan["home_transition_stages"],10)
        self.assertEqual(plan["waypoints"][0]["stage"],"home_high_align")
        self.assertAlmostEqual(plan["waypoints"][0]["xyz_mm"][2],182.572,places=2)
        self.assertIn("hover_ball",[x["stage"] for x in plan["waypoints"]])
        zs=[x["xyz_mm"][2] for x in plan["waypoints"]
            if x["kind"]=="cartesian"]
        self.assertLessEqual(max(abs(b-a) for a,b in zip(zs,zs[1:])),20.01)
        self.assertFalse(plan["motion_authorized"])

    def test_not_at_taught_home_or_far_xy_does_not_move(self):
        start={"x":360.339,"y":-4.975,"z":182.572,
               "tit":0.31,"r":0.02}
        with self.assertRaisesRegex(GraspCandidateError,"does not match taught Home"):
            build_grasp_candidate(Q,POSES,
                                  start_status={**start,"z":100.0})
        with self.assertRaisesRegex(GraspCandidateError,"local corridor"):
            build_grasp_candidate({**Q,"robot_xy_preview_mm":[380,-30]},
                                  POSES,start_status=start,
                                  max_xy_shift_mm=50)

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
