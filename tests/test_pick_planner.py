"""Tests for software-only pick-and-place preview; no hardware movement."""
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from control_ui.pick_plan import (
    PickPlanningError, plan_pick_place, validate_pick_profile,
    save_pick_profile, build_from_saved_files,
)
from control_ui.pick_teach import observe_tool_pose


def corners():
    return [[100.0 + c*40,100.0 + r*40] for r in range(6) for c in range(9)]


def run_plan(ball, prof=None, cal=None, observed=None):
    return plan_pick_place(ball, prof or profile(), cal or calibration(),
                           image_size=[1920,1080],
                           corners_px=observed if observed is not None else corners())


def calibration():
    pts=[("P1",1,1),("P2",7,1),("P3",1,4),("P4",7,4),("V1",4,2)]
    return {
      "fit":{"validated":True,"matrix_2x2":[[0,-1],[-1,0]],"offset_mm":[310,75]},
      "records":[{"id":n,"board_mm":[x*24,y*24],
        "robot_xyz_mm":[310-y*24,75-x*24,100]} for n,x,y in pts],
      "square_mm":24.0,"image_size":[1920,1080],"corners_px":corners(),
    }


def profile():
    return {"grip_z_mm":100.0,"place_z_mm":105.0,"travel_z_mm":165.0,
            "place_xy_mm":[230.0,10.0],"pitch_rad":0.0,"roll_rad":0.0,
            "open_g_rad":1.5,"close_g_rad":2.75,"ball_diameter_mm":39.0}


class TestPickPlan(unittest.TestCase):
    def test_4_plus_1_calibration_produces_supervised_pick_plan(self):
        result=run_plan([96,60])
        self.assertEqual(result["ball_robot_xy_mm"],[250.0,-21.0])
        stages=[x["stage"] for x in result["waypoints"]]
        for name in ["hover_ball","open_before_pick","descend_pick","close_ball",
                     "lift_ball","descend_place","release_ball","retreat"]:
            self.assertIn(name,stages)
        self.assertFalse(result["auto_execute"])
        self.assertTrue(all(w["requires_operator_confirm"] for w in result["waypoints"]))
        moves=[w for w in result["waypoints"] if w["kind"]=="cartesian"]
        self.assertTrue(all(w["spd"]<=0.15 for w in moves))
        # Vertical travel is now intentionally split into small safe increments.
        self.assertTrue(all(100 <= w["xyz_mm"][2] <= 165 for w in moves))
        for before, after in zip(moves, moves[1:]):
            self.assertLessEqual(
                math.dist(before["xyz_mm"][:2], after["xyz_mm"][:2]),35.01)
            self.assertLessEqual(
                abs(before["xyz_mm"][2]-after["xyz_mm"][2]),20.01)

    def test_outside_measured_work_area_is_not_treated_as_safe(self):
        for xy in ([200,60],[40,130],[25,25],[0,0]):
            with self.subTest(xy=xy),self.assertRaises(PickPlanningError):
                run_plan(xy)

    def test_no_calibration_no_move_plan(self):
        cal=calibration();cal["fit"]["validated"]=False
        with self.assertRaises(PickPlanningError):
            run_plan([96,60],cal=cal)

    def test_moved_camera_or_chessboard_blocks_planning(self):
        displaced=[[x+25,y] for x,y in corners()]
        with self.assertRaises(PickPlanningError):
            run_plan([96,60],observed=displaced)

    def test_changed_resolution_blocks_planning(self):
        with self.assertRaises(PickPlanningError):
            plan_pick_place([96,60],profile(),calibration(),
                            image_size=[1280,720],corners_px=corners())

    def test_grip_must_be_taught_with_clearance_and_gripper_range(self):
        p=profile();p["travel_z_mm"]=120
        with self.assertRaises(PickPlanningError):
            validate_pick_profile(p)
        p=profile();p["open_g_rad"]=3.0;p["close_g_rad"]=2.4
        with self.assertRaises(PickPlanningError):
            validate_pick_profile(p)
        p=profile();p["ball_diameter_mm"]=float("nan")
        with self.assertRaises(PickPlanningError):
            validate_pick_profile(p)

    def test_negative_robot_base_z_profile_can_be_saved_and_previewed(self):
        # Actual Mac teaching sample: negative Z is valid in base coordinates.
        p = {
          "grip_z_mm": -116.18, "pitch_rad": 1.24252,
          "roll_rad": 0.0092, "travel_z_mm": -38.82,
          "place_z_mm": -103.87, "place_xy_mm": [230.0, 10.0],
          "open_g_rad": 2.34392, "close_g_rad": 2.53260,
          "ball_diameter_mm": 34.7,
        }
        valid = validate_pick_profile(p)
        self.assertEqual(valid["grip_z_mm"], -116.18)
        self.assertEqual(valid["travel_z_mm"], -38.82)
        plan = run_plan([96,60], prof=p)
        self.assertEqual(plan["waypoints"][0]["xyz_mm"][2], -38.82)
        points = [w for w in plan["waypoints"] if w["kind"]=="cartesian"]
        self.assertEqual(min(w["xyz_mm"][2] for w in points), -116.18)
        for prev, next_step in zip(points,points[1:]):
            self.assertLessEqual(abs(next_step["xyz_mm"][2] - prev["xyz_mm"][2]), 20.01)

    def test_signed_z_does_not_bypass_lift_clearance_or_range(self):
        p = profile()
        p.update({"grip_z_mm": -116.18, "place_z_mm": -103.87,
                  "travel_z_mm": -70.0})
        with self.assertRaisesRegex(PickPlanningError, "净抬升"):
            validate_pick_profile(p)
        p["travel_z_mm"] = -38.82
        p["grip_z_mm"] = -410.0
        with self.assertRaisesRegex(PickPlanningError, "超出"):
            validate_pick_profile(p)

    def test_user_place_xy_still_rejected_if_outside_current_reach(self):
        p = profile()
        p.update({"grip_z_mm": -116.18, "place_z_mm": -103.87,
                  "travel_z_mm": -38.82,
                  "place_xy_mm": [285.35, 275.04]})
        self.assertTrue(validate_pick_profile(p))
        with self.assertRaisesRegex(PickPlanningError, "放置点距基座水平半径"):
            run_plan([96,60], prof=p)

    def test_preview_only_allows_actual_taught_396mm_place_but_blocks_execution(self):
        p = profile()
        p.update({
            "grip_z_mm": -116.18, "place_z_mm": -103.87,
            "travel_z_mm": -38.82, "pitch_rad": 1.24252,
            "roll_rad": 0.0092, "open_g_rad": 2.34392,
            "close_g_rad": 2.5326, "ball_diameter_mm": 34.7,
            "place_xy_mm": [285.35, 275.04],
        })
        preview = plan_pick_place([96, 60], p, calibration(),
            image_size=[1920,1080], corners_px=corners(), preview_only=True)
        self.assertEqual(preview["place_robot_xy_mm"], [285.35, 275.04])
        self.assertTrue(preview["preview_only"])
        self.assertFalse(preview["execution_eligible"])
        self.assertFalse(preview["auto_execute"])
        self.assertTrue(any("396.3" in text for text in preview["reachability_warnings"]))
        self.assertTrue(preview["waypoints"])
        self.assertTrue(all(w["requires_operator_confirm"] for w in preview["waypoints"]))
        with self.assertRaisesRegex(PickPlanningError, "试验阈值"):
            run_plan([96, 60], prof=p)

    def test_preview_only_also_explains_arbitrary_transfer_span_limit(self):
        p = profile()
        p["place_xy_mm"] = [210.0, 295.0]
        preview = plan_pick_place([96, 60], p, calibration(),
            image_size=[1920,1080], corners_px=corners(), preview_only=True)
        self.assertFalse(preview["execution_eligible"])
        self.assertTrue(any("280mm" in text for text in preview["reachability_warnings"]))

    def test_save_then_preview_from_files_never_drives_arm(self):
        import json
        with tempfile.TemporaryDirectory() as td:
            calpath=Path(td)/"calibration.json"
            propath=Path(td)/"profile.json"
            calpath.write_text(json.dumps(calibration()))
            with patch("control_ui.pick_plan.profile_path",return_value=propath):
                save_pick_profile(profile())
                result=build_from_saved_files([96,60],calibration_file=calpath,image_size=[1920,1080],corners_px=corners())
                self.assertFalse(result["auto_execute"])


class FakeService:
    def __init__(self):
        import threading
        self.lock=threading.RLock()
        self.calls=0
    def health(self):
        return {"connected":True,"pending":None,"fault":None}
    def status(self):
        self.calls+=1
        return {"T":1051,"x":200.0,"y":10.0,"z":120.0,
                "tit":0.3,"t":-0.8,"r":0.1,"g":1.6,
                **{"torswitch"+x:1 for x in "BSETRG"}}


class TestTeach(unittest.TestCase):
    def test_teach_uses_tip_pitch_not_wrist_joint_angle(self):
        service=FakeService()
        result=observe_tool_pose(service,sleep_fn=lambda _:None)
        self.assertAlmostEqual(result["pitch_rad"],0.3)
        self.assertNotEqual(result["pitch_rad"],-0.8)
        self.assertEqual(service.calls,2)

    def test_missing_tip_pitch_rejected(self):
        svc=FakeService()
        def bad():
            result=FakeService.status(svc)
            del result["tit"]
            return result
        svc.status=bad
        with self.assertRaises(PickPlanningError):
            observe_tool_pose(svc,sleep_fn=lambda _:None)


if __name__=="__main__":
    unittest.main()
