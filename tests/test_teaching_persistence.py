"""Persistent partial pick teaching is isolated from real hardware motion."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from control_ui.pick_plan import (
    PickPlanningError, save_pick_draft, load_pick_progress,
    save_pick_profile, build_from_saved_files
)


def taught():
    return {
      "grip_z_mm":100.0,"place_z_mm":105.0,"travel_z_mm":165.0,
      "place_xy_mm":[230,10],"pitch_rad":0.0,"roll_rad":0.0,
      "open_g_rad":1.5,"close_g_rad":2.75,"ball_diameter_mm":39.0
    }


class TestTeachingPersistence(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        folder=Path(self.tmp.name)
        for module_path,local in [
            ("control_ui.pick_plan.draft_path","draft.json"),
            ("control_ui.pick_plan.profile_path","profile.json"),
        ]:
            patcher=patch(module_path,return_value=folder/local)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_partial_stage_persists_and_recovers_across_new_ui_load(self):
        first={"grip_z_mm":115.0,"pitch_rad":0.3,"roll_rad":0.0}
        saved=save_pick_draft(first)
        self.assertTrue(saved["saved"])
        restarted=load_pick_progress()
        self.assertFalse(restarted["saved"])
        self.assertEqual(restarted["profile"],first)
        updated={**first,"travel_z_mm":180.0}
        save_pick_draft(updated)
        self.assertEqual(load_pick_progress()["profile"],updated)

    def test_final_profile_supersedes_draft_but_does_not_move_arm(self):
        save_pick_draft({"grip_z_mm":100})
        output=save_pick_profile(taught())
        self.assertTrue(output["saved"])
        state=load_pick_progress()
        self.assertTrue(state["saved"])
        self.assertEqual(state["profile"]["ball_diameter_mm"],39)

    def test_invalid_values_refuse_overwriting_saved_draft(self):
        save_pick_draft({"grip_z_mm":110})
        with self.assertRaises(PickPlanningError):
            save_pick_draft({"grip_z_mm":float("nan")})
        self.assertEqual(load_pick_progress()["profile"]["grip_z_mm"],110)

    def test_stale_xy_session_blocks_motion_preview(self):
        from control_ui import robot_calibration as calibration
        with patch.object(calibration,"session_path",return_value=Path(self.tmp.name)/"xy_draft.json"):
            (Path(self.tmp.name)/"xy_draft.json").write_text('{"saved":false}')
            with self.assertRaisesRegex(PickPlanningError,"尚未验收"):
                build_from_saved_files([50,50])


if __name__=="__main__":
    unittest.main()
