"""Removed motion UIs remain retired even through old bookmarked routes."""
from pathlib import Path
import unittest

from control_ui import main

ROOT = Path(__file__).resolve().parents[1]/"control_ui"


class WorkspacePages(unittest.TestCase):
    def test_legacy_bookmarks_redirect_without_controller_calls(self):
        for fn, target in ((main.home, "/calibration"), (main.fast_teaching_page, "/calibration"),
                           (main.pick_page, "/experiments"), (main.camera_page, "/calibration#camera")):
            result = fn()
            self.assertEqual(result.status_code, 307)
            self.assertEqual(result.headers["location"], target)

    def test_workspaces_have_consistent_navigation_without_joint_controls(self):
        for filename in ("manual_calibration.html", "experiments.html", "coordinates.html"):
            text = (ROOT/filename).read_text()
            self.assertIn('href="/calibration"', text)
            self.assertIn('href="/experiments"', text)
            self.assertIn('href="/coordinates"', text)
            for obsolete in ('href="/teach"', 'href="/pick"', 'data-cal-jog', 'id="unlock"', 'id="joint"'):
                self.assertNotIn(obsolete, text)
        script = (ROOT/"workspace.js").read_text()
        for obsolete in ("/api/step", "/api/teach/move", "/api/unlock-once", "/api/home"):
            self.assertNotIn(obsolete, script)
