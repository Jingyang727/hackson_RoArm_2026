"""Shared EvoArm nav and fast-teaching UX regression checks (no robot control)."""
import re
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/"control_ui"
PAGES={
    "index.html":"/",
    "camera.html":"/camera",
    "calibration.html":"/calibration",
    "pick.html":"/pick",
    "fast_teach.html":"/teach",
}
EXPECTED=["/","/camera","/calibration","/pick","/teach"]


class NavigationAndTeachUxTests(unittest.TestCase):
    def test_each_page_has_same_five_links_and_active_page(self):
        for filename,current in PAGES.items():
            with self.subTest(page=filename):
                html=(ROOT/filename).read_text(encoding="utf-8")
                matched=re.search(r'<div class="topbar"><nav class="nav-links"[^>]*>(.*?)</nav>',html,re.S)
                self.assertIsNotNone(matched)
                anchors=re.findall(r'<a href="([^"]+)"([^>]*)>',matched.group(1))
                self.assertEqual([a[0] for a in anchors],EXPECTED)
                active=[url for url,attrs in anchors if 'aria-current="page"' in attrs]
                self.assertEqual(active,[current])
                self.assertIn("connection-chip",html)

    def test_teach_keeps_target_during_feedback_refresh(self):
        html=(ROOT/"fast_teach.html").read_text(encoding="utf-8")
        # Refreshing feedback must not call syncSlider(selectedAngle()).
        refresh=re.search(r'\$\("refresh"\)\.onclick=async\(\)=>\{(.*?)\};',html,re.S)
        self.assertIsNotNone(refresh)
        self.assertNotIn("syncSlider(",refresh.group(1))
        self.assertIn("function formatMotionReceipt(",html)
        self.assertIn("function motionBlockReason(",html)
        self.assertIn("动作已完成",html)
        self.assertNotIn("动作回执："+'" +JSON.stringify(',html)

    def test_fast_teach_light_theme_has_explicit_readable_text_contrast(self):
        html=(ROOT/"fast_teach.html").read_text(encoding="utf-8")
        self.assertIn('<body class="teach-page">',html)
        self.assertIn('id="teach-accessibility-overrides"',html)
        for selector in (
            ".teach-page .notice",
            ".teach-page .pose",
            ".teach-page .chip",
            ".teach-page button:disabled",
            ".teach-page .warning",
        ):
            self.assertIn(selector,html)
        # Accessible teaching theme must be declared after the original
        # dark local rules, otherwise the dark palette returns.
        self.assertGreater(html.index("teach-accessibility-overrides"),
                           html.index(".pose{background:#263b55"))

    def test_calibration_uses_persisted_reference_and_embedded_jog(self):
        html=(ROOT/"calibration.html").read_text(encoding="utf-8")
        self.assertIn("function restoreSavedCorners(state)",html)
        self.assertIn("restoreSavedCorners(previous)",html)
        self.assertIn('data-cal-jog="-5"',html)
        self.assertIn('data-cal-jog="10"',html)
        self.assertIn('/api/teach/move-relative',html)
        self.assertNotIn('vision=null;shot=null;modifiedPoints.clear();drawOverlay();',html)
        self.assertIn("data.checkerboard.corners_px=original",html)
        self.assertIn("target.col=record.grid[0]",html)

    def test_navigation_helper_is_exposed_by_server(self):
        app=(ROOT/"main.py").read_text(encoding="utf-8")
        self.assertIn('@app.get("/assets/nav_status.js")',app)
        self.assertTrue((ROOT/"nav_status.js").exists())


if __name__=="__main__":
    unittest.main()
