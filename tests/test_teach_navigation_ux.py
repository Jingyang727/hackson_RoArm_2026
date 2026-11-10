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
        refresh=re.search(r'\$("refresh"\)\.onclick=async\(\)=>\{(.*?)\};',html,re.S)
        self.assertIsNotNone(refresh)
        self.assertNotIn("syncSlider(",refresh.group(1))
        self.assertIn("function formatMotionReceipt(",html)
        self.assertIn("function motionBlockReason(",html)
        self.assertIn("剩余",html)
        self.assertNotIn("动作回执："+'" +JSON.stringify(',html)

    def test_navigation_helper_is_exposed_by_server(self):
        app=(ROOT/"main.py").read_text(encoding="utf-8")
        self.assertIn('@app.get("/assets/nav_status.js")',app)
        self.assertTrue((ROOT/"nav_status.js").exists())


if __name__=="__main__":
    unittest.main()
