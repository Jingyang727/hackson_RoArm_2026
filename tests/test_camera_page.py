"""Static smoke tests for the browser-only camera page (no hardware required)."""
from pathlib import Path
import unittest


class CameraPageTests(unittest.TestCase):
    def test_preview_exists_and_never_calls_robot_control(self):
        page = (Path(__file__).resolve().parents[1] / "control_ui" / "camera.html").read_text(encoding="utf-8")
        self.assertIn("getUserMedia", page)
        self.assertIn("保存一帧 PNG", page)
        self.assertNotIn("/api/step", page)
        self.assertNotIn("/api/unlock-once", page)
        self.assertNotIn("EVOARM_ENABLE_MOTION", page)


if __name__ == "__main__":
    unittest.main()
