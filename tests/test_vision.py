"""P1 synthetic-camera tests. Never read or write the robot serial port."""
import unittest
import cv2
import numpy as np
from control_ui.vision import analyze_image, VisionError, PATTERN


def example_image():
    # 10 x 7 SQUARES => 9 x 6 internal chessboard corners.
    image = np.full((960, 1280, 3), 240, dtype=np.uint8)
    size = 58
    top_x, top_y = 240, 180
    for row in range(7):
        for column in range(10):
            tone = 10 if (row + column) % 2 == 0 else 245
            x = top_x + column * size
            y = top_y + row * size
            cv2.rectangle(image, (x, y), (x + size - 1, y + size - 1),
                          (tone, tone, tone), -1)
    cv2.circle(image, (1040, 660), 35, (0, 230, 255), -1)  # Yellow, outside board
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    return encoded.tobytes()


class ChessboardVisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.photo = example_image()

    def test_detects_board_and_yellow_ball(self):
        result = analyze_image(self.photo)
        self.assertEqual(result["image"], {"width": 1280, "height": 960})
        self.assertTrue(result["checkerboard"]["detected"])
        self.assertEqual(result["checkerboard"]["corners_count"], 54)
        self.assertEqual(result["checkerboard"]["pattern_inner"], list(PATTERN))
        self.assertTrue(result["ball"]["detected"])
        self.assertAlmostEqual(result["ball"]["center_px"][0], 1040, delta=3)
        self.assertAlmostEqual(result["ball"]["center_px"][1], 660, delta=3)
        self.assertFalse(result["ball"]["inside_inner_board"])
        self.assertFalse(result["robot_motion_enabled"])

    def test_square_size_enables_board_mm_only(self):
        result = analyze_image(self.photo, square_mm=20.0)
        ball = result["ball"]
        self.assertIn("board_mm_xy", ball)
        self.assertAlmostEqual(ball["board_mm_xy"][0],
                               ball["board_grid_xy"][0] * 20, delta=0.15)
        self.assertTrue(any("不可直接用于机械臂运动" in w for w in result["warnings"]))

    def test_unknown_square_size_does_not_invent_mm(self):
        result = analyze_image(self.photo)
        self.assertNotIn("board_mm_xy", result["ball"])
        self.assertIsNone(result["checkerboard"]["square_size_mm"])

    def test_bad_image_refused(self):
        with self.assertRaises(VisionError):
            analyze_image(b"not an image")

    def test_nonphysical_square_size_refused(self):
        for bad in (-5, 0, float("nan"), float("inf"), 201):
            with self.subTest(bad=bad), self.assertRaises(VisionError):
                analyze_image(self.photo, square_mm=bad)


if __name__ == "__main__":
    unittest.main()
