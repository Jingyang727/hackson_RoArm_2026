"""P1 synthetic-camera tests. Never read or write the robot serial port."""
import unittest
import cv2
import numpy as np
from control_ui.vision import analyze_image, VisionError, PATTERN, confirm_empty_board


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

    def test_ball_occlusion_uses_fresh_visible_corners_not_old_image_blindly(self):
        blank = cv2.imdecode(np.frombuffer(self.photo, dtype=np.uint8),cv2.IMREAD_COLOR)
        cv2.rectangle(blank,(1020,630),(1080,690),(240,240,240),-1)
        # Fresh frame with unobscured board for the persisted 4+1 reference.
        baseline = analyze_image(cv2.imencode(".png",blank)[1].tobytes(),square_mm=24)
        self.assertTrue(baseline["checkerboard"]["detected"])
        reference={
          "image_size":[1280,960],
          "corners_px":baseline["checkerboard"]["corners_px"],
          "square_mm":24,
        }
        occluded = blank.copy()
        # A 35-pixel yellow ball directly on a central inner corner makes
        # the full 9x6 OpenCV chessboard detector fail.
        cv2.circle(occluded,(530,354),35,(0,230,255),-1)
        raw = cv2.imencode(".png",occluded)[1].tobytes()
        without = analyze_image(raw,square_mm=24)
        self.assertFalse(without["checkerboard"]["detected"])
        self.assertTrue(without["ball"]["detected"])
        self.assertNotIn("board_mm_xy",without["ball"])
        result = analyze_image(raw,square_mm=24,reference=reference)
        self.assertTrue(result["checkerboard"]["detected"])
        self.assertEqual(result["checkerboard"]["source"],"reference_partial_verified")
        self.assertGreaterEqual(result["checkerboard"]["visible_match_count"],32)
        self.assertTrue(result["ball"]["inside_inner_board"])
        self.assertIn("board_mm_xy",result["ball"])
        self.assertEqual(result["checkerboard"]["corners_count"],54)

    def test_reference_fallback_rejects_moved_board_and_changed_resolution(self):
        base=cv2.imdecode(np.frombuffer(self.photo,dtype=np.uint8),cv2.IMREAD_COLOR)
        baseline=analyze_image(cv2.imencode(".png",base)[1].tobytes(),square_mm=24)
        ref={"image_size":[1280,960],
             "corners_px":baseline["checkerboard"]["corners_px"],
             "square_mm":24}
        occluded=base.copy()
        cv2.circle(occluded,(530,354),35,(0,230,255),-1)
        # A shifted physical board must NOT be trusted from its saved XY map.
        transformed=cv2.warpAffine(
            occluded,np.float32([[1,0,25],[0,1,20]]),(1280,960))
        blocked=analyze_image(cv2.imencode(".png",transformed)[1].tobytes(),
                              square_mm=24,reference=ref)
        self.assertFalse(blocked["checkerboard"]["detected"])
        self.assertNotIn("board_mm_xy",blocked["ball"])
        resized_ref={**ref,"image_size":[1920,1080]}
        wrong_size=analyze_image(cv2.imencode(".png",occluded)[1].tobytes(),
                                 square_mm=24,reference=resized_ref)
        self.assertFalse(wrong_size["checkerboard"]["detected"])

    def test_empty_board_first_then_yellow_ball_inside_no_recalibration(self):
        # Regression: first scan has NO yellow ball. Next scan has a yellow
        # ball blocking central corners. Both use the original saved XY corners.
        base = cv2.imdecode(np.frombuffer(self.photo,dtype=np.uint8),cv2.IMREAD_COLOR)
        cv2.rectangle(base,(1010,620),(1090,700),(240,240,240),-1)
        clear_png=cv2.imencode(".png",base)[1].tobytes()
        baseline=analyze_image(clear_png,square_mm=24)
        stored={
            "square_mm":24,
            "image_size":[1280,960],
            "corners_px":baseline["checkerboard"]["corners_px"],
        }
        reference,receipt=confirm_empty_board(clear_png,24,stored)
        self.assertTrue(receipt["confirmed"])
        self.assertEqual(receipt["corners_count"],54)
        occupied=base.copy()
        cv2.circle(occupied,(530,354),35,(0,230,255),-1)
        occupied_png=cv2.imencode(".png",occupied)[1].tobytes()
        visible=analyze_image(occupied_png,square_mm=24,reference=reference)
        self.assertEqual(visible["checkerboard"]["source"],"empty_board_frame_verified")
        self.assertTrue(visible["ball"]["detected"])
        self.assertTrue(visible["ball"]["inside_inner_board"])
        self.assertIn("board_mm_xy",visible["ball"])
        self.assertGreaterEqual(visible["checkerboard"]["visible_match_count"],24)
        # Reference source is read-only; it is the exact original stored map.
        self.assertEqual(reference["corners_px"],stored["corners_px"])

    def test_empty_board_confirmation_rejects_camera_motion(self):
        base=cv2.imdecode(np.frombuffer(self.photo,dtype=np.uint8),cv2.IMREAD_COLOR)
        cv2.rectangle(base,(1010,620),(1090,700),(240,240,240),-1)
        raw=cv2.imencode(".png",base)[1].tobytes()
        original=analyze_image(raw,square_mm=24)
        stored={
            "square_mm":24,
            "image_size":[1280,960],
            "corners_px":original["checkerboard"]["corners_px"],
        }
        broken={**stored,"corners_px":[[x+22,y+17] for x,y in stored["corners_px"]]}
        with self.assertRaisesRegex(VisionError,"当前棋盘与已保存标定不一致"):
            confirm_empty_board(raw,24,broken)
        reference,_=confirm_empty_board(raw,24,stored)
        moved=cv2.warpAffine(base,np.float32([[1,0,25],[0,1,20]]),(1280,960))
        cv2.circle(moved,(555,374),35,(0,230,255),-1)
        moved_result=analyze_image(cv2.imencode(".png",moved)[1].tobytes(),
                                   square_mm=24,reference=reference)
        self.assertFalse(moved_result["checkerboard"]["detected"])
        self.assertNotIn("board_mm_xy",moved_result["ball"])

    def test_bad_image_refused(self):
        with self.assertRaises(VisionError):
            analyze_image(b"not an image")

    def test_nonphysical_square_size_refused(self):
        for bad in (-5, 0, float("nan"), float("inf"), 201):
            with self.subTest(bad=bad), self.assertRaises(VisionError):
                analyze_image(self.photo, square_mm=bad)


if __name__ == "__main__":
    unittest.main()
