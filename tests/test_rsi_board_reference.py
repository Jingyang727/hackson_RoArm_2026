"""One-time Gemini board reference reused with yellow ball already present."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from evoarm_rsi.board_reference import create_reference
from evoarm_rsi.localize import locate_saved_trial
from tests.test_rsi_localize import FakeVideo


class FiniteVideo:
    def __init__(self,frame,frames=4):
        self.frame=frame
        self.frames=frames
        self.index=0
    def isOpened(self): return True
    def read(self):
        if self.index>=self.frames: return False,None
        self.index+=1
        return True,self.frame.copy()
    def release(self): pass


class PersistentGeminiTests(unittest.TestCase):
    def _fixtures(self,root):
        root=Path(root)
        reference=root/"trial_0031"
        new_trial=root/"trial_new_ball_only"
        reference.mkdir()
        new_trial.mkdir()
        empty=np.zeros((240,320,3),dtype=np.uint8)
        ball=empty.copy()
        cv2.circle(ball,(160,110),12,(0,255,255),-1)
        corners=np.asarray([
            [80+col*20,60+row*20]
            for row in range(6) for col in range(9)
        ],dtype=np.float32).reshape(6,9,2)
        board_mm=np.asarray([(col,row)
                  for row in range(6) for col in range(9)],dtype=np.float32)
        H,_=cv2.findHomography(corners.reshape(54,2),board_mm,0)
        (reference/"rgb.mp4").write_bytes(b"fake")
        (reference/"manifest.json").write_text(json.dumps({
            "simulated":False,"trial_id":"0031"
        }))
        (reference/"gemini_color_intrinsics.json").write_text(json.dumps({
            "fx":609.182,"fy":609.079,"cx":638.265,"cy":361.901,
            "width":320,"height":240,"aligned_to":"color",
            "verified":False
        }))
        (new_trial/"rgb.mp4").write_bytes(b"fake")
        (new_trial/"manifest.json").write_text(json.dumps({
            "simulated":False,"trial_id":"fresh-ball"
        }))
        (new_trial/"frames.jsonl").write_text("\n".join(json.dumps({
            "host_monotonic_ns":1_000_000_000+i*100_000_000
        }) for i in range(4))+"\n")
        depth=np.full((240,320),600,dtype=np.uint16)
        cv2.circle(depth,(160,110),7,560,-1)
        np.savez_compressed(new_trial/"depth_samples.npz",
             depth_mm=np.stack((depth,depth)),
             monotonic_ns=np.asarray([1_000_000_000,1_200_000_000],
                                      dtype=np.int64))
        return reference,new_trial,empty,ball,corners,H

    def test_reuse_trial0031_board_even_when_no_current_frame_has_board(self):
        with tempfile.TemporaryDirectory() as d:
            reference,trial,empty,ball,corners,H=self._fixtures(d)
            saved=Path(d)/"gemini_board_reference.json"
            with (patch("cv2.VideoCapture",return_value=FiniteVideo(empty)),
                  patch("control_ui.vision.find_chessboard",
                        return_value=(corners,H))):
                extracted=create_reference(reference,output=saved)
            self.assertEqual(extracted["frames_verified"],4)
            self.assertTrue(saved.exists())
            with (patch("cv2.VideoCapture",return_value=FakeVideo(ball)),
                  patch("control_ui.vision.find_chessboard",return_value=(None,None))):
                localized=locate_saved_trial(
                    trial,board_reference_file=saved,
                    output_file=Path(d)/"localized.json"
                )
            self.assertEqual(localized["board_frames_detected"],0)
            self.assertEqual(localized["ball_frames_with_live_board_xy"],2)
            self.assertEqual(localized["persistent_reference_source_trial_id"],"0031")
            self.assertFalse(localized["persistent_reference_placement_verified"])
            self.assertEqual(localized["observations"][0]["ball_board_xy_source"],
                             "persistent_gemini_board_reference_placement_unverified")
            self.assertAlmostEqual(localized["observations"][0]["ball_board_xy_mm"][0],96,delta=.1)
            self.assertAlmostEqual(localized["observations"][0]["ball_board_xy_mm"][1],60,delta=.1)
            self.assertFalse(localized["motion_authorized"])

    def test_displaced_visible_board_is_recorded_and_not_trusted(self):
        with tempfile.TemporaryDirectory() as d:
            reference,trial,empty,ball,corners,H=self._fixtures(d)
            saved=Path(d)/"reference.json"
            with (patch("cv2.VideoCapture",return_value=FiniteVideo(empty)),
                  patch("control_ui.vision.find_chessboard",
                        return_value=(corners,H))):
                create_reference(reference,output=saved)
            moved=corners.copy()
            moved[:,:,0]+=30
            with (patch("cv2.VideoCapture",return_value=FakeVideo(ball)),
                  patch("control_ui.vision.find_chessboard",
                        return_value=(moved,H))):
                localized=locate_saved_trial(
                    trial,board_reference_file=saved,
                    output_file=Path(d)/"localized.json")
            self.assertEqual(localized["persistent_reference_displaced_board_frames"],2)
            self.assertEqual(localized["ball_frames_detected"],0)
            self.assertFalse(localized["motion_authorized"])


if __name__=="__main__":
    unittest.main()
