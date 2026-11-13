"""Multimodal RSI reviewer tests without sending network requests."""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from evoarm_rsi.vision_review import (
    build_keyframes, describe_evidence, review_trial, _validate_review
)
from evoarm_rsi.gateway import CompanyModelGateway


class FakeResponse:
    status=200
    def __init__(self, data):
        self.payload=json.dumps({"choices":[{"message":{"content":json.dumps(data)}}]}).encode()
    def __enter__(self): return self
    def __exit__(self,*a): pass
    def read(self, size): return self.payload[:size]


class VisionReviewTests(unittest.TestCase):
    def test_never_calls_an_unconfigured_public_model(self):
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaisesRegex(RuntimeError,"not configured"):
                review_trial("/nonexistent")

    def test_camera_only_trial_is_not_reported_as_grasp_success(self):
        claimed={
            "observed_outcome":"picked_up",
            "confidence":1,
            "failure_type":"none",
            "evidence":["yellow ball visible"],
            "changes":{"offset_x_mm":3},
        }
        result=_validate_review(claimed,motion_recorded=False)
        self.assertEqual(result["observed_outcome"],"not_attempted")
        self.assertEqual(result["failure_type"],"no_attempt")
        self.assertEqual(result["changes"],{})

    def test_rejects_invalid_model_output(self):
        with self.assertRaises(ValueError):
            _validate_review({
                "observed_outcome":"magic", "confidence":0.5,
                "failure_type":"none", "evidence":[],
                "changes":{},
            },motion_recorded=True)

    def test_company_call_includes_ordered_images_and_depth_but_no_commands(self):
        with patch.dict(os.environ,{"EVOARM_COMPANY_LLM_TOKEN":"test-no-secret"}):
            gateway=CompanyModelGateway(
                base_url="https://inference.tesla.com/v1",
                model="grok-4.7-build",
                timeout_s=10
            )
            keyframes=[{"frame_index":0,"elapsed_s":0.0,
                        "data_url":"data:image/jpeg;base64,aGVsbG8="},
                       {"frame_index":10,"elapsed_s":1.5,
                        "data_url":"data:image/jpeg;base64,d29ybGQ="}]
            metadata={
                "trial_id":"real1", "simulated":False,
                "robot_motion_executed":True,
                "depth_samples_available":True,
                "robot_feedback_samples":10,
            }
            captured=[]
            def mock_urlopen(req,timeout):
                captured.append(json.loads(req.data))
                self.assertEqual(req.get_method(),"POST")
                self.assertEqual(timeout,10)
                return FakeResponse({
                    "observed_outcome":"unknown","confidence":0.3,
                    "failure_type":"unknown","evidence":["occlusion"],
                    "changes":{},
                })
            with (patch("evoarm_rsi.vision_review.build_keyframes",return_value=keyframes),
                  patch("evoarm_rsi.vision_review.describe_evidence",return_value=metadata),
                  patch("evoarm_rsi.vision_review.request.urlopen",side_effect=mock_urlopen)):
                output=review_trial("/unused",gateway=gateway,frames=2)
            self.assertEqual(output["observed_outcome"],"not_attempted")
            self.assertTrue(output["image_payload_accepted_by_company_gateway"])
            parts=captured[0]["messages"][1]["content"]
            self.assertEqual([p["type"] for p in parts],["text","image_url","image_url"])
            self.assertIn("robot_feedback_samples",parts[0]["text"])
            self.assertIn("elapsed_s",parts[0]["text"])
            self.assertNotIn("Authorization",json.dumps(captured))

    def test_input_depth_stats_are_labeled_not_ball_specific(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            (folder/"manifest.json").write_text(json.dumps({
                "trial_id":"a","simulated":False,
                "kind":"real_camera_observation_only",
                "robot_motion_executed":False
            }))
            np.savez_compressed(folder/"depth_samples.npz",
                                depth_mm=np.array([[[0,500],[600,500]]],
                                                  dtype=np.uint16))
            e=describe_evidence(folder)
            self.assertEqual(e["depth_statistics"]["median_valid_mm"],500)
            self.assertFalse(e["robot_motion_executed"])
            self.assertIn("not ball-specific",e["depth_statistics"]["note"])

    def test_invalid_frame_count_fails_before_camera_access(self):
        with self.assertRaisesRegex(ValueError,"2..8"):
            build_keyframes("/missing",count=1)


if __name__=="__main__":
    unittest.main()
