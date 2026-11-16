"""Pre-dispatch scene checks retain evidence without relaxing target checks."""
import json
import unittest
from unittest.mock import patch

import cv2
from fastapi import FastAPI

from control_ui import main,rsi_trial
from tests import test_rsi_loop as fixtures
from tests.test_rsi_pick_place import scene


class LiveSceneEvidence(unittest.TestCase):
    request=fixtures.RSIApi.request
    prepare=fixtures.RSIApi.prepare
    start_trial=fixtures.RSIApi.start_trial

    def setUp(self):
        fixtures.RSIApi.setUp(self)
        app=FastAPI();rsi_trial.install(app,lambda:self.service)
        p=patch.object(main,'app',app);p.start();self.addCleanup(p.stop)

    def trial_folder(self,key):return self.home/'.evoarm/rsi/pick_place_runs'/key

    def test_matching_scene_keeps_measured_pixels_and_the_actual_checked_frame(self):
        key=self.prepare();response,answer=self.start_trial(key)
        self.assertEqual(response['status'],202,answer)
        evidence=json.loads((self.trial_folder(key)/'live-scene-validation.json').read_text())
        self.assertEqual(evidence['status'],'matched');self.assertEqual(evidence['phase'],'before_motion')
        self.assertLessEqual(evidence['funnel']['delta_px'],evidence['funnel']['limit_px'])
        self.assertTrue((self.trial_folder(key)/'preflight-live.jpg').is_file())

    def test_a_real_funnel_move_still_rejects_before_authorization_and_dispatch(self):
        key=self.prepare();_,moved,_=scene(self.home,self.session,funnel=(290.,-170.,-69.6))
        _,jpeg=cv2.imencode('.jpg',moved)
        with patch.object(rsi_trial,'preview_frame',return_value={'jpeg':jpeg.tobytes()}):
            response,error=self.start_trial(key)
        self.assertEqual(response['status'],409,error);self.assertIn('漏斗位置已改变',error['detail'])
        evidence=json.loads((self.trial_folder(key)/'live-scene-validation.json').read_text())
        self.assertEqual(evidence['status'],'rejected');self.assertFalse(evidence['grasp_motion_sent_before_check'])
        self.assertGreater(evidence['funnel']['delta_px'],evidence['funnel']['limit_px'])
        self.assertEqual((self.trial_folder(key)/'preflight-live.jpg').read_bytes(),jpeg.tobytes())
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.execute_mock.call_count,0)
        self.assertEqual(self.service.armed_until,0)

    def test_detection_failure_is_saved_instead_of_losing_the_preflight_image(self):
        key=self.prepare()
        with patch.object(rsi_trial,'detect_funnel',side_effect=ValueError('synthetic rim obscured')):
            response,error=self.start_trial(key)
        self.assertEqual(response['status'],409,error)
        evidence=json.loads((self.trial_folder(key)/'live-scene-validation.json').read_text())
        self.assertEqual(evidence['error'],'synthetic rim obscured')
        self.assertTrue((self.trial_folder(key)/'preflight-live.jpg').is_file())
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.execute_mock.call_count,0)
