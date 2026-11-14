"""Paired visual evidence, feedback attribution and effect-driven RSI, no devices."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from evoarm_rsi import grok_review
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from evoarm_rsi.rsi_comparison import build_comparison,camera_view,comparison_panel,validate_assessment
from evoarm_rsi.rsi_policy import propose
from tests.test_rsi_policy import facing_scene


def paired_trials(root):
    scene=facing_scene();states=[]
    before=copy.deepcopy(DEFAULT_PARAMETERS);after=copy.deepcopy(before);after['pickup_xy_mm']=[0.,8.]
    for i,parameters in enumerate((before,after)):
        key=('a' if i==0 else 'b')*32;folder=root/key;folder.mkdir(parents=True)
        image=np.full((720,1280,3),220,np.uint8)
        cv2.circle(image,(640,360),20,(0,210,230),-1)
        cv2.rectangle(image,(675-i*9,338),(683-i*9,380),(20,20,20),-1)
        rows=[]
        for n,label in enumerate(('before','close_review','lift_review','release')):
            name=f'evidence-{n:02d}.jpg';cv2.imwrite(str(folder/name),image)
            rows.append({'label':label,'labels':[label],'frame_index':n,'elapsed_s':float(n),'file':name})
        (folder/'keyframes.json').write_text(json.dumps({'frames':rows}))
        state={'id':key,'policy_version':i+1,'calibration_fingerprint':'same-calibration',
               'plan':{'parameters':parameters,'scene':copy.deepcopy(scene),'grasp_pitch_deg':80.,
                       'grasp_tip_xyz_mm':[300.,float(i*8),-100.],'release_tip_xyz_mm':[320.,-100.,-70.]},
               'result':{'capture_trial_id':key,'grasp_attempted':True,'placement_attempted':True,
                         'feedback':[{'stage':'close','status':{'x':300.,'y':float(i*8),'z':-100.}}]}}
        states.append(state)
    review={'plan_id':'a'*32,'before':before,'after':after,'outcome':'grasp_missed','direction':'image_right',
            'proposal':{'predicted_pixel_delta':[-8.2,0.],'step_mm':8.},'note':'gripper was image-right'}
    return states[0],states[1],review


class PairedComparison(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.experiments=self.root/'experiments';self.previous,self.current,self.review=paired_trials(self.experiments)
        self.trial=self.root/'review';self.trial.mkdir()

    def comparison(self):return build_comparison(self.current,self.previous,self.review,self.experiments)

    def test_pair_binds_real_frames_to_the_parameters_actually_used(self):
        value=self.comparison();self.assertTrue(value['available'])
        self.assertEqual(value['reference_plan_id'],self.previous['id'])
        self.assertEqual(value['requested_xy_delta_mm'],[0.,8.])
        self.assertEqual(value['feedback']['reported_xy_delta_mm'],[0.,8.])
        self.assertEqual(value['feedback']['status'],'consistent')
        self.assertEqual(value['previous_frame']['role'],'previous')
        self.assertEqual(value['current_frame']['frame_index'],1)

    def test_feedback_subtracts_target_relocation_instead_of_learning_it_as_an_offset(self):
        self.current['plan']['scene']['ball']['center_robot_mm'][0]+=10
        self.current['plan']['grasp_tip_xyz_mm'][0]+=10
        self.current['result']['feedback'][0]['status']['x']+=10
        value=self.comparison();self.assertTrue(value['available'])
        self.assertEqual(value['feedback']['reported_xy_delta_mm'],[0.,8.])

    def test_opposite_reported_motion_is_separate_evidence_and_rolls_back(self):
        self.current['result']['feedback'][0]['status']['y']=-8
        value=self.comparison();self.assertEqual(value['feedback']['status'],'opposite')
        assessment={'reference_plan_id':self.previous['id'],'effect':'worsened','movement':'opposite','note':'pair shows wrong-way shift'}
        after,_,decision=propose(self.current['plan']['parameters'],self.current['plan']['scene'],'grasp_missed','image_right',
                                history=[self.review],comparison=value,assessment=assessment)
        self.assertEqual(after,DEFAULT_PARAMETERS);self.assertFalse(decision['allow_retry'])
        self.assertEqual(decision['comparison'],'direction_disproved')

    def test_no_visible_motion_does_not_increase_an_unexecuted_correction(self):
        self.current['result']['feedback'][0]['status']['y']=0.
        value=self.comparison();self.assertEqual(value['feedback']['status'],'too_small')
        assessment={'reference_plan_id':self.previous['id'],'effect':'unchanged','movement':'no_visible_motion','note':'same gripper position'}
        after,_,decision=propose(self.current['plan']['parameters'],self.current['plan']['scene'],'grasp_missed','image_right',
                                history=[self.review],comparison=value,assessment=assessment)
        self.assertEqual(after,DEFAULT_PARAMETERS);self.assertFalse(decision['allow_retry'])

    def test_missing_feedback_does_not_fabricate_a_measured_displacement(self):
        self.current['result']['feedback']=[]
        value=self.comparison();self.assertTrue(value['available'])
        self.assertFalse(value['feedback']['available']);self.assertNotIn('reported_xy_delta_mm',value['feedback'])

    def test_feedback_with_large_cross_axis_error_is_not_called_consistent(self):
        self.current['result']['feedback'][0]['status']['x']+=100
        self.assertEqual(self.comparison()['feedback']['status'],'inconclusive')

    def test_new_position_pose_or_unrelated_parameter_history_is_not_a_correction_comparison(self):
        for mutation in ('position','pitch','parameters','reference'):
            previous,current,review=copy.deepcopy((self.previous,self.current,self.review))
            if mutation=='position':current['plan']['scene']['ball']['center_robot_mm'][0]+=40
            if mutation=='pitch':current['plan']['grasp_pitch_deg']=60
            if mutation=='parameters':current['plan']['parameters']['travel_speed']=.9
            if mutation=='reference':review['plan_id']='c'*32
            self.assertFalse(build_comparison(current,previous,review,self.experiments)['available'])

    def test_command_start_frame_alone_does_not_claim_a_completed_grasp_comparison(self):
        path=self.experiments/self.current['id']/'keyframes.json'
        rows=json.loads(path.read_text())
        rows['frames'][1].update(label='close',labels=['close'])
        path.write_text(json.dumps(rows))
        value=self.comparison();self.assertFalse(value['available'])
        self.assertIn('动作开始帧',value['reason'])

    def test_reused_capture_id_cannot_be_two_trials_of_evidence(self):
        self.current['result']['capture_trial_id']=self.previous['result']['capture_trial_id']
        self.assertFalse(self.comparison()['available'])

    def test_camera_labels_and_expected_arrow_do_not_mirror_or_cover_the_recorded_pixels(self):
        raw=cv2.imread(self.comparison()['current_frame']['path'])
        annotated=camera_view(raw,'CURRENT',[-10,0])
        np.testing.assert_array_equal(annotated[58:58+raw.shape[0],:raw.shape[1]],raw)
        panel=cv2.imread(comparison_panel(self.comparison(),self.trial))
        self.assertEqual(panel.shape[:2],(538,1120));self.assertGreater(float(panel.mean()),100.)

    def test_grok_receives_previous_and_current_real_images_and_checked_effect_is_required(self):
        context={'plan_id':self.current['id'],'comparison':self.comparison(),
                 'video_path':str(self.experiments/self.current['id']/'rgb.mp4')}
        verdict={'outcome':'grasp_missed','direction':'image_right','ball_state':'table','magnitude':'medium',
                 'confidence':.9,'note':'still right but closer','comparison':{'reference_plan_id':self.previous['id'],
                 'effect':'improved','movement':'as_expected','note':'previous and current close views show a smaller gap'}}
        config={'configured':True,'model':'grok-test','backend':'company_api'};updates=[]
        with (patch.object(grok_review,'configuration',return_value=config),
              patch.object(grok_review,'complete_content',return_value=json.dumps(verdict)) as api):
            result=grok_review.review(context,self.trial,threading.Event(),on_update=updates.append)
        self.assertEqual(result,verdict)
        texts=[p['text'] for p in api.call_args.args[1] if p['type']=='text']
        self.assertTrue(any('PREVIOUS' in t and self.previous['id'][:10] in t for t in texts))
        self.assertTrue(any('CURRENT' in t and self.current['id'][:10] in t for t in texts))
        self.assertTrue((self.trial/'comparison.jpg').is_file())
        request=json.loads((self.trial/'grok-request.json').read_text())
        self.assertTrue(all(Path(frame['model_image_path']).is_file() for frame in request['reviewed_frames']))
        self.assertEqual(updates[0]['type'],'comparison')
        for bad in ({k:v for k,v in verdict.items() if k!='comparison'},
                    {**verdict,'comparison':{**verdict['comparison'],'reference_plan_id':'c'*32}}):
            with patch.object(grok_review,'configuration',return_value=config),patch.object(grok_review,'complete_content',return_value=json.dumps(bad)),self.assertRaises(ValueError):
                grok_review.review(context,self.trial,threading.Event())

    def test_assessment_schema_rejects_motor_parameters_and_claimed_comparison_without_reference(self):
        value={'reference_plan_id':None,'effect':'not_compared','movement':'unknown','note':'first trial'}
        self.assertEqual(validate_assessment(value),value)
        for change in ({'effect':'improved'},{'movement':'opposite'},{'delta_mm':10}):
            with self.assertRaises(ValueError):validate_assessment({**value,**change})
