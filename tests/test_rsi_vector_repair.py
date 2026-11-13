"""Regression of the uploaded saturated-Y trial, not a zero-offset-only test."""
import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from evoarm_rsi.policy_repair import repair_clipped_policy
from evoarm_rsi.rsi_policy import application,propose
from tests.test_rsi_policy import facing_scene


def legacy_policy():
    before=copy.deepcopy(DEFAULT_PARAMETERS);after=copy.deepcopy(before)
    before['pickup_xy_mm']=[-14.242209,30.];after['pickup_xy_mm']=[-15.632220,30.]
    return {'version':18,'parameters':after,'reviews':[{'plan_id':'d'*32,'outcome':'grasp_missed',
            'before':before,'after':copy.deepcopy(after),'proposal':{'step_mm':8.}}]}


def oblique_scene(tilted=True):
    scene=facing_scene();angle=math.radians(10);c=math.cos(angle);s=math.sin(angle)
    yaw=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    tilt=.18 if tilted else 0.;c=math.cos(tilt);s=math.sin(tilt)
    pitch=np.array([[1,0,0],[0,c,-s],[0,s,c]])
    scene['camera_geometry']['R']=(yaw@pitch@np.array(scene['camera_geometry']['R'])).tolist()
    return scene


class FullVector(unittest.TestCase):
    def test_oblique_camera_full_step_at_recorded_saturated_y_for_all_four_sides(self):
        before=legacy_policy()['parameters'];scene=oblique_scene()
        for direction in ('image_left','image_right','image_up','image_down'):
            with self.subTest(direction=direction):
                after,_,decision=propose(before,scene,'grasp_missed',direction,'medium')
                self.assertAlmostEqual(math.dist(before['pickup_xy_mm'],after['pickup_xy_mm']),8.)
                self.assertGreater(decision['projection_direction_alignment'],.99999)
                self.assertTrue(decision['projection_direction_verified'])
        after,_,_=propose(before,scene,'grasp_missed','image_right','large')
        self.assertGreater(after['pickup_xy_mm'][1],30.)
        self.assertAlmostEqual(math.dist(before['pickup_xy_mm'],after['pickup_xy_mm']),12.)

    def test_one_point_four_mm_with_tiny_correct_component_and_eighty_degree_error_rejected(self):
        before=legacy_policy()['parameters'];clipped=copy.deepcopy(before);clipped['pickup_xy_mm'][0]-=1.389
        with patch('evoarm_rsi.rsi_policy.adapt',return_value=(clipped,['bad clipped delta'])):
            after,_,decision=propose(before,oblique_scene(False),'grasp_missed','image_right')
        self.assertEqual(after,before);self.assertFalse(decision['requested_step_preserved'])
        self.assertAlmostEqual(math.degrees(math.acos(decision['projection_direction_alignment'])),80.)
        self.assertFalse(decision['allow_retry']);self.assertEqual(decision['saved_xy_step_mm'],0.)

    def test_agent_can_choose_visible_step_beyond_automatic_sixteen_without_hidden_clip(self):
        before=legacy_policy()['parameters']
        after,_,decision=propose(before,oblique_scene(),'grasp_missed','image_right',requested_step_mm=20.)
        self.assertAlmostEqual(math.dist(before['pickup_xy_mm'],after['pickup_xy_mm']),20.)
        self.assertEqual(decision['decision'],'agent_step')
        self.assertTrue(decision['projection_direction_verified'])

    def test_unchanged_is_an_assessment_not_verified_improvement(self):
        before=copy.deepcopy(DEFAULT_PARAMETERS);after=copy.deepcopy(before);after['pickup_xy_mm']=[0.,8.]
        previous={'plan_id':'a'*32,'before':before,'after':after,'proposal':{'step_mm':8.}}
        _,_,decision=propose(after,facing_scene(),'grasp_missed','image_right',history=[previous],
            comparison={'available':True,'reference_plan_id':'a'*32,'feedback':{'status':'consistent'}},
            assessment={'reference_plan_id':'a'*32,'effect':'unchanged','movement':'as_expected','note':'same miss'})
        self.assertTrue(decision['effect_assessed']);self.assertFalse(decision['effect_verified'])


class Quarantine(unittest.TestCase):
    def test_repairs_only_bad_xy_preserves_history_and_other_parameters_once(self):
        old=legacy_policy();old['parameters']['squeeze_deg']=4.;old['reviews'][-1]['after']['squeeze_deg']=4.
        original=copy.deepcopy(old);new,record=repair_clipped_policy(old)
        self.assertEqual(old,original);self.assertEqual(new['parameters']['pickup_xy_mm'],[0.,0.])
        self.assertEqual(new['parameters']['squeeze_deg'],4.);self.assertEqual(new['reviews'],old['reviews'])
        self.assertEqual(new['version'],19);self.assertEqual(record['source_trial_id'],'d'*32)
        audit=application(new['parameters'],new['parameters'],new['reviews'],new['baseline_resets'])
        self.assertEqual(audit['baseline_reset_id'],record['id'])
        self.assertEqual(repair_clipped_policy(new),(new,None))
        with self.assertRaises(ValueError):application(new['parameters'],new['parameters'],new['reviews'])

    def test_legitimate_small_or_unsaturated_updates_not_reset(self):
        for mutate in ('small','unsaturated','success','different_policy'):
            old=legacy_policy()
            if mutate=='small':old['reviews'][-1]['proposal']['step_mm']=1.389
            if mutate=='unsaturated':
                old['parameters']['pickup_xy_mm'][1]=29.;old['reviews'][-1]['after']['pickup_xy_mm'][1]=29.
            if mutate=='success':old['reviews'][-1]['outcome']='placed'
            if mutate=='different_policy':old['parameters']['pickup_xy_mm'][0]=0.
            self.assertIsNone(repair_clipped_policy(old)[1],mutate)

    def test_successful_baseline_requires_real_success_review(self):
        old=legacy_policy();success=copy.deepcopy(DEFAULT_PARAMETERS);success['pickup_xy_mm']=[1.,2.]
        old['successful_policy']={'plan_id':'a'*32,'parameters':success}
        self.assertEqual(repair_clipped_policy(old)[0]['parameters']['pickup_xy_mm'],[0.,0.])
        old['reviews'].insert(0,{'plan_id':'a'*32,'outcome':'placed','after':success})
        self.assertEqual(repair_clipped_policy(old)[0]['parameters']['pickup_xy_mm'],[1.,2.])
