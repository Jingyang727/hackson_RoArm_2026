"""Camera-relative corrections and persisted policy application, no devices."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from evoarm_rsi.rsi_policy import application,project,propose,target_audit


def facing_scene():
    # A proper rotation: image right is robot -Y, image down is robot -X.
    rotation=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
    intrinsic=np.array([[600.,0.,640.],[0.,600.,360.],[0.,0.,1.]])
    scene={'camera_geometry':{'K':intrinsic.tolist(),'inverse_K':np.linalg.inv(intrinsic).tolist(),
                             'R':rotation.tolist(),'t':[300.,0.,500.],'normal_camera':[0.,0.,-1.]},
           'table_plane':{'normal_up':[0.,0.,1.],'d_mm':100.},'d_camera':600.,
           'ball':{'center_robot_mm':[300.,0.,-83.]},
           'funnel':{'mouth_center_robot_mm':[320.,-100.,-54.6]}}
    scene['ball']['pixel_center']=project(scene,scene['ball']['center_robot_mm'])
    scene['funnel']['pixel_center']=project(scene,scene['funnel']['mouth_center_robot_mm'])
    return scene


class CameraRelativePolicy(unittest.TestCase):
    def setUp(self):self.scene=facing_scene();self.parameters=copy.deepcopy(DEFAULT_PARAMETERS)

    def entry(self,before,after,proposal,**extra):
        return {'plan_id':'a'*32,'outcome':'grasp_missed','direction':'image_right','magnitude':'small',
                'before':before,'after':after,'proposal':proposal,
                'ball_xyz_mm':self.scene['ball']['center_robot_mm'],**extra}

    def test_camera_facing_robot_right_miss_requires_positive_robot_y(self):
        after,changes,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        self.assertAlmostEqual(after['pickup_xy_mm'][1],2.)
        self.assertAlmostEqual(after['pickup_xy_mm'][0],0.)
        self.assertLess(proposal['predicted_pixel_delta'][0],0)
        self.assertEqual(proposal['correction_direction'],'image_left')
        self.assertTrue(proposal['projection_direction_verified'])
        self.assertEqual(proposal['parameters_changed'],['pickup_xy_mm'])
        self.assertIn('画面左侧',' '.join(changes))

    def test_image_up_correction_is_not_robot_z_down(self):
        after,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_up')
        self.assertLess(after['pickup_xy_mm'][0],0)
        self.assertGreater(proposal['predicted_pixel_delta'][1],0)
        self.assertEqual(after['grasp_z_offset_mm'],self.parameters['grasp_z_offset_mm'])

    def test_same_miss_increases_experiment_step_and_records_next_plan_application(self):
        after,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        history=[self.entry(self.parameters,after,proposal)]
        second,_,decision=propose(after,self.scene,'grasp_missed','image_right','small',history=history)
        self.assertAlmostEqual(second['pickup_xy_mm'][1]-after['pickup_xy_mm'][1],4.)
        self.assertEqual(decision['comparison'],'same_error_repeated')
        audit=application(second,copy.deepcopy(second),[self.entry(after,second,decision)])
        self.assertTrue(audit['parameters_verified']);self.assertEqual(audit['source_trial_id'],'a'*32)
        with self.assertRaises(ValueError):application(second,self.parameters,history)
        with self.assertRaises(ValueError):application(second,second,history)

    def test_opposite_error_halves_previous_step_instead_of_oscillating(self):
        after,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        second,_,decision=propose(after,self.scene,'grasp_missed','image_left','small',history=[self.entry(self.parameters,after,proposal)])
        self.assertEqual(decision['comparison'],'direction_reversed');self.assertEqual(decision['step_mm'],1.)
        self.assertAlmostEqual(second['pickup_xy_mm'][1],1.)

    def test_worse_error_rolls_back_and_does_not_repeat_blindly(self):
        after,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        second,changes,decision=propose(after,self.scene,'grasp_missed','image_right','large',history=[self.entry(self.parameters,after,proposal)])
        self.assertEqual(second,self.parameters);self.assertFalse(decision['allow_retry'])
        self.assertEqual(decision['status'],'rolled_back');self.assertIn('恢复',' '.join(changes))

    def test_wrong_projection_and_exhausted_parameter_do_not_claim_a_new_correction(self):
        wrong=copy.deepcopy(self.parameters);wrong['pickup_xy_mm']=[0.,-4.]
        with patch('evoarm_rsi.rsi_policy.adapt',return_value=(wrong,['incorrect change'])):
            after,changes,decision=propose(self.parameters,self.scene,'grasp_missed','image_right')
        self.assertEqual(after,self.parameters);self.assertFalse(decision['allow_retry'])
        self.assertNotIn('incorrect change',' '.join(changes))
        at_limit=copy.deepcopy(self.parameters);at_limit['pickup_xy_mm']=[0.,30.]
        after,_,decision=propose(at_limit,self.scene,'grasp_missed','image_right')
        self.assertEqual(after,at_limit);self.assertFalse(decision['allow_retry'])

    def test_funnel_in_image_right_can_have_robot_negative_y(self):
        audit=target_audit(self.scene,self.parameters)
        self.assertEqual(audit['funnel_side_of_ball'],'right')
        self.assertLess(self.scene['funnel']['mouth_center_robot_mm'][1],self.scene['ball']['center_robot_mm'][1])

    def test_existing_squeeze_migration_is_accepted_without_rewriting_history(self):
        old=copy.deepcopy(self.parameters);old.pop('squeeze_deg')
        history=[{'plan_id':'a'*32,'after':old}]
        self.assertTrue(application(self.parameters,self.parameters,history)['parameters_verified'])
        self.assertNotIn('squeeze_deg',history[0]['after'])
