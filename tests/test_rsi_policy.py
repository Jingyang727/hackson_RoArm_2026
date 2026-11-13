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

    def evidence(self,effect,movement='as_expected'):
        return {'comparison':{'available':True,'reference_plan_id':'a'*32,'feedback':{'status':'consistent'}},
                'assessment':{'reference_plan_id':'a'*32,'effect':effect,'movement':movement,'note':'paired actual frames'}}

    def test_camera_facing_robot_right_miss_requires_positive_robot_y(self):
        after,changes,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        self.assertAlmostEqual(after['pickup_xy_mm'][1],4.)
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
        second,_,decision=propose(after,self.scene,'grasp_missed','image_right','small',history=history,**self.evidence('unchanged'))
        self.assertAlmostEqual(second['pickup_xy_mm'][1]-after['pickup_xy_mm'][1],8.)
        self.assertEqual(decision['comparison'],'same_error_repeated')
        audit=application(second,copy.deepcopy(second),[self.entry(after,second,decision)])
        self.assertTrue(audit['parameters_verified']);self.assertEqual(audit['source_trial_id'],'a'*32)
        with self.assertRaises(ValueError):application(second,self.parameters,history)
        with self.assertRaises(ValueError):application(second,second,history)

    def test_opposite_error_halves_previous_step_instead_of_oscillating(self):
        after,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        second,_,decision=propose(after,self.scene,'grasp_missed','image_left','small',history=[self.entry(self.parameters,after,proposal)],**self.evidence('overshot'))
        self.assertEqual(decision['comparison'],'overshot');self.assertEqual(decision['step_mm'],2.)
        self.assertAlmostEqual(second['pickup_xy_mm'][1],2.)

    def test_worse_error_rolls_back_and_does_not_repeat_blindly(self):
        after,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        second,changes,decision=propose(after,self.scene,'grasp_missed','image_right','large',history=[self.entry(self.parameters,after,proposal)],**self.evidence('worsened'))
        self.assertEqual(second,self.parameters);self.assertFalse(decision['allow_retry'])
        self.assertEqual(decision['status'],'rolled_back');self.assertIn('恢复',' '.join(changes))

    def test_initial_obvious_error_uses_ball_scaled_step_and_improvement_reduces_it(self):
        first,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','large')
        self.assertAlmostEqual(first['pickup_xy_mm'][1],12.)
        after,_,decision=propose(first,self.scene,'grasp_missed','image_right','medium',
                                history=[self.entry(self.parameters,first,proposal)],**self.evidence('improved'))
        self.assertEqual(decision['step_mm'],4.);self.assertEqual(after['pickup_xy_mm'][1],16.)

    def test_repeated_direction_alone_does_not_prove_need_to_increase(self):
        first,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','small')
        _,_,decision=propose(first,self.scene,'grasp_missed','image_right','small',history=[self.entry(self.parameters,first,proposal)])
        self.assertEqual(decision['step_mm'],4.);self.assertFalse(decision['effect_verified'])
        self.assertNotEqual(decision['comparison'],'same_error_repeated')

    def test_opposite_motion_undoes_and_rejected_policy_is_not_forgotten(self):
        first,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','medium')
        history=[self.entry(self.parameters,first,proposal)]
        restored,_,undo=propose(first,self.scene,'grasp_missed','image_right','medium',history=history,**self.evidence('worsened','opposite'))
        self.assertEqual(restored,self.parameters);self.assertEqual(undo['comparison'],'direction_disproved')
        self.assertFalse(undo['allow_retry'])
        history.append(self.entry(first,restored,undo))
        retried,_,decision=propose(restored,self.scene,'grasp_missed','image_right','medium',history=history)
        self.assertEqual(retried,restored);self.assertFalse(decision['allow_retry'])
        self.assertEqual(decision['decision'],'avoid_rejected_policy')

    def test_wrong_reference_does_not_control_the_policy(self):
        first,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right')
        evidence=self.evidence('worsened');evidence['assessment']['reference_plan_id']='b'*32
        with self.assertRaises(ValueError):
            propose(first,self.scene,'grasp_missed','image_right',history=[self.entry(self.parameters,first,proposal)],**evidence)

    def test_tiny_perception_changes_do_not_resurrect_a_rejected_strategy(self):
        angle=.2;c=np.cos(angle);s=np.sin(angle)
        rotation=np.asarray(self.scene['camera_geometry']['R'])@np.array([[c,0,s],[0,1,0],[-s,0,c]])
        self.scene['camera_geometry']['R']=rotation.tolist()
        self.scene['camera_geometry']['normal_camera']=(rotation.T@np.array([0,0,1.])).tolist()
        self.scene['ball']['pixel_center']=project(self.scene,self.scene['ball']['center_robot_mm'])
        first,_,proposal=propose(self.parameters,self.scene,'grasp_missed','image_right','medium')
        history=[self.entry(self.parameters,first,proposal)]
        restored,_,undo=propose(first,self.scene,'grasp_missed','image_right','medium',history=history,**self.evidence('worsened','opposite'))
        history.append(self.entry(first,restored,undo))
        self.scene['ball']['center_robot_mm'][0]+=.5
        self.scene['ball']['pixel_center']=project(self.scene,self.scene['ball']['center_robot_mm'])
        fresh,_,_=propose(restored,self.scene,'grasp_missed','image_right','medium')
        self.assertNotEqual(fresh['pickup_xy_mm'],first['pickup_xy_mm'])
        after,_,decision=propose(restored,self.scene,'grasp_missed','image_right','medium',history=history)
        self.assertEqual(after,restored);self.assertEqual(decision['decision'],'avoid_rejected_policy')

    def test_legacy_two_mm_probe_not_visible_is_replaced_by_a_visible_probe_not_a_false_direction_fault(self):
        old=copy.deepcopy(self.parameters);old['pickup_xy_mm']=[0.,2.]
        history=[self.entry(self.parameters,old,{'step_mm':2.})]
        after,_,decision=propose(old,self.scene,'grasp_missed','image_right','small',history=history,**self.evidence('unchanged','no_visible_motion'))
        self.assertEqual(decision['step_mm'],8.);self.assertTrue(decision['allow_retry'])
        self.assertAlmostEqual(after['pickup_xy_mm'][1],10.)

    def test_wrong_projection_and_exhausted_parameter_do_not_claim_a_new_correction(self):
        wrong=copy.deepcopy(self.parameters);wrong['pickup_xy_mm']=[0.,-4.]
        with patch('evoarm_rsi.rsi_policy.adapt',return_value=(wrong,['incorrect change'])):
            after,changes,decision=propose(self.parameters,self.scene,'grasp_missed','image_right')
        self.assertEqual(after,self.parameters);self.assertFalse(decision['allow_retry'])
        self.assertEqual(decision['robot_delta_mm'],[0.,0.])
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
