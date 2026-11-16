"""Persist speed changes without movement, preserving the RSI policy lineage."""
import json
import unittest
from unittest.mock import patch

from fastapi import FastAPI
from control_ui import main,rsi_trial
from evoarm_rsi.coordinate_test import fingerprint
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from tests import test_rsi_loop as fixtures


class SpeedAPI(unittest.TestCase):
    request=fixtures.RSIApi.request
    prepare=fixtures.RSIApi.prepare
    start_trial=fixtures.RSIApi.start_trial

    def setUp(self):
        fixtures.RSIApi.setUp(self)
        app=FastAPI();rsi_trial.install(app,lambda:self.service)
        p=patch.object(main,'app',app);p.start();self.addCleanup(p.stop)
        self.path=self.home/'.evoarm/rsi/pick_place_policies'/self.session['setup_id']/(fingerprint(self.session)+'.json')

    def settings(self):return self.request('state',method='GET')[1]['speed_settings']

    def save(self,travel=1.,approach=.5):
        return self.request('speed',{**self.settings(),'travel_speed':travel,'approach_speed':approach})

    def test_read_defaults_does_not_create_policy_or_move(self):
        value=self.settings();self.assertEqual(value['travel_speed'],.65);self.assertEqual(value['approach_speed'],.3)
        self.assertFalse(self.path.exists());self.assertEqual(self.arm.commands,[])

    def test_save_replans_all_cartesian_stages_with_chosen_speeds_and_survives_restart(self):
        response,answer=self.save();self.assertEqual(response['status'],200,answer)
        self.assertTrue(answer['changed']);self.assertFalse(answer['robot_moved']);self.assertEqual(self.arm.commands,[])
        saved=json.loads(self.path.read_text());self.assertEqual(saved['parameters'],{**DEFAULT_PARAMETERS,'travel_speed':1.,'approach_speed':.5})
        app=FastAPI();rsi_trial.install(app,lambda:self.service)
        with patch.object(main,'app',app):
            self.assertEqual(self.settings()['travel_speed'],1.)
            key=self.prepare();_,context=self.request('context/'+key,method='GET')
        plan=context['plan']
        for step in plan['steps']:
            if step['kind']=='cartesian':
                self.assertEqual(step['spd'],.5 if step['stage'] in ('align_above_ball','descend','lower_to_release') else 1.)
            else:
                self.assertEqual(step['spd'],DEFAULT_PARAMETERS['gripper_speed']);self.assertEqual(step['acc'],DEFAULT_PARAMETERS['gripper_acc'])
        self.assertEqual(self.execute_mock.call_count,0);self.assertEqual(self.arm.commands,[])

    def test_previous_plan_is_not_reused_after_speed_change(self):
        key=self.prepare();self.save()
        response,error=self.start_trial(key)
        self.assertEqual(response['status'],409,error);self.assertEqual(self.execute_mock.call_count,0)
        self.assertEqual(self.arm.commands,[])

    def test_speed_changes_preserve_learned_offsets_and_review_chain(self):
        key=self.prepare();self.start_trial(key)
        response,review=self.request('review',{'plan_id':key,'outcome':'grasp_missed','direction':'image_left','source':'human'})
        self.assertEqual(response['status'],200,review)
        prior=json.loads(self.path.read_text());self.save(.9,.45);self.save(1.,.5)
        saved=json.loads(self.path.read_text())
        self.assertEqual(saved['reviews'],prior['reviews']);self.assertEqual(saved['parameters']['pickup_xy_mm'],prior['parameters']['pickup_xy_mm'])
        self.assertEqual(len(saved['baseline_resets']),2)
        self.prepare();_,state=self.request('state',method='GET')
        self.assertTrue(state['plan']['policy_application']['parameters_verified'])
        self.assertEqual(state['plan']['policy_application']['source_trial_id'],key)
        self.assertEqual(state['plan']['policy_application']['baseline_reset_id'],saved['baseline_resets'][-1]['id'])
        self.assertEqual(self.execute_mock.call_count,1)

    def test_same_settings_are_noop_and_stale_version_or_calibration_cannot_overwrite(self):
        current=self.settings()
        self.assertFalse(self.request('speed',current)[1]['changed']);self.assertFalse(self.path.exists())
        self.save();before=self.path.read_bytes()
        self.assertEqual(self.request('speed',current)[0]['status'],409)
        self.assertEqual(self.request('speed',{**self.settings(),'calibration_fingerprint':'wrong'})[0]['status'],409)
        self.assertEqual(self.path.read_bytes(),before);self.assertEqual(self.arm.commands,[])

    def test_pending_or_automation_blocks_settings_without_motion(self):
        body={**self.settings(),'travel_speed':1.}
        for field,value in (('pending',{'kind':'other'}),('automation_owner','other')):
            with patch.object(self.service,field,value):
                response,error=self.request('speed',body)
                self.assertEqual(response['status'],409,error)
        self.assertFalse(self.path.exists());self.assertEqual(self.arm.commands,[])

    def test_invalid_speed_is_rejected_before_policy_write(self):
        body=self.settings()
        for value in (0,1.1,'NaN',True):
            response,error=self.request('speed',{**body,'travel_speed':value})
            self.assertEqual(response['status'],422,error)
        self.assertFalse(self.path.exists());self.assertEqual(self.arm.commands,[])

    def test_failed_save_preserves_active_policy_and_prepared_plan(self):
        self.save();key=self.prepare();before=self.path.read_bytes();write=rsi_trial._write
        def disk_fail(path,value):
            if path==self.path:raise OSError('synthetic disk failure')
            return write(path,value)
        with patch.object(rsi_trial,'_write',side_effect=disk_fail):response,error=self.save(.9,.45)
        self.assertEqual(response['status'],409,error);self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(self.request('state',method='GET')[1]['plan']['status'],'prepared')
        self.assertEqual(self.arm.commands,[])
