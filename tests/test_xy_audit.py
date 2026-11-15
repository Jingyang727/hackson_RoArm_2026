"""Reported evidence must distinguish cumulative policy, raw motion and attribution."""
import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from evoarm_rsi.xy_audit import audit
from tests import test_rsi_loop as fixtures


def reported_states():
    data=json.loads((Path(__file__).parent/'fixtures/reported_xy_trials.json').read_text())['trials']
    return [{'id':r['id'],'status':'failed','plan':{'parameters':{'pickup_xy_mm':r['used']},
        'scene':{'ball':{'center_robot_mm':r['ball']}},'grasp_pitch_deg':r['pitch'],'grasp_tip_xyz_mm':r['goal']},
        'result':{'feedback':[{'stage':stage,'status':dict(zip('xyz',r[key]))} for stage,key in
            (('approach_ball','approach'),('close','close'))],
            'dispatches':[{'stage':'descend','command':{'T':104,**dict(zip('xyz',r['descend']))}}]},
        'review':{'after':{'pickup_xy_mm':r['next']},'direction':r['direction'],'proposal':{'decision':r['decision']}}
        if r['next'] is not None else None} for r in data]


class ReportedAudit(unittest.TestCase):
    def test_supplied_four_rounds_contain_only_one_positive_x_update_not_four(self):
        rows=audit(reported_states())
        np.testing.assert_allclose(rows[1]['versus_previous']['used_compensation_delta_mm'],[-.094,3.999])
        np.testing.assert_allclose(rows[2]['versus_previous']['used_compensation_delta_mm'],[0.,0.])
        np.testing.assert_allclose(rows[3]['versus_previous']['used_compensation_delta_mm'],[3.995,.213])
        self.assertIsNone(rows[-1]['saved_for_next_pickup_xy_mm'])
        self.assertIsNone(rows[-1]['descend_feedback_xyz_mm'])

    def test_actual_four_mm_update_reaches_both_nominal_plan_and_raw_close_feedback(self):
        last=audit(reported_states())[-1];pair=last['versus_previous']
        np.testing.assert_allclose(pair['nominal_grasp_delta_xyz_mm'],[3.932,.419,.026])
        np.testing.assert_allclose(pair['raw_close_delta_xyz_mm'],[3.395,1.014,-.448])
        np.testing.assert_allclose(pair['baseline_adjusted_feedback_delta_mm'],[3.458,.808],atol=1e-9)
        np.testing.assert_allclose(last['close_minus_descend_command_mm'],[-5.655,-1.557,-9.205])
        self.assertAlmostEqual(math.hypot(*last['descend_minus_nominal_grasp_mm'][:2]),9.782,places=3)

    def test_adjusted_motion_can_appear_positive_even_when_absolute_feedback_is_unchanged(self):
        states=reported_states()[2:];states[1]=copy.deepcopy(states[0]);states[1]['id']='synthetic-cancellation'
        states[1]['plan']['parameters']['pickup_xy_mm'][0]+=10
        rows=audit(states);pair=rows[1]['versus_previous']
        self.assertEqual(pair['raw_close_delta_xyz_mm'],[0.,0.,0.])
        self.assertEqual(pair['nominal_grasp_delta_xyz_mm'],[0.,0.,0.])
        self.assertEqual(pair['localization_and_pose_baseline_delta_mm'],[-10.,0.])
        self.assertEqual(pair['baseline_adjusted_feedback_delta_mm'],[10.,0.])


class ReadOnlyAuditAPI(unittest.TestCase):
    setUp=fixtures.RSIApi.setUp
    request=fixtures.RSIApi.request
    prepare=fixtures.RSIApi.prepare
    start_trial=fixtures.RSIApi.start_trial

    def test_four_new_trials_use_zero_ten_twenty_thirty_and_save_forty_for_next_trial(self):
        keys=[]
        for _ in range(4):
            key=self.prepare();keys.append(key);self.start_trial(key)
            response,entry=self.request('review',{'plan_id':key,'source':'human','outcome':'grasp_missed',
                'direction':'image_up','magnitude':'small','ball_state':'table'})
            self.assertEqual(response['status'],200,entry)
        calls=self.execute_mock.call_count
        with patch.object(self.service,'status',side_effect=AssertionError('audit must not sample robot')):
            response,value=self.request('diagnostics/xy',method='GET')
        self.assertEqual(response['status'],200,value);self.assertFalse(value['robot_sampled'])
        self.assertEqual([r['plan_id'] for r in value['trials']],keys)
        rows=value['trials'];self.assertEqual(len(rows),4)
        self.assertEqual([round(math.hypot(*r['used_pickup_xy_mm'])) for r in rows],[0,10,20,30])
        self.assertAlmostEqual(math.hypot(*rows[-1]['saved_for_next_pickup_xy_mm']),40.)
        for row in rows:self.assertIsNone(row['descend_command_xyz_mm'])
        for row in rows[1:]:
            pair=row['versus_previous']
            np.testing.assert_allclose(pair['nominal_grasp_delta_xyz_mm'][:2],pair['used_compensation_delta_mm'],atol=.01)
        self.assertEqual(self.execute_mock.call_count,calls);self.assertEqual(self.arm.commands,[])
