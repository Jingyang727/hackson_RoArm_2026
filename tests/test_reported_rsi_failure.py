"""The reported layout must work without relaxing motor/plane constraints."""
import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from control_ui import rsi_trial
from evoarm_rsi.agent_tools import invoke,parser
from evoarm_rsi.ball_trial import check_segment
from evoarm_rsi.pick_place import build_plan,DEFAULT_PARAMETERS,_opening_direction
from evoarm_rsi.roarm_geometry import inverse,MotionConstraintError,height_above_plane
from tests import test_rsi_loop as fixtures

DATA=json.loads((Path(__file__).parent/'fixtures/reported_rsi_layout.json').read_text())


def world(case):
    return {'ball':{'center_robot_mm':case['ball_center_mm']},
        'funnel':{'mouth_center_robot_mm':case['funnel_center_mm']},
        'table_plane':case['table_plane'],'warnings':[]}


class RecordedLayout(unittest.TestCase):
    def verify_path(self,plan):
        current=plan['initial_status'];plane=plan['table_plane']
        for step in plan['steps']:
            if step['kind']=='gripper':current={**current,'g':step['gripper_rad']};continue
            check_segment(current,step,plane)
            if step['vertical_only']:np.testing.assert_allclose(step['xyz_mm'][:2],[current[k] for k in 'xy'],atol=1e-6)
            else:
                floor=plan['route_policy']['minimum_transfer_height_mm']
                self.assertGreaterEqual(height_above_plane([current[k] for k in 'xyz'],plane),floor-.5)
                self.assertGreaterEqual(height_above_plane(step['xyz_mm'],plane),floor-.5)
            current=inverse(step['xyz_mm'],step['pitch_rad'],step['roll_rad'],step['gripper_rad'])

    def test_both_failed_observations_retain_vertical_grasp_and_use_reachable_transport(self):
        for case in DATA['cases']:
            p=copy.deepcopy(DEFAULT_PARAMETERS)
            plan=build_plan({'setup_id':'recorded-layout'},case['initial_status'],world(case),p)
            self.assertEqual(plan['grasp_pitch_deg'],80.)
            self.assertLess(plan['release_pitch_deg'],plan['grasp_pitch_deg'])
            self.assertEqual(plan['parameters'],DEFAULT_PARAMETERS)
            self.verify_path(plan)
            names=[s['stage'] for s in plan['steps']]
            self.assertLess(names.index('descend'),names.index('close'))
            self.assertLess(names.index('close'),names.index('lift'))
            self.assertEqual(names[-2:],['lower_to_release','release'])

    def test_changed_drop_pitch_recalculates_fingertip_to_keep_ball_over_mouth(self):
        case=DATA['cases'][0];plan=build_plan({'setup_id':'replay'},case['initial_status'],world(case),copy.deepcopy(DEFAULT_PARAMETERS))
        roll=case['initial_status']['r'];pitch=math.radians(plan['release_pitch_deg'])
        opening=_opening_direction(case['funnel_center_mm'],pitch,roll)
        center=np.asarray(plan['release_tip_xyz_mm'])+opening*DEFAULT_PARAMETERS['finger_offset_mm']
        np.testing.assert_allclose(center,plan['object_geometry']['release_ball_center_xyz_mm'],atol=1e-7)
        expected=np.asarray(case['funnel_center_mm'])+np.asarray(case['table_plane']['normal_up'])*35.
        np.testing.assert_allclose(center,expected,atol=1e-7)

    def test_later_coordinate_pose_is_not_confused_with_failure_time_feedback(self):
        case=DATA['cases'][0];raw=DATA['later_coordinate_status']
        plan=build_plan({'setup_id':'replay'},raw,world(case),copy.deepcopy(DEFAULT_PARAMETERS))
        self.verify_path(plan)

    def test_no_candidate_reports_stage_xyz_and_search_scope(self):
        case=DATA['cases'][0];scene=world(case);scene['funnel']['mouth_center_robot_mm']=[9999.,0.,-75.]
        with self.assertRaises(MotionConstraintError) as caught:
            build_plan({'setup_id':'replay'},case['initial_status'],scene,copy.deepcopy(DEFAULT_PARAMETERS))
        detail=caught.exception.detail
        self.assertEqual(detail['code'],'pick_place_search_exhausted')
        self.assertTrue(detail['attempts'])
        for attempt in detail['attempts']:
            self.assertIn('stage',attempt);self.assertEqual(len(attempt['xyz_mm']),3)


class StartupContract(unittest.TestCase):
    setUp=fixtures.RSIApi.setUp
    request=fixtures.RSIApi.request
    prepare=fixtures.RSIApi.prepare
    start_trial=fixtures.RSIApi.start_trial

    def test_fresh_locked_disarmed_feedback_is_ready_and_run_requests_authorization(self):
        self.service.disarm()
        response,ready=self.request('agent/readiness',method='GET')
        self.assertEqual(response['status'],200,ready)
        self.assertTrue(ready['ready_to_prepare']);self.assertTrue(ready['execution_enabled'])
        self.assertFalse(ready['health']['armed']);self.assertEqual(self.service.armed_until,0)
        self.assertFalse(ready['authorization_contract']['manual_unlock_button_required'])
        self.assertTrue(ready['feedback_verified']);self.assertEqual(ready['robot']['T'],1051)
        self.assertEqual(self.arm.commands,[])
        key=self.prepare();prior=self.execute_mock.side_effect
        def execute_armed(service,*args,**kwargs):
            self.assertTrue(service.health()['armed']);return prior(service,*args,**kwargs)
        self.execute_mock.side_effect=execute_armed
        with patch.object(self.service,'unlock_once',wraps=self.service.unlock_once) as unlock:
            self.assertEqual(self.start_trial(key)[0]['status'],202);self.assertEqual(unlock.call_count,1)

    def test_readiness_does_not_connect_or_hide_a_feedback_failure(self):
        with patch.object(self.service,'status',side_effect=OSError('fresh status unavailable')):
            _,ready=self.request('agent/readiness',method='GET')
        self.assertFalse(ready['feedback_verified']);self.assertFalse(ready['ready_to_prepare'])
        self.service.arm=None
        with patch.object(self.service,'connect',side_effect=AssertionError('do not connect')):
            _,ready=self.request('agent/readiness',method='GET')
        self.assertFalse(ready['health']['connected']);self.assertFalse(ready['ready_to_prepare'])
        self.assertEqual(self.arm.commands,[])

    def test_rejected_planning_preserves_actual_inputs_before_any_motion(self):
        rejection=MotionConstraintError('no candidate',code='pick_place_search_exhausted',attempts=[
            {'stage':'transport','xyz_mm':[440.,0.,0.],'pitch_deg':80.,'message':'endpoint cannot be reached'}])
        with patch.object(rsi_trial,'build_plan',side_effect=rejection):
            response,error=self.request('prepare')
        self.assertEqual(response['status'],409)
        self.assertEqual(error['detail']['code'],'pick_place_search_exhausted')
        key=error['detail']['image_url'].split('/')[-2]
        folder=self.home/'.evoarm/rsi/pick_place_runs'/key
        inputs=json.loads((folder/'planning-input.json').read_text())
        self.assertEqual(inputs['initial_status']['x'],self.arm.status()['x'])
        self.assertEqual(inputs['parameters'],DEFAULT_PARAMETERS)
        self.assertEqual(json.loads((folder/'calibration-snapshot.json').read_text()),self.session)
        self.assertEqual(json.loads((folder/'prepare-error.json').read_text())['detail']['attempts'][0]['stage'],'transport')
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.execute_mock.call_count,0)


class OpenCodeEntry(unittest.TestCase):
    def test_readiness_tool_identifies_the_same_controller_and_client_roots(self):
        from unittest.mock import Mock
        api=Mock();api.url='http://127.0.0.1:8765';api.request.return_value={'ready_to_prepare':True}
        value=invoke(parser().parse_args(['readiness']),api)
        api.request.assert_called_once_with('/agent/readiness')
        self.assertEqual(value['controller_url'],api.url)
        self.assertEqual(value['client_project_root'],str(Path(__file__).resolve().parents[1]))
