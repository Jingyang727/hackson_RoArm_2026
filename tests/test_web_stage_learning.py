"""Web-only learning across a late timeout; all hardware and model replies are synthetic."""
import copy
import json
import math
import unittest
from unittest.mock import MagicMock,patch

from evoarm_rsi.company_api import APISettings
from evoarm_rsi.grasp_once import export_keyframes
from evoarm_rsi.rsi_policy import center_target,project
from tests.test_execution_learning import failed_report
from tests import test_rsi_learning as learning_fixture
from tests.test_rsi_learning import verdict,phase_facts


class WebStageLearning(unittest.TestCase):
    setUp=learning_fixture.AutoAPI.setUp
    request=learning_fixture.AutoAPI.request
    start=learning_fixture.AutoAPI.start

    def setup_failure(self,*,category='time_budget',pickup_complete=True):
        original=self.fixture.execute_mock.side_effect;self.plans=[]
        def execute(service,plan,root,stop,publish,**kwargs):
            self.plans.append(copy.deepcopy(plan))
            result=original(service,plan,root,stop,publish,**kwargs)
            if len(self.plans)==1:
                result.update(failed_report(plan,category))
                result['failure']['stage']='lower_to_release' if category=='time_budget' else 'transport'
                if not pickup_complete:
                    result['events']=[e for e in result['events'] if e['stage']!='lift_completed']
                    result['failure']['stage']='lift'
            return result
        self.fixture.execute_mock.side_effect=execute

    def run_two(self,first):
        with (patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':'场景清晰'},first,verdict()]),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'}) as recovery):
            self.start()
        _,state=self.request('state',method='GET')
        self.assertEqual(state['learning']['status'],'completed',state['learning'].get('error'))
        self.assertEqual(len(self.plans),2);self.assertEqual(recovery.call_count,2)
        file=self.fixture.home/'.evoarm/rsi/pick_place_runs'/state['learning']['trials'][0]['plan_id']/'state.json'
        return json.loads(file.read_text())['review']

    def test_late_timeout_preserves_pickup_evidence_and_image_up_moves_down_in_calibrated_projection(self):
        self.setup_failure();first={**verdict('grasp_missed','image_up'),'adjustment':'pickup_xy'}
        review=self.run_two(first);a,b=self.plans
        self.assertEqual(review['learning_basis']['scope'],'visual_pickup')
        self.assertEqual(review['execution_diagnosis']['category'],'time_budget')
        self.assertEqual(a['parameters']['travel_speed'],b['parameters']['travel_speed'])
        self.assertEqual(a['parameters']['grasp_z_offset_mm'],b['parameters']['grasp_z_offset_mm'])
        before=project(a['scene'],center_target(a['scene'],a['parameters']))
        after=project(a['scene'],center_target(a['scene'],b['parameters']))
        self.assertGreater(after[1],before[1]);self.assertLess(abs(after[0]-before[0]),1.)
        self.assertAlmostEqual(math.dist(a['parameters']['pickup_xy_mm'],b['parameters']['pickup_xy_mm']),12.)

    def test_actual_height_error_changes_height_only_despite_late_transport_timeout(self):
        self.setup_failure();review=self.run_two({**verdict('grasp_missed','too_high'),'adjustment':'grasp_height'})
        self.assertEqual(review['before']['pickup_xy_mm'],review['after']['pickup_xy_mm'])
        self.assertEqual(review['before']['grasp_z_offset_mm']-2,review['after']['grasp_z_offset_mm'])
        self.assertEqual(review['before']['travel_speed'],review['after']['travel_speed'])

    def test_visible_slip_is_not_overridden_by_an_automatic_speed_increase(self):
        self.setup_failure();review=self.run_two({**verdict('slipped'),'adjustment':'transport_slower'})
        self.assertEqual(review['learning_basis']['scope'],'visual_grip')
        self.assertEqual(review['proposal']['parameters_changed'],['travel_speed'])
        self.assertLess(review['after']['travel_speed'],review['before']['travel_speed'])

    def test_early_timeout_selects_and_persists_one_speed_trial_without_guessing_xy(self):
        self.setup_failure(pickup_complete=False)
        review=self.run_two(verdict('execution_failed'))
        self.assertEqual(review['proposal']['parameters_changed'],['travel_speed'])
        self.assertEqual(review['before']['travel_speed'],.65);self.assertEqual(review['after']['travel_speed'],.85)
        self.assertEqual(review['before']['pickup_xy_mm'],review['after']['pickup_xy_mm'])
        self.assertEqual(self.plans[1]['parameters']['travel_speed'],.85)

    def test_clearance_shortfall_selects_measured_lift_trial_and_keeps_required_floor(self):
        self.setup_failure(category='clearance_shortfall')
        review=self.run_two(verdict('execution_failed'))
        self.assertEqual(review['proposal']['parameters_changed'],['lift_height_mm'])
        self.assertEqual(review['after']['lift_height_mm'],126.4)
        self.assertEqual(self.plans[0]['route_policy']['minimum_transfer_height_mm'],self.plans[1]['route_policy']['minimum_transfer_height_mm'])

    def test_web_real_http_client_sends_images_and_keeps_raw_reply_without_an_opencode_subagent(self):
        self.setup_failure();export_keyframes(self.fixture.folder,[],sample_count=6)
        replies=[{'ready':True,'note':'scene'},dict(verdict('grasp_missed','image_up'),adjustment='pickup_xy'),verdict()]
        requests=[]
        def opened(request,**kwargs):
            requests.append(json.loads(request.data));reply=replies[len(requests)-1]
            if 'outcome' in reply:
                metadata=json.loads(requests[-1]['messages'][-1]['content'][0]['text'])
                frame=next(f['frame_index'] for f in metadata['reviewed_frames'] if f.get('role')=='current')
                reply={**reply,'phases':phase_facts(reply['outcome'],frame)}
            response=MagicMock();response.__enter__.return_value=response;response.headers={'Content-Type':'application/json'}
            response.read.return_value=json.dumps({'choices':[{'finish_reason':'stop','message':{'content':json.dumps(reply)}}]}).encode()
            return response
        opener=MagicMock();opener.open.side_effect=opened
        with (patch('evoarm_rsi.company_api.settings',return_value=APISettings('https://company.invalid/v1','fake-token','grok-test')),
              patch('evoarm_rsi.grok_review.configuration',return_value={'configured':True,'backend':'company_api','model':'grok-test'}),
              patch('evoarm_rsi.company_api.request.build_opener',return_value=opener),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'})):
            self.start()
        _,state=self.request('state',method='GET')
        self.assertEqual(state['learning']['status'],'completed',state['learning'].get('error'))
        self.assertEqual(len(requests),3)
        for request in requests:
            images=[p for p in request['messages'][1]['content'] if p['type']=='image_url']
            self.assertGreater(len(images),0)
            self.assertTrue(all(p['image_url']['url'].startswith('data:image/jpeg;base64,') for p in images))
        saved=self.fixture.home/'.evoarm/rsi/pick_place_runs'/state['learning']['trials'][0]['plan_id']
        self.assertEqual(json.loads((saved/'grok-review.json').read_text())['verdict']['direction'],'image_up')
        self.assertIn('grasp_missed',(saved/'grok-output.txt').read_text())

    def test_candidate_that_fails_full_ik_is_not_saved_or_retried(self):
        self.setup_failure();from evoarm_rsi.pick_place import build_plan
        def checked(saved,fresh,scene,parameters):
            if parameters['pickup_xy_mm']!=[0.,0.]:raise ValueError('synthetic candidate unreachable')
            return build_plan(saved,fresh,scene,parameters)
        with (patch('control_ui.rsi_trial.build_plan',side_effect=checked),
              patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':''},verdict('grasp_missed','image_up')]),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'}) as recovery):
            self.start()
        _,state=self.request('state',method='GET');entry=state['plan']['review']
        self.assertEqual(len(self.plans),1);self.assertEqual(recovery.call_count,1)
        self.assertEqual(entry['before'],entry['after']);self.assertFalse(entry['proposal']['allow_retry'])
        self.assertNotIn('robot_delta_mm',entry['proposal'])
        self.assertIn('synthetic candidate unreachable',state['learning']['error'])
