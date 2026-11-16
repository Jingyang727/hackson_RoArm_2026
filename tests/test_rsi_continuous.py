"""Same-policy retries and continuous learning, with simulated devices only."""
import json
import unittest
from unittest.mock import patch

from evoarm_rsi.pick_place import DEFAULT_PARAMETERS
from evoarm_rsi.rsi_learning import LEARNING_DECLARATION
from tests import test_rsi_learning as fixtures


class ContinuousAPI(unittest.TestCase):
    setUp=fixtures.AutoAPI.setUp
    request=fixtures.AutoAPI.request

    def start(self,limit=None):
        return self.request('learning/start',{'declaration':LEARNING_DECLARATION,'max_attempts':limit})

    def test_default_is_continuous_and_finite_limits_are_not_capped_at_twenty(self):
        with patch('evoarm_rsi.rsi_learning._launch_learning'):
            response,answer=self.request('learning/start',{'declaration':LEARNING_DECLARATION})
            self.assertEqual(response['status'],202,answer);self.assertIsNone(answer['max_attempts'])
            _,state=self.request('state',method='GET')
            self.assertTrue(state['learning']['continuous']);self.assertEqual(state['continuous_learning_version'],1)
        # Fresh service for a separate requested task, not an automatic restart.
        other=fixtures.AutoAPI();other.setUp();self.addCleanup(other.doCleanups)
        with patch('evoarm_rsi.rsi_learning._launch_learning'):
            response,answer=other.request('learning/start',{'declaration':LEARNING_DECLARATION,'max_attempts':1000})
            self.assertEqual(response['status'],202,answer);self.assertEqual(answer['max_attempts'],1000)

    def test_continuous_crosses_five_rounds_with_identical_parameters_and_stops_on_success(self):
        replies=[{'ready':True,'note':'scene'}]+[fixtures.verdict('grasp_missed','none')]*11+[fixtures.verdict()]
        with (patch('evoarm_rsi.grok_review.review',side_effect=replies),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'}) as recovery):
            response,answer=self.start()
        self.assertEqual(response['status'],202,answer);self.assertEqual(len(self.policies),12)
        self.assertEqual(recovery.call_count,12);self.assertTrue(all(p==DEFAULT_PARAMETERS for p in self.policies))
        _,state=self.request('state',method='GET');self.assertEqual(state['learning']['phase'],'completed')
        self.assertEqual(state['learning']['trial_count'],12);self.assertIsNone(state['learning']['max_attempts'])
        self.assertIsNone(self.fixture.service.automation_owner)
        self.assertEqual(self.fixture.execute_mock.call_count,12)
        first=state['learning']['trials'][0]['plan_id']
        response,context=self.request('context/'+first,method='GET')
        self.assertEqual(response['status'],200,context);self.assertEqual(context['plan_id'],first)
        self.assertEqual(self.fixture.execute_mock.call_count,12)
        events=state['process']['events'];self.assertTrue(any(e.get('phase')=='retrying' for e in events))

    def test_stop_during_review_prevents_another_attempt_and_leaves_controller_idle(self):
        def model(context,root,stop,**kwargs):
            if kwargs.get('preflight'):return {'ready':True,'note':'scene'}
            if len(self.policies)==3:stop.set()
            return fixtures.verdict('grasp_missed','none')
        with (patch('evoarm_rsi.grok_review.review',side_effect=model),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'})):
            self.start()
        self.assertEqual(len(self.policies),3)
        _,state=self.request('state',method='GET');self.assertEqual(state['learning']['status'],'stopped')
        self.assertIsNone(self.fixture.service.automation_owner);self.assertIsNone(self.fixture.service.pending)

    def test_rejected_visual_candidate_keeps_baseline_and_does_not_stop_completed_trials(self):
        from evoarm_rsi.pick_place import build_plan
        def checked(saved,fresh,scene,parameters):
            if parameters['pickup_xy_mm']!=[0.,0.]:raise ValueError('synthetic new candidate unreachable')
            return build_plan(saved,fresh,scene,parameters)
        with (patch('control_ui.rsi_trial.build_plan',side_effect=checked),
              patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':'scene'}]+[fixtures.verdict('grasp_missed','image_right')]*3),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'})):
            self.start(3)
        self.assertEqual(len(self.policies),3);self.assertTrue(all(p==DEFAULT_PARAMETERS for p in self.policies))
        _,state=self.request('state',method='GET');self.assertEqual(state['learning']['phase'],'exhausted')
        self.assertEqual(state['plan']['review']['proposal']['retry_mode'],'same_parameters')
        self.assertIn('未通过完整路径检查',state['plan']['review']['changes'][0])

    def test_controller_feedback_failure_stops_even_in_continuous_mode(self):
        original=self.fixture.execute_mock.side_effect
        def execute(*args,**kwargs):
            result=original(*args,**kwargs)
            return {**result,'motion_completed':False,'motion_error':'serial feedback timeout'}
        with (patch.object(self.fixture.execute_mock,'side_effect',execute),
              patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':'scene'},fixtures.verdict('unknown')]),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'})):
            self.start()
        self.assertEqual(len(self.policies),1)
        _,state=self.request('state',method='GET');self.assertEqual(state['learning']['status'],'failed')
        self.assertIn('feedback timeout',state['learning']['error'])

    def test_invalid_round_counts_do_not_launch(self):
        for value in (0,-1,True,2.5,'10'):
            response,error=self.start(value);self.assertEqual(response['status'],422,error)
        self.assertEqual(self.policies,[])
