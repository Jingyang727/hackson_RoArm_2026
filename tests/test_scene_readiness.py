"""Home-pose gripper outside RGB must not veto visible pickup/place targets."""
import json
import unittest
from unittest.mock import patch

from evoarm_rsi import grok_review
from evoarm_rsi.grasp_once import export_keyframes
from tests import test_rsi_learning as fixtures


def facts(**changes):
    return {'ball_visible':True,'funnel_visible':True,'gripper_state':'unknown',
            'note':'黄球和漏斗开口清晰；夹爪在画面外。',**changes}


class SceneFacts(unittest.TestCase):
    def test_hidden_or_occluded_gripper_is_unknown_not_confirmed_empty(self):
        value=facts(ready=False)
        result=grok_review._final_json(json.dumps(value),preflight=True)
        self.assertTrue(result['ready']);self.assertEqual(result['gripper_state'],'unknown')
        self.assertFalse(value['ready'])
        self.assertEqual(grok_review._scene_verdict(result),result)

    def test_missing_target_or_positively_held_ball_still_blocks_even_if_model_says_ready(self):
        for change in ({'ball_visible':False},{'funnel_visible':False},{'gripper_state':'holding_ball'}):
            with self.subTest(change=change):
                result=grok_review._scene_verdict(facts(ready=True,**change))
                self.assertFalse(result['ready'])

    def test_visible_empty_gripper_is_allowed_but_not_required(self):
        self.assertTrue(grok_review._scene_verdict(facts(gripper_state='empty'))['ready'])

    def test_missing_malformed_or_extra_facts_cannot_assert_readiness(self):
        malformed=[facts(ball_visible='true'),facts(funnel_visible=1),facts(gripper_state=[]),
                   facts(gripper_state='not_visible'),facts(ready=1),facts(command={'T':100})]
        partial=facts();partial.pop('ball_visible');malformed.append(partial)
        for value in malformed:
            with self.subTest(value=value),self.assertRaises(ValueError):grok_review._scene_verdict(value)
        self.assertEqual(grok_review._scene_verdict({'ready':False,'note':'wrong funnel'}),
                         {'ready':False,'note':'wrong funnel'})

    def test_home_pose_rejection_does_not_prevent_a_trial_when_both_targets_are_visible(self):
        case=fixtures.AutoAPI();case.setUp();self.addCleanup(case.doCleanups)
        export_keyframes(case.fixture.folder,[],sample_count=6)
        observed=[]
        def model(system,content,stop,**kwargs):
            observed.append(system)
            if system==grok_review.SCENE_SYSTEM:return json.dumps(facts(ready=False))
            frames=json.loads(content[0]['text'])['reviewed_frames']
            return json.dumps({**fixtures.verdict(),'phases':fixtures.phase_facts('placed',frames[-1]['frame_index'])})
        with (patch.object(grok_review,'configuration',return_value={'configured':True,'model':'DeepSeek-V4.1-Flash','backend':'company_api'}),
              patch.object(grok_review,'complete_content',side_effect=model),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'})):
            case.start()
        _,state=case.request('state',method='GET')
        self.assertEqual(len(case.policies),1)
        self.assertEqual(state['learning']['phase'],'completed',state['learning'].get('error'))
        self.assertEqual(state['plan']['review']['phases']['release']['result'],'entered')
        scene=json.loads((case.fixture.home/'.evoarm/rsi/pick_place_runs'/state['learning']['trials'][0]['plan_id']/'grok-scene-review.json').read_text())
        self.assertEqual(scene['verdict']['gripper_state'],'unknown');self.assertTrue(scene['verdict']['ready'])

    def test_held_ball_and_missing_targets_are_not_bypassed_by_the_learning_loop(self):
        for change in ({'ball_visible':False},{'funnel_visible':False},{'gripper_state':'holding_ball'}):
            with self.subTest(change=change):
                case=fixtures.AutoAPI();case.setUp()
                try:
                    with (patch.object(grok_review,'configuration',return_value={'configured':True,'model':'DeepSeek-V4.1-Flash','backend':'company_api'}),
                          patch.object(grok_review,'complete_content',return_value=json.dumps(facts(ready=True,**change))),
                          patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'})):
                        case.start()
                    self.assertEqual(case.policies,[])
                finally:case.doCleanups()
