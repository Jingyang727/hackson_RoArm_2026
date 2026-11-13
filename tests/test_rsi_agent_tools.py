"""OpenCode tools reuse the current server and preserve trial/calibration scope."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch

from evoarm_rsi.agent_tools import Controller,invoke,objects,parser
from evoarm_rsi.coordinate_test import fingerprint
from tests import test_rsi_loop as loop_fixtures
from tests.test_rsi_vector_repair import legacy_policy


class Tools(unittest.TestCase):
    def test_readonly_tools_do_not_send_post(self):
        for command,path in (('state','/state'),('context','/agent/context'),('history','/history'),('coordinate-evidence','/agent/coordinate-evidence')):
            api=Mock();invoke(parser().parse_args([command]),api);api.request.assert_called_once_with(path)

    def test_motion_requires_explicit_execution_scope_and_no_auto_reissue(self):
        for command in (['run','a'*32],['reset']):
            api=Mock()
            with self.assertRaises(ValueError):invoke(parser().parse_args(command),api)
            api.request.assert_not_called()
            invoke(parser().parse_args(command+['--execute']),api)
            self.assertEqual(api.request.call_count,1)
        api=Mock();api.request.side_effect=ValueError('timeout')
        with self.assertRaises(ValueError):invoke(parser().parse_args(['run','a'*32,'--execute']),api)
        self.assertEqual(api.request.call_count,1)

    def test_review_inline_supports_agent_without_file_editing(self):
        api=Mock();payload={'plan_id':'a'*32,'outcome':'grasp_missed','source':'human','requested_step_mm':12}
        invoke(parser().parse_args(['review','--json',json.dumps(payload)]),api)
        sent=api.request.call_args.args
        self.assertEqual(sent[0],'/review');self.assertEqual(sent[1]['source'],'opencode_agent')
        self.assertEqual(sent[1]['requested_step_mm'],12)

    def test_existing_service_only_and_missing_observation_explicit(self):
        for url in ('https://example.com','http://127.0.0.1.evil.test','http://secret@localhost','http://localhost/api'):
            with self.assertRaises(ValueError):Controller(url)
        self.assertFalse(objects({'plan':None})['available'])


class AgentAPI(unittest.TestCase):
    setUp=loop_fixtures.RSIApi.setUp
    request=loop_fixtures.RSIApi.request
    prepare=loop_fixtures.RSIApi.prepare
    start_trial=loop_fixtures.RSIApi.start_trial

    def test_agent_context_and_memory_persist_without_robot_commands_or_api_config(self):
        response,context=self.request('agent/context',method='GET');self.assertEqual(response['status'],200)
        self.assertEqual(context['direction_contract']['observed_side_to_correction']['image_right'],'image_left')
        self.assertEqual(context['memory']['notes'],[])
        self.assertEqual(self.request('agent/memory',{'note':'原图右偏应向原图左纠正'})[0]['status'],200)
        _,context=self.request('agent/context',method='GET');self.assertEqual(len(context['memory']['notes']),1)
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.execute_mock.call_count,0)

    def test_explicit_step_wrong_mirror_and_visual_numeric_values_rejected(self):
        key=self.prepare();self.start_trial(key)
        body={'plan_id':key,'outcome':'grasp_missed','direction':'image_right','correction_direction':'image_right',
              'adjustment':'pickup_xy','requested_step_mm':12,'source':'opencode_agent'}
        self.assertEqual(self.request('review',body)[0]['status'],409)
        body.update(correction_direction='image_left',source='company_grok')
        self.assertEqual(self.request('review',body)[0]['status'],409)
        body['source']='opencode_agent';response,review=self.request('review',body)
        self.assertEqual(response['status'],200,review)
        self.assertAlmostEqual(review['proposal']['saved_xy_step_mm'],12.)
        self.assertEqual(self.execute_mock.call_count,1);self.assertEqual(self.arm.commands,[])
        path=self.home/'.evoarm/rsi/pick_place_runs'/key/'agent-review.json'
        self.assertTrue(json.loads(path.read_text())['ik_preflight'])

    def test_unreachable_agent_step_not_persisted_and_never_moves_robot(self):
        key=self.prepare();self.start_trial(key)
        body={'plan_id':key,'outcome':'grasp_missed','direction':'image_right','adjustment':'pickup_xy',
              'requested_step_mm':10000,'source':'opencode_agent'}
        self.assertEqual(self.request('review',body)[0]['status'],409)
        _,context=self.request('agent/context',method='GET')
        self.assertEqual(context['policy']['version'],1);self.assertEqual(context['policy']['reviews'],[])
        self.assertEqual(self.arm.commands,[])

    def test_legacy_repair_only_on_prepare_with_backup_and_explicit_lineage(self):
        path=self.home/'.evoarm/rsi/pick_place_policies'/self.session['setup_id']/(fingerprint(self.session)+'.json')
        path.parent.mkdir(parents=True);original=legacy_policy();path.write_text(json.dumps(original))
        self.request('agent/context',method='GET');self.assertEqual(json.loads(path.read_text()),original)
        self.prepare();new=json.loads(path.read_text());reset=new['baseline_resets'][-1]
        self.assertEqual(json.loads(Path(reset['backup_file']).read_text()),original)
        self.assertEqual(new['parameters']['pickup_xy_mm'],[0.,0.])
        _,state=self.request('state',method='GET')
        self.assertEqual(state['plan']['policy_application']['baseline_reset_id'],reset['id'])
        self.assertEqual(self.arm.commands,[])

    def test_coordinate_evidence_reads_persisted_commands_and_feedback_without_sampling(self):
        folder=self.home/'.evoarm/rsi/coordinate_runs'/('e'*32);folder.mkdir(parents=True)
        report={'plan_id':'e'*32,'status':'completed','target_xyz_mm':[338.48,-70,-70],
                'actual_xyz_mm':[338.,-69.,-70.],'actual_minus_target_mm':[-.48,1.,0.],
                'plan':{'pose_preserved_rad':{'pitch':1.4},'calibration_fingerprint':fingerprint(self.session)}}
        (folder/'report.json').write_text(json.dumps(report))
        with patch.object(self.service,'status',side_effect=AssertionError('read stored records only')):
            response,data=self.request('agent/coordinate-evidence',method='GET')
        self.assertEqual(response['status'],200,data);self.assertFalse(data['robot_sampled'])
        self.assertEqual(data['coordinate_reports'][0]['actual_xyz_mm'],report['actual_xyz_mm'])
        self.assertEqual(self.arm.commands,[])
