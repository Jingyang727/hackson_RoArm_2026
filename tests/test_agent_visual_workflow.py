"""OpenCode controls; the visual task sees images and qualitative context only."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch

from evoarm_rsi.agent_tools import invoke,parser
from evoarm_rsi.grasp_once import export_keyframes
from tests import test_rsi_loop as fixtures
from control_ui.rsi_trial import DECLARATION


def verdict(direction='image_right'):
    return {'outcome':'grasp_missed','direction':direction,'ball_state':'table','magnitude':'medium',
            'adjustment':'grasp_height' if direction=='too_high' else 'pickup_xy',
            'confidence':.8,'note':'实际闭爪与提升帧显示球仍在桌面',
            'comparison':{'reference_plan_id':None,'effect':'not_compared','movement':'unknown','note':'首轮'}}


class Workflow(unittest.TestCase):
    setUp=fixtures.RSIApi.setUp
    request=fixtures.RSIApi.request
    prepare=fixtures.RSIApi.prepare
    start_trial=fixtures.RSIApi.start_trial

    def visual(self,key):
        export_keyframes(self.folder,[],sample_count=3)
        response,packet=self.request('vision-context/'+key,method='GET')
        self.assertEqual(response['status'],200,packet);return packet

    def test_scene_packet_has_no_robot_geometry_or_strategy_and_no_motion(self):
        key=self.prepare();packet=self.visual(key);self.assertEqual(packet['phase'],'scene')
        encoded=json.dumps(packet)
        for field in ('center_robot_mm','camera_geometry','parameters','grasp_pitch','robot_xyz','board_to_robot'):
            self.assertNotIn(field,encoded)
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.execute_mock.call_count,0)

    def test_bound_visual_result_and_agent_step_enter_policy_with_provenance(self):
        key=self.prepare();self.start_trial(key);packet=self.visual(key);visual=verdict()
        body={**visual,'plan_id':key,'source':'opencode_agent','note':'选择12mm可见试验，不是模型测量',
              'visual_review':visual,'vision_packet_id':packet['vision_packet_id'],'requested_step_mm':12.}
        response,result=self.request('review',body);self.assertEqual(response['status'],200,result)
        self.assertEqual(result['vision_packet_id'],packet['vision_packet_id'])
        self.assertEqual(result['visual_review'],visual);self.assertAlmostEqual(result['proposal']['saved_xy_step_mm'],12.)
        self.assertEqual(self.execute_mock.call_count,1);self.assertEqual(self.arm.commands,[])
        _,state=self.request('state',method='GET')
        self.assertIn('grok_validated',[row['kind'] for row in state['process']['events']])
        self.assertIn('policy',[row['kind'] for row in state['process']['events']])

    def test_false_scene_review_cannot_move_robot_and_true_review_is_bound_to_scene_packet(self):
        key=self.prepare();packet=self.visual(key)
        body={'plan_id':key,'declaration':DECLARATION,'vision_packet_id':packet['vision_packet_id'],
              'scene_review':{'ready':False,'note':'标错漏斗'}}
        self.assertEqual(self.request('run',body)[0]['status'],409);self.assertEqual(self.execute_mock.call_count,0)
        body['scene_review']={'ready':True,'note':'球和漏斗开口可见'}
        self.assertEqual(self.request('run',body)[0]['status'],202);self.assertEqual(self.execute_mock.call_count,1)

    def test_scene_packet_cannot_prove_result_and_parent_cannot_rewrite_observation(self):
        key=self.prepare();scene_packet=self.visual(key);self.start_trial(key)
        visual=verdict();body={**visual,'plan_id':key,'source':'opencode_agent','visual_review':visual,
                              'vision_packet_id':scene_packet['vision_packet_id']}
        self.assertEqual(self.request('review',body)[0]['status'],409)
        packet=self.visual(key);body.update(vision_packet_id=packet['vision_packet_id'],direction='image_left')
        self.assertEqual(self.request('review',body)[0]['status'],409)

    def test_agent_can_choose_one_scalar_trial_after_qualitative_height_error(self):
        key=self.prepare();self.start_trial(key);packet=self.visual(key);visual=verdict('too_high')
        body={**visual,'plan_id':key,'source':'opencode_agent','visual_review':visual,'vision_packet_id':packet['vision_packet_id'],
              'parameter_trial':{'parameter':'grasp_z_offset_mm','value':0.},'note':'只降低夹持中心3mm再看实帧'}
        response,result=self.request('review',body);self.assertEqual(response['status'],200,result)
        self.assertEqual(result['proposal']['parameters_changed'],['grasp_z_offset_mm'])
        self.assertEqual(result['after']['grasp_z_offset_mm'],0.);self.assertEqual(self.arm.commands,[])


class DecisionTool(unittest.TestCase):
    def test_decision_derives_observation_from_visual_result_and_requires_reason(self):
        api=Mock();value={'plan_id':'a'*32,'vision_packet_id':'b'*16,'visual_review':verdict(),
                         'reason':'看完前后图后选择12mm','requested_step_mm':12}
        invoke(parser().parse_args(['decision','--json',json.dumps(value)]),api)
        payload=api.request.call_args.args[1]
        self.assertEqual(payload['direction'],'image_right');self.assertEqual(payload['source'],'opencode_agent')
        value['visual_review']['delta_mm']=12
        with self.assertRaises(ValueError):invoke(parser().parse_args(['decision','--json',json.dumps(value)]),Mock())

    def test_visual_download_is_checked_and_contains_only_image_task_context(self):
        data=b'image-data';packet={'plan_id':'a'*32,'vision_packet_id':'b'*16,'phase':'result','instructions':'只看图',
            'images':[{'url':'/api/bench/rsi/plans/'+('a'*32)+'/vision-'+('b'*16)+'-0','sha256':hashlib.sha256(data).hexdigest()}]}
        api=Mock();api.request.side_effect=[packet,data]
        with tempfile.TemporaryDirectory() as tmp,patch.object(Path,'cwd',return_value=Path(tmp)):
            result=invoke(parser().parse_args(['vision','a'*32]),api)
            self.assertTrue(Path(result['images'][0]['local_path']).is_file())
            self.assertNotIn('camera_geometry',result['vision_task'])
