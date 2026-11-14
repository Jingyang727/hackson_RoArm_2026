"""Observed failures must yield usable, bounded Agent decisions, not replays."""
import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import Mock,patch

from control_ui.rsi_trial import DECLARATION
from evoarm_rsi.agent_summary import summarize
from evoarm_rsi.agent_tools import invoke,parser
from evoarm_rsi.execution_diagnosis import diagnose
from evoarm_rsi.grasp_once import export_keyframes
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS,build_plan
from evoarm_rsi.roarm_geometry import height_above_plane,inverse
from evoarm_rsi.rsi_route import at_height
from tests import test_rsi_loop as fixtures
from tests.test_reported_rsi_failure import DATA,world


def visual():
    return {'outcome':'execution_failed','direction':'none','ball_state':'unknown','magnitude':'unknown',
        'adjustment':'unknown','confidence':.5,'note':'投放未执行，图片不足以确认抓起',
        'comparison':{'reference_plan_id':None,'effect':'not_compared','movement':'unknown','note':'首轮'}}


def failed_report(plan,category):
    plane=plan['table_plane'];goal=at_height([340.,-10.,0.],100.,plane)
    actual=at_height(goal,56.,plane)
    raw=inverse(actual,math.radians(50),0.,2.6)
    start=1_000_000_000
    # Timing taken from the supplied account; XYZ for the 44mm undershoot is
    # synthetic because that account did not include an absolute feedback XYZ.
    events=[('start',0),('approach_ball',0),('approach_ball_completed',1.986),('align_above_ball',2.034),
        ('align_above_ball_completed',4.604),('descend',4.660),('descend_completed',6.257),('close',6.305),
        ('close_completed',7.768),('lift',7.817),('lift_completed',9.534),('transport',9.535),('fault',9.536)]
    return {'motion_completed':False,'grasp_attempted':True,'placement_attempted':False,
        'motion_error':'RuntimeError: 本轮10秒录像窗口即将结束，停止发送后续动作' if category=='time_budget' else
            'MotionConstraintError: 路径0.0%处：指尖低于本段要求的离桌高度72.4mm',
        'failure':{'stage':'transport','code':'time_budget' if category=='time_budget' else 'tip_clearance','progress':0},
        'events':[{'stage':s,'host_monotonic_ns':start+round(t*1e9)} for s,t in events],
        'dispatches':[{'stage':'lift','command':{'T':104,**dict(zip('xyz',goal))}}],
        'feedback':[{'stage':'lift','status':raw}],
        'last_preflight':{'stage':'transport','status':raw},'camera_error':None}


class DiagnosisAndSummaries(unittest.TestCase):
    def test_clearance_is_relative_to_tilted_table_and_preserves_actual_xyz(self):
        case=DATA['cases'][0];plan=build_plan({'setup_id':'replay'},case['initial_status'],world(case),copy.deepcopy(DEFAULT_PARAMETERS))
        report=failed_report(plan,'clearance_shortfall');value=diagnose(report,plan)
        self.assertEqual(value['category'],'clearance_shortfall')
        self.assertAlmostEqual(value['lift']['target_height_mm'],100.)
        self.assertAlmostEqual(value['lift']['actual_height_mm'],56.)
        self.assertNotAlmostEqual(value['lift']['actual_xyz_mm'][2],56.)
        self.assertAlmostEqual(value['lift']['undershoot_mm'],44.)
        self.assertEqual(value['suggested_trial'],{'parameter':'lift_height_mm','value':126.4,
            'meaning':'single measured execution trial; full parameter bounds and IK still required'})
        self.assertEqual(value['stage_timings'][0]['duration_s'],1.986)
        self.assertEqual(value['elapsed_s'],9.536)

    def test_finished_job_with_long_historical_feedback_is_not_reported_running(self):
        report=failed_report({'table_plane':{'normal_up':[0.,0.,1.],'d_mm':115.}},'time_budget')
        report['feedback']*=1500
        value={'health':{'connected':True,'pending':None},'job':{'status':'failed','phase':'finished','result':report},
            'learning':{'status':'idle'},'plan':{'scene':{'table_plane':{'normal_up':[0.,0.,1.],'d_mm':115.}}}}
        compact=summarize('state',value)
        self.assertTrue(compact['task_finished']);self.assertEqual(compact['waiting_for'],[])
        self.assertLess(len(json.dumps(compact,ensure_ascii=False)),5000)
        self.assertEqual(compact['job']['result']['execution_diagnosis']['category'],'time_budget')
        value['learning']={'status':'running','phase':'recovering'}
        self.assertFalse(summarize('wait',value)['task_finished'])

    def test_cli_accepts_documented_wait_and_pasted_plan_id_without_issuing_motion(self):
        self.assertEqual(parser().parse_args(['wait','--timeout','30']).seconds,30.)
        self.assertEqual(parser().parse_args(['context','PLAN_ID='+'a'*32]).plan_id,'a'*32)
        api=Mock();api.request.return_value={'health':{'pending':None},'job':{'status':'failed'},'learning':{'status':'idle'}}
        value=invoke(parser().parse_args(['wait']),api)
        self.assertEqual(value['job']['status'],'failed');api.request.assert_called_once_with('/state')

    def test_recorded_layout_avoids_redundant_alignment_and_lifts_in_transport_pose(self):
        for case in DATA['cases']:
            plan=build_plan({'setup_id':'replay'},case['initial_status'],world(case),copy.deepcopy(DEFAULT_PARAMETERS))
            steps=plan['steps'];names=[s['stage'] for s in steps]
            self.assertTrue(plan['route_policy']['merged_high_approach'])
            self.assertNotIn('align_above_ball',names)
            approach=next(s for s in steps if s['stage']=='approach_ball')
            self.assertTrue(approach['open_while_moving'])
            self.assertGreaterEqual(height_above_plane(approach['xyz_mm'],plan['table_plane']),72.4)
            lift=next(s for s in steps if s['stage']=='lift')
            self.assertEqual(lift['pitch_rad'],math.radians(plan['release_pitch_deg']))
            self.assertFalse(lift['hold_orientation'])


class AgentExecutionLearning(unittest.TestCase):
    setUp=fixtures.RSIApi.setUp
    request=fixtures.RSIApi.request

    def failed(self,category='time_budget'):
        def execute(service,plan,root,stop,publish,**kw):
            service.pending=None;service.disarm()
            return {**failed_report(plan,category),'capture_trial_id':self.folder.name,'recording_covers_motion':True}
        self.execute_mock.side_effect=execute
        response,prepared=self.request('prepare',{'source':'opencode_agent'})
        self.assertEqual(response['status'],200,prepared);key=prepared['plan_id']
        _,scene=self.request('vision-context/'+key,method='GET')
        response,run=self.request('run',{'plan_id':key,'declaration':DECLARATION,
            'vision_packet_id':scene['vision_packet_id'],'scene_review':{'ready':True,'note':'实帧目标可见'}})
        self.assertEqual(response['status'],202,run)
        return key

    def review(self,key,trial=None,verdict=None):
        export_keyframes(self.folder,[],sample_count=3)
        _,packet=self.request('vision-context/'+key,method='GET');v=verdict or visual()
        body={**v,'plan_id':key,'source':'opencode_agent','vision_packet_id':packet['vision_packet_id'],'visual_review':v,
            'note':'按阶段时序或实测抬升不足选择单项执行试验，不修改XY'}
        if trial:body['parameter_trial']=trial
        return self.request('review',body)

    def test_unreviewed_failed_trial_blocks_next_agent_motion(self):
        self.failed()
        response,error=self.request('prepare',{'source':'opencode_agent'})
        self.assertEqual(response['status'],409,error)
        self.assertEqual(error['detail']['code'],'trial_review_required')
        self.assertEqual(self.execute_mock.call_count,1)

    def test_unchanged_timeout_is_not_replayed_after_recording_review(self):
        key=self.failed();response,value=self.review(key)
        self.assertEqual(response['status'],200,value);self.assertFalse(value['proposal']['allow_retry'])
        response,error=self.request('prepare',{'source':'opencode_agent'})
        self.assertEqual(response['status'],409,error)
        self.assertEqual(error['detail']['code'],'execution_retry_requires_change')
        self.assertEqual(self.execute_mock.call_count,1)

    def test_timeout_permits_one_speed_trial_and_next_plan_contains_the_new_value(self):
        key=self.failed();response,value=self.review(key,{'parameter':'travel_speed','value':.8})
        self.assertEqual(response['status'],200,value)
        self.assertEqual(value['proposal']['decision'],'execution_parameter_trial')
        self.assertEqual(value['proposal']['parameters_changed'],['travel_speed'])
        response,new=self.request('prepare',{'source':'opencode_agent'})
        self.assertEqual(response['status'],200,new)
        _,state=self.request('state',method='GET');self.assertEqual(state['plan']['parameters']['travel_speed'],.8)
        self.assertEqual(self.execute_mock.call_count,1)

    def test_clearance_permits_measured_lift_trial_without_lowering_transport_floor(self):
        key=self.failed('clearance_shortfall');response,value=self.review(key,{'parameter':'lift_height_mm','value':126.4})
        self.assertEqual(response['status'],200,value)
        self.assertEqual(value['after']['lift_height_mm'],126.4)
        self.assertEqual(value['after']['pickup_xy_mm'],[0.,0.])
        self.assertEqual(self.arm.commands,[])

    def test_execution_error_does_not_grant_arbitrary_grasp_changes(self):
        key=self.failed();response,error=self.review(key,{'parameter':'finger_offset_mm','value':25.})
        self.assertEqual(response['status'],409,error)
        _,context=self.request('agent/context',method='GET');self.assertEqual(context['policy']['parameters'],DEFAULT_PARAMETERS)
        self.assertEqual(self.arm.commands,[])

    def test_visible_miss_does_not_hide_timeout_with_automatic_xy_learning(self):
        key=self.failed();v={**visual(),'outcome':'grasp_missed','direction':'image_right',
            'ball_state':'table','magnitude':'medium','adjustment':'pickup_xy'}
        response,value=self.review(key,verdict=v)
        self.assertEqual(response['status'],200,value)
        self.assertEqual(value['visual_review'],v)
        self.assertEqual(value['before'],value['after'])
        self.assertFalse(value['proposal']['allow_retry'])
        self.assertNotIn('robot_delta_mm',value['proposal'])

    def test_visible_miss_does_not_grant_unrelated_trial_before_timeout_is_resolved(self):
        key=self.failed();v={**visual(),'outcome':'grasp_missed','direction':'image_right',
            'ball_state':'table','magnitude':'medium','adjustment':'pickup_xy'}
        response,error=self.review(key,{'parameter':'finger_offset_mm','value':25.},v)
        self.assertEqual(response['status'],409,error)
        _,context=self.request('agent/context',method='GET')
        self.assertEqual(context['policy']['parameters'],DEFAULT_PARAMETERS)
