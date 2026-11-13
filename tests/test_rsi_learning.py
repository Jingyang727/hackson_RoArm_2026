"""Automatic RSI and recovery with synthetic devices/model; no real hardware."""
import asyncio
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
from fastapi import FastAPI

from control_ui import main
from control_ui.rsi_trial import install
from control_ui.service import ArmService
from control_ui.safety import UnsafeMotion
from evoarm_rsi import grok_review
from evoarm_rsi.grasp_once import export_keyframes
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS, build_plan, observe_scene
from evoarm_rsi.roarm_geometry import forward, inverse
from evoarm_rsi.rsi_execution import execute,observe_pending, reservation
from evoarm_rsi.rsi_journal import archive_trial, pending_exports, publish_trace
from evoarm_rsi.rsi_learning import LEARNING_DECLARATION
from evoarm_rsi.rsi_recovery import recover
from tests.test_ball_trial import layout
from tests import test_rsi_loop as loop_fixtures
from tests.test_rsi_loop import InstantArm
from tests.test_rsi_pick_place import scene


def verdict(outcome='placed', direction='none', magnitude='medium', ball_state=None):
    return {'outcome': outcome, 'direction': direction, 'magnitude': magnitude,
            'ball_state': ball_state or ('funnel' if outcome == 'placed' else 'table'),
            'confidence': .92, 'note': 'synthetic visual evidence'}


class AutoAPI(unittest.TestCase):
    def setUp(self):
        self.fixture = loop_fixtures.RSIApi(); self.fixture.setUp(); self.addCleanup(self.fixture.doCleanups)
        self.fixture.arm=FactoryArm();self.fixture.service.arm=self.fixture.arm
        app = FastAPI(); install(app, lambda: self.fixture.service)
        for item in (patch.object(main, 'app', app),
                     patch('evoarm_rsi.grok_review.configuration', return_value={'configured': True, 'model': 'company/grok-test'}),
                     patch('evoarm_rsi.rsi_learning._launch_learning', side_effect=lambda target: target())):
            item.start(); self.addCleanup(item.stop)
        self.policies = []
        def execute(service, plan, root, stop, publish, **kwargs):
            self.assertEqual(kwargs['automation_id'], service.automation_owner)
            self.policies.append(copy.deepcopy(plan['parameters']))
            service.pending = None; service.disarm()
            return {'motion_completed': True, 'grasp_attempted': True, 'placement_attempted': True,
                    'physical_elapsed_s': 6., 'within_10s': True, 'robot_moved': True,
                    'recording_covers_motion': True, 'motion_error': None, 'camera_error': None,
                    'capture_trial_id': self.fixture.folder.name, 'keyframes_file': 'synthetic.jpg'}
        self.fixture.execute_mock.side_effect = execute

    def request(self, *args, **kwargs): return self.fixture.request(*args, **kwargs)

    def start(self):
        return self.request('learning/start', {'declaration': LEARNING_DECLARATION, 'max_attempts': 5})

    def test_miss_is_reviewed_recovered_and_retried_then_success_is_remembered(self):
        with (patch('evoarm_rsi.grok_review.review', side_effect=[{'ready': True, 'note': 'scene visible'},
                   verdict('grasp_missed', 'image_left', 'large'), verdict()]) as model,
                patch('control_ui.rsi_trial.recover_trial', return_value={'status': 'completed'}) as recovery):
            response, answer = self.start()
        self.assertEqual(response['status'], 202, answer)
        _, state = self.request('state', method='GET')
        self.assertEqual(state['learning']['status'], 'completed')
        self.assertEqual(state['learning']['attempt'], 2)
        self.assertEqual(model.call_count, 3); self.assertEqual(recovery.call_count, 1)
        self.assertEqual(len(self.policies), 2)
        self.assertAlmostEqual(math.hypot(*self.policies[1]['pickup_xy_mm']), 12.)
        self.assertIsNone(self.fixture.service.automation_owner)
        pol = next((self.fixture.home / '.evoarm/rsi/pick_place_policies').glob('*/*.json'))
        saved = json.loads(pol.read_text())
        self.assertEqual(saved['statistics'], {'reviewed_trials': 2, 'successes': 1})
        self.assertEqual(saved['successful_policy']['parameters'], self.policies[1])
        self.assertEqual(len(list(pending_exports(self.fixture.home))), 2)

    def test_no_grok_config_means_no_robot_write(self):
        with patch('evoarm_rsi.grok_review.configuration', return_value={'configured': False, 'reason': 'company Grok missing'}):
            response, _ = self.start()
        self.assertEqual(response['status'], 409); self.assertEqual(self.fixture.arm.commands, [])
        self.assertEqual(self.policies, [])

    def paired_run(self,*,opposite):
        original=self.fixture.execute_mock.side_effect;captured=[];contexts=[]
        def execute(service,plan,root,stop,publish,**kwargs):
            result=original(service,plan,root,stop,publish,**kwargs)
            key=f'{len(captured)+100:032x}';folder=self.fixture.home/'.evoarm/rsi/experiments'/key;folder.mkdir(parents=True)
            image=self.fixture.image.copy();rows=[]
            for index,label in enumerate(('before','close_review','lift_review','release')):
                name=f'evidence-{index:02d}.jpg';cv2.imwrite(str(folder/name),image)
                rows.append({'file':name,'label':label,'frame_index':index,'elapsed_s':float(index)})
            (folder/'keyframes.json').write_text(json.dumps({'frames':rows}))
            measured=np.asarray(plan['grasp_tip_xyz_mm']).copy()
            if captured and opposite:
                measured[:2]-=2*(np.asarray(plan['parameters']['pickup_xy_mm'])-captured[-1]['parameters']['pickup_xy_mm'])
            captured.append(copy.deepcopy(plan))
            result.update(capture_trial_id=key,feedback=[{'stage':'close','status':dict(zip('xyz',measured.tolist()))}])
            return result
        def model(system,content,stop,**options):
            context=json.loads(content[0]['text']);contexts.append(context)
            if system==grok_review.SCENE_SYSTEM:answer={'ready':True,'note':'synthetic scene'}
            else:
                pair=context['comparison'];first=len(captured)==1
                answer=verdict('grasp_missed' if first or opposite or len(captured)==2 else 'placed','image_right',
                               'medium' if first else 'small')
                answer['comparison']={'reference_plan_id':pair['reference_plan_id'] if pair['available'] else None,
                    'effect':'not_compared' if first else 'worsened' if opposite else 'improved',
                    'movement':'unknown' if first else 'opposite' if opposite else 'as_expected','note':'synthetic paired evidence'}
            return json.dumps(answer)
        self.fixture.execute_mock.side_effect=execute
        with (patch.object(grok_review,'configuration',return_value={'configured':True,'model':'grok-test','backend':'company_api'}),
              patch.object(grok_review,'complete_content',side_effect=model),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'}) as recovery):
            self.start()
        _,state=self.request('state',method='GET')
        return state,contexts,recovery.call_count

    def test_real_review_pipeline_compares_trials_and_rolls_back_wrong_way_instead_of_third_attempt(self):
        state,contexts,resets=self.paired_run(opposite=True)
        self.assertEqual(len(self.policies),2);self.assertEqual(resets,2)
        self.assertEqual(state['learning']['status'],'failed',state['learning'].get('error'))
        entry=state['plan']['review'];self.assertEqual(entry['proposal']['comparison'],'direction_disproved')
        self.assertEqual(entry['after'],DEFAULT_PARAMETERS)
        self.assertEqual(entry['comparison']['feedback']['status'],'opposite')
        self.assertEqual(entry['comparison']['assessment']['movement'],'opposite')
        self.assertTrue(entry['comparison']['panel_url'])
        self.assertTrue(contexts[-1]['comparison']['available'])
        self.assertTrue(any(row['kind']=='comparison' for row in state['process']['events']))
        response,image=asyncio.run(loop_fixtures.api_request(entry['comparison']['panel_url'],{},method='GET',decode_json=False))
        self.assertEqual(response['status'],200);self.assertGreater(len(image),1000)
        import zipfile
        with zipfile.ZipFile(self.fixture.home/'.evoarm/rsi/backups'/f"{state['plan']['plan_id']}.zip") as archive:
            self.assertIn('trial/comparison.jpg',archive.namelist())
            self.assertEqual(archive.read('trial/model-frames/raw-review-00.jpg'),Path(contexts[-1]['comparison']['previous_frame']['path']).read_bytes())

    def test_real_review_pipeline_uses_improvement_to_shrink_and_remembers_successful_actual_policy(self):
        state,contexts,_=self.paired_run(opposite=False)
        self.assertEqual(state['learning']['status'],'completed',state['learning'].get('error'))
        self.assertEqual(len(self.policies),3)
        self.assertAlmostEqual(math.dist(self.policies[0]['pickup_xy_mm'],self.policies[1]['pickup_xy_mm']),8.)
        self.assertAlmostEqual(math.dist(self.policies[1]['pickup_xy_mm'],self.policies[2]['pickup_xy_mm']),4.)
        self.assertEqual(state['plan']['review']['proposal']['decision'],'keep')
        self.assertEqual(state['plan']['review']['after'],self.policies[-1])

    def test_missing_paired_evidence_does_not_escalate_step_but_application_is_saved(self):
        miss=verdict('grasp_missed','image_right','small')
        with (patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':''},miss,miss,verdict()]),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'})):
            self.start()
        self.assertEqual(len(self.policies),3)
        first=np.asarray(self.policies[1]['pickup_xy_mm'])-self.policies[0]['pickup_xy_mm']
        second=np.asarray(self.policies[2]['pickup_xy_mm'])-self.policies[1]['pickup_xy_mm']
        self.assertAlmostEqual(float(np.linalg.norm(first)),4.)
        self.assertAlmostEqual(float(np.linalg.norm(second)),4.)
        _,state=self.request('state',method='GET')
        self.assertEqual(state['learning']['status'],'completed',state['learning'].get('error'))
        audit=state['plan']['policy_application'];self.assertTrue(audit['parameters_verified'])
        saved=json.loads((self.fixture.home/'.evoarm/rsi/pick_place_runs'/state['plan']['plan_id']/'state.json').read_text())
        self.assertEqual(saved['result']['policy_application'],audit)

    def test_no_effective_new_parameter_recovers_and_stops_instead_of_replaying(self):
        miss=verdict('grasp_missed','image_right','small')
        from evoarm_rsi.rsi_policy import propose
        def bounded(*args,**kwargs):
            parameters,changes,proposal=propose(*args,**kwargs)
            proposal['allow_retry']=False
            return parameters,changes,proposal
        with (patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':''},miss]),
              patch('control_ui.rsi_trial.propose',side_effect=bounded),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'}) as recovery):
            self.start()
        self.assertEqual(len(self.policies),1);self.assertEqual(recovery.call_count,1)
        _,state=self.request('state',method='GET')
        self.assertIn('没有可验证的新修正',state['learning']['error'])

    def test_scene_rejection_stops_before_physical_attempt(self):
        with patch('evoarm_rsi.grok_review.review', return_value={'ready': False, 'note': 'wrong funnel'}):
            self.start()
        _, state = self.request('state', method='GET')
        self.assertEqual(state['learning']['status'], 'failed'); self.assertEqual(self.policies, [])
        self.assertEqual(self.fixture.arm.commands[-1],{'T':100})
        self.assertIn('已恢复官方初始姿态',state['learning']['error'])

    def test_low_far_pose_planning_failure_resets_then_replans_from_official_home(self):
        self.fixture.arm.raw=inverse([425.,0.,-70.],math.radians(64.2),0.,3.14)
        with patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':''},verdict()]):
            response,data=self.start()
        self.assertEqual(response['status'],202,data)
        _,state=self.request('state',method='GET')
        self.assertEqual(state['learning']['status'],'completed',state['learning'].get('error'))
        self.assertEqual(self.fixture.arm.commands,[{'T':100}])
        self.assertEqual(len(self.policies),1)
        event=state['learning']['recovery_events'][0]
        self.assertEqual(event['phase'],'planning');self.assertIn('383.2',event['cause'])

    def test_new_planning_failure_is_not_suppressed_by_previous_recovery(self):
        from fastapi import HTTPException
        from control_ui import rsi_trial
        original=rsi_trial.observe_scene;observations=0
        def observe(*args):
            nonlocal observations
            observations+=1
            if observations>=2:raise ValueError('synthetic unreachable next plan')
            return original(*args)
        with (patch('control_ui.rsi_trial.observe_scene',side_effect=observe),
              patch('evoarm_rsi.grok_review.review',side_effect=[{'ready':True,'note':''},verdict('grasp_missed')]),
              patch('control_ui.rsi_trial.recover_trial',return_value={'status':'completed'}) as resets):
            self.start()
        _,state=self.request('state',method='GET')
        self.assertEqual(state['learning']['status'],'failed')
        self.assertEqual(resets.call_count,3) # failed trial, NEW planning failure, failed replan
        self.assertEqual(len(state['learning']['recovery_events']),2)
        self.assertIn('已恢复官方初始姿态',state['learning']['error'])
        self.assertNotIn('HTTPException',state['learning']['error'])
        self.assertTrue(state['learning']['session_backup_file'])

    def test_reset_button_does_not_require_grok_configuration_or_an_ik_plan(self):
        self.fixture.arm.raw=inverse([425.,0.,-70.],math.radians(64.2),0.,3.14)
        with patch('evoarm_rsi.grok_review.configuration',return_value={'configured':False,'reason':'missing API'}):
            response,data=self.request('learning/reset',{'declaration':LEARNING_DECLARATION})
        self.assertEqual(response['status'],202,data)
        self.assertEqual(self.fixture.arm.commands,[{'T':100}])
        _,state=self.request('state',method='GET');self.assertEqual(state['learning']['phase'],'reset_completed')

    def test_uncertain_official_reset_is_not_reissued_for_the_same_planning_failure(self):
        self.fixture.arm.raw=inverse([425.,0.,-70.],math.radians(64.2),0.,3.14)
        def uncertain(command):
            self.fixture.arm.commands.append(command)
            raise OSError('reset may have arrived')
        with patch.object(self.fixture.arm,'send',side_effect=uncertain):self.start()
        _,state=self.request('state',method='GET')
        self.assertEqual(self.fixture.arm.commands,[{'T':100}]);self.assertEqual(self.policies,[])
        self.assertEqual(state['learning']['status'],'failed')
        event=state['learning']['recovery_events'][0]
        self.assertTrue(event['t100_attempted']);self.assertFalse(event['t100_sent'])

    def test_unknown_review_recovers_once_and_does_not_train_or_repeat_blindly(self):
        with (patch('evoarm_rsi.grok_review.review', side_effect=[{'ready': True, 'note': ''}, verdict('unknown')]),
                patch('control_ui.rsi_trial.recover_trial', return_value={'status': 'completed'}) as recovery):
            self.start()
        self.assertEqual(len(self.policies), 1); self.assertEqual(recovery.call_count, 1)
        _, state = self.request('state', method='GET'); self.assertEqual(state['learning']['status'], 'failed')

    def test_model_transport_failure_after_motion_still_recovers_and_backs_up(self):
        with (patch('evoarm_rsi.grok_review.review', side_effect=[{'ready': True, 'note': ''}, RuntimeError('Grok offline')]),
                patch('control_ui.rsi_trial.recover_trial', return_value={'status': 'completed'}) as recovery):
            self.start()
        self.assertEqual(recovery.call_count, 1); self.assertEqual(len(self.policies), 1)
        self.assertEqual(len(list(pending_exports(self.fixture.home))), 1)

    def test_active_automation_blocks_manual_paths_and_double_start(self):
        key = self.fixture.prepare()
        with patch('evoarm_rsi.rsi_learning._launch_learning'):
            self.assertEqual(self.start()[0]['status'], 202)
            self.assertEqual(self.start()[0]['status'], 409)
            self.assertEqual(self.request('prepare')[0]['status'], 409)
            self.assertEqual(self.request('state',method='GET')[1]['job']['status'],'ready')
            self.assertEqual(self.fixture.start_trial(key)[0]['status'], 409)
            with self.assertRaises(UnsafeMotion): self.fixture.service.factory_reset()
            with self.assertRaises(UnsafeMotion): self.fixture.service.set_torque(False, '已托住机械臂并确认释放全部关节')
            self.assertTrue(self.request('learning/stop')[1]['stop_requested'])
        self.assertEqual(self.fixture.arm.commands, [])

    def test_bad_limit_or_missing_scope_does_not_start(self):
        self.assertEqual(self.request('learning/start', {'declaration': '', 'max_attempts': 5})[0]['status'], 409)
        self.assertEqual(self.request('learning/start', {'declaration': LEARNING_DECLARATION, 'max_attempts': True})[0]['status'], 422)
        self.assertEqual(self.fixture.arm.commands, [])

    def test_v1_policy_adds_only_the_new_preload_default_without_losing_history(self):
        self.fixture.prepare()
        _, state = self.request('state', method='GET')
        from evoarm_rsi.coordinate_test import fingerprint
        file=self.fixture.home/'.evoarm/rsi/pick_place_policies/bench-layout-v2'/f'{fingerprint(self.fixture.session)}.json'
        file.parent.mkdir(parents=True,exist_ok=True)
        old={k:v for k,v in DEFAULT_PARAMETERS.items() if k!='squeeze_deg'}
        file.write_text(json.dumps({'version':4,'parameters':old,'reviews':[]}))
        self.fixture.prepare();_,state=self.request('state',method='GET')
        self.assertEqual(state['plan']['policy_version'],5)
        self.assertEqual(state['plan']['parameters']['squeeze_deg'],2.)
        self.assertEqual(json.loads(file.read_text())['migrations'][0]['added_defaults'],['squeeze_deg'])


class EvidenceAndTracking(unittest.TestCase):
    def test_one_contact_preload_is_retained_during_lift_and_transport(self):
        class ContactArm(InstantArm):
            def send(self,command):
                super().send(command)
                self.raw['g']=min(self.raw['g'],math.radians(158))
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp);saved=layout(home);folder,image,depth=scene(home,saved)
            observed,_,_,_=observe_scene(folder,saved);arm=ContactArm();arm.contact=True
            service=ArmService();service.arm=arm;plan=build_plan(saved,arm.status(),observed,copy.deepcopy(DEFAULT_PARAMETERS))
            key='e'*32;done=threading.Event();camera_state={}
            capture=home/'.evoarm/rsi/experiments'/('f'*32);capture.mkdir(parents=True)
            def camera(task_home,method,path,payload=None):
                if method=='GET':return dict(camera_state)
                stamp=time.monotonic_ns();camera_state.update(recording=True,phase='recording',
                    recording_request_id=payload['request_id'],recording_started_monotonic_ns=stamp)
                done.wait(12)
                writer=cv2.VideoWriter(str(capture/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),.3,(1280,720))
                for _ in range(4):writer.write(image)
                writer.release();stamps=[stamp+int(i*10_000_000_000/3) for i in range(4)]
                (capture/'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns':s}) for s in stamps))
                (capture/'manifest.json').write_text(json.dumps({'trial_id':capture.name,'status':'recorded','simulated':False}))
                np.savez_compressed(capture/'depth_samples.npz',depth_mm=np.stack([depth]*4),monotonic_ns=stamps)
                return {'trial_id':capture.name,'request_id':payload['request_id']}
            def publish(phase,**kwargs):
                if phase=='saving_recording':done.set()
            with patch.object(Path,'home',return_value=home),patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
                service.unlock_once('已检查设备并清空运动区域');service.pending=reservation(key)
                result=execute(service,plan,home/key,threading.Event(),publish,camera=camera)
            self.assertTrue(result['motion_completed'],result['motion_error'])
            preload=[d for d in result['dispatches'] if d['stage']=='hold_contact']
            self.assertEqual(len(preload),1)
            target=math.radians(160)
            self.assertAlmostEqual(preload[0]['command']['cmd'],target)
            for stage in ('lift','transport'):
                command=next(d['command'] for d in result['dispatches'] if d['stage']==stage)
                self.assertAlmostEqual(command['g'],target)
            self.assertFalse(result['contact_preload']['force_measured'])

    def test_large_stable_tracking_residual_is_evidence_and_does_not_veto_close(self):
        with tempfile.TemporaryDirectory() as temp:
            service = ArmService(); service.arm = InstantArm(); actual = service.arm.status()
            target = [actual[k] + (18 if k == 'x' else 0) for k in 'xyz']
            report = {'plan': {'target_xyz_mm': target, 'pose_preserved_rad':
                       {'pitch': actual['tit'], 'roll': actual['r'], 'gripper': actual['g']}},
                      'feedback': [], 'report_file': str(Path(temp) / 'step.json')}
            pending = {'kind': 'rsi_trial', 'stage_kind': 'cartesian', 'since': 0,
                       'index': 1, 'run_id': 'a' * 32, 'report': report}
            service.pending = pending; service.last_command = {}
            for at in (.9, 1.05, 1.2): observe_pending(service, pending, actual, at)
            self.assertIsNone(service.fault)
            self.assertEqual(service.pending['stage_kind'], 'reserved')
            self.assertAlmostEqual(report['position_error_mm'], 18)
            self.assertEqual(report['status'], 'settled_with_offset')

    def test_opening_is_embedded_in_approach_and_not_discarded(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); saved = layout(home); folder, _, _ = scene(home, saved)
            observed, _, _, _ = observe_scene(folder, saved)
            plan = build_plan(saved, InstantArm().status(), observed, copy.deepcopy(DEFAULT_PARAMETERS))
            opening = next(step for step in plan['steps'] if step.get('open_while_moving'))
            self.assertIn(opening['stage'], ('align', 'approach_ball'))
            self.assertAlmostEqual(opening['gripper_rad'], math.radians(110))
            self.assertNotIn('open', [step['stage'] for step in plan['steps']])

    def test_odd_keyframe_count_contains_no_black_padding(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp); writer = cv2.VideoWriter(str(folder / 'rgb.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 1, (160, 90))
            for index in range(4): writer.write(np.full((90, 160, 3), 130 + index * 20, np.uint8))
            writer.release(); stamps = [1_000_000_000 * (i + 1) for i in range(4)]
            (folder / 'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns': stamp}) for stamp in stamps))
            output = export_keyframes(folder, [{'stage': 'close', 'host_monotonic_ns': stamps[1]}])
            image = cv2.imread(output)
            self.assertEqual(image.shape[:2], (810, 480)); self.assertGreater(float(image[-200:].mean()), 50)
            self.assertEqual(json.loads((folder / 'keyframes.json').read_text())['padding_frames'], 0)

    def test_uniform_sample_does_not_erase_close_event_label(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);writer=cv2.VideoWriter(str(folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),1,(160,90))
            for _ in range(4):writer.write(np.full((90,160,3),180,np.uint8))
            writer.release();stamps=[1_000_000_000*i for i in range(4)]
            (folder/'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns':s}) for s in stamps))
            export_keyframes(folder,[{'stage':'close','host_monotonic_ns':stamps[1]+100}],sample_count=4)
            frames=json.loads((folder/'keyframes.json').read_text())['frames']
            self.assertEqual(len(frames),4);self.assertIn('close',frames[1]['labels'])
            self.assertIn('sample',frames[1]['labels'])

    def test_contact_evidence_is_before_next_motion_and_no_release_is_invented(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);writer=cv2.VideoWriter(str(folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),1,(160,90))
            for index in range(6):writer.write(np.full((90,160,3),130+index*15,np.uint8))
            writer.release();stamps=[(i+1)*1_000_000_000 for i in range(6)]
            (folder/'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns':s}) for s in stamps))
            events=[{'stage':stage,'host_monotonic_ns':int(t*1e9)} for stage,t in
                    [('close',2.1),('lift',4.1),('transport',5.1),('release',5.4),('finish',6.1)]]
            export_keyframes(folder,events,sample_count=6)
            rows=json.loads((folder/'keyframes.json').read_text())['frames']
            by_label={label:row['frame_index'] for row in rows for label in row['labels']}
            self.assertEqual(by_label['close_review'],3);self.assertEqual(by_label['lift_review'],4)
            self.assertEqual(by_label['release_review'],5)
            export_keyframes(folder,[e for e in events if e['stage'] not in ('release','transport')])
            rows=json.loads((folder/'keyframes.json').read_text())['frames']
            self.assertFalse(any('release_review' in row['labels'] for row in rows))

    def test_backup_hashes_and_langfuse_contract_preserve_raw_evidence(self):
        import zipfile
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); root = home / '.evoarm/rsi/pick_place_runs' / ('a' * 32); root.mkdir(parents=True)
            state = {'calibration_fingerprint': 'test', 'plan': {'setup_id': 'bench-layout-v2', 'parameters': DEFAULT_PARAMETERS,
                     'scene': {'ball': {'center_robot_mm': [300, 0, -98]}, 'funnel': {'mouth_center_robot_mm': [260, 100, -70]}}}, 'result': {}}
            (root / 'state.json').write_text(json.dumps(state)); (root / 'journal.jsonl').write_text('{"name":"fault"}\n')
            archive = archive_trial(root, calibration={'verified_points': 5}, home=home)
            with zipfile.ZipFile(archive) as file:
                index = json.loads(file.read('archive-manifest.json'))
                for row in index['files']: self.assertEqual(hashlib.sha256(file.read(row['path'])).hexdigest(), row['sha256'])
                self.assertIn('trial/calibration-snapshot.json', file.namelist())
            packets = list(pending_exports(home)); self.assertEqual(len(packets), 1)
            self.assertFalse(packets[0]['remote_submitted'])
            client = MagicMock(); publish_trace(packets[0], client); client.flush.assert_called_once()


class FactoryArm(InstantArm):
    def send(self, command):
        if command['T'] != 100: return super().send(command)
        self.commands.append(dict(command))
        angles = dict(b=0., s=0., e=math.pi / 2, t=0., r=0., g=math.pi)
        model = forward({'T':1051,**angles})
        self.raw = {'T': 1051, **angles, **dict(zip('xyz', model['xyz_mm'])), 'tit': model['pitch_rad']}


class AutomaticRecovery(unittest.TestCase):
    def test_old_transport_fault_can_reset_after_fresh_feedback_proves_connection_and_stillness(self):
        with tempfile.TemporaryDirectory() as temp:
            service=ArmService();service.arm=FactoryArm();service.automation_owner='owner'
            service.fault={'code':'feedback_timeout','message':'old timeout'}
            with patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
                report=recover(service,None,{}, {},Path(temp),threading.Event(),lambda *a,**kw:None,automation_id='owner')
            self.assertEqual(service.arm.commands,[{'T':100}]);self.assertIsNone(service.fault)
            self.assertEqual(report['previous_fault']['code'],'feedback_timeout')
    def test_failed_empty_grasp_sends_one_official_reset_without_cartesian_retreat(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp); saved = layout(home); folder, _, _ = scene(home, saved)
            observed, _, _, _ = observe_scene(folder, saved); service = ArmService(); arm = FactoryArm(); service.arm = arm
            plan = build_plan(saved, arm.status(), observed, copy.deepcopy(DEFAULT_PARAMETERS))
            arm.raw = inverse(plan['grasp_tip_xyz_mm'], math.radians(plan['grasp_pitch_deg']), 0., math.radians(110))
            root = home / ('b' * 32); service.automation_owner = 'owner'; service.fault = {'code': 'rsi_position_unverified', 'message': 'old residual gate'}
            with patch.dict('os.environ', {'EVOARM_ENABLE_MOTION': '1'}):
                result = recover(service, plan, {'grasp_attempted': False}, verdict('grasp_missed'), root,
                                 threading.Event(), lambda *a, **kw: None, automation_id='owner')
            self.assertEqual(result['status'], 'completed')
            self.assertEqual(arm.commands,[{'T':100}])
            self.assertEqual(sum(command['T'] == 100 for command in arm.commands), 1)
            self.assertFalse(any(command['T'] == 210 for command in arm.commands))
            self.assertEqual(service.armed_until, 0)

    def test_transport_fault_and_user_stop_never_auto_reset(self):
        for stopped in (False, True):
            with self.subTest(stopped=stopped), tempfile.TemporaryDirectory() as temp:
                service = ArmService(); service.arm = FactoryArm(); service.automation_owner = 'owner'
                service.fault = {'code': 'feedback_timeout', 'message': 'serial unavailable'}
                stop = threading.Event()
                if stopped: stop.set()
                with (patch.dict('os.environ', {'EVOARM_ENABLE_MOTION': '1'}),
                      patch.object(service.arm,'status',side_effect=OSError('no fresh feedback')),
                      self.assertRaises((OSError,RuntimeError))):
                    recover(service, {'table_plane': {'normal_up': [0, 0, 1], 'd_mm': 115}}, {}, {}, Path(temp), stop,
                            lambda *a, **kw: None, automation_id='owner')
                self.assertEqual(service.arm.commands, [])

    def test_potentially_held_ball_is_released_before_direct_reset_without_funnel_path(self):
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp);saved=layout(home);folder,_,_=scene(home,saved)
            observed,_,_,_=observe_scene(folder,saved);service=ArmService();arm=FactoryArm();service.arm=arm
            plan=build_plan(saved,arm.status(),observed,copy.deepcopy(DEFAULT_PARAMETERS))
            arm.raw=inverse(plan['grasp_tip_xyz_mm'],math.radians(plan['grasp_pitch_deg']),0.,math.radians(158))
            service.automation_owner='owner'
            with patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
                result=recover(service,plan,{'grasp_attempted':True,'placement_attempted':False},
                               verdict('execution_failed',ball_state='gripper'),home/('c'*32),threading.Event(),
                               lambda *a,**kw:None,automation_id='owner',validate_funnel=lambda:plan['release_tip_xyz_mm'])
            self.assertTrue(result['possible_ball_released_on_bench'])
            release_index=next(i for i,c in enumerate(arm.commands) if c['T']==106)
            reset_index=next(i for i,c in enumerate(arm.commands) if c['T']==100)
            self.assertLess(release_index,reset_index)
            self.assertEqual([command['T'] for command in arm.commands],[106,100])


class GrokAdapter(unittest.TestCase):
    def test_public_partial_output_is_visible_but_validation_waits_for_complete_reply(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);cv2.imwrite(str(root/'preview.jpg'),np.full((90,160,3),200,np.uint8))
            updates=[];answer={'ready':True,'note':'黄球与漏斗可见'}
            def streaming(system,content,stop,**options):
                options['on_mode']('streaming');options['on_delta']('黄球与漏斗可见。\n')
                self.assertFalse(any(row['type']=='validated' for row in updates))
                options['on_delta'](json.dumps(answer,ensure_ascii=False))
                return '黄球与漏斗可见。\n'+json.dumps(answer,ensure_ascii=False)
            with (patch('evoarm_rsi.grok_review.configuration',return_value={'configured':True,'model':'grok-test','backend':'company_api'}),
                  patch('evoarm_rsi.grok_review.complete_content',side_effect=streaming)):
                result=grok_review.review({},root,threading.Event(),preflight=True,on_update=updates.append)
            self.assertEqual(result,answer);self.assertEqual(updates[-1]['type'],'validated')
            self.assertTrue(any(row['type']=='text' for row in updates))
            self.assertIn('黄球与漏斗',(root/'grok-scene-output.txt').read_text())

    def test_invalid_streamed_reply_never_publishes_a_validated_result(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);cv2.imwrite(str(root/'preview.jpg'),np.full((90,160,3),200,np.uint8));updates=[]
            def invalid(*args,**options):options['on_delta']('看起来成功了，但没有结果JSON');return '看起来成功了，但没有结果JSON'
            with (patch('evoarm_rsi.grok_review.configuration',return_value={'configured':True,'model':'grok-test','backend':'company_api'}),
                  patch('evoarm_rsi.grok_review.complete_content',side_effect=invalid),self.assertRaises(ValueError)):
                grok_review.review({},root,threading.Event(),preflight=True,on_update=updates.append)
            self.assertEqual(updates[-1]['type'],'failed')
            self.assertFalse(any(row['type']=='validated' for row in updates))

    def test_bounded_frame_selection_preserves_close_lift_and_release(self):
        index=[{'label':'sample','frame_index':i} for i in range(20)]
        for i,label in ((0,'before'),(4,'close'),(6,'lift'),(17,'release'),(19,'last')):index[i]['label']=label
        selected=grok_review.select_frames(index)
        self.assertEqual(len(selected),8)
        self.assertTrue({'before','close','lift','release','last'}<=set(r['label'] for r in selected))
        self.assertEqual([r['frame_index'] for r in selected],sorted(r['frame_index'] for r in selected))

    def test_wrapped_final_json_is_accepted_but_two_answers_are_rejected(self):
        value=verdict()
        def event(text):return text
        self.assertEqual(grok_review._final_json(event('Analysis complete.\n'+json.dumps(value))),value)
        with self.assertRaises(ValueError):
            grok_review._final_json(event(json.dumps(value)+'\n'+json.dumps(value)))
        with self.assertRaises(ValueError):
            grok_review._final_json(event(json.dumps({'result':value,'motor_command':{'T':100}})))

    def test_stop_before_review_does_not_call_model_api(self):
        stopped=threading.Event();stopped.set()
        with patch('evoarm_rsi.grok_review.complete_content') as process,self.assertRaises(RuntimeError):
            grok_review.review({},Path('/unused'),stopped,preflight=True)
        process.assert_not_called()
    def test_reasoning_and_extra_motor_fields_are_not_accepted(self):
        payload = verdict()
        self.assertEqual(grok_review._final_json(json.dumps(payload)), payload)
        with self.assertRaises(ValueError): grok_review.validate_verdict({**payload, 'motor_command': {'T': 100}})
        self.assertEqual(grok_review.validate_verdict({**payload, 'confidence': .1})['confidence'], .1)

    def test_review_calls_direct_api_with_images_and_never_starts_cli(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            cv2.imwrite(str(root / 'preview.jpg'), np.full((90, 160, 3), 200, np.uint8))
            with (patch('evoarm_rsi.grok_review.configuration',return_value={'configured':True,'model':'grok-4.7-build','backend':'company_api'}),
                  patch('evoarm_rsi.grok_review.complete_content',return_value=json.dumps({'ready':True,'note':'synthetic scene'})) as api,
                  patch('subprocess.Popen',side_effect=AssertionError('CLI must not run'))):
                result = grok_review.review({}, root, threading.Event(), preflight=True)
            self.assertTrue(result['ready']); self.assertTrue((root / 'grok-scene-review.json').is_file())
            parts=api.call_args.args[1];self.assertTrue(any(p['type']=='image_url' for p in parts))
