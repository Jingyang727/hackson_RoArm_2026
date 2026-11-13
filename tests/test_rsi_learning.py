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
        self.assertAlmostEqual(math.hypot(*self.policies[1]['pickup_xy_mm']), 6.)
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

    def test_scene_rejection_stops_before_physical_attempt(self):
        with patch('evoarm_rsi.grok_review.review', return_value={'ready': False, 'note': 'wrong funnel'}):
            self.start()
        _, state = self.request('state', method='GET')
        self.assertEqual(state['learning']['status'], 'failed'); self.assertEqual(self.policies, [])

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
    def test_failed_empty_grasp_retreats_then_sends_one_official_reset(self):
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
            self.assertEqual(arm.commands[0]['T'], 104); self.assertEqual(arm.commands[-1]['T'], 100)
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
                with patch.dict('os.environ', {'EVOARM_ENABLE_MOTION': '1'}), self.assertRaises(RuntimeError):
                    recover(service, {'table_plane': {'normal_up': [0, 0, 1], 'd_mm': 115}}, {}, {}, Path(temp), stop,
                            lambda *a, **kw: None, automation_id='owner')
                self.assertEqual(service.arm.commands, [])

    def test_potentially_held_ball_is_released_over_funnel_before_reset(self):
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
            self.assertTrue(result['possible_ball_released_to_funnel'])
            release_index=next(i for i,c in enumerate(arm.commands) if c['T']==106)
            reset_index=next(i for i,c in enumerate(arm.commands) if c['T']==100)
            self.assertLess(release_index,reset_index)
            np.testing.assert_allclose([arm.commands[release_index-1][k] for k in 'xyz'],plan['release_tip_xyz_mm'])


class GrokAdapter(unittest.TestCase):
    def test_wrapped_final_json_is_accepted_but_two_answers_are_rejected(self):
        value=verdict()
        def event(text):return json.dumps({'type':'text','part':{'text':text}})
        self.assertEqual(grok_review._final_json(event('Analysis complete.\n'+json.dumps(value))),value)
        with self.assertRaises(ValueError):
            grok_review._final_json(event(json.dumps(value)+'\n'+json.dumps(value)))
        with self.assertRaises(ValueError):
            grok_review._final_json(event(json.dumps({'result':value,'motor_command':{'T':100}})))

    def test_stop_before_review_does_not_launch_model_process(self):
        stopped=threading.Event();stopped.set()
        with patch('evoarm_rsi.grok_review.subprocess.Popen') as process,self.assertRaises(RuntimeError):
            grok_review.review({},Path('/unused'),stopped,preflight=True)
        process.assert_not_called()
    def test_reasoning_and_extra_motor_fields_are_not_accepted(self):
        payload = verdict()
        events = [{'type': 'reasoning', 'part': {'text': 'private reasoning'}},
                  {'type': 'text', 'part': {'text': json.dumps(payload)}}]
        self.assertEqual(grok_review._final_json('\n'.join(map(json.dumps, events))), payload)
        with self.assertRaises(ValueError): grok_review.validate_verdict({**payload, 'motor_command': {'T': 100}})
        self.assertEqual(grok_review.validate_verdict({**payload, 'confidence': .1})['confidence'], .1)

    def test_local_cli_is_tool_denied_and_uses_only_configured_grok(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); command = root / 'opencode'
            command.write_text('#!/usr/bin/env python3\nimport json,os,sys\np=json.loads(os.environ["OPENCODE_CONFIG_CONTENT"])\nassert p["permission"]["*"]=="deny"\nassert "company/grok-test" in sys.argv\nprint(json.dumps({"type":"text","part":{"text":json.dumps({"ready":True,"note":"synthetic scene"})}}))\n')
            command.chmod(0o755)
            cv2.imwrite(str(root / 'preview.jpg'), np.full((90, 160, 3), 200, np.uint8))
            with patch.dict('os.environ', {'EVOARM_OPENCODE_BIN': str(command), 'EVOARM_OPENCODE_MODEL': 'company/grok-test'}):
                result = grok_review.review({}, root, threading.Event(), preflight=True)
            self.assertTrue(result['ready']); self.assertTrue((root / 'grok-scene-review.json').is_file())
