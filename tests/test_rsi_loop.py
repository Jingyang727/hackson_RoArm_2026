"""RSI API/review and one complete synthetic-device execution, never real motors."""
import asyncio
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
import uuid
from unittest.mock import patch

import cv2
import numpy as np

from control_ui import main
from control_ui.service import ArmService
from control_ui.rsi_trial import DECLARATION
from control_ui.safety import UnsafeMotion
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS,observe_scene,build_plan
from evoarm_rsi.rsi_execution import execute,reservation,observe_pending
from evoarm_rsi.browser_video import browser_video
from evoarm_rsi.roarm_geometry import inverse
from tests.test_ball_trial import SimulatedArm,layout
from tests.test_rsi_pick_place import scene
from tests.test_serial_transport_errors import api_request


class InstantArm(SimulatedArm):
    def send(self,command):
        self.commands.append(dict(command))
        if command['T']==104:self.raw=inverse([command[k] for k in 'xyz'],command['t'],command['r'],command['g'])
        elif command['T']==106:self.raw['g']=command['cmd']
        else:raise AssertionError('unexpected motor command')


class BrowserPlayback(unittest.TestCase):
    def test_h264_copy_is_cached_and_does_not_modify_raw_recording(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);source=folder/'rgb.mp4'
            writer=cv2.VideoWriter(str(source),cv2.VideoWriter_fourcc(*'mp4v'),10,(160,90))
            for _ in range(10):writer.write(np.full((90,160,3),180,np.uint8))
            writer.release();original=source.read_bytes()
            output=browser_video(folder);self.assertNotEqual(output,source)
            self.assertEqual(source.read_bytes(),original)
            cap=cv2.VideoCapture(str(output))
            try:self.assertEqual(int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),10)
            finally:cap.release()
            with patch('evoarm_rsi.browser_video.subprocess.run',side_effect=AssertionError('cache should be used')):
                self.assertEqual(browser_video(folder),output)


class RSIApi(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        self.session=layout(self.home);self.folder,self.image,_=scene(self.home,self.session)
        self.arm=InstantArm();self.service=ArmService();self.service.arm=self.arm
        _,jpeg=cv2.imencode('.jpg',self.image)
        def camera(home,method,path,payload=None):
            if path=='/health':return {'camera_service':'ready','robot_control':False,'recording':False,'recording_request_ids':True}
            return {'trial_id':self.folder.name}
        def execution(service,plan,root,stop,publish,**kwargs):
            service.pending=None;service.disarm()
            return {'motion_completed':True,'grasp_attempted':True,'placement_attempted':True,'physical_elapsed_s':7.2,
                    'within_10s':True,'recording_covers_motion':True,'motion_error':None,'camera_error':None,
                    'capture_trial_id':self.folder.name,'keyframes_file':str(self.folder/'grasp-keyframes.jpg')}
        self.execute_patch=patch('control_ui.rsi_trial.execute',side_effect=execution);self.execute_mock=self.execute_patch.start();self.addCleanup(self.execute_patch.stop)
        for p in (patch.object(main,'service',self.service),patch.object(Path,'home',return_value=self.home),
                  patch('control_ui.rsi_trial.client_request',side_effect=camera),
                  patch('control_ui.rsi_trial.preview_frame',return_value={'jpeg':jpeg.tobytes()}),
                  patch('control_ui.rsi_trial._launch',side_effect=lambda target:target()),
                  patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1','EVOARM_SETUP_ID':'bench-layout-v2'})):
            p.start();self.addCleanup(p.stop)

    def request(self,path,body=None,method='POST'):
        return asyncio.run(api_request('/api/bench/rsi/'+path,body or {},method=method))
    def prepare(self):
        response,data=self.request('prepare');self.assertEqual(response['status'],200,data);return data['plan_id']
    def start_trial(self,key):return self.request('run',{'plan_id':key,'declaration':DECLARATION})

    def test_prepare_is_read_only_uses_depth_and_contains_release(self):
        key=self.prepare();_,data=self.request('state',method='GET')
        self.assertTrue(data['plan']['scene']['ball']['depth_evidence']['depth_used'])
        self.assertIn('release',data['plan']['steps'])
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)
        response,image=asyncio.run(api_request('/api/bench/rsi/plans/'+key+'/depth',{},method='GET',decode_json=False))
        self.assertEqual(response['status'],200);self.assertGreater(len(image),100)

    def test_stale_parameters_from_planner_are_rejected_before_any_dispatch(self):
        original=build_plan
        def stale(*args,**kwargs):
            plan=original(*args,**kwargs);plan['parameters']=copy.deepcopy(plan['parameters'])
            plan['parameters']['pickup_xy_mm']=[9.,9.]
            return plan
        with patch('control_ui.rsi_trial.build_plan',side_effect=stale):response,error=self.request('prepare')
        self.assertEqual(response['status'],409);self.assertIn('当前RSI参数',error['detail']['message'])
        self.assertEqual(self.execute_mock.call_count,0);self.assertEqual(self.arm.commands,[])

    def test_one_run_requires_confirmation_and_does_not_replay(self):
        key=self.prepare()
        self.assertEqual(self.request('run',{'plan_id':key,'declaration':''})[0]['status'],409)
        self.assertEqual(self.start_trial(key)[0]['status'],202)
        self.assertEqual(self.start_trial(key)[0]['status'],409)
        self.assertEqual(self.execute_mock.call_count,1)
        _,state=self.request('state',method='GET');self.assertEqual(state['job']['status'],'completed')
        self.assertTrue(state['job']['result']['placement_attempted'])

    def test_review_updates_once_and_next_prepare_uses_new_policy(self):
        key=self.prepare();self.start_trial(key)
        body={'plan_id':key,'outcome':'grasp_missed','direction':'image_left','source':'human'}
        response,review=self.request('review',body);self.assertEqual(response['status'],200,review)
        self.assertEqual(review['after_version'],2)
        self.assertAlmostEqual(math.hypot(*review['after']['pickup_xy_mm']),4.)
        self.assertEqual(self.request('review',body)[1]['after_version'],2)
        self.prepare();_,state=self.request('state',method='GET')
        self.assertEqual(state['plan']['policy_version'],2)
        self.assertEqual(state['plan']['parameters']['pickup_xy_mm'],review['after']['pickup_xy_mm'])
        self.assertEqual(self.execute_mock.call_count,1)

    def test_old_prepared_plan_is_invalidated_after_new_observation_failure(self):
        key=self.prepare()
        with patch('control_ui.rsi_trial.observe_scene',side_effect=ValueError('funnel unavailable')):
            self.assertEqual(self.request('prepare')[0]['status'],409)
        self.assertEqual(self.start_trial(key)[0]['status'],409)
        _,state=self.request('state',method='GET');self.assertIsNone(state['plan'])
        self.assertEqual(self.execute_mock.call_count,0)

    def test_changed_pose_calibration_and_pending_reject_before_execution(self):
        key=self.prepare();self.arm.raw['b']+=math.radians(2)
        self.assertEqual(self.start_trial(key)[0]['status'],409);self.assertEqual(self.execute_mock.call_count,0)
        self.assertEqual(self.arm.commands,[])

    def test_review_schema_cannot_supply_motor_values(self):
        key=self.prepare();self.start_trial(key)
        for extra in ({'motor_command':{'T':100}},{'delta_mm':100}):
            self.assertEqual(self.request('review',{'plan_id':key,'outcome':'grasp_missed',**extra})[0]['status'],422)
        self.assertEqual(self.arm.commands,[])

    def test_review_cannot_claim_success_without_placement(self):
        key=self.prepare()
        result={'motion_completed':False,'grasp_attempted':False,'placement_attempted':False,'motion_error':'no motion','camera_error':None}
        with patch('control_ui.rsi_trial.execute',return_value=result):self.start_trial(key)
        self.assertEqual(self.request('review',{'plan_id':key,'outcome':'placed'})[0]['status'],409)
        self.assertEqual(self.request('review',{'plan_id':key,'outcome':'execution_failed'})[0]['status'],200)

    def test_historical_context_and_review_survive_service_restart_without_replay(self):
        key=self.prepare();self.start_trial(key)
        from fastapi import FastAPI
        from control_ui.rsi_trial import install
        app=FastAPI();install(app,lambda:self.service)
        with patch.object(main,'app',app):
            response,context=self.request('context/'+key,method='GET')
            self.assertEqual(response['status'],200,context);self.assertIn('定性',context['instructions'])
            self.assertEqual(self.start_trial(key)[0]['status'],409)
            self.assertEqual(self.request('review',{'plan_id':key,'outcome':'placed'})[0]['status'],200)


class Execution(unittest.TestCase):
    def test_shorter_rsi_pacing_is_explicit_and_default_manual_gap_is_unchanged(self):
        service=ArmService();service.arm=InstantArm();service.armed_until=time.monotonic()+60
        service.last_motion_at=time.monotonic()-.2
        with patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
            with self.assertRaisesRegex(UnsafeMotion,'间隔'):service._preflight(1)
            self.assertEqual(service._preflight(1,minimum_gap_s=.12)['T'],1051)
            for value in (-1,float('nan'),True):
                with self.assertRaises(UnsafeMotion):service._preflight(1,minimum_gap_s=value)
        self.assertEqual(service.arm.commands,[])

    def test_camera_failure_before_motion_disarms_without_any_motor_write(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);home=Path(temp.name);saved=layout(home)
        folder,_,_=scene(home,saved);world,_,_,_=observe_scene(folder,saved);arm=InstantArm();service=ArmService();service.arm=arm
        plan=build_plan(saved,arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS));key=uuid.uuid4().hex
        def unavailable(*args,**kwargs):raise OSError('camera unavailable')
        with patch.object(Path,'home',return_value=home),patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
            service.unlock_once('已检查设备并清空运动区域');service.pending=reservation(key)
            result=execute(service,plan,home/key,threading.Event(),lambda *a,**kw:None,camera=unavailable)
        self.assertEqual(arm.commands,[]);self.assertIs(result['robot_moved'],False)
        self.assertEqual(service.armed_until,0);self.assertIsNone(service.pending)
    def test_uncertain_rsi_write_is_consumed_and_never_retried(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);home=Path(temp.name);saved=layout(home)
        folder,_,_=scene(home,saved);world,_,_,_=observe_scene(folder,saved);arm=InstantArm();service=ArmService();service.arm=arm
        plan=build_plan(saved,arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS));key=uuid.uuid4().hex;done=threading.Event();camera_state={}
        def camera(task_home,method,path,payload=None):
            if method=='GET':return dict(camera_state)
            camera_state.update(recording=True,phase='recording',recording_request_id=payload['request_id'],recording_started_monotonic_ns=time.monotonic_ns())
            done.wait(5);return {'trial_id':folder.name,'request_id':payload['request_id']}
        def lost(command):arm.commands.append(command);raise OSError('write may have arrived')
        def publish(phase,**kwargs):
            if phase=='saving_recording':done.set()
        with patch.object(Path,'home',return_value=home),patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}),patch.object(arm,'send',side_effect=lost):
            service.unlock_once('已检查设备并清空运动区域');service.pending=reservation(key)
            result=execute(service,plan,home/key,threading.Event(),publish,camera=camera)
            with self.assertRaisesRegex(RuntimeError,'收据'):execute(service,plan,home/key,threading.Event(),publish,camera=camera)
        self.assertEqual(len(arm.commands),1);self.assertIsNone(result['robot_moved'])
        self.assertFalse(result['grasp_attempted']);self.assertEqual(service.fault['code'],'motion_transmission_unverified')
        self.assertEqual(service.armed_until,0)

    def test_status_contact_candidate_never_writes_and_reserved_session_blocks_foreign_motion(self):
        service=ArmService();service.arm=InstantArm();service.armed_until=time.monotonic()+60
        key=uuid.uuid4().hex;service.pending=reservation(key)
        with patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
            with self.assertRaises(UnsafeMotion):service.step(1,5)
            with self.assertRaises(UnsafeMotion):service.factory_reset()
        pending={'kind':'rsi_trial','run_id':key,'index':2,'stage_kind':'close','start':1.9,'target':3.14,'since':0.}
        service.pending=pending;raw={**service.arm.status(),'g':2.8,'tG':0}
        for now in (.3,.5,.7):observe_pending(service,pending,raw,now)
        self.assertTrue(pending['contact_candidate']);self.assertEqual(service.arm.commands,[])

    def test_complete_pick_place_and_real_ten_second_recording_contract(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);home=Path(temp.name);saved=layout(home)
        folder,image,depth=scene(home,saved);world,_,_,_=observe_scene(folder,saved)
        arm=InstantArm();service=ArmService();service.arm=arm
        plan=build_plan(saved,arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS));key=uuid.uuid4().hex
        root=home/'.evoarm/rsi/pick_place_runs'/key;camera_state={};camera_started=threading.Event()
        def camera(task_home,method,path,payload=None):
            if method=='GET':return dict(camera_state)
            stamp=time.monotonic_ns();camera_state.update(recording=True,phase='recording',recording_request_id=payload['request_id'],recording_started_monotonic_ns=stamp)
            camera_started.set()
            time.sleep(10)
            trial=home/'.evoarm/rsi/experiments'/uuid.uuid4().hex;trial.mkdir(parents=True)
            writer=cv2.VideoWriter(str(trial/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(1280,720))
            for _ in range(101):writer.write(image)
            writer.release();stamps=[stamp+i*100_000_000 for i in range(101)]
            (trial/'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns':s}) for s in stamps))
            (trial/'manifest.json').write_text(json.dumps({'trial_id':trial.name,'status':'recorded','simulated':False}))
            np.savez_compressed(trial/'depth_samples.npz',depth_mm=np.stack([depth]*3),monotonic_ns=[stamp,stamp+5_000_000_000,stamp+10_000_000_000])
            return {'trial_id':trial.name,'request_id':payload['request_id']}
        def publish(phase,**values):
            if phase=='saving_recording':self.assertEqual(service.pending['stage_kind'],'reserved')
        with patch.object(Path,'home',return_value=home),patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
            service.unlock_once('已检查设备并清空运动区域');service.pending=reservation(key)
            result=execute(service,plan,root,threading.Event(),publish,camera=camera)
        self.assertTrue(result['motion_completed'],result.get('motion_error'))
        self.assertTrue(result['placement_attempted']);self.assertTrue(result['recording_covers_motion'])
        self.assertTrue(result['within_10s']);self.assertIsNone(result['visual_success'])
        self.assertEqual(arm.commands[-1]['T'],106);self.assertAlmostEqual(arm.commands[-1]['cmd'],math.radians(110))
        self.assertTrue(any(c['T']==104 and c['spd']==.65 for c in arm.commands))
        self.assertTrue(any(c['T']==106 and c['spd']==1000 and c['acc']==20 for c in arm.commands))
        self.assertFalse(any('acc' in c for c in arm.commands if c['T']==104))
        self.assertEqual(service.armed_until,0);self.assertIsNone(service.pending)
