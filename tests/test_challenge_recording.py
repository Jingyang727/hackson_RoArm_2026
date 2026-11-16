"""Opt-in action-length challenge videos; ordinary recording stays unchanged."""
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
import uuid
from unittest.mock import patch

import cv2
import numpy as np

from control_ui.service import ArmService
from evoarm_rsi.camera_service import CameraServer,client_request
from evoarm_rsi.camera_hub import CameraHub
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS,observe_scene,build_plan
from evoarm_rsi.rsi_execution import execute,reservation
from tests.test_camera_hub import SyntheticSource,wait_for
from tests.test_ball_trial import layout
from tests.test_rsi_loop import InstantArm
from tests.test_rsi_pick_place import scene


class AdaptiveCapture(unittest.TestCase):
    def test_only_matching_opted_in_capture_can_finish_early_with_paired_last_depth(self):
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp);sock=home/'camera.sock';srv=CameraServer(sock,os.getuid(),os.getgid(),home,hub=CameraHub(SyntheticSource))
            thread=threading.Thread(target=srv.serve_forever,daemon=True);thread.start()
            try:
                with patch('evoarm_rsi.camera_service.socket_path',return_value=sock),ThreadPoolExecutor(max_workers=1) as pool:
                    key=uuid.uuid4().hex
                    future=pool.submit(client_request,home,'POST','/record',{'seconds':20,'request_id':key,'allow_early_finish':True})
                    wait_for(lambda:client_request(home,'GET','/health')['phase']=='recording')
                    self.assertTrue(client_request(home,'GET','/health')['recording_finish'])
                    with self.assertRaisesRegex(RuntimeError,'HTTP 409'):client_request(home,'POST','/record/finish',{'request_id':'f'*32})
                    self.assertFalse(future.done());time.sleep(.18)
                    self.assertTrue(client_request(home,'POST','/record/finish',{'request_id':key})['finish_requested'])
                    result=future.result(timeout=3);folder=home/'.evoarm/rsi/experiments'/result['trial_id']
                    self.assertTrue(result['capture']['ended_early'])
                    stamps=[json.loads(r)['host_monotonic_ns'] for r in (folder/'frames.jsonl').read_text().splitlines()]
                    self.assertGreaterEqual(len(stamps),3);self.assertLess((stamps[-1]-stamps[0])/1e9,2)
                    with np.load(folder/'depth_samples.npz',allow_pickle=False) as depth:self.assertEqual(int(depth['monotonic_ns'][-1]),stamps[-1])
                    # An ordinary request with an ID still records the whole
                    # interval; the finish endpoint cannot shorten it.
                    key=uuid.uuid4().hex
                    future=pool.submit(client_request,home,'POST','/record',{'seconds':.4,'request_id':key})
                    wait_for(lambda:client_request(home,'GET','/health')['phase']=='recording')
                    with self.assertRaisesRegex(RuntimeError,'HTTP 409'):client_request(home,'POST','/record/finish',{'request_id':key})
                    self.assertNotIn('ended_early',future.result(timeout=3)['capture'])
            finally:srv.shutdown();srv.server_close();thread.join(timeout=2)

    def test_executor_requests_finish_after_release_and_accepts_complete_short_video(self):
        with tempfile.TemporaryDirectory() as temp:
            home=Path(temp);saved=layout(home);folder,image,depth=scene(home,saved)
            world,*_=observe_scene(folder,saved);service=ArmService();service.arm=InstantArm()
            plan=build_plan(saved,service.arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS))
            key=uuid.uuid4().hex;run=home/key;state={};done=threading.Event();calls=[]
            def camera(task_home,method,path,payload=None):
                calls.append((method,path,payload))
                if path=='/health':return dict(state)
                if path=='/record/finish':
                    self.assertTrue(any(c['T']==106 for c in service.arm.commands));self.assertEqual(payload['request_id'],state['recording_request_id'])
                    done.set();return {'finish_requested':True}
                self.assertTrue(payload['allow_early_finish']);start=time.monotonic_ns()
                state.update(recording=True,phase='recording',recording_request_id=payload['request_id'],recording_started_monotonic_ns=start)
                if not done.wait(12):raise RuntimeError('test did not request finish')
                end=time.monotonic_ns()+80_000_000;stamps=list(range(start-10_000_000,end,70_000_000))+[end]
                writer=cv2.VideoWriter(str(folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),15,(1280,720))
                for _ in stamps:writer.write(image)
                writer.release();(folder/'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns':s}) for s in stamps))
                np.savez_compressed(folder/'depth_samples.npz',depth_mm=np.stack([depth,depth]),monotonic_ns=[stamps[0],stamps[-1]])
                return {'trial_id':folder.name,'request_id':payload['request_id']}
            with patch.object(Path,'home',return_value=home),patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1'}):
                service.unlock_once('已检查设备并清空运动区域');service.pending=reservation(key)
                report=execute(service,plan,run,threading.Event(),lambda *a,**v:None,camera=camera,finish_recording=True)
            self.assertTrue(report['motion_completed'],report.get('motion_error'));self.assertTrue(report['placement_attempted'])
            self.assertIsNone(report['camera_error']);self.assertTrue(report['recording_covers_motion'])
            self.assertLess(report['recording_duration_s'],20);self.assertEqual(report['recording_mode'],'until_motion_end')
            self.assertEqual(len([c for c in calls if c[1]=='/record/finish']),1)
            self.assertFalse(any(c['T']==100 for c in service.arm.commands))
