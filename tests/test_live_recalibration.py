"""Moved layouts get a fresh board reference, never an old pose or motor write."""
import asyncio
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from control_ui import bench_ui,main
from control_ui.service import ArmService
from evoarm_rsi import bench_calibration,bench_start
from evoarm_rsi.bench_layout import active_session,begin_revision,calibration_root
from tests.test_bench_calibration import make_session
from tests.test_robot_calibration import FakeArm
from tests.test_serial_transport_errors import api_request
from tests.test_vision import example_image


class LiveRecalibration(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        self.root=calibration_root(self.home,'bench-layout-v2');self.root.mkdir(parents=True)
        self.old=self.root/'session.json';make_session(self.old);self.original=self.old.read_bytes()
        self.service=ArmService();self.service.arm=FakeArm()
        for p in (patch.object(Path,'home',return_value=self.home),patch.object(main,'service',self.service),
                  patch.dict('os.environ',{'EVOARM_SETUP_ID':'bench-layout-v2'})):
            p.start();self.addCleanup(p.stop)

    def request(self,path,payload=None,method='POST',**kwargs):
        return asyncio.run(api_request('/api/bench/'+path,payload or {},method=method,**kwargs))

    def camera(self,home,method,path,payload=None):
        if path=='/health':return {'camera_service':'ready','robot_control':False,'recording':False}
        self.assertEqual((method,path,payload),('POST','/record',{'seconds':2}))
        return {'trial_id':'a'*32}

    def source(self,*,board=True):
        folder=self.home/'.evoarm/rsi/experiments'/('a'*32);folder.mkdir(parents=True)
        image=cv2.imdecode(np.frombuffer(example_image(),np.uint8),cv2.IMREAD_COLOR) if board else np.full((960,1280,3),220,np.uint8)
        writer=cv2.VideoWriter(str(folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(1280,960))
        for _ in range(3):writer.write(image)
        writer.release()
        (folder/'manifest.json').write_text(json.dumps({'simulated':False,'trial_id':folder.name,'status':'recorded'}))
        (folder/'gemini_color_intrinsics.json').write_text(json.dumps({'width':1280,'height':960}))

    def test_detects_actual_fifty_four_corners_and_keeps_old_files_and_torque(self):
        self.source()
        with patch('evoarm_rsi.camera_service.client_request',side_effect=self.camera), \
             patch.object(self.service,'status',side_effect=AssertionError('no robot reads')):
            response,result=self.request('recalibrate',{'square_mm':24.})
        self.assertEqual(response['status'],200,result);self.assertEqual(result['corner_count'],54)
        self.assertEqual(result['next_point'],'P1');self.assertIsNone(result['fit'])
        self.assertFalse(result['motion_commands_sent']);self.assertEqual(self.service.arm.commands,[])
        self.assertEqual(self.old.read_bytes(),self.original)
        target=active_session(self.home,'bench-layout-v2');self.assertNotEqual(target,self.old)
        _,session=bench_calibration.load_session(target)
        self.assertEqual(session['records'],{});self.assertEqual(len(session['reference']['corners_px']),54)
        self.assertTrue(target.with_suffix('.png').is_file());self.assertEqual(bench_ui.session_path(),target)

    def test_detection_failure_keeps_old_data_but_does_not_fall_back_to_old_layout(self):
        self.source(board=False)
        with patch('evoarm_rsi.camera_service.client_request',side_effect=self.camera):
            response,error=self.request('recalibrate',{'square_mm':24.})
        self.assertEqual(response['status'],409,error)
        self.assertNotEqual(bench_ui.session_path(),self.old);self.assertFalse(bench_ui.session_path().exists())
        _,status=self.request('status',method='GET')
        self.assertTrue(status['recalibration_required']);self.assertFalse(status['session_exists'])
        self.assertEqual(self.old.read_bytes(),self.original);self.assertEqual(self.service.arm.commands,[])
        with patch('evoarm_rsi.bench_start._controller_get',side_effect=AssertionError('old layout must not be used')):
            with self.assertRaises(FileNotFoundError):bench_start.run(setup_id='bench-layout-v2',home=self.home)

    def test_camera_failure_also_leaves_layout_in_need_of_recalibration(self):
        with patch('evoarm_rsi.camera_service.client_request',side_effect=RuntimeError('camera disconnected')):
            self.assertEqual(self.request('recalibrate',{'square_mm':24.})[0]['status'],409)
        _,state=self.request('status',method='GET');self.assertTrue(state['recalibration_required'])
        self.assertEqual(self.old.read_bytes(),self.original)

    def test_active_motion_cannot_switch_layout_or_start_camera_capture(self):
        self.service.pending={'kind':'coordinate_test'}
        with patch('evoarm_rsi.camera_service.client_request',side_effect=AssertionError('must not capture')):
            self.assertEqual(self.request('recalibrate',{'square_mm':24.})[0]['status'],409)
        self.assertEqual(bench_ui.session_path(),self.old);self.assertEqual(self.old.read_bytes(),self.original)

    def test_raw_live_preview_is_available_before_any_reference_or_five_points(self):
        begin_revision(self.home,'bench-layout-v2')
        frame={'jpeg':b'current camera','width':900,'height':500,'sequence':1,'age_ms':0.,'stamp_ns':time.monotonic_ns()}
        with patch.object(self.service,'status',side_effect=AssertionError('no robot access')), \
             patch('evoarm_rsi.camera_service.client_request',return_value={'started':True}), \
             patch('evoarm_rsi.camera_service.preview_frame',return_value=frame):
            self.assertEqual(self.request('live/start')[0]['status'],200)
            response,image=self.request('live-frame',method='GET',decode_json=False)
        self.assertEqual(response['status'],200);self.assertEqual(image,frame['jpeg'])

    def test_invalid_revision_pointer_cannot_escape_or_silently_reactivate_old_session(self):
        (self.root/'active-session.json').write_text(json.dumps({'revision_id':'../../old'}))
        with self.assertRaises(ValueError):bench_ui.session_path()
        self.assertEqual(self.old.read_bytes(),self.original)
