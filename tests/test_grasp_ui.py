"""UI orchestration and perception failures with simulated hardware only."""
import asyncio
import json
import threading
import time
import unittest
from unittest.mock import patch

import cv2

from control_ui.ball_trial import UI_DECLARATION
from evoarm_rsi import ball_trial
from tests import test_ball_trial as fixtures

scene=fixtures.scene
from tests.test_serial_transport_errors import api_request


def rewrite_rgb(folder, image):
    writer=cv2.VideoWriter(str(folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(1280,720))
    for _ in range(6):writer.write(image)
    writer.release()


class GraspUI(unittest.TestCase):
    setUp=fixtures.BallAPITest.setUp
    request=fixtures.BallAPITest.request
    prepare=fixtures.BallAPITest.prepare
    end_prepared_plans=fixtures.BallAPITest.end_prepared_plans
    def get(self, path, *, image=False):
        return asyncio.run(api_request('/api/bench/grasp/'+path,{},method='GET',decode_json=not image))

    def wait_done(self):
        until=time.monotonic()+3
        while time.monotonic()<until:
            _,state=self.get('ui-state')
            if state['job']['status']!='running':return state
            time.sleep(.01)
        self.fail('UI worker did not finish')

    def test_state_reads_never_unlock_connect_or_move(self):
        self.service.armed_until=0
        self.get('ui-state');self.get('ui-state')
        self.assertEqual(self.service.armed_until,0)
        self.assertEqual(self.arm.commands,[])

    def test_size_failure_includes_numbers_and_actual_mask_without_motion(self):
        image,_=scene()
        cv2.circle(image,(715,381),34,(0,210,230),-1)
        rewrite_rgb(self.folder,image)
        response,data=self.request('prepare',{})
        self.assertEqual(response['status'],409,data)
        detail=data['detail'];self.assertEqual(detail['code'],'ball_size_mismatch')
        self.assertGreater(detail['size']['relative_error_percent'],22)
        self.assertEqual(detail['size']['allowed_relative_error_percent'],22)
        self.assertFalse(detail['size']['passed'])
        for name in ('ball-size-diagnostic.json','ball-size-mask.png','ball-size-overlay.jpg'):
            self.assertTrue((self.folder/name).is_file())
        for url in (detail['image_url'],detail['mask_url']):
            response,raw=asyncio.run(api_request(url,{},method='GET',decode_json=False))
            self.assertEqual(response['status'],200)
            self.assertGreater(len(raw),100)
        _,state=self.get('ui-state')
        self.assertEqual(state['job']['last_error']['size'],detail['size'])
        self.assertIsNone(state['plan']);self.assertEqual(self.arm.commands,[])

    def test_small_attached_yellow_spur_does_not_enlarge_ball_to_enclosing_circle(self):
        image,_=scene()
        cv2.rectangle(image,(731,378),(750,384),(0,210,230),-1)
        rewrite_rgb(self.folder,image)
        ball,_,_=ball_trial.observe_ball(self.folder,json.loads((self.home/'.evoarm/rsi/calibrations/bench-layout-v2/session.json').read_text()))
        size=ball['size_evidence']
        self.assertTrue(size['passed'],size)
        self.assertLess(abs(size['relative_error_percent']),10,size)
        self.assertGreater(size['observed_enclosing_diameter_px']/size['expected_diameter_px'],1.22,size)

    def test_ui_run_requires_declaration_then_runs_once_and_revokes_lease(self):
        plan=self.prepare();key=plan['plan_id'];self.service.armed_until=0
        response,_=self.request('run',{'plan_id':key,'declaration':'unchecked'})
        self.assertEqual(response['status'],409);self.assertEqual(self.service.armed_until,0)
        entered=threading.Event();release=threading.Event()
        def runner(plan_id,**kwargs):
            self.assertTrue(self.service.health()['armed']);entered.set()
            kwargs['progress']({'phase':'hover','health':self.service.health()})
            release.wait(3)
            return {'motion_completed':True,'grasp_attempted':True,'recording_covers_motion':True}
        with patch('evoarm_rsi.grasp_once.execute',side_effect=runner) as execute:
            response,_=self.request('run',{'plan_id':key,'declaration':UI_DECLARATION})
            self.assertEqual(response['status'],202);self.assertTrue(entered.wait(1))
            response,_=self.request('run',{'plan_id':key,'declaration':UI_DECLARATION})
            self.assertEqual(response['status'],409)
            _,state=self.get('ui-state');self.assertEqual(state['job']['phase'],'hover')
            release.set();state=self.wait_done()
            self.assertEqual(state['job']['status'],'completed',state)
            self.assertEqual(execute.call_count,1)
        self.assertEqual(self.service.armed_until,0);self.assertEqual(self.arm.commands,[])

    def test_stop_ack_does_not_wait_for_serial_lock_and_prevents_next_command(self):
        key=self.prepare()['plan_id'];entered=threading.Event();release=threading.Event()
        with patch('evoarm_rsi.grasp_once.execute',side_effect=lambda *a,**k:(entered.set(),release.wait(3),{'motion_completed':False})[-1]):
            response,data=self.request('run',{'plan_id':key,'declaration':UI_DECLARATION})
            self.assertEqual(response['status'],202,data);self.assertTrue(entered.wait(1))
            locked=threading.Event();unlock=threading.Event()
            def serial_busy():
                with self.service.lock:locked.set();unlock.wait(3)
            thread=threading.Thread(target=serial_busy);thread.start();self.assertTrue(locked.wait(1))
            try:
                before=time.monotonic();response,data=self.request('stop',{'plan_id':key})
                self.assertEqual(response['status'],200);self.assertTrue(data['stop_requested'])
                self.assertLess(time.monotonic()-before,.5)
                # Polling uses cached health, so it also stays responsive.
                _,state=self.get('ui-state');self.assertEqual(state['job']['phase'],'stopping')
            finally:unlock.set();thread.join(2)
            release.set();self.assertEqual(self.wait_done()['job']['status'],'stopped')
        # Even another explicit lease cannot revive this stopped plan.
        self.service.armed_until=time.monotonic()+60
        response,_=self.request('begin',{'plan_id':key,'camera_request_id':self.nonce})
        self.assertEqual(response['status'],409);self.assertEqual(self.arm.commands,[])

    def test_thread_start_failure_revokes_authorization(self):
        key=self.prepare()['plan_id'];self.service.armed_until=0
        original=threading.Thread.start
        def start(thread):
            if thread.name=='evoarm-ui-grasp':raise RuntimeError('thread unavailable')
            return original(thread)
        with patch('control_ui.ball_trial.threading.Thread.start',new=start):
            response,_=self.request('run',{'plan_id':key,'declaration':UI_DECLARATION})
        self.assertEqual(response['status'],409);self.assertEqual(self.service.armed_until,0)
        self.assertEqual(self.get('ui-state')[1]['job']['status'],'failed')
        self.assertEqual(self.arm.commands,[])

    def test_fixed_image_routes_cannot_read_other_files_or_symlinks(self):
        key=self.prepare()['plan_id']
        self.assertEqual(self.get('plans/'+key+'/preview',image=True)[0]['status'],200)
        for path in ('diagnostics/'+self.folder.name+'/manifest.json','plans/not-a-uuid/preview','evidence/'+'a'*32+'/keyframes'):
            self.assertEqual(self.get(path,image=True)[0]['status'],404)
        path=self.home/'.evoarm/rsi/grasp_plans'/key/'preview.jpg';path.unlink();path.symlink_to(self.folder/'rgb.mp4')
        self.assertEqual(self.get('plans/'+key+'/preview',image=True)[0]['status'],404)
