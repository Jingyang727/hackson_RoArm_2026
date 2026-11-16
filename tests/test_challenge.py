"""Multi-colour RGB-D and the cleanup loop; all motors and models are fake."""
import asyncio
import base64
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
import uuid
from unittest.mock import patch

import cv2
import numpy as np
from fastapi import FastAPI

from control_ui import main
from control_ui.challenge import install
from control_ui.service import ArmService
from evoarm_rsi import challenge_scene as vision,challenge_loop as loop
from evoarm_rsi.ball_trial import camera_geometry
from evoarm_rsi.coordinate_test import fingerprint
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS,build_plan
from evoarm_rsi.rsi_route import TRANSFER_HEIGHT_POLICY
from tests.test_ball_trial import layout,K,SimulatedArm
from tests.test_serial_transport_errors import api_request

POSITIONS={'red':[330.,-40.,-98.],'green':[330.,10.,-98.],'blue':[330.,60.,-98.]}
BGR={'red':(0,0,235),'green':(0,210,0),'blue':(235,0,0),'white':(245,245,245),'black':(20,20,20)}


def synthetic(saved,positions=POSITIONS,*,holes=False):
    g=camera_geometry(saved,K);image=np.full((720,1280,3),220,np.uint8);depth=np.full((720,1280),500,np.uint16)
    def project(xyz):
        camera=g['R'].T@(np.asarray(xyz)-g['t']);uv=g['K']@camera
        return uv[:2]/uv[2],camera
    mouth=np.array([290.,-110.,-69.6]);uv,cam=project(mouth);rim=600*38/cam[2]
    cv2.circle(image,tuple(np.round(uv).astype(int)),round(rim),(18,18,18),-1)
    cv2.circle(depth,tuple(np.round(uv).astype(int)),round(rim),round(cam[2]),5)
    angle=np.linspace(0,2*math.pi,96,endpoint=False)
    boundary=[project(mouth+[38*math.cos(a),38*math.sin(a),0])[0].tolist() for a in angle]
    bowl={'mouth_center_robot_mm':mouth.tolist(),'pixel_center':uv.tolist(),'mouth_boundary_px':boundary,'radius_mm':38.,'height_mm':45.4}
    boxes=[]
    for color,xyz in positions.items():
        uv,cam=project(xyz);radius=600*17/cam[2];pixel=tuple(np.round(uv).astype(int))
        cv2.circle(image,pixel,round(radius),BGR[color],-1)
        yy,xx=np.indices(depth.shape);ys,xs=np.nonzero((xx-uv[0])**2+(yy-uv[1])**2<radius**2)
        rays=np.c_[xs,ys,np.ones(len(xs))]@g['inverse_K'].T
        a=np.sum(rays*rays,axis=1);b=rays@cam;c=cam@cam-17**2;disc=b*b-a*c;good=disc>=0
        z=(b[good]-np.sqrt(disc[good]))/a[good];depth[ys[good],xs[good]]=0 if holes else np.round(z).astype(np.uint16)
        boxes.append({'color':color,'bbox_norm':[(uv[0]-radius)/1280,(uv[1]-radius)/720,(uv[0]+radius)/1280,(uv[1]+radius)/720]})
    frame={'trial_id':uuid.uuid4().hex,'bgr':image,'depth_mm':depth,'frame_index':3,'host_monotonic_ns':1234}
    balls,unresolved=vision.locate(frame,g,bowl,boxes if holes else ())
    review={'view_clear':True,'table_balls':boxes,'bowl_ball_colors':[],'note':'synthetic visible table'}
    scene={'balls':balls,'unresolved':unresolved,'bowl':bowl,'view_clear':True,'bowl_ball_colors':[],
        'inventory_review':review,'fresh_model_review':True,'source_trial_id':frame['trial_id'],'frame_index':3,
        'host_monotonic_ns':1234,'image_size':[1280,720],'table_plane':g['plane'],
        'camera_geometry':{k:g[k].tolist() for k in ('K','inverse_K','R','t','normal_camera')},'warnings':[]}
    return scene,g,image,frame,review


class ColourDepth(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name);self.saved=layout(self.home)

    def test_three_colours_use_surface_depth_and_fixed_34mm_geometry(self):
        scene,g,*_=synthetic(self.saved)
        self.assertEqual({b['color'] for b in scene['balls']},set(POSITIONS))
        for ball in scene['balls']:
            self.assertTrue(ball['depth_evidence']['depth_used']);self.assertEqual(ball['radius_mm'],17)
            np.testing.assert_allclose(ball['center_robot_mm'],POSITIONS[ball['color']],atol=1.)
            plan=build_plan(self.saved,SimulatedArm().status(),vision.single_scene(scene,ball),copy.deepcopy(DEFAULT_PARAMETERS))
            self.assertEqual(plan['route_policy']['transfer_height_policy'],TRANSFER_HEIGHT_POLICY)
            self.assertIn('release',[s['stage'] for s in plan['steps']])
            self.assertGreater(plan['route_policy']['transfer_height_mm'],160)

    def test_white_and_black_require_depth_foreground_not_table_or_container(self):
        scene,*_=synthetic(self.saved,{'white':POSITIONS['red'],'black':POSITIONS['green']})
        self.assertEqual({b['color'] for b in scene['balls']},{'white','black'})

    def test_red_hue_wrap_is_the_same_target_colour(self):
        scene,g,image,frame,*_=synthetic(self.saved,{'red':POSITIONS['red']})
        mask=cv2.inRange(image,np.array([0,0,230]),np.array([1,1,255]))
        wrapped=cv2.cvtColor(np.uint8([[[179,255,235]]]),cv2.COLOR_HSV2BGR)[0,0]
        frame['bgr'][mask>0]=wrapped
        balls,_=vision.locate(frame,g,scene['bowl']);self.assertEqual([b['color'] for b in balls],['red'])

    def test_ball_inside_container_is_excluded_but_outside_ball_remains(self):
        scene,*_=synthetic(self.saved,{'red':[290,-110,-98],'blue':POSITIONS['blue']})
        self.assertEqual([b['color'] for b in scene['balls']],['blue'])

    def test_flat_colour_patch_is_not_a_sphere_and_depth_holes_are_explicit_estimates(self):
        _,g,_,frame,review=synthetic(self.saved)
        frame['depth_mm'][:]=500;scene=synthetic(self.saved)[0]
        self.assertEqual(vision.locate(frame,g,scene['bowl'])[0],[])
        scene,*_=synthetic(self.saved,holes=True)
        self.assertEqual(len(scene['balls']),3)
        self.assertTrue(all(not b['depth_evidence']['depth_used'] for b in scene['balls']))

    def test_inventory_sends_actual_768px_image_once_and_saves_model_evidence(self):
        scene,g,image,frame,review=synthetic(self.saved);requests=[]
        def complete(system,content,stop,**kwargs):
            requests.append(content);return json.dumps(review)
        with patch.object(vision,'complete_content',side_effect=complete),patch.object(vision,'configuration',return_value={'model':'test-vlm'}):
            actual=vision.inventory(image,scene['bowl'],scene['balls'],self.home/'evidence',threading.Event())
        self.assertEqual(actual,review);self.assertEqual(len(requests),1)
        data=requests[0][-1]['image_url']['url'];decoded=cv2.imdecode(np.frombuffer(base64.b64decode(data.split(',')[1]),np.uint8),1)
        self.assertEqual(max(decoded.shape[:2]),768)
        self.assertTrue((self.home/'evidence/inventory-review.json').is_file())

    def test_inventory_rejects_bad_boxes_and_unsupported_colours(self):
        for row in ({'color':'red','bbox_norm':[0,0,2,1]},{'color':'mystery','bbox_norm':[0,0,.1,.1]}):
            with self.assertRaises(ValueError):vision.validate_inventory({'view_clear':True,'table_balls':[row]})

    def test_post_model_refresh_does_not_rescue_targets_from_old_boxes(self):
        scene,g,image,frame,review=synthetic(self.saved,holes=True)
        folder=self.home/'capture';folder.mkdir();(folder/'gemini_color_intrinsics.json').write_text(json.dumps(K))
        root=self.home/'refresh';root.mkdir()
        with patch.object(vision,'load_rgbd',return_value=frame),patch.object(vision,'detect_funnel',return_value=scene['bowl']):
            updated,*_=vision.observe(folder,self.saved,root,threading.Event(),model_review=review)
        self.assertEqual(updated['balls'],[]);self.assertFalse(updated['fresh_model_review'])


class CleanupLoop(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name);self.saved=layout(self.home)
        self.service=ArmService();self.service.arm=SimulatedArm();self.runner=loop.Challenge(lambda:self.service)
        self.world=synthetic(self.saved)[0];self.motion=[];self.reviews=[];self.colors=set(POSITIONS);self.outcomes=[];self.placed=set()
        for item in (patch.object(Path,'home',return_value=self.home),patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1','EVOARM_SETUP_ID':'bench-layout-v2'}),
                     patch.object(loop.grok_review,'configuration',return_value={'configured':True,'model':'test-vlm'}),
                     patch.object(loop,'launch',side_effect=lambda target:target()),patch.object(loop,'recover',side_effect=self.recover),
                     patch.object(self.runner,'_capture',side_effect=self.capture),patch.object(loop,'execute',side_effect=self.execute),
                     patch.object(loop.grok_review,'review',side_effect=self.review)):
            item.start();self.addCleanup(item.stop)
        self.positions=copy.deepcopy(POSITIONS)

    def recover(self,service,plan,result,verdict,root,stop,publish,**kwargs):
        self.assertEqual(service.automation_owner,kwargs['automation_id']);service.pending=None;service.fault=None;service.disarm()
        return {'status':'completed'}

    def capture(self,saved,folder,**kwargs):
        if not self.colors and not self.placed: # Missing balls may not be declared placed.
            if self.runner.snapshot()['empty_checks']>=2:self.runner.stop.set();raise RuntimeError('test stops missing-ball wait')
        scene,g,image,*_=synthetic(self.saved,{c:self.positions[c] for c in self.colors})
        scene['bowl_ball_colors']=sorted(self.placed)
        return scene,g,image

    def execute(self,service,plan,root,stop,publish,**kwargs):
        self.assertEqual(service.automation_owner,kwargs['automation_id']);self.assertEqual(service.pending['kind'],'rsi_trial')
        self.motion.append(copy.deepcopy(plan));service.pending=None;service.disarm()
        return {'motion_completed':True,'grasp_attempted':True,'placement_attempted':True,'physical_elapsed_s':8.1,
                'recording_covers_motion':True,'capture_trial_id':uuid.uuid4().hex,'motion_error':None,'camera_error':None}

    def review(self,context,root,stop,**kwargs):
        color=context['visual_target']['color'];self.reviews.append(color)
        outcome=self.outcomes.pop(0) if self.outcomes else 'placed'
        if outcome=='placed':self.colors.discard(color);self.placed.add(color)
        elif outcome=='slipped':self.positions[color][1]+=12
        return {'outcome':outcome,'direction':'none','magnitude':'unknown','adjustment':'unknown',
                'ball_state':'funnel' if outcome=='placed' else 'table','confidence':.9,'note':'test visual pixels'}

    def start(self):return self.runner.start(loop.DECLARATION)

    def test_three_sequential_targets_do_not_stop_on_first_success_and_require_empty_rechecks(self):
        before=(self.home/'.evoarm/rsi/calibrations/bench-layout-v2/session.json').read_bytes()
        self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(state['attempt'],3)
        self.assertEqual(state['remaining'],0);self.assertEqual(state['empty_checks'],2);self.assertEqual(len(state['confirmed_colors']),3)
        self.assertEqual(set(self.reviews),set(POSITIONS));self.assertIsNone(self.service.automation_owner)
        self.assertEqual((self.home/'.evoarm/rsi/calibrations/bench-layout-v2/session.json').read_bytes(),before)
        self.assertEqual(len(list((self.home/'.evoarm/rsi/backups').glob('*.zip'))),3)
        for file in (self.home/'.evoarm/rsi/langfuse-outbox').glob('*.json'):
            self.assertEqual(json.loads(file.read_text())['task']['kind'],'challenge_upgrade')

    def test_slip_retry_uses_fresh_position_and_continues_to_remaining_balls(self):
        self.outcomes=['slipped'];self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(len(self.motion),4)
        slipped=self.reviews[0];self.assertEqual(self.reviews.count(slipped),2)
        plans=[p for p in self.motion if p['scene']['ball']['color']==slipped]
        self.assertGreater(math.dist(plans[0]['scene']['ball']['center_robot_mm'],plans[1]['scene']['ball']['center_robot_mm']),10)
        self.assertLess(self.motion[1]['parameters']['travel_speed'],self.motion[0]['parameters']['travel_speed'])

    def test_unknown_failure_may_retry_unchanged_parameters(self):
        self.outcomes=['grasp_missed'];self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(len(self.motion),4)
        self.assertEqual(self.motion[0]['parameters'],self.motion[1]['parameters'])

    def test_more_than_five_attempts_are_allowed(self):
        self.outcomes=['grasp_missed']*6;self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(len(self.motion),9)

    def test_model_error_after_motion_retains_backup_and_does_not_repeat(self):
        with patch.object(loop.grok_review,'review',side_effect=RuntimeError('test model unavailable')):self.start()
        state=self.runner.snapshot();self.assertEqual(state['status'],'failed');self.assertEqual(len(self.motion),1)
        self.assertEqual(len(state['trials']),1);self.assertEqual(state['trials'][0]['outcome'],'unknown')
        self.assertEqual(len(list((self.home/'.evoarm/rsi/backups').glob('*.zip'))),1)

    def test_unclear_table_never_dispatches_or_reports_empty_success(self):
        original=self.capture;calls=[]
        def unclear(*args,**kwargs):
            calls.append(1)
            if len(calls)>2:self.runner.stop.set();self.runner.guard()
            scene,g,image=original(*args,**kwargs);scene['view_clear']=False;return scene,g,image
        with patch.object(self.runner,'_capture',side_effect=unclear):self.start()
        self.assertEqual(self.motion,[]);self.assertEqual(self.runner.snapshot()['status'],'stopped')

    def test_claimed_placement_without_release_is_not_counted(self):
        original=self.execute
        def incomplete(*args,**kwargs):
            report=original(*args,**kwargs);report['placement_attempted']=False;return report
        def review(*args,**kwargs):return {'outcome':'placed','ball_state':'funnel','direction':'none','magnitude':'unknown'}
        calls=[];capture=self.capture
        def limited(*args,**kwargs):
            if len(self.motion)>1:self.runner.stop.set();self.runner.guard()
            return capture(*args,**kwargs)
        with patch.object(loop,'execute',side_effect=incomplete),patch.object(loop.grok_review,'review',side_effect=review),patch.object(self.runner,'_capture',side_effect=limited):self.start()
        self.assertEqual(self.runner.snapshot()['confirmed_colors'],[])

    def test_observation_is_motion_free_and_reserves_shared_resources(self):
        original=self.capture
        def checked(*args,**kwargs):
            self.assertIsNotNone(self.service.automation_owner);return original(*args,**kwargs)
        with patch.object(self.runner,'_capture',side_effect=checked):scene=self.runner.observe()
        self.assertEqual(len(scene['balls']),3);self.assertIsNone(self.service.automation_owner)
        self.assertEqual(self.motion,[]);self.assertEqual(self.service.arm.commands,[])

    def test_empty_table_with_no_placement_evidence_is_not_success(self):
        original=self.capture
        def disappearing(saved,folder,**kwargs):
            if self.runner.snapshot()['attempt']:self.colors.clear();self.placed.clear()
            return original(saved,folder,**kwargs)
        self.outcomes=['unknown']
        with patch.object(self.runner,'_capture',side_effect=disappearing):self.start()
        self.assertEqual(self.runner.snapshot()['status'],'stopped');self.assertEqual(self.runner.snapshot()['confirmed_colors'],[])

    def test_unreachable_target_does_not_turn_into_clear_table(self):
        with patch.object(loop,'build_plan',side_effect=lambda saved,raw,scene,params: (_ for _ in ()).throw(ValueError('test unreachable')) if scene['ball']['color']=='red' else build_plan(saved,raw,scene,params)):
            self.start()
        state=self.runner.snapshot();self.assertEqual(state['status'],'failed');self.assertEqual(len(self.motion),2)
        self.assertEqual(state['remaining'],1);self.assertIn('完整抓放路径',state['error'])

    def test_foreign_owner_or_read_only_prevents_start_armed_false_does_not(self):
        self.service.automation_owner='other-task'
        with self.assertRaises(ValueError):self.start()
        self.service.automation_owner=None
        with patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'0'}):
            with self.assertRaises(ValueError):self.start()
        self.assertFalse(self.service.health()['armed']);self.start();self.assertEqual(self.runner.snapshot()['status'],'completed')

    def test_policy_is_seeded_without_rewriting_single_ball_file(self):
        path=self.home/'.evoarm/rsi/pick_place_policies/bench-layout-v2'/f'{fingerprint(self.saved)}.json'
        path.parent.mkdir(parents=True);policy={'version':7,'parameters':{**DEFAULT_PARAMETERS,'travel_speed':.9},'reviews':[]}
        path.write_text(json.dumps(policy));before=path.read_bytes();self.start()
        self.assertEqual(self.motion[0]['parameters']['travel_speed'],.9);self.assertEqual(path.read_bytes(),before)

    def test_cancel_interrupts_before_first_dispatch_and_releases_owner(self):
        def stop_before_observation(*args,**kwargs):self.runner.cancel();self.runner.guard()
        with patch.object(self.runner,'_capture',side_effect=stop_before_observation):self.start()
        self.assertEqual(self.motion,[]);self.assertEqual(self.runner.snapshot()['status'],'stopped');self.assertIsNone(self.service.automation_owner)


class ChallengeAPI(unittest.TestCase):
    def test_loading_and_state_are_read_only_and_no_dimension_input_is_added(self):
        app=FastAPI();service=ArmService();runner=install(app,lambda:service)
        with patch.object(main,'app',app):
            response,payload=asyncio.run(api_request('/challenge',{},method='GET',decode_json=False))
            self.assertEqual(response['status'],200);self.assertIn('挑战升级'.encode(),payload)
            self.assertNotIn(b'type="number"',payload);self.assertIsNone(service.automation_owner)
            response,data=asyncio.run(api_request('/api/challenge/state',{},method='GET'))
            self.assertEqual(response['status'],200);self.assertEqual(data['challenge_api_version'],1)
            self.assertFalse(data['health']['armed']);self.assertEqual(data['challenge']['status'],'idle')
            response,data=asyncio.run(api_request('/api/challenge/start',{'declaration':''}))
            self.assertEqual(response['status'],409);self.assertIn('确认',data['detail'])
            response,_=asyncio.run(api_request('/api/challenge/sessions/not-an-id/observations/not-an-id',{},method='GET'))
            self.assertEqual(response['status'],404)
