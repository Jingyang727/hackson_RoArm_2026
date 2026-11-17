"""Multi-colour RGB-D and the cleanup loop; all motors and models are fake."""
import asyncio
import base64
import copy
import json
import math
import time
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
from control_ui.safety import FACTORY_DEGREES
from evoarm_rsi import challenge_scene as vision,challenge_loop as loop
from evoarm_rsi.ball_trial import camera_geometry
from evoarm_rsi.coordinate_test import fingerprint
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS,build_plan
from evoarm_rsi.rsi_route import TRANSFER_HEIGHT_POLICY
from evoarm_rsi.roarm_geometry import inverse
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
        'host_monotonic_ns':time.monotonic_ns(),'image_size':[1280,720],'table_plane':g['plane'],
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

    def test_container_colour_evidence_requires_visible_colour_inside_mouth(self):
        scene,g,image,frame,*_=synthetic(self.saved,{'red':[290,-110,-52.6],'blue':POSITIONS['blue']})
        colors=vision.container_colours(frame,g,scene['bowl'])
        self.assertIn('red',colors);self.assertNotIn('blue',colors);self.assertNotIn('black',colors)

    def test_sparse_depth_stays_unresolved_until_a_current_contour_is_verified(self):
        scene,g,image,frame,review=synthetic(self.saved,holes=True)
        self.assertEqual(vision.locate(frame,g,scene['bowl'])[0],[])
        self.assertEqual(len(vision.locate(frame,g,scene['bowl'])[1]),3)
        folder=self.home/'capture';folder.mkdir();(folder/'gemini_color_intrinsics.json').write_text(json.dumps(K))
        root=self.home/'fast';root.mkdir()
        with patch.object(vision,'load_rgbd',return_value=frame),patch.object(vision,'detect_funnel',return_value=scene['bowl']):
            updated,*_=vision.observe(folder,self.saved,root,threading.Event(),fast=True,verified_colors={'red','green','blue'})
        self.assertEqual(len(updated['balls']),3);self.assertEqual(updated['unresolved'],[])

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

    def test_locked_mouth_survives_occlusion_while_balls_are_measured_again(self):
        initial,*_=synthetic(self.saved)
        reference=vision.container_reference(initial)
        positions={'red':[340,-35,-98],'blue':[290,-110,-52.6]}
        scene,g,image,frame,*_=synthetic(self.saved,positions)
        # Gripper touching the image edge obscures the rim. Detection would
        # fail, but it must not be attempted again with a locked reference.
        u,v=np.round(reference['bowl']['pixel_center']).astype(int)
        cv2.rectangle(image,(u-12,0),(u+12,v-12),(18,18,18),-1)
        folder=self.home/'capture';folder.mkdir();(folder/'gemini_color_intrinsics.json').write_text(json.dumps(K))
        root=self.home/'post';root.mkdir()
        with patch.object(vision,'load_rgbd',return_value=frame),patch.object(vision,'detect_funnel',side_effect=ValueError('rim occluded')) as detector:
            updated,*_=vision.observe(folder,self.saved,root,threading.Event(),fast=True,bowl_reference=reference)
        detector.assert_not_called()
        self.assertEqual([b['color'] for b in updated['balls']],['red'])
        np.testing.assert_allclose(updated['balls'][0]['center_robot_mm'],positions['red'],atol=1)
        self.assertIn('blue',updated['bowl_ball_colors'])
        self.assertEqual(updated['bowl'],initial['bowl'])
        self.assertEqual(updated['source_trial_id'],frame['trial_id'])
        self.assertNotEqual(updated['source_trial_id'],updated['bowl_location']['source_trial_id'])
        self.assertEqual(updated['bowl_location']['mode'],'challenge_locked')
        self.assertEqual(json.loads((root/'scene.json').read_text())['bowl_location'],updated['bowl_location'])
        updated['bowl']['mouth_center_robot_mm'][0]+=100
        self.assertEqual(reference['bowl'],initial['bowl'])

    def test_locked_mouth_does_not_confirm_a_ball_by_disappearance(self):
        scene,g,image,frame,*_=synthetic(self.saved,{})
        folder=self.home/'capture';folder.mkdir();(folder/'gemini_color_intrinsics.json').write_text(json.dumps(K))
        root=self.home/'post';root.mkdir()
        with patch.object(vision,'load_rgbd',return_value=frame):
            updated,*_=vision.observe(folder,self.saved,root,threading.Event(),fast=True,bowl_reference=vision.container_reference(scene))
        self.assertEqual(updated['balls'],[]);self.assertEqual(updated['bowl_ball_colors'],[])

    def test_new_observation_still_requires_a_measured_mouth(self):
        scene,g,image,frame,*_=synthetic(self.saved)
        folder=self.home/'capture';folder.mkdir();(folder/'gemini_color_intrinsics.json').write_text(json.dumps(K))
        with patch.object(vision,'load_rgbd',return_value=frame),patch.object(vision,'detect_funnel',side_effect=ValueError('rim absent')):
            with self.assertRaisesRegex(ValueError,'rim absent'):
                vision.observe(folder,self.saved,self.home,threading.Event(),fast=True)

    def test_camera_profile_change_invalidates_locked_pixels(self):
        scene,g,image,frame,*_=synthetic(self.saved);reference=vision.container_reference(scene)
        folder=self.home/'capture';folder.mkdir();intr={**K,'fx':K['fx']+1}
        (folder/'gemini_color_intrinsics.json').write_text(json.dumps(intr))
        with patch.object(vision,'load_rgbd',return_value=frame):
            with self.assertRaisesRegex(ValueError,'内参或标定已变化'):
                vision.observe(folder,self.saved,self.home,threading.Event(),fast=True,bowl_reference=reference)


class CleanupLoop(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name);self.saved=layout(self.home)
        self.service=ArmService();self.service.arm=SimulatedArm();self.runner=loop.Challenge(lambda:self.service)
        self.world=synthetic(self.saved)[0];self.motion=[];self.reviews=[];self.colors=set(POSITIONS);self.outcomes=[];self.placed=set();self.executed_outcomes={};self.reset_calls=[]
        for item in (patch.object(Path,'home',return_value=self.home),patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1','EVOARM_SETUP_ID':'bench-layout-v2'}),
                     patch.object(loop.grok_review,'configuration',return_value={'configured':True,'model':'test-vlm'}),
                     patch.object(loop,'launch',side_effect=lambda target:target()),patch.object(loop,'recover',side_effect=self.recover),
                     patch.object(self.runner,'_capture',side_effect=self.capture),patch.object(loop,'execute',side_effect=self.execute),
                     patch.object(loop.grok_review,'review',side_effect=self.review)):
            item.start();self.addCleanup(item.stop)
        self.positions=copy.deepcopy(POSITIONS)

    def recover(self,service,plan,result,verdict,root,stop,publish,**kwargs):
        self.reset_calls.append(root)
        self.assertEqual(service.automation_owner,kwargs['automation_id']);service.pending=None;service.fault=None;service.disarm()
        return {'status':'completed'}

    def capture(self,saved,folder,**kwargs):
        scene,g,image,*_=synthetic(self.saved,{c:self.positions[c] for c in self.colors})
        scene['bowl_ball_colors']=sorted(self.placed);scene['fresh_model_review']=kwargs.get('fast',True) is False
        return scene,g,image

    def execute(self,service,plan,root,stop,publish,**kwargs):
        self.assertEqual(service.automation_owner,kwargs['automation_id']);self.assertEqual(service.pending['kind'],'rsi_trial')
        self.motion.append(copy.deepcopy(plan));service.pending=None;service.disarm()
        color=plan['scene']['ball']['color'];outcome=self.outcomes.pop(0) if self.outcomes else 'placed'
        self.executed_outcomes[root.name]=outcome
        if outcome=='placed':self.colors.discard(color);self.placed.add(color)
        elif outcome=='slipped':self.positions[color][1]+=12
        end=next(s for s in reversed(plan['steps']) if s['kind']=='cartesian')
        service.arm.raw=inverse(end['xyz_mm'],end['pitch_rad'],end['roll_rad'],plan['steps'][-1]['gripper_rad'])
        return {'motion_completed':True,'grasp_attempted':True,'placement_attempted':True,'physical_elapsed_s':8.1,
                'recording_covers_motion':True,'capture_trial_id':uuid.uuid4().hex,'motion_error':None,'camera_error':None}

    def review(self,context,root,stop,**kwargs):
        color=context['visual_target']['color'];self.reviews.append(color)
        outcome=self.executed_outcomes[root.name]
        return {'outcome':outcome,'direction':'none','magnitude':'unknown','adjustment':'transport_slower' if outcome=='slipped' else 'unknown',
                'ball_state':'funnel' if outcome=='placed' else 'table','confidence':.9,'note':'test visual pixels'}

    def start(self):return self.runner.start(loop.DECLARATION)

    def test_continuous_targets_do_not_reset_and_finish_when_table_empty(self):
        before=(self.home/'.evoarm/rsi/calibrations/bench-layout-v2/session.json').read_bytes()
        self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(state['attempt'],3)
        self.assertEqual(state['remaining'],0);self.assertTrue(state['placement_confirmed']);self.assertEqual(len(state['confirmed_colors']),3)
        self.assertEqual(self.reviews,[]);self.assertEqual(self.reset_calls,[]);self.assertIsNone(self.service.automation_owner)
        for previous,following in zip(self.motion,self.motion[1:]):
            final=next(s for s in reversed(previous['steps']) if s['kind']=='cartesian')
            np.testing.assert_allclose([following['initial_status'][k] for k in 'xyz'],final['xyz_mm'],atol=.001)
        self.assertEqual((self.home/'.evoarm/rsi/calibrations/bench-layout-v2/session.json').read_bytes(),before)
        self.assertEqual(len(list((self.home/'.evoarm/rsi/backups').glob('*.zip'))),3)

    def test_container_reference_is_shared_through_census_and_new_each_challenge(self):
        captures=[];original=self.capture
        def checked(*args,**kwargs):
            captures.append(kwargs);return original(*args,**kwargs)
        with patch.object(self.runner,'_capture',side_effect=checked):
            self.runner.observe();self.assertIsNone(captures[-1].get('bowl_reference'))
            references=[]
            for _ in range(2):
                captures.clear();self.colors=set(POSITIONS);self.placed.clear()
                self.start();state=self.runner.snapshot()
                self.assertEqual(state['status'],'completed',state.get('error'))
                self.assertIsNone(captures[0].get('bowl_reference'))
                reference=state['container_reference'];references.append(reference)
                self.assertTrue(all(c['bowl_reference']==reference for c in captures[1:]))
                self.assertTrue(any(c.get('source') for c in captures))
                self.assertTrue(any(c.get('fast') is False for c in captures))
                saved=json.loads((self.runner.root(state['session_id'])/'state.json').read_text())
                self.assertEqual(saved['container_reference'],reference)
            self.assertNotEqual(references[0]['source_trial_id'],references[1]['source_trial_id'])
        self.assertEqual(self.reset_calls,[])

    def test_first_drop_occluding_rim_does_not_pause_following_grabs(self):
        measurements=[]
        def detector(*args):
            measurements.append(1)
            if len(measurements)>1:raise ValueError('gripper occludes the rim after first release')
            return copy.deepcopy(self.world['bowl'])
        def capture(saved,folder,**kwargs):
            positions={c:self.positions[c] for c in self.colors}
            positions.update({c:[290,-110,-52.6] for c in self.placed})
            scene,g,image,frame,review=synthetic(saved,positions)
            review['table_balls']=[r for r in review['table_balls'] if r['color'] in self.colors]
            review['bowl_ball_colors']=sorted(self.placed)
            u,v=np.round(scene['bowl']['pixel_center']).astype(int)
            if self.motion:cv2.rectangle(image,(u-12,0),(u+12,v-12),(18,18,18),-1)
            recording=self.home/'recording';recording.mkdir(exist_ok=True)
            (recording/'gemini_color_intrinsics.json').write_text(json.dumps(K))
            folder.mkdir(parents=True,exist_ok=True)
            with patch.object(vision,'load_rgbd',return_value=frame),patch.object(vision,'inventory',return_value=review):
                return vision.observe(recording,saved,folder,self.runner.stop,
                    fast=kwargs.get('fast',True),bowl_reference=kwargs.get('bowl_reference'))
        with patch.object(self.runner,'_capture',side_effect=capture),patch.object(vision,'detect_funnel',side_effect=detector):self.start()
        state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'))
        self.assertEqual(len(self.motion),3);self.assertEqual(len(measurements),1)
        self.assertEqual(state['scene']['bowl_location']['mode'],'challenge_locked')
        self.assertEqual(self.reset_calls,[])

    def test_perception_failure_has_a_persisted_stage_and_event_reason(self):
        reason='当前没有找到完整的大漏斗口沿'
        with patch.object(self.runner,'_capture',side_effect=ValueError(reason)):self.start()
        state=self.runner.snapshot();self.assertEqual(state['status'],'failed')
        self.assertEqual(state['failure_phase'],'observing');self.assertEqual(state['error'],reason)
        events=self.runner.events.after()['events']
        self.assertEqual(events[-1]['phase'],'failed');self.assertEqual(events[-1]['message'],reason)
        saved=json.loads((self.runner.root(state['session_id'])/'state.json').read_text())
        self.assertEqual(saved['error'],reason);self.assertEqual(self.motion,[])
        self.assertIsNone(self.service.automation_owner)
        for file in (self.home/'.evoarm/rsi/langfuse-outbox').glob('*.json'):
            self.assertEqual(json.loads(file.read_text())['task']['kind'],'challenge_upgrade')

    def test_slip_retry_uses_fresh_position_and_continues_to_remaining_balls(self):
        self.outcomes=['slipped'];self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(len(self.motion),4)
        slipped=self.motion[0]['scene']['ball']['color']
        plans=[p for p in self.motion if p['scene']['ball']['color']==slipped]
        self.assertGreater(math.dist(plans[0]['scene']['ball']['center_robot_mm'],plans[1]['scene']['ball']['center_robot_mm']),10)
        self.assertEqual(self.reset_calls,[])

    def test_unknown_failure_may_retry_unchanged_parameters(self):
        self.outcomes=['grasp_missed'];self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(len(self.motion),4)
        self.assertEqual(self.motion[0]['parameters'],self.motion[1]['parameters'])

    def test_more_than_five_attempts_are_allowed(self):
        self.outcomes=['grasp_missed']*6;self.start();state=self.runner.snapshot()
        self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(len(self.motion),9)

    def test_background_model_error_does_not_block_next_ball(self):
        self.outcomes=['grasp_missed']
        with patch.object(loop.grok_review,'review',side_effect=RuntimeError('test model unavailable')):self.start()
        state=self.runner.snapshot();self.assertEqual(state['status'],'completed');self.assertEqual(len(self.motion),4)
        self.assertEqual(len(list((self.home/'.evoarm/rsi/backups').glob('*.zip'))),4)

    def test_detected_ball_count_can_be_one_two_or_four(self):
        for positions in ({'red':POSITIONS['red']},{'red':POSITIONS['red'],'blue':POSITIONS['blue']},
                {**POSITIONS,'white':[285,10,-98]}):
            self.colors=set(positions);self.positions=copy.deepcopy(positions);self.placed.clear();self.motion.clear()
            self.start();state=self.runner.snapshot()
            self.assertEqual(state['status'],'completed',state.get('error'));self.assertEqual(state['detected_total'],len(positions))
            self.assertEqual(len(self.motion),len(positions));self.assertEqual(len(state['confirmed_colors']),len(positions))

    def test_slow_visual_review_runs_while_following_pick_place_continues(self):
        self.outcomes=['grasp_missed'];entered=threading.Event();finished=threading.Event();continued=[]
        original=self.execute
        def slow(context,root,stop,**kwargs):
            entered.set();stop.wait(5);finished.set()
            return {'outcome':'unknown','ball_state':'unknown','direction':'none','magnitude':'unknown'}
        def execution(*args,**kwargs):
            if self.motion:
                self.assertTrue(entered.wait(1));continued.append(not finished.is_set())
            return original(*args,**kwargs)
        with patch.object(loop.grok_review,'review',side_effect=slow),patch.object(loop,'execute',side_effect=execution):self.start()
        self.assertTrue(all(continued));self.assertEqual(len(self.motion),4);self.assertTrue(finished.wait(1))
        self.assertEqual(self.reset_calls,[])

    def test_unclear_table_never_dispatches_or_reports_empty_success(self):
        original=self.capture;calls=[]
        def unclear(*args,**kwargs):
            calls.append(1)
            if len(calls)>2:self.runner.stop.set();self.runner.guard()
            scene,g,image=original(*args,**kwargs);scene['view_clear']=False;return scene,g,image
        with patch.object(self.runner,'_capture',side_effect=unclear):self.start()
        self.assertEqual(self.motion,[]);self.assertEqual(self.runner.snapshot()['status'],'stopped')

    def test_claimed_placement_without_release_is_not_counted(self):
        self.outcomes=['unknown']*10
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

    def test_existing_fault_recovers_once_but_normal_following_trials_do_not(self):
        self.service.fault={'code':'test_fault','message':'synthetic fault'}
        self.start();self.assertEqual(self.runner.snapshot()['status'],'completed')
        self.assertEqual(len(self.reset_calls),1);self.assertEqual(self.reset_calls[0].name,'startup')

    def test_real_execution_error_still_recovers_and_does_not_blindly_continue(self):
        original=self.execute
        def failed(*args,**kwargs):
            report=original(*args,**kwargs);report.update(motion_completed=False,motion_error='synthetic unverified move')
            self.service.fault={'code':'test_unverified','message':'synthetic unverified move'};return report
        with patch.object(loop,'execute',side_effect=failed):self.start()
        self.assertEqual(self.runner.snapshot()['status'],'failed');self.assertEqual(len(self.motion),1)
        self.assertEqual(len(self.reset_calls),1);self.assertIsNone(self.service.automation_owner)

    def test_old_background_review_does_not_overwrite_a_new_policy(self):
        self.start();state=self.runner.snapshot();trial=state['trials'][0]['plan_id']
        current=json.loads((self.runner.root(state['session_id'])/'attempts'/trial/'state.json').read_text())
        pol=self.runner.policy(self.saved);pol['parameters']['pickup_xy_mm']=[10,2]
        verdict={'outcome':'grasp_missed','ball_state':'table','direction':'image_left','magnitude':'medium'}
        self.runner._apply_review(self.saved,current,verdict,pol,source='model')
        self.assertEqual(pol['parameters']['pickup_xy_mm'],[10,2])
        self.assertEqual(current['review']['proposal']['decision'],'stale_review')

    def test_late_failure_does_not_change_parameters_after_same_colour_succeeds(self):
        self.start();state=self.runner.snapshot();trial=state['trials'][0]['plan_id']
        current=json.loads((self.runner.root(state['session_id'])/'attempts'/trial/'state.json').read_text())
        pol=self.runner.policy(self.saved);before=copy.deepcopy(pol['parameters'])
        pol['reviews'].append({'session_id':state['session_id'],'target_color':current['task']['target_color'],
            'outcome':'placed','attempt':current['task']['attempt']+1})
        verdict={'outcome':'grasp_missed','ball_state':'table','direction':'image_left','magnitude':'medium'}
        self.runner._apply_review(self.saved,current,verdict,pol,source='model')
        self.assertEqual(pol['parameters'],before);self.assertEqual(current['review']['proposal']['decision'],'stale_review')

    def test_faster_existing_approach_speed_is_preserved(self):
        path=self.home/'.evoarm/rsi/pick_place_policies/bench-layout-v2'/f'{fingerprint(self.saved)}.json'
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps({'version':1,'parameters':{**DEFAULT_PARAMETERS,'approach_speed':.9},'reviews':[]}))
        self.assertEqual(self.runner.policy(self.saved)['parameters']['approach_speed'],.9)

    def test_empty_table_with_no_placement_evidence_is_not_success(self):
        original=self.capture
        def disappearing(saved,folder,**kwargs):
            if self.runner.snapshot()['attempt']:self.colors.clear();self.placed.clear()
            return original(saved,folder,**kwargs)
        self.outcomes=['unknown']
        with patch.object(self.runner,'_capture',side_effect=disappearing):self.start()
        self.assertEqual(self.runner.snapshot()['phase'],'cleared');self.assertFalse(self.runner.snapshot()['placement_confirmed']);self.assertEqual(self.runner.snapshot()['confirmed_colors'],[])

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
        self.assertEqual(self.motion[0]['parameters']['travel_speed'],1.);self.assertEqual(self.motion[0]['parameters']['approach_speed'],.6);self.assertEqual(path.read_bytes(),before)

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
            self.assertEqual(response['status'],200);self.assertEqual(data['challenge_api_version'],2)
            self.assertFalse(data['health']['armed']);self.assertEqual(data['challenge']['status'],'idle')
            response,data=asyncio.run(api_request('/api/challenge/start',{'declaration':''}))
            self.assertEqual(response['status'],409);self.assertIn('确认',data['detail'])
            response,_=asyncio.run(api_request('/api/challenge/sessions/not-an-id/observations/not-an-id',{},method='GET'))
            self.assertEqual(response['status'],404)


class ChallengePendingAPI(unittest.TestCase):
    """A paused challenge must still observe another page's unowned motion."""
    def setUp(self):
        app=FastAPI();self.service=ArmService();self.arm=SimulatedArm();self.service.arm=self.arm
        self.runner=install(app,lambda:self.service)
        self.runner.value.update(session_id='a'*32,status='failed',phase='failed',error='漏斗口沿未识别')
        self.arm.raw.update({k:math.radians(FACTORY_DEGREES[j]) for j,k in enumerate('bsetrg',1)})
        self.arm.raw['g']=math.radians(154.25)
        self.service.pending={'kind':'factory','target':'official_init','since':time.monotonic()-3.5,
            'elapsed_s':3.5,'initial_angles':{**FACTORY_DEGREES,6:110.66},'stable_intervals':0}
        self.service.last_command={'status':'factory_reset_sent_unverified','command':{'T':100}}
        for item in (patch.object(main,'app',app),patch.object(self.runner,'session',return_value={}),
                patch.object(loop.grok_review,'configuration',return_value={'configured':True,'model':'test-vlm'})):
            item.start();self.addCleanup(item.stop)

    def state(self):
        response,value=asyncio.run(api_request('/api/challenge/state',{},method='GET'))
        self.assertEqual(response['status'],200)
        return value

    def test_paused_challenge_updates_factory_feedback_and_releases_verified_pending(self):
        with patch.object(self.arm,'status',wraps=self.arm.status) as read:
            value=self.state();pending=value['health']['pending']
            self.assertEqual(pending['kind'],'factory');self.assertEqual(pending['max_error_deg'],25.75)
            self.assertEqual(pending['worst_joint'],6);self.assertIsNone(value['health']['automation_owner'])
            self.assertIsNone(value['health']['fault']);self.assertEqual(value['challenge']['error'],'漏斗口沿未识别')
            read.assert_called_once_with(timeout=20)
            self.arm.raw['g']=math.radians(180)
            value=self.state()
        self.assertIsNone(value['health']['pending']);self.assertIsNone(value['health']['fault'])
        self.assertEqual(value['health']['last_command']['status'],'feedback_confirmed')
        self.assertFalse(value['health']['armed']);self.assertEqual(self.arm.commands,[])

    def test_unowned_cartesian_pending_also_gets_real_feedback(self):
        start=[self.arm.raw[k] for k in 'xyz'];target=[start[0]+50,*start[1:]]
        self.service.pending={'kind':'cartesian','since':time.monotonic()-3.5,'xyz_start':start,'xyz_target':target}
        with patch.object(self.arm,'status',wraps=self.arm.status) as read:
            value=self.state();self.assertEqual(value['health']['pending']['position_error_mm'],50)
            read.assert_called_once_with(timeout=30)
            self.arm.raw['x']+=50;value=self.state()
        self.assertIsNone(value['health']['pending']);self.assertEqual(self.arm.commands,[])

    def test_stalled_factory_becomes_visible_fault_instead_of_pending_forever(self):
        self.service.pending['since']=96.5
        with patch('control_ui.service.time') as clock:
            for now in (100.,101.,102.):
                clock.monotonic.return_value=now;value=self.state()
        self.assertIsNone(value['health']['pending'])
        self.assertEqual(value['health']['fault']['code'],'motion_stalled_or_offset')
        self.assertEqual(value['health']['fault']['error_deg'],25.75)
        self.assertFalse(value['health']['armed']);self.assertEqual(self.arm.commands,[])

    def test_factory_timeout_is_reported_without_resetting_or_clearing_the_fault(self):
        self.service.pending['since']-=50
        value=self.state()
        self.assertIsNone(value['health']['pending'])
        self.assertEqual(value['health']['fault']['code'],'motion_not_confirmed')
        self.arm.raw['g']=math.radians(180);again=self.state()
        self.assertEqual(again['health']['fault'],value['health']['fault'])
        self.assertEqual(self.arm.commands,[])

    def test_serial_read_failure_is_explicit_and_never_replays_reset(self):
        with patch.object(self.arm,'status',side_effect=OSError('synthetic serial unavailable')):
            value=self.state()
        self.assertIn('synthetic serial unavailable',value['feedback_error'])
        self.assertEqual(value['health']['fault']['code'],'feedback_timeout')
        self.assertEqual(self.arm.commands,[])

    def test_active_automation_keeps_exclusive_control_of_its_feedback(self):
        for owner in ('another-job',self.runner.value['session_id']):
            self.service.automation_owner=owner
            before=copy.deepcopy(self.service.pending)
            with patch.object(self.arm,'status',side_effect=AssertionError('active worker owns serial feedback')) as read:
                value=self.state()
            read.assert_not_called();self.assertEqual(value['health']['automation_owner'],owner)
            self.assertEqual(self.service.pending,before)
            self.assertEqual(value['health']['pending'],json.loads(json.dumps(before)));self.assertEqual(self.arm.commands,[])
