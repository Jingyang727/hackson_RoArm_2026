"""Board-free RGB-D pick/place and qualitative policy updates, no real devices."""
import copy
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
import uuid

import cv2
import numpy as np

from tests.test_ball_trial import layout,K,SimulatedArm
from evoarm_rsi.ball_trial import camera_geometry,check_segment
from evoarm_rsi.pick_place import observe_scene,build_plan,DEFAULT_PARAMETERS,adapt,image_correction,FUNNEL_HEIGHT
from evoarm_rsi.roarm_geometry import inverse,height_above_plane


def scene(home,session,ball=(330.,0.,-98.),funnel=(290.,-110.,-69.6),*,holes=False):
    geometry=camera_geometry(session,K);image=np.full((720,1280,3),220,np.uint8)
    depth=np.full(image.shape[:2],500,np.uint16)
    def project(point):
        camera=geometry['R'].T@(np.asarray(point)-geometry['t']);uv=geometry['K']@camera
        return uv[:2]/uv[2],camera
    uv,cam=project(funnel);pixel=tuple(np.round(uv).astype(int));radius=round(600*38/cam[2])
    cv2.circle(image,pixel,radius,(18,18,18),-1)
    cv2.circle(depth,pixel,radius,round(cam[2]),5)  # Rim valid, hole still sees TABLE.
    uv,cam=project(ball);radius=round(600*17/cam[2]);pixel=tuple(np.round(uv).astype(int))
    cv2.circle(image,pixel,radius,(0,210,230),-1)
    yy,xx=np.indices(depth.shape);mask=(xx-uv[0])**2+(yy-uv[1])**2<radius**2
    ys,xs=np.nonzero(mask);rays=np.c_[xs,ys,np.ones(len(xs))]@geometry['inverse_K'].T
    a=np.sum(rays*rays,axis=1);b=rays@cam;c=cam@cam-17**2;disc=b*b-a*c
    good=disc>=0;z=(b[good]-np.sqrt(disc[good]))/a[good]
    depth[ys[good],xs[good]]=np.round(z).astype(np.uint16)
    if holes:depth[mask]=0
    key=uuid.uuid4().hex;folder=home/'.evoarm/rsi/experiments'/key;folder.mkdir(parents=True)
    writer=cv2.VideoWriter(str(folder/'rgb.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(1280,720))
    for _ in range(4):writer.write(image)
    writer.release();stamps=[time.monotonic_ns()-400_000_000+i*100_000_000 for i in range(4)]
    (folder/'frames.jsonl').write_text('\n'.join(json.dumps({'host_monotonic_ns':s}) for s in stamps))
    np.savez_compressed(folder/'depth_samples.npz',depth_mm=np.stack([depth]*4),monotonic_ns=stamps)
    (folder/'gemini_color_intrinsics.json').write_text(json.dumps(K))
    (folder/'manifest.json').write_text(json.dumps({'trial_id':key,'simulated':False,'status':'recorded'}))
    return folder,image,depth


class PickPlace(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        self.session=layout(self.home);self.arm=SimulatedArm()

    def test_actual_depth_fits_ball_center_and_hole_is_not_funnel_rim(self):
        folder,_,_=scene(self.home,self.session)
        world,geometry,_,_=observe_scene(folder,self.session)
        self.assertTrue(world['ball']['depth_evidence']['depth_used'])
        self.assertEqual(world['ball']['depth_evidence']['method'],'rgbd_known_radius_sphere_fit')
        np.testing.assert_allclose(world['ball']['center_robot_mm'],[330,0,-98],atol=2.)
        self.assertFalse(world['funnel']['depth_evidence']['hole_depth_used_as_rim'])
        self.assertAlmostEqual(height_above_plane(world['funnel']['mouth_center_robot_mm'],geometry['plane']),FUNNEL_HEIGHT)
        self.assertLess(math.dist(world['funnel']['mouth_center_robot_mm'][:2],[290,-110]),3.)
        self.assertFalse(world['ball']['board_check']['board_required'])

    def test_missing_ball_depth_remains_explicit_planar_estimate_not_a_fake_measurement(self):
        folder,_,_=scene(self.home,self.session,holes=True)
        world,_,_,_=observe_scene(folder,self.session)
        self.assertFalse(world['ball']['depth_evidence']['depth_used'])
        self.assertIn('保存桌面',world['warnings'][-1] if '球面' in world['warnings'][-1] else '\n'.join(world['warnings']))

    def test_complete_plan_has_release_and_real_stage_specific_speed_parameters(self):
        folder,_,_=scene(self.home,self.session);world,_,_,_=observe_scene(folder,self.session)
        plan=build_plan(self.session,self.arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS))
        names=[s['stage'] for s in plan['steps']]
        for name in ('descend','close','lift','transport','release'):self.assertIn(name,names)
        self.assertEqual(plan['steps'][-1]['stage'],'release')
        self.assertGreater(height_above_plane(plan['release_tip_xyz_mm'],plan['table_plane']),FUNNEL_HEIGHT)
        for step in plan['steps']:
            if step['kind']=='cartesian':self.assertGreater(step['spd'],.12);self.assertNotIn('acc',step)
            else:self.assertEqual(step['spd'],1000);self.assertEqual(step['acc'],20)

    def test_several_ball_positions_are_scene_targets_not_old_taught_positions(self):
        targets=[]
        for ball in ([300,30,-98],[350,35,-98],[360,-35,-98]):
            folder,_,_=scene(self.home,self.session,ball=ball)
            world,_,_,_=observe_scene(folder,self.session)
            plan=build_plan(self.session,self.arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS))
            targets.append(plan['grasp_tip_xyz_mm'])
        self.assertGreater(math.dist(targets[0],targets[-1]),40.)

    def test_funnel_moves_and_release_target_moves_with_it(self):
        targets=[]
        for funnel in ([270,-100,-69.6],[310,-110,-69.6]):
            folder,_,_=scene(self.home,self.session,funnel=funnel)
            world,_,_,_=observe_scene(folder,self.session)
            targets.append(build_plan(self.session,self.arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS))['release_tip_xyz_mm'])
        self.assertGreater(math.dist(*targets),30.)

    def test_direction_uses_camera_transform_and_one_review_changes_one_parameter(self):
        folder,_,_=scene(self.home,self.session);world,_,_,_=observe_scene(folder,self.session)
        p=copy.deepcopy(DEFAULT_PARAMETERS);new,changes=adapt(p,world,'grasp_missed','image_left')
        self.assertAlmostEqual(math.hypot(*new['pickup_xy_mm']),8.)
        self.assertEqual(new['release_xy_mm'],[0.,0.]);self.assertEqual(p['pickup_xy_mm'],[0.,0.])
        changed=[key for key in p if p[key]!=new[key]];self.assertEqual(changed,['pickup_xy_mm'])
        opposite=image_correction(world,'image_right');np.testing.assert_allclose(opposite,-np.asarray(new['pickup_xy_mm']),atol=.01)

    def test_height_slip_bounce_and_unknown_feedback_have_bounded_effects(self):
        folder,_,_=scene(self.home,self.session);world,_,_,_=observe_scene(folder,self.session);p=copy.deepcopy(DEFAULT_PARAMETERS)
        high,_=adapt(p,world,'grasp_missed','too_high');self.assertEqual(high['grasp_z_offset_mm'],1)
        slip,_=adapt(p,world,'slipped','none');self.assertLess(slip['travel_speed'],p['travel_speed'])
        bounced,_=adapt(p,world,'bounced_out','none');self.assertEqual(bounced['release_clearance_mm'],14)
        self.assertEqual(adapt(p,world,'unknown','none')[0],p)
        self.assertEqual(adapt(p,world,'placed','none')[0],p)

    def test_corrupt_parameters_and_unreachable_objects_do_not_make_motor_targets(self):
        folder,_,_=scene(self.home,self.session);world,_,_,_=observe_scene(folder,self.session)
        for change in ({'travel_speed':float('nan')},{'gripper_acc':1000},{'open_deg':181}):
            with self.assertRaises(ValueError):build_plan(self.session,self.arm.status(),world,{**DEFAULT_PARAMETERS,**change})
        world['ball']['center_robot_mm']=[900,0,-98]
        with self.assertRaises(ValueError):build_plan(self.session,self.arm.status(),world,copy.deepcopy(DEFAULT_PARAMETERS))
