"""Regress reported low release, wrong scalar rollback, and recovery ordering."""
import copy
import json
import math
from pathlib import Path
import threading
import time
import unittest
import uuid
from unittest.mock import patch

import numpy as np

from evoarm_rsi.bench_calibration import load_session
from evoarm_rsi.ball_trial import camera_geometry
from evoarm_rsi.pick_place import DEFAULT_PARAMETERS,adapt,build_plan
from evoarm_rsi.release_geometry import verify_release,upgrade_release_policy
from evoarm_rsi.rsi_policy import propose,application,project
from evoarm_rsi.rsi_execution import execute,reservation
from evoarm_rsi.rsi_route import at_height
from evoarm_rsi.roarm_geometry import height_above_plane,inverse,MotionConstraintError
from evoarm_rsi.learning_basis import execution_proposal
from tests.test_rsi_learning import FactoryArm,verdict
from tests import test_rsi_learning as learning_fixtures
from tests import test_rsi_loop as fixtures

REPORT=json.loads(Path(__file__).with_name('fixtures').joinpath('reported_height_rollback.json').read_text())


def reported_scene():
    root=Path(__file__).resolve().parents[1]/'calibrations/bench-layout-v2/2026-11-15'
    saved=load_session(next(root.glob('calibrations/*/revisions/*/session.json')))[1]
    intrinsics=json.loads(next(root.glob('experiments/*/gemini_color_intrinsics.json')).read_text())
    g=camera_geometry(saved,intrinsics);r=REPORT['current']
    scene={'table_plane':g['plane'],'camera_geometry':{k:g[k].tolist() for k in ('K','R','t','inverse_K','normal_camera')},
           'ball':{'center_robot_mm':r['ball_xyz_mm']},'funnel':{'mouth_center_robot_mm':r['funnel_xyz_mm']},'warnings':[]}
    for name,field in (('ball','center_robot_mm'),('funnel','mouth_center_robot_mm')):
        scene[name]['pixel_center']=project(scene,scene[name][field])
    return saved,scene


class ReportedRegressions(unittest.TestCase):
    def test_bad_height_trial_is_undone_and_new_xy_advice_keeps_iterating(self):
        _,scene=reported_scene();r=REPORT['current']
        after,changes,p=propose(r['before'],scene,r['outcome'],r['direction'],r['magnitude'],r['adjustment'],
            history=[REPORT['previous']],comparison=r['comparison'],assessment=r['comparison']['assessment'])
        self.assertTrue(p['allow_retry']);self.assertEqual(p['decision'],'rollback_and_trial')
        self.assertEqual(after['grasp_z_offset_mm'],1.)
        self.assertAlmostEqual(math.dist(after['pickup_xy_mm'],r['before']['pickup_xy_mm']),12.)
        self.assertEqual(p['rollback_parameters'],['grasp_z_offset_mm'])
        self.assertEqual(p['trial_parameters'],['pickup_xy_mm'])
        self.assertLess(p['predicted_pixel_delta'][0],0.)
        self.assertTrue(p['projection_direction_verified'])

    def test_reported_far_funnel_has_reachable_high_release_from_home(self):
        saved,scene=reported_scene();p=copy.deepcopy(REPORT['current']['before']);p['release_clearance_mm']=100.
        arm=FactoryArm();arm.send({'T':100});plan=build_plan(saved,arm.status(),scene,p)
        clearance=height_above_plane(plan['release_tip_xyz_mm'],plan['table_plane'])-plan['release_policy']['lip_height_mm']
        self.assertGreater(clearance,95.)
        self.assertGreaterEqual(plan['release_policy']['minimum_tip_clearance_mm'],60.)
        # Simulate the reported ~31mm stable downward offset.
        actual=dict(zip('xyz',at_height(plan['release_tip_xyz_mm'],
            height_above_plane(plan['release_tip_xyz_mm'],plan['table_plane'])-31.,plan['table_plane'])))
        self.assertTrue(verify_release(actual,plan)['verified'])

    def test_low_legacy_policy_upgrades_without_losing_pickup_learning_or_history(self):
        parameters=copy.deepcopy(REPORT['current']['before'])
        old={'version':20,'parameters':parameters,'reviews':[REPORT['previous']]}
        before=copy.deepcopy(old);new,record=upgrade_release_policy(old)
        self.assertEqual(old,before);self.assertEqual(new['parameters']['release_clearance_mm'],100.)
        for name in parameters:
            if name!='release_clearance_mm':self.assertEqual(new['parameters'][name],parameters[name])
        self.assertEqual(new['reviews'],old['reviews'])
        self.assertTrue(application(new['parameters'],new['parameters'],new['reviews'],new['baseline_resets'])['parameters_verified'])
        self.assertIsNone(upgrade_release_policy(new)[1])

    def test_force_advice_does_not_change_squeeze_or_invent_height_correction(self):
        _,scene=reported_scene();p=copy.deepcopy(DEFAULT_PARAMETERS)
        self.assertEqual(adapt(p,scene,'grasp_missed','none',adjustment='grip_tighten')[0],p)
        corrected,_=adapt(p,scene,'grasp_missed','image_right',adjustment='grip_tighten')
        self.assertNotEqual(corrected['pickup_xy_mm'],p['pickup_xy_mm'])
        self.assertEqual(corrected['squeeze_deg'],p['squeeze_deg']);self.assertEqual(corrected['grasp_z_offset_mm'],p['grasp_z_offset_mm'])
        slipped,_=adapt(p,scene,'slipped','none',adjustment='grip_tighten')
        self.assertEqual(slipped['squeeze_deg'],p['squeeze_deg']);self.assertLess(slipped['travel_speed'],p['travel_speed'])


class ReleaseDispatch(unittest.TestCase):
    setUp=fixtures.RSIApi.setUp
    request=fixtures.RSIApi.request

    def test_fresh_low_feedback_prevents_open_write_and_selects_higher_next_trial(self):
        self.execute_patch.stop()
        from evoarm_rsi.pick_place import observe_scene
        scene,_,_,_=observe_scene(self.folder,self.session)
        plan=build_plan(self.session,self.arm.status(),scene,copy.deepcopy(DEFAULT_PARAMETERS))
        plan['steps']=[next(s for s in plan['steps'] if s['stage']=='release')]
        self.arm.raw=inverse(at_height([300.,0.,0.],plan['release_policy']['lip_height_mm']+20.,plan['table_plane']),math.radians(40),0.,3.14)
        key=uuid.uuid4().hex;root=self.home/'.evoarm/rsi/pick_place_runs'/key
        cam={};done=threading.Event()
        def camera(home,method,path,payload=None):
            if method=='GET':return dict(cam)
            cam.update(recording=True,phase='recording',recording_request_id=payload['request_id'],recording_started_monotonic_ns=time.monotonic_ns())
            done.wait(5)
            return {'trial_id':self.folder.name,'request_id':payload['request_id']}
        def publish(phase,**kwargs):
            if phase=='saving_recording':done.set()
        self.service.unlock_once('已检查设备并清空运动区域');self.service.pending=reservation(key)
        result=execute(self.service,plan,root,threading.Event(),publish,camera=camera)
        self.assertEqual(self.arm.commands,[]);self.assertFalse(result['placement_attempted'])
        self.assertEqual(result['failure']['code'],'release_clearance')
        after,_,p=execution_proposal(plan['parameters'],result['execution_diagnosis'])
        self.assertTrue(p['allow_retry']);self.assertGreater(after['release_clearance_mm'],100.)

    def test_prepare_backs_up_and_migrates_existing_low_policy(self):
        from evoarm_rsi.coordinate_test import fingerprint
        path=self.home/'.evoarm/rsi/pick_place_policies'/self.session['setup_id']/(fingerprint(self.session)+'.json')
        path.parent.mkdir(parents=True,exist_ok=True)
        p=copy.deepcopy(DEFAULT_PARAMETERS);p['release_clearance_mm']=18.
        original=json.dumps({'version':21,'parameters':p,'reviews':[]})
        path.write_text(original)
        response,data=self.request('prepare');self.assertEqual(response['status'],200,data)
        saved=json.loads(path.read_text());self.assertEqual(saved['version'],23)
        self.assertEqual(saved['parameters']['release_clearance_mm'],100.)
        high=next(r for r in saved['baseline_resets'] if r['kind']=='high_release_clearance')
        self.assertEqual(json.loads(Path(high['backup_file']).read_text()),json.loads(original))


class RecoveryOrder(unittest.TestCase):
    def test_each_trial_including_success_recovers_before_video_review_once(self):
        case=learning_fixtures.AutoAPI();case.setUp();self.addCleanup(case.doCleanups)
        recovered=[];reviewed=[]
        def reset(*args,**kwargs):recovered.append(len(case.policies));return {'status':'completed'}
        def review(context,root,stop,*,preflight=False,**kwargs):
            if preflight:return {'ready':True,'note':'visible'}
            self.assertEqual(recovered[-1],len(case.policies))
            self.assertEqual(len(recovered),len(case.policies))
            reviewed.append(len(case.policies))
            return verdict('grasp_missed','image_right') if len(case.policies)==1 else verdict()
        with patch('control_ui.rsi_trial.recover_trial',side_effect=reset),patch('evoarm_rsi.grok_review.review',side_effect=review):
            case.start()
        _,state=case.request('state',method='GET')
        self.assertEqual(state['learning']['phase'],'completed');self.assertEqual(recovered,[1,2]);self.assertEqual(reviewed,[1,2])
