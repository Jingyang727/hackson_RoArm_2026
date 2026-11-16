"""Workspace snapshots use the real planner without authorizing any motion."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
import zipfile
from unittest.mock import patch

import numpy as np
from fastapi import HTTPException

from evoarm_rsi.pick_place import build_plan,observe_scene,DEFAULT_PARAMETERS
from evoarm_rsi.workspace_preview import compute_workspace
from evoarm_rsi.planning_diagnostics import explain,search_summary
from evoarm_rsi.roarm_geometry import MotionConstraintError
from evoarm_rsi.rsi_learning import error_info,ResetHandledError
from tests.test_ball_trial import layout
from tests.test_rsi_pick_place import scene
from tests.test_rsi_learning import FactoryArm
from tests import test_rsi_loop as api_fixtures
from tests import test_rsi_learning as learning_fixtures


class WorkspaceGeometry(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        self.saved=layout(self.home);folder,*_=scene(self.home,self.saved)
        self.scene,*_=observe_scene(folder,self.saved);self.arm=FactoryArm();self.arm.send({'T':100});self.arm.commands.clear()

    def test_green_samples_replay_as_complete_routes_with_tilt_and_camera_transform(self):
        normal=np.array([.025,.01,1.]);normal/=np.linalg.norm(normal)
        self.scene['table_plane']={'normal_up':normal.tolist(),'d_mm':115.}
        raw=self.arm.status();p=copy.deepcopy(DEFAULT_PARAMETERS)
        result=compute_workspace(self.saved,raw,self.scene,p,columns=8,rows=6,budget_s=5)
        self.assertEqual(result['status'],'completed');self.assertGreater(result['planned_samples'],0)
        self.assertGreater(result['rejected_samples'],0);self.assertEqual(result['unknown_samples'],0)
        self.assertEqual(self.arm.commands,[]);R=np.array(self.scene['camera_geometry']['R']);t=np.array(self.scene['camera_geometry']['t'])
        K=np.array(self.scene['camera_geometry']['inverse_K']);width,height=result['image_size']
        points=[]
        for row,text in enumerate(result['grid']):
            for col,value in enumerate(text):
                if value!='+':continue
                ray=K@[(col+.5)*width/8,(row+.5)*height/6,1.]
                center=R@(ray*((17.-(normal@t+115.))/(ray@(R.T@normal))))+t
                candidate={**self.scene,'ball':{**self.scene['ball'],'center_robot_mm':center.tolist()}}
                plan=build_plan(self.saved,raw,candidate,p);points.append(center)
                self.assertIn('release',[s['stage'] for s in plan['steps']])
                if len(points)==4:break
            if len(points)==4:break
        self.assertEqual(len(points),4)

    def test_time_budget_leaves_unknown_cells_instead_of_inventing_unreachable_area(self):
        updates=[]
        result=compute_workspace(self.saved,self.arm.status(),self.scene,copy.deepcopy(DEFAULT_PARAMETERS),
                                 columns=8,rows=6,budget_s=.000001,progress=updates.append)
        self.assertEqual(result['status'],'partial');self.assertEqual(result['unknown_samples'],48)
        self.assertEqual(result['rejected_samples'],0);self.assertEqual(result['selected_ball']['status'],'unknown')
        self.assertEqual(updates[-1]['grid'],result['grid']);self.assertEqual(self.arm.commands,[])


class WorkspaceAPI(unittest.TestCase):
    setUp=api_fixtures.RSIApi.setUp
    request=api_fixtures.RSIApi.request

    def test_snapshot_preview_is_allowed_in_readonly_mode_and_preserves_calibration_and_policy(self):
        from evoarm_rsi.coordinate_test import fingerprint
        policy=self.home/'.evoarm/rsi/pick_place_policies'/self.session['setup_id']/(fingerprint(self.session)+'.json')
        policy.parent.mkdir(parents=True);policy.write_text(json.dumps({'version':4,'parameters':copy.deepcopy(DEFAULT_PARAMETERS),'reviews':[]}))
        original=policy.read_bytes();saved=copy.deepcopy(self.session)
        response,observation=self.request('observe');self.assertEqual(response['status'],200,observation)
        def compute(*args,**kwargs):return compute_workspace(*args,**kwargs,columns=6,rows=4,budget_s=3)
        with (patch('control_ui.rsi_trial._launch_workspace',side_effect=lambda fn:fn()),
              patch('control_ui.rsi_trial.compute_workspace',side_effect=compute),
              patch.object(self.service,'_send_motion',side_effect=AssertionError('no motor write')),
              patch.object(self.service,'unlock_once',side_effect=AssertionError('no authorization')),
              patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'0'})):
            response,value=self.request('workspace',{'observation_id':observation['observation_id']})
            self.assertEqual(response['status'],202,value)
            _,status=self.request('workspace/'+value['workspace_id'],method='GET')
        self.assertEqual(status['status'],'completed',status)
        self.assertFalse(status['robot_moved']);self.assertEqual(self.arm.commands,[])
        self.assertEqual(policy.read_bytes(),original);self.assertEqual(saved,self.session)
        self.assertEqual(self.execute_mock.call_count,0)
        inputs=self.home/'.evoarm/rsi/workspace_previews'/value['workspace_id']/'input.json'
        self.assertTrue(inputs.is_file());self.assertEqual(json.loads(inputs.read_text())['session'],saved)

    def test_old_calibration_observation_is_rejected_before_computation(self):
        _,observation=self.request('observe')
        source=self.home/'.evoarm/rsi/pick_place_observations'/observation['observation_id']/'scene.json'
        value=json.loads(source.read_text());value['calibration_fingerprint']='different';source.write_text(json.dumps(value))
        with patch('control_ui.rsi_trial.compute_workspace') as compute:
            response,data=self.request('workspace',{'observation_id':observation['observation_id']})
        self.assertEqual(response['status'],409,data);compute.assert_not_called();self.assertEqual(self.arm.commands,[])


class PlanningDiagnostics(unittest.TestCase):
    def test_deepest_stage_example_survives_truncation_of_attempts(self):
        early={'stage':'raise_current','code':'joint_limit','message':'起步高度关节范围'}
        late={'stage':'transport','code':'ik_workspace','message':'运输目标距离过远','xyz_mm':[500.,200.,20.]}
        summary=search_summary([early,late])
        detail={'code':'pick_place_search_exhausted','message':'search failed','attempts':[early],**summary}
        result=explain(detail)
        self.assertEqual(result['stage'],'transport');self.assertEqual(result['reason'],late['message'])
        self.assertEqual(result['evidence']['xyz_mm'],late['xyz_mm'])
        wrapped=ResetHandledError(HTTPException(status_code=409,detail=detail),{'status':'completed'})
        self.assertEqual(error_info(wrapped)['planning_diagnosis']['stage'],'transport')

    def test_official_home_planning_rejection_checks_once_and_does_not_replan_or_send_t100(self):
        case=learning_fixtures.AutoAPI();case.setUp();self.addCleanup(case.doCleanups)
        arm=FactoryArm();arm.send({'T':100});arm.commands.clear();case.fixture.service.arm=arm
        problem=MotionConstraintError('target cannot be reached',code='pick_place_search_exhausted',attempts=[
            {'stage':'lower_to_release','code':'ik_workspace','message':'漏斗投放点过远','xyz_mm':[600.,0.,40.]}])
        with patch('control_ui.rsi_trial.build_plan',side_effect=problem) as planner:
            case.start()
        _,state=case.request('state',method='GET');learning=state['learning']
        self.assertEqual(planner.call_count,1);self.assertEqual(arm.commands,[])
        self.assertEqual(learning['status'],'failed');self.assertEqual(learning['planning_diagnosis']['target'],'funnel')
        self.assertIn('已在官方初始姿态',learning['error']);self.assertIn('投放点',learning['error'])
        self.assertEqual(len(learning['recovery_events']),1)
        self.assertTrue(learning['recovery_events'][0]['already_at_official_pose'])
        self.assertFalse(learning['recovery_events'][0]['t100_attempted'])
        self.assertEqual(case.policies,[])
        with zipfile.ZipFile(learning['session_backup_file']) as archive:
            self.assertTrue(any(name.endswith('/planning-diagnosis.json') for name in archive.namelist()))
