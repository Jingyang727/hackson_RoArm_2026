"""XYZ validation and measured-depth projection without physical devices."""
import asyncio
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from control_ui import main
from control_ui.coordinates import DECLARATION, observe_pending
from control_ui.service import ArmService
from evoarm_rsi.coordinate_test import plan_point, check_coordinate_segment
from evoarm_rsi.roarm_geometry import inverse
from tests import test_ball_trial as fixtures
from tests.test_serial_transport_errors import api_request


class Coordinates(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.home=Path(self.temp.name)
        self.session=fixtures.layout(self.home);self.arm=fixtures.SimulatedArm();self.service=ArmService();self.service.arm=self.arm
        for p in (patch.object(main,'service',self.service),patch.object(Path,'home',return_value=self.home),
                  patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1','EVOARM_SETUP_ID':'bench-layout-v2'})):
            p.start();self.addCleanup(p.stop)
    def request(self,path,body=None,method='POST',image=False):
        return asyncio.run(api_request('/api/coordinates/'+path,body or {},method=method,decode_json=not image))
    def prepare(self,target=(350.,0.,145.)):
        response,data=self.request('prepare',{'xyz_mm':list(target)})
        self.assertEqual(response['status'],200,data);return data
    def move(self,prepared):return self.request('move',{'plan_id':prepared['plan_id'],'declaration':DECLARATION})
    def settle(self,now=None):
        pending=self.service.pending;start=pending['since'] if now is None else now
        for elapsed in (0.,.45,.9):observe_pending(self.service,pending,self.arm.status(),start+elapsed)

    def test_state_and_preview_never_move_unlock_or_require_camera(self):
        response,data=self.request('state',method='GET')
        self.assertEqual(response['status'],200,data);self.assertFalse(data['board_required']);self.assertFalse(data['camera_required'])
        plan=self.prepare()['plan']
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)
        self.assertEqual(plan['command']['T'],104)
        for source,target in [('tit','t'),('r','r'),('g','g')]:self.assertEqual(plan['command'][target],self.arm.raw[source])
        self.assertEqual(plan['physical_accuracy'],'unverified_until_operator_checks')

    def test_reported_180_8_gripper_keeps_raw_feedback_and_uses_a_legal_hold_command(self):
        self.arm.raw=inverse([353.6,-2.7,189.9],math.radians(6.2),math.radians(.4),math.radians(180.8))
        prepared=self.prepare([400.,50.,50.]);plan=prepared['plan']
        self.assertAlmostEqual(math.degrees(plan['initial_status']['g']),180.8)
        self.assertTrue(plan['gripper_hold']['normalized'])
        self.assertEqual(plan['command']['g'],3.14)
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)
        response,result=self.move(prepared)
        self.assertEqual(response['status'],200,result)
        self.assertEqual([c['T'] for c in self.arm.commands],[104])
        self.assertEqual(self.arm.commands[0]['g'],3.14)
        self.assertEqual(self.arm.commands[0]['t'],plan['initial_status']['tit'])
        self.assertEqual(self.arm.commands[0]['r'],plan['initial_status']['r'])
        saved=json.loads(Path(result['report_file']).read_text())
        self.assertAlmostEqual(math.degrees(saved['plan']['initial_status']['g']),180.8)
        self.assertAlmostEqual(math.degrees(saved['dispatch_status']['g']),180.8)
        self.assertEqual(self.move(prepared)[0]['status'],409)
        self.assertEqual(len(self.arm.commands),1)

    def test_gripper_feedback_outside_existing_tolerance_is_not_hidden_by_normalizing(self):
        self.arm.raw['g']=math.radians(183)
        response,detail=self.request('prepare',{'xyz_mm':[350.,0.,145.]})
        self.assertEqual(response['status'],409,detail)
        self.assertIn('超出允许偏差',detail['detail'])
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)

    def test_normalized_preview_still_rechecks_fresh_feedback_band_before_execution(self):
        self.arm.raw['g']=math.radians(181.8);prepared=self.prepare()
        # Less than the 1-degree plan drift limit, but beyond the existing
        # 2-degree stop-feedback tolerance. The command must still reject.
        self.arm.raw['g']=math.radians(182.5)
        response,detail=self.move(prepared)
        self.assertEqual(response['status'],409,detail)
        self.assertIn('超出允许偏差',detail['detail'])
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)

    def test_legal_hold_does_not_expand_active_gripper_or_other_joint_limits(self):
        prepared=self.prepare();stage=dict(prepared['plan']['stage']);stage['gripper_rad']=math.radians(181)
        with self.assertRaisesRegex(ValueError,'目标角度'):
            check_coordinate_segment(self.arm.status(),stage,prepared['plan']['table_plane'])
        self.arm.raw['g']=math.radians(180.8)
        target=[340*math.cos(math.radians(80)),340*math.sin(math.radians(80)),145.]
        response,detail=self.request('prepare',{'xyz_mm':target})
        self.assertEqual(response['status'],409,detail)
        self.assertIn('joint 1 training range',detail['detail'])
        self.assertEqual(self.arm.commands,[])

    def test_move_sends_exactly_once_records_per_axis_and_manual_verification(self):
        prepared=self.prepare();response,data=self.move(prepared)
        self.assertEqual(response['status'],200,data);self.assertTrue(data['sent'])
        self.assertEqual(len(self.arm.commands),1)
        self.assertEqual(self.move(prepared)[0]['status'],409)
        self.settle()
        self.assertIsNone(self.service.pending);self.assertEqual(self.service.armed_until,0)
        response,state=self.request('state',method='GET');result=state['result']
        self.assertEqual(result['status'],'feedback_within_tolerance')
        np.testing.assert_allclose(result['actual_minus_target_mm'],[0,0,0],atol=.001)
        self.assertEqual(result['physical_accuracy'],'not_checked')
        response,verified=self.request('verify',{'plan_id':prepared['plan_id'],'result':'offset','note':'实物偏右，尚未量距离'})
        self.assertEqual(response['status'],200,verified)
        saved=json.loads(Path(verified['report_file']).read_text())
        self.assertEqual(saved['physical_accuracy'],'offset');self.assertEqual(len(saved['feedback']),3)

    def test_timeout_keeps_measured_error_and_blocks_following_motion(self):
        prepared=self.prepare();original=dict(self.arm.raw);self.move(prepared);self.arm.raw=original
        pending=self.service.pending
        observe_pending(self.service,pending,self.arm.status(),pending['since']+21)
        self.assertEqual(self.service.fault['code'],'coordinate_timeout')
        self.assertGreater(self.service.fault['position_error_mm'],9)
        self.assertEqual(self.move(prepared)[0]['status'],409);self.assertEqual(len(self.arm.commands),1)

    def test_new_preview_supersedes_old_and_late_near_target_feedback_gets_settling_samples(self):
        old=self.prepare();new=self.prepare((355.,0.,145.))
        self.assertEqual(self.move(old)[0]['status'],409)
        self.move(new);pending=self.service.pending
        for elapsed in (25.,25.45,25.9):observe_pending(self.service,pending,self.arm.status(),pending['since']+elapsed)
        self.assertIsNone(self.service.fault);self.assertIsNone(self.service.pending)
        self.assertEqual(len(self.arm.commands),1)
        self.assertEqual(len(self.arm.commands),1)

    def test_settled_offset_is_reported_without_success_or_automatic_retry(self):
        prepared=self.prepare();self.move(prepared)
        self.arm.raw=inverse([340.,15.,145.],self.arm.raw['tit'],self.arm.raw['r'],self.arm.raw['g'])
        pending=self.service.pending;self.settle(now=pending['since']+3)
        self.assertIs(self.service.pending,pending)  # A short T104 plateau is not completion.
        self.settle(now=pending['since']+8)
        self.assertIsNone(self.service.pending);self.assertIsNone(self.service.fault)
        _,state=self.request('state',method='GET')
        self.assertEqual(state['result']['status'],'settled_with_offset')
        self.assertEqual(state['result']['physical_accuracy'],'not_checked')
        np.testing.assert_allclose(state['result']['actual_minus_target_mm'],[-10,15,0],atol=.001)
        self.assertEqual(len(self.arm.commands),1)

    def test_xyz_confirmation_does_not_wait_for_a_gripper_angle_threshold(self):
        prepared=self.prepare();self.move(prepared)
        self.arm.raw['g']-=math.radians(3)
        self.settle()
        self.assertIsNone(self.service.pending);self.assertIsNone(self.service.fault)
        _,state=self.request('state',method='GET')
        self.assertEqual(state['result']['status'],'feedback_within_tolerance')
        self.assertAlmostEqual(state['result']['pose_error_deg']['gripper'],-3,places=4)

    def test_unreachable_table_and_changed_pose_reject_before_any_motor_write(self):
        for target in ([1000,0,100],[350,0,-115]):
            self.assertEqual(self.request('prepare',{'xyz_mm':target})[0]['status'],409)
        prepared=self.prepare();self.arm.raw=inverse([330,0,145],self.arm.raw['tit'],0,self.arm.raw['g'])
        self.assertEqual(self.move(prepared)[0]['status'],409)
        self.assertEqual(self.arm.commands,[])

    def test_noop_read_only_mode_and_uncertain_transmission_cannot_replay(self):
        prepared=self.prepare([self.arm.raw[k] for k in 'xyz']);response,data=self.move(prepared)
        self.assertEqual(response['status'],200,data);self.assertFalse(data['sent']);self.assertEqual(self.arm.commands,[])
        self.assertEqual(self.service.armed_until,0)
        prepared=self.prepare()
        with patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'0'}):self.assertEqual(self.move(prepared)[0]['status'],409)
        self.assertEqual(self.arm.commands,[])
        self.arm.failed=True
        self.assertEqual(self.move(prepared)[0]['status'],409)
        self.assertEqual(self.service.fault['code'],'motion_transmission_unverified')
        self.assertEqual(self.move(prepared)[0]['status'],409);self.assertEqual(len(self.arm.commands),1)

    def test_previous_fault_is_preserved_and_never_automatically_recovered(self):
        self.service.fault={'code':'trial_cartesian_unverified','message':'previous failure'}
        prepared=self.prepare()
        self.assertEqual(self.move(prepared)[0]['status'],409)
        self.assertEqual(self.service.fault['code'],'trial_cartesian_unverified');self.assertEqual(self.arm.commands,[])

    def test_depth_snapshot_projects_measured_surface_instead_of_assuming_the_plane(self):
        folder=fixtures.observation(self.home)
        response,snapshot=self.request('depth-snapshot',method='GET')
        self.assertEqual(response['status'],200,snapshot)
        self.assertEqual(snapshot['source'],'recorded_rgbd_not_live')
        response,point=self.request('depth-point',{'snapshot_id':snapshot['snapshot_id'],'u':640,'v':360})
        self.assertEqual(response['status'],200,point);self.assertTrue(point['valid'])
        np.testing.assert_allclose(point['camera_xyz_mm'],[0,0,500],atol=.01)
        self.assertLess(abs(point['height_mm']),.01)
        # At the yellow sphere, measured visible surface is above the plane,
        # not clamped to table height or mislabeled as its 17mm-high center.
        _,surface=self.request('depth-point',{'snapshot_id':snapshot['snapshot_id'],'u':715,'v':381})
        self.assertGreater(surface['height_mm'],30)
        self.assertEqual(surface['meaning'],'measured_visible_surface_not_ball_center')
        self.assertFalse(surface['motion_sent']);self.assertEqual(self.arm.commands,[])
        for url in (snapshot['rgb_url'],snapshot['depth_url']):
            response,image=asyncio.run(api_request(url,{},method='GET',decode_json=False))
            self.assertEqual(response['status'],200);self.assertGreater(len(image),100)
        # A hole remains unknown; never infer a spatial point from zero depth.
        with np.load(folder/'depth_samples.npz') as saved:depth=saved['depth_mm'].copy();stamps=saved['monotonic_ns'].copy()
        depth[:,360,640]=0;np.savez_compressed(folder/'depth_samples.npz',depth_mm=depth,monotonic_ns=stamps)
        _,snapshot=self.request('depth-snapshot',method='GET')
        _,hole=self.request('depth-point',{'snapshot_id':snapshot['snapshot_id'],'u':640,'v':360})
        self.assertFalse(hole['valid']);self.assertNotIn('robot_xyz_mm',hole)

    def test_depth_is_optional_and_cannot_block_xyz_preview(self):
        self.assertEqual(self.request('depth-snapshot',method='GET')[0]['status'],409)
        self.prepare();self.assertEqual(self.arm.commands,[])
