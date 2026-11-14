"""Collision-relevant phase invariants, including feedback offsets and tilt."""
import copy
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from evoarm_rsi.pick_place import build_plan,observe_scene,DEFAULT_PARAMETERS
from evoarm_rsi.rsi_route import at_height,execution_target
from evoarm_rsi.roarm_geometry import height_above_plane,inverse
from evoarm_rsi.ball_trial import check_segment
from tests.test_ball_trial import layout,SimulatedArm
from tests.test_rsi_pick_place import scene


class Routes(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        self.saved=layout(self.home);folder,_,_=scene(self.home,self.saved)
        self.world,_,_,_=observe_scene(folder,self.saved)

    def plan(self,status):return build_plan(self.saved,status,self.world,copy.deepcopy(DEFAULT_PARAMETERS))

    def test_low_start_first_lifts_without_xy_rotation_or_opening(self):
        raw=inverse([300.,0.,-105.],math.radians(80),0.,math.pi);plan=self.plan(raw)
        first=plan['steps'][0]
        self.assertEqual(first['stage'],'raise_current');self.assertTrue(first['vertical_only'])
        np.testing.assert_allclose(first['xyz_mm'][:2],[raw['x'],raw['y']])
        self.assertGreater(first['xyz_mm'][2],raw['z'])
        self.assertEqual(first['pitch_rad'],raw['tit']);self.assertFalse(first.get('open_while_moving',False))

    def test_all_vertical_legs_keep_xy_and_lateral_legs_stay_high(self):
        for tilted in (False,True):
            if tilted:
                n=np.array([.04,.02,1.]);n/=np.linalg.norm(n)
                self.world['table_plane']={'normal_up':n.tolist(),'d_mm':115.}
            raw=inverse([300.,0.,-90.],math.radians(80),0.,math.pi);plan=self.plan(raw)
            current=[raw[k] for k in 'xyz'];plane=plan['table_plane'];floor=plan['route_policy']['minimum_transfer_height_mm']
            for step in plan['steps']:
                if step['kind']!='cartesian':continue
                if step['vertical_only']:np.testing.assert_allclose(step['xyz_mm'][:2],current[:2],atol=1e-7)
                else:
                    self.assertGreaterEqual(height_above_plane(current,plane),floor-.5)
                    self.assertGreaterEqual(height_above_plane(step['xyz_mm'],plane),floor-.5)
                current=step['xyz_mm']

    def test_ball_radius_tool_reference_and_stationary_release_are_explicit(self):
        plan=self.plan(SimulatedArm().status());geo=plan['object_geometry'];plane=plan['table_plane']
        self.assertEqual(geo['ball_diameter_mm'],34.)
        self.assertAlmostEqual(math.dist(geo['ball_top_xyz_mm'],self.world['ball']['center_robot_mm']),17.)
        self.assertAlmostEqual(math.dist(geo['grasp_center_xyz_mm'],plan['grasp_tip_xyz_mm']),19.)
        close=next(i for i,s in enumerate(plan['steps']) if s['stage']=='close')
        self.assertEqual(plan['steps'][close-1]['stage'],'descend')
        self.assertEqual(plan['steps'][-2]['stage'],'lower_to_release')
        self.assertEqual(plan['steps'][-1]['stage'],'release');self.assertGreater(plan['steps'][-2]['settle_s'],0)
        mouth=self.world['funnel']['mouth_center_robot_mm'];center=geo['release_ball_center_xyz_mm']
        np.testing.assert_allclose(center[:2],mouth[:2],atol=1e-7)
        self.assertAlmostEqual(height_above_plane(center,plane)-17.-height_above_plane(mouth,plane),18.)

    def test_runtime_vertical_descent_uses_measured_xy_after_tracking_offset(self):
        plan=self.plan(SimulatedArm().status());step=next(s for s in plan['steps'] if s['stage']=='descend')
        actual=inverse([step['xyz_mm'][0]+15.,step['xyz_mm'][1]-8.,0.],step['pitch_rad'],0.,math.radians(110))
        target=execution_target(step,actual,plan['table_plane'])
        self.assertEqual(target['xyz_mm'][:2],[actual['x'],actual['y']])
        self.assertEqual(step['xyz_mm'],plan['grasp_tip_xyz_mm'])

    def test_feedback_below_transit_clearance_cannot_start_lateral_segment(self):
        plan=self.plan(SimulatedArm().status());step=next(s for s in plan['steps'] if s['stage']=='transport')
        actual=inverse(at_height([300.,0.,0.],30.,plan['table_plane']),step['pitch_rad'],0.,math.pi)
        with self.assertRaises(ValueError):check_segment(actual,step,plan['table_plane'])
