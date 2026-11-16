"""The reported below-plane feedback must permit upward departure, not drag."""
import copy
import json
import math
from pathlib import Path
import unittest

import numpy as np

from evoarm_rsi.ball_trial import check_segment
from evoarm_rsi.roarm_geometry import inverse,height_above_plane
from evoarm_rsi.rsi_route import execution_target,at_height

DATA=json.loads((Path(__file__).parent/'fixtures/reported_lift_start_20261116.json').read_text())


class ReportedLift(unittest.TestCase):
    def setUp(self):
        self.plane=DATA['table_plane'];self.xyz=DATA['actual_xyz_mm']
        # The supplied report omitted roll. Keep that assumption explicit.
        self.raw=inverse(self.xyz,DATA['actual_pitch_rad'],DATA['synthetic_roll_rad_for_ik_test'],DATA['actual_gripper_rad'])
        self.step={'stage':'lift','kind':'cartesian','xyz_mm':at_height(self.xyz,DATA['transfer_height_mm'],self.plane),
            'pitch_rad':DATA['lift_pitch_rad'],'roll_rad':0.,'gripper_rad':DATA['actual_gripper_rad'],
            'minimum_tip_height_mm':0.,'vertical_only':True,'spd':1.}

    def test_report_reproduces_old_zero_percent_rejection_and_upward_departure_passes(self):
        self.assertAlmostEqual(height_above_plane(self.xyz,self.plane),DATA['expected_start_height_mm'])
        with self.assertRaisesRegex(ValueError,'路径0.0%'):
            check_segment(self.raw,self.step,self.plane)
        original=copy.deepcopy(self.step);target=execution_target(self.step,self.raw,self.plane)
        check_segment(self.raw,target,self.plane)
        self.assertEqual(self.step,original);self.assertEqual(target['xyz_mm'][:2],self.xyz[:2])
        self.assertGreater(target['xyz_mm'][2],self.xyz[2])
        self.assertEqual(target['gripper_rad'],DATA['actual_gripper_rad'])
        self.assertEqual(target['minimum_tip_height_mm'],DATA['expected_start_height_mm'])
        self.assertEqual(target['table_departure']['direction'],'upward_only')
        # The whole fixed-XY line rises monotonically in this tilted plane.
        heights=[height_above_plane((np.asarray(self.xyz)*(1-a)+np.asarray(target['xyz_mm'])*a).tolist(),self.plane) for a in np.linspace(0,1,51)]
        self.assertTrue(all(b>=a for a,b in zip(heights,heights[1:])))

    def test_downward_lift_or_endpoint_below_table_cannot_use_departure(self):
        for height in (-2.,-.2):
            with self.assertRaises(ValueError):
                execution_target({**self.step,'xyz_mm':at_height(self.xyz,height,self.plane)},self.raw,self.plane)
        with self.assertRaises(ValueError):execution_target({**self.step,'vertical_only':False},self.raw,self.plane)

    def test_horizontal_transport_and_descent_still_reject_below_plane_start(self):
        for name,minimum in (('transport',72.4),('descend',0.)):
            step={**self.step,'stage':name,'minimum_tip_height_mm':minimum,'vertical_only':name=='descend'}
            target=execution_target(step,self.raw,self.plane)
            self.assertEqual(target['minimum_tip_height_mm'],minimum)
            with self.assertRaises(ValueError):check_segment(self.raw,target,self.plane)

    def test_lift_settled_too_low_still_cannot_transport(self):
        actual=inverse(at_height(self.xyz,50.,self.plane),DATA['lift_pitch_rad'],0.,DATA['actual_gripper_rad'])
        step={**self.step,'stage':'transport','xyz_mm':at_height(DATA['release_tip_xyz_mm'],DATA['transfer_height_mm'],self.plane),
            'vertical_only':False,'minimum_tip_height_mm':DATA['minimum_transfer_height_mm']}
        with self.assertRaisesRegex(ValueError,'72.4'):check_segment(actual,execution_target(step,actual,self.plane),self.plane)
