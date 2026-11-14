"""The coordinate page uses verified waypoints, keeping the direct API intact."""
import json
import math
from pathlib import Path
from unittest.mock import patch
import unittest

from tests import test_coordinates as fixtures
from evoarm_rsi.roarm_geometry import inverse


class CoordinateRoutes(unittest.TestCase):
    setUp=fixtures.Coordinates.setUp
    request=fixtures.Coordinates.request
    move=fixtures.Coordinates.move

    def routed(self):
        self.arm.raw=inverse([300.,0.,-105.],math.radians(75),0.,3.14)
        response,plan=self.request('prepare',{'xyz_mm':[330.,20.,-105.],'path_mode':'lift'})
        self.assertEqual(response['status'],200,plan);return plan

    def test_low_coordinate_preview_has_an_up_over_down_polyline_without_motor_writes(self):
        prepared=self.routed();plan=prepared['plan'];segments=plan['segments']
        self.assertEqual([s['stage'] for s in segments],['raise_current','coordinate_transfer','coordinate_descend'])
        self.assertEqual(segments[0]['xyz_mm'][:2],[300.,0.])
        self.assertGreater(segments[0]['xyz_mm'][2],-105.)
        self.assertEqual(segments[-1]['xyz_mm'],[330.,20.,-105.])
        self.assertGreater(len(plan['path_xyz_mm']),3);self.assertEqual(self.arm.commands,[])

    def test_each_waypoint_waits_for_feedback_and_route_cannot_be_replayed(self):
        prepared=self.routed()
        with patch('control_ui.coordinates._launch_route',side_effect=lambda work:work()):
            response,result=self.move(prepared)
        self.assertEqual(response['status'],200,result)
        self.assertEqual(result['status'],'feedback_within_tolerance')
        self.assertEqual(len(self.arm.commands),3)
        self.assertEqual([c['T'] for c in self.arm.commands],[104,104,104])
        self.assertEqual(self.arm.commands[0]['x'],300.);self.assertEqual(self.arm.commands[0]['y'],0.)
        self.assertEqual(self.arm.commands[1]['z'],self.arm.commands[0]['z'])
        self.assertEqual(self.arm.commands[2]['x'],self.arm.commands[1]['x'])
        self.assertEqual(self.arm.commands[2]['y'],self.arm.commands[1]['y'])
        self.assertIsNone(self.service.automation_owner);self.assertIsNone(self.service.pending)
        self.assertEqual(self.move(prepared)[0]['status'],409);self.assertEqual(len(self.arm.commands),3)
        self.assertEqual(len(json.loads(Path(result['report_file']).read_text())['commands']),3)

    def test_uncertain_first_write_does_not_dispatch_other_waypoints(self):
        prepared=self.routed();self.arm.failed=True
        with patch('control_ui.coordinates._launch_route',side_effect=lambda work:work()):
            response,result=self.move(prepared)
        self.assertEqual(response['status'],200,result)
        self.assertEqual(result['status'],'transmission_uncertain');self.assertEqual(len(self.arm.commands),1)
        self.assertIsNone(self.service.automation_owner);self.assertIsNotNone(self.service.fault)
