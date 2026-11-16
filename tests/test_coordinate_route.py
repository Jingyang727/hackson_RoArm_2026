"""The coordinate page uses verified waypoints, keeping the direct API intact."""
import json
import math
from pathlib import Path
from unittest.mock import patch
import unittest

from tests import test_coordinates as fixtures
from evoarm_rsi.roarm_geometry import inverse
from control_ui.coordinates import observe_pending


class CoordinateRoutes(unittest.TestCase):
    setUp=fixtures.Coordinates.setUp
    request=fixtures.Coordinates.request
    move=fixtures.Coordinates.move

    def routed(self,speed=None):
        self.arm.raw=inverse([300.,0.,-105.],math.radians(75),0.,3.14)
        body={'xyz_mm':[330.,20.,-105.],'path_mode':'lift'}
        if speed is not None:body['speed']=speed
        response,plan=self.request('prepare',body)
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

    def test_default_speed_is_independent_from_the_legacy_grasp_speed(self):
        from evoarm_rsi.ball_trial import SPEED
        prepared=self.routed();plan=prepared['plan']
        self.assertEqual(plan['motion_profile']['speed'],.65)
        self.assertTrue(all(s['spd']==.65 for s in plan['segments']))
        self.assertEqual(plan['command']['spd'],.65)
        self.assertEqual(SPEED,.12);self.assertEqual(self.arm.commands,[])

    def test_selected_fast_speed_reaches_each_single_dispatch_and_persisted_preview(self):
        prepared=self.routed(speed=1.)
        with patch('control_ui.coordinates._launch_route',side_effect=lambda work:work()):
            response,result=self.move(prepared)
        self.assertEqual(response['status'],200,result)
        self.assertEqual(len(self.arm.commands),3)
        self.assertTrue(all(c['spd']==1. for c in self.arm.commands))
        saved=json.loads(Path(result['report_file']).read_text())
        self.assertEqual(saved['plan']['motion_profile']['speed'],1.)
        self.assertTrue(all(c['command']['spd']==1. for c in saved['commands']))

    def test_invalid_speed_is_rejected_before_a_motion_plan_or_command(self):
        for speed in (0.,1.1,'NaN'):
            response,error=self.request('prepare',{'xyz_mm':[330.,20.,100.],'speed':speed})
            self.assertEqual(response['status'],422,error)
        from evoarm_rsi.coordinate_motion import motion_profile
        for speed in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):motion_profile(speed)
        self.assertEqual(self.arm.commands,[])

    def waypoint_feedback(self):
        raw=self.arm.status();xyz=[raw[k] for k in 'xyz']
        report={'plan':{'initial_status':{**raw,'x':raw['x']-20.},'target_xyz_mm':[xyz[0]+10.,xyz[1],xyz[2]],
                       'pose_preserved_rad':{'pitch':raw['tit'],'roll':raw['r'],'gripper':raw['g']}},
                'feedback':[],'report_file':str(self.home/'waypoint.json')}
        pending={'kind':'coordinate_test','route_owner':'a'*32,'since':0.,'report':report}
        self.service.pending=pending;self.service.last_command={}
        return pending,raw,report

    def test_stationary_waypoint_offset_can_continue_after_point_eight_seconds_not_eight(self):
        pending,raw,report=self.waypoint_feedback()
        for now in (.1,.45,.85):observe_pending(self.service,pending,raw,now)
        self.assertIsNone(self.service.pending);self.assertEqual(report['status'],'settled_with_offset')
        self.assertAlmostEqual(report['position_error_mm'],10.)
        self.assertEqual(self.arm.commands,[])

    def test_waypoint_cannot_continue_while_joints_are_still_moving_at_fixed_xyz(self):
        pending,raw,report=self.waypoint_feedback()
        for now,degrees in ((.8,0.),(1.,1.),(1.2,2.)):
            observe_pending(self.service,pending,{**raw,'r':raw['r']+math.radians(degrees)},now)
        self.assertIs(self.service.pending,pending)
        for now in (3.,3.2,3.4):observe_pending(self.service,pending,{**raw,'r':raw['r']+math.radians(2)},now)
        self.assertIsNone(self.service.pending);self.assertEqual(report['status'],'settled_with_offset')
