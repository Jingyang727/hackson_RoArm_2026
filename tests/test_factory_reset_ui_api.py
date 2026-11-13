"""Explicit browser reset through ArmService; all motor I/O is synthetic."""
import asyncio
import json
import math
import tempfile
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

from control_ui import main
from control_ui.service import ArmService, FACTORY_RESET_DECLARATION
from control_ui.safety import FACTORY_DEGREES
from evoarm_rsi.roarm_geometry import forward
from tests.test_ball_trial import SimulatedArm, layout
from tests.test_serial_transport_errors import api_request


class ResetArm(SimulatedArm):
    fail_reset=False
    def send(self,payload):
        if payload['T']!=100:return super().send(payload)
        self.commands.append(payload)
        if self.fail_reset:raise OSError('synthetic lost reset acknowledgement')
        self.raw.update({k:math.radians(FACTORY_DEGREES[j]) for j,k in enumerate('bsetrg',1)})
        model=forward(self.raw);self.raw.update(zip('xyz',model['xyz_mm']));self.raw['tit']=model['pitch_rad']


class FactoryResetUI(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.home=Path(temp.name)
        layout(self.home);self.arm=ResetArm();self.service=ArmService();self.service.arm=self.arm
        self.request_id=uuid.uuid4().hex
        for p in (patch.object(main,'service',self.service),patch.object(Path,'home',return_value=self.home),
                  patch('control_ui.service.RECOVERY_SAMPLE_GAP_S',0),
                  patch.dict('os.environ',{'EVOARM_ENABLE_MOTION':'1','EVOARM_SETUP_ID':'bench-layout-v2','EVOARM_ALLOWED_JOINTS':'1,2,3,4,5,6'})):
            p.start();self.addCleanup(p.stop)

    def call(self,path,body=None,method='POST'):
        path,_,query=path.partition('?')
        return asyncio.run(api_request(path,body or {},method=method,query_string=query.encode()))

    def reset(self,**changes):
        return self.call('/api/coordinates/factory-reset',{'request_id':self.request_id,
                         'declaration':FACTORY_RESET_DECLARATION,**changes})

    def receipt(self):return self.home/'.evoarm/rsi/factory_resets'/f'{self.request_id}.json'

    def test_preview_is_read_only_and_shows_official_targets(self):
        response,body=self.call('/api/home/preview?kind=factory',method='GET')
        self.assertEqual(response['status'],200,body)
        self.assertEqual([r['target_deg'] for r in body['rows']],[0,0,90,0,0,180])
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)

    def test_confirmation_is_required_and_failed_attempt_disarms(self):
        self.service.armed_until=1e20
        response,_=self.reset(declaration='')
        self.assertEqual(response['status'],409)
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)
        self.assertFalse(self.receipt().exists())

    def test_one_reset_consumes_request_and_invalidates_old_xyz_plan(self):
        response,prepared=self.call('/api/coordinates/prepare',{'xyz_mm':[350,0,145]})
        self.assertEqual(response['status'],200,prepared)
        original_send=self.arm.send
        def send(command):
            receipt=json.loads(self.receipt().read_text())
            self.assertEqual(receipt['status'],'dispatching')
            self.assertEqual(receipt['initial_status']['g'],self.arm.raw['g'])
            original_send(command)
        with patch.object(self.arm,'send',side_effect=send):
            response,body=self.reset()
        self.assertEqual(response['status'],200,body)
        self.assertEqual(self.arm.commands,[{'T':100}]);self.assertEqual(self.service.armed_until,0)
        self.assertEqual(self.service.pending['kind'],'factory')
        self.service.status()
        self.assertIsNone(self.service.pending)
        self.assertEqual(self.service.last_command['status'],'feedback_confirmed')
        from control_ui.coordinates import DECLARATION
        self.assertEqual(self.call('/api/coordinates/move',{'plan_id':prepared['plan_id'],'declaration':DECLARATION})[0]['status'],409)
        self.assertEqual(self.reset()[0]['status'],409)
        self.assertEqual(self.arm.commands,[{'T':100}])

    def test_consumed_request_stays_consumed_after_service_restart(self):
        self.assertEqual(self.reset()[0]['status'],200)
        new_service=ArmService();new_service.arm=ResetArm()
        with patch.object(main,'service',new_service):
            self.assertEqual(self.reset()[0]['status'],409)
        self.assertEqual(new_service.arm.commands,[])

    def test_stationary_fault_is_verified_then_reset_without_torque_commands(self):
        fault={'code':'coordinate_timeout','message':'previous movement unconfirmed'}
        self.service.fault=dict(fault)
        response,body=self.reset()
        self.assertEqual(response['status'],200,body)
        self.assertEqual(self.arm.commands,[{'T':100}]);self.assertIsNone(self.service.fault)
        self.assertEqual(json.loads(self.receipt().read_text())['previous_fault'],fault)

    def test_pending_movement_is_not_cleared_or_overlaid_with_reset(self):
        from control_ui.coordinates import DECLARATION
        _,plan=self.call('/api/coordinates/prepare',{'xyz_mm':[350,0,145]})
        self.call('/api/coordinates/move',{'plan_id':plan['plan_id'],'declaration':DECLARATION})
        response,body=self.reset()
        self.assertEqual(response['status'],409,body)
        self.assertIn('不能叠加复位',body['detail'])
        self.assertEqual([c['T'] for c in self.arm.commands],[104])
        self.assertIsNotNone(self.service.pending);self.assertEqual(self.service.armed_until,0)

    def test_unlocked_torque_never_sends_or_clears_existing_fault(self):
        self.service.fault={'code':'coordinate_timeout','message':'existing fault'}
        status=self.arm.status
        with patch.object(self.arm,'status',side_effect=lambda timeout=None:{**status(timeout),'torswitchR':0}):
            self.assertEqual(self.reset()[0]['status'],409)
        self.assertEqual(self.arm.commands,[])
        self.assertEqual(self.service.fault['code'],'coordinate_timeout')

    def test_changing_feedback_rejects_before_any_reset(self):
        status=self.arm.status;reads=0
        def changing(timeout=None):
            nonlocal reads
            reads+=1;raw=status(timeout)
            if reads>=3:raw['b']+=math.radians(2)
            return raw
        with patch.object(self.arm,'status',side_effect=changing):
            self.assertEqual(self.reset()[0]['status'],409)
        self.assertEqual(self.arm.commands,[]);self.assertFalse(self.receipt().exists())

    def test_read_failure_and_missing_joint_authorization_never_reset(self):
        with patch.object(self.arm,'status',side_effect=OSError('feedback lost')):
            self.assertEqual(self.reset()[0]['status'],409)
        with patch.dict('os.environ',{'EVOARM_ALLOWED_JOINTS':'6'}):
            self.assertEqual(self.reset()[0]['status'],409)
        self.assertEqual(self.arm.commands,[])

    def test_uncertain_write_is_consumed_and_not_replayed(self):
        self.arm.fail_reset=True
        response,_=self.reset();self.assertEqual(response['status'],409)
        self.assertEqual(self.arm.commands,[{'T':100}])
        self.assertEqual(self.service.fault['code'],'motion_transmission_unverified')
        self.assertEqual(self.service.armed_until,0)
        self.assertEqual(self.reset()[0]['status'],409)
        self.assertEqual(len(self.arm.commands),1)
        self.assertIn('uncertain',json.loads(self.receipt().read_text())['status'])

    def test_receipt_write_failure_cannot_move(self):
        with patch('evoarm_rsi.task_agent._write',side_effect=OSError('disk full')):
            self.assertEqual(self.reset()[0]['status'],409)
        self.assertEqual(self.arm.commands,[]);self.assertEqual(self.service.armed_until,0)
