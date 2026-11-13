"""Old wrist-recovery clients cannot cause an unnecessary motor command."""
import asyncio
import unittest
from unittest.mock import Mock, patch

from control_ui import main
from evoarm.limits import JOINTS
from tests.test_serial_transport_errors import api_request


class RetiredStartMotion(unittest.TestCase):
    def payload(self):
        return {'reference_sha256':'a'*64,'expected_joint_rad':[0.,0.,1.57,1.4,0.,3.14],
                'camera_request_id':'b'*32}

    def test_capabilities_explain_the_replacement(self):
        response,data=asyncio.run(api_request('/api/bench/motion/capabilities',{},method='GET'))
        self.assertEqual(response['status'],200)
        self.assertFalse(data['high_wrist_recovery']);self.assertTrue(data['retired'])
        self.assertEqual(data['replacement'],'/coordinates')
        self.assertEqual(JOINTS[4][2:],(-90.,90.))

    def test_previously_valid_request_never_reads_or_moves_hardware(self):
        service=Mock()
        with patch.object(main,'service',service):
            for _ in range(2):
                response,data=asyncio.run(api_request('/api/bench/motion/start-once',self.payload()))
                self.assertEqual(response['status'],409)
                self.assertIn('无需先收回腕部',data['detail'])
        self.assertEqual(service.mock_calls,[])

    def test_arbitrary_targets_still_fail_schema_validation(self):
        response,_=asyncio.run(api_request('/api/bench/motion/start-once',{**self.payload(),'target_deg':55.}))
        self.assertEqual(response['status'],422)
