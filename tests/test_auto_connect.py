"""USB auto-connect: shared, idempotent, fail-closed and never moves servos."""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from control_ui.service import ArmService
from control_ui.safety import UnsafeMotion


class MockSerial:
    opened = []
    closed = []
    commands = []

    def __init__(self, port):
        self.port = port

    def __enter__(self):
        self.opened.append(self.port)
        return self

    def __exit__(self, *_):
        self.closed.append(self.port)

    def status(self, timeout=None):
        return {
            "T": 1051, "x": 250.0, "y": 10.0, "z": 120.0,
            "tit": 0.0, "r": 0.0, "g": 2.1,
            **{"torswitch"+k: 1 for k in "BSETRG"}
        }

    def send(self, command):
        self.commands.append(command)


class AutoConnectTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        MockSerial.opened.clear()
        MockSerial.closed.clear()
        MockSerial.commands.clear()
        self.last_file = Path(self.tmp.name) / "port.json"
        self.ports = [{"device": "/dev/cu.usbserial-130", "description": "RoArm USB"}]

    def mock(self, service):
        from contextlib import ExitStack
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch("control_ui.service.RoArmSerial", MockSerial))
        stack.enter_context(patch.object(service, "ports", return_value=self.ports))
        stack.enter_context(patch.object(service, "_last_port_path", return_value=self.last_file))

    def test_one_auto_connection_reused_by_every_page_without_motor_commands(self):
        with patch.dict(os.environ, {"EVOARM_SERIAL_PORT": ""}):
            service = ArmService()
            self.mock(service)
            initial=service.auto_connect()
            self.assertTrue(initial["connected"])
            self.assertFalse(initial["reused"])
            self.assertEqual(service.port,"/dev/cu.usbserial-130")
            self.assertEqual(json.loads(self.last_file.read_text())["port"],service.port)
            again=service.auto_connect()
            self.assertTrue(again["reused"])
            service.connect("/dev/cu.usbserial-130") # second UI opened same port
            self.assertEqual(MockSerial.opened, ["/dev/cu.usbserial-130"])
            self.assertEqual(MockSerial.closed, [])
            self.assertEqual(MockSerial.commands, [])
            self.assertFalse(service.health()["armed"])
            service.disconnect()

    def test_saved_port_wins_when_multiple_usb_candidates_are_present(self):
        with patch.dict(os.environ, {"EVOARM_SERIAL_PORT": ""}):
            self.last_file.write_text(json.dumps({"port":"/dev/cu.usbserial-130"}))
            service=ArmService()
            self.ports.append({"device":"/dev/cu.usbserial-222","description":"Other USB"})
            self.mock(service)
            result=service.auto_connect()
            self.assertTrue(result["connected"])
            self.assertEqual(result["port"],"/dev/cu.usbserial-130")
            self.assertEqual(len(MockSerial.opened),1)

    def test_ambiguous_ports_fail_closed_without_guess(self):
        with patch.dict(os.environ, {"EVOARM_SERIAL_PORT": ""}):
            service=ArmService()
            self.ports.append({"device":"/dev/cu.usbserial-222","description":"Other USB"})
            self.mock(service)
            result=service.auto_connect()
            self.assertFalse(result["connected"])
            self.assertEqual(MockSerial.opened,[])
            self.assertEqual(MockSerial.commands,[])

    def test_motion_pending_or_teach_release_cannot_switch_serial(self):
        service=ArmService()
        self.ports.append({"device":"/dev/cu.usbserial-222","description":"Other USB"})
        self.mock(service)
        service.connect("/dev/cu.usbserial-130")
        service.teach_active=True
        with self.assertRaises(UnsafeMotion):
            service.connect("/dev/cu.usbserial-222")
        self.assertEqual(MockSerial.opened,["/dev/cu.usbserial-130"])
        self.assertEqual(MockSerial.commands,[])


if __name__=="__main__":
    unittest.main()
