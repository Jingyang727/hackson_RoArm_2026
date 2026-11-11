"""Real termios-error regressions with fake serial I/O, never actual USB."""
import asyncio
import json
import os
import termios
import unittest
from unittest.mock import Mock, patch

from control_ui import main
from control_ui.service import ArmService
from evoarm.serial_arm import RoArmSerial, ArmCommunicationError


async def api_request(path, value):
    body = json.dumps(value).encode()
    messages = []
    received = False

    async def receive():
        nonlocal received
        if not received:
            received = True
            return {"type": "http.request", "body": body, "more_body": False}
        return {"type": "http.disconnect"}

    async def send(message):
        messages.append(message)

    await main.app({"type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1",
        "method": "POST", "scheme": "http", "path": path, "root_path": "",
        "query_string": b"", "headers": [(b"content-type", b"application/json")],
        "server": ("localhost", 8765), "client": ("127.0.0.1", 10000)}, receive, send)
    start = next(m for m in messages if m["type"] == "http.response.start")
    payload = b"".join(m.get("body", b"") for m in messages if m["type"] == "http.response.body")
    return start, json.loads(payload)


class TransportErrors(unittest.TestCase):
    def arm(self):
        arm = RoArmSerial("/dev/cu.fake")
        arm.ser = Mock(is_open=True)
        return arm

    def test_termios_is_not_oserror_and_unlock_returns_json_409_not_plain_500(self):
        error = termios.error(6, "Device not configured")
        self.assertNotIsInstance(error, OSError)
        arm = self.arm()
        arm.ser.reset_input_buffer.side_effect = error
        service = ArmService()
        service.arm = arm
        with patch.dict(os.environ, {"EVOARM_ENABLE_MOTION": "1"}), patch.object(main, "service", service):
            start, body = asyncio.run(api_request("/api/unlock-once", {
                "declaration": "已检查设备并清空运动区域"}))
        self.assertEqual(start["status"], 409)
        self.assertIn(b"application/json", dict(start["headers"])[b"content-type"])
        self.assertIn("termios.error", body["detail"])
        self.assertIn("Device not configured", body["detail"])
        self.assertEqual(service.fault["code"], "feedback_timeout")
        self.assertEqual(service.armed_until, 0)
        self.assertIsNone(service.pending)
        arm.ser.write.assert_not_called()

    def test_calibration_transport_error_is_also_a_json_response(self):
        with patch.object(main.calibration, "capture", side_effect=ArmCommunicationError("device gone")):
            start, body = asyncio.run(api_request("/api/calibration/capture", {
                "session_id": "test", "point_id": "P1", "col": 1, "row": 1}))
        self.assertEqual(start["status"], 409)
        self.assertEqual(body["error_code"], "serial_communication_error")

    def test_uncertain_motor_write_revokes_lease_and_latches_fault(self):
        arm = self.arm()
        arm.ser.flush.side_effect = termios.error(6, "Device not configured")
        service = ArmService()
        service.arm = arm
        service.armed_until = 9999999999
        with self.assertRaises(ArmCommunicationError):
            service._send_motion({"T": 121, "joint": 1, "angle": 1})
        self.assertEqual(service.armed_until, 0)
        self.assertEqual(service.fault["code"], "motion_transmission_unverified")
        self.assertEqual(service.last_command["status"], "unverified_fault")
        arm.ser.write.assert_called_once()

    def test_write_failure_is_normalized_and_never_retried(self):
        arm = self.arm()
        arm.ser.write.side_effect = termios.error(6, "Device not configured")
        with self.assertRaises(ArmCommunicationError) as caught:
            arm.send({"T": 121, "joint": 1, "angle": 1})
        self.assertIsInstance(caught.exception.__cause__, termios.error)
        arm.ser.write.assert_called_once()
        arm.ser.flush.assert_not_called()

    def test_flush_failure_does_not_replay_potentially_transmitted_command(self):
        arm = self.arm()
        arm.ser.flush.side_effect = termios.error(6, "Device not configured")
        with self.assertRaises(ArmCommunicationError):
            arm.send({"T": 105})
        arm.ser.write.assert_called_once()
        arm.ser.flush.assert_called_once()

    def test_read_failure_is_normalized_after_one_readonly_status_request(self):
        arm = self.arm()
        arm.ser.readline.side_effect = termios.error(6, "Device not configured")
        with self.assertRaises(ArmCommunicationError):
            arm.status()
        arm.ser.readline.assert_called_once()
        self.assertEqual(json.loads(arm.ser.write.call_args.args[0]), {"T": 105})


if __name__ == "__main__":
    unittest.main()
