"""Read-only serial status and explicit JSON command transport for RoArm-M3."""
import json
import time


class ArmCommunicationError(RuntimeError):
    pass


class RoArmSerial:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def __enter__(self):
        import serial
        try:
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baudrate
            ser.timeout = 0.2
            ser.write_timeout = 2
            ser.xonxoff = ser.rtscts = ser.dsrdtr = False
            ser.dtr = ser.rts = False
            ser.open()
            self.ser = ser
            self._wait_until_quiet(cap=12)
            return self
        except Exception as exc:
            raise ArmCommunicationError(f"打开串口失败: {exc}") from exc

    def __exit__(self, *args):
        if self.ser:
            self.ser.close()
            self.ser = None

    def send(self, payload):
        if not self.ser or not self.ser.is_open:
            raise ArmCommunicationError("串口未打开")
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        self.ser.write(line.encode())
        self.ser.flush()

    def _wait_until_quiet(self, cap):
        last = time.monotonic()
        deadline = last + cap
        saw_data = False
        while time.monotonic() < deadline:
            if self.ser.readline():
                saw_data = True
                last = time.monotonic()
            elif saw_data and time.monotonic() - last > 1.2:
                return

    def status(self, timeout=4.0):
        if not self.ser:
            raise ArmCommunicationError("串口未打开")
        self.ser.reset_input_buffer()
        self.send({"T": 105})
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            data = self.ser.readline().decode("utf-8", errors="replace").strip()
            if "{" not in data:
                continue
            try:
                obj = json.loads(data[data.index("{"):])
            except ValueError:
                continue
            if isinstance(obj, dict) and obj.get("T") == 1051:
                return obj
        raise ArmCommunicationError("状态反馈 T=1051 超时")
