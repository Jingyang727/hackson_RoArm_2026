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

    @staticmethod
    def _serial_io(operation, function, *args):
        """Normalize driver/termios errors without retrying any I/O.

        POSIX termios.error is NOT an OSError. Restrict this wrapper to
        transport calls so parsing/planning errors are not concealed.
        """
        try:
            return function(*args)
        except Exception as exc:
            kind = type(exc).__module__ + "." + type(exc).__name__
            raise ArmCommunicationError(
                f"串口{operation}失败（{kind}: {exc}）；请核查设备连接，未自动重连或重发"
            ) from exc

    def send(self, payload):
        if not self.ser or not self.ser.is_open:
            raise ArmCommunicationError("串口未打开")
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        self._serial_io("写入", self.ser.write, line.encode())
        self._serial_io("刷新输出", self.ser.flush)

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
        self._serial_io("清空接收缓冲区", self.ser.reset_input_buffer)
        self.send({"T": 105})
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            raw = self._serial_io("读取", self.ser.readline)
            data = raw.decode("utf-8", errors="replace").strip()
            if "{" not in data:
                continue
            try:
                obj = json.loads(data[data.index("{"):])
            except ValueError:
                continue
            if isinstance(obj, dict) and obj.get("T") == 1051:
                return obj
        raise ArmCommunicationError("状态反馈 T=1051 超时")
