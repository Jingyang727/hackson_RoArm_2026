"""Single serial-port owner; read-only by default; one permit per tiny move."""
import os
import threading
import time
from serial.tools import list_ports
from evoarm.serial_arm import RoArmSerial
from .safety import JOINTS, UnsafeMotion, plan_single_step


class ArmService:
    def __init__(self):
        self.lock = threading.RLock()
        self.arm = None
        self.port = None
        self.armed_until = 0.0
        self.last_motion = 0.0

    @property
    def motion_enabled(self):
        return os.getenv("EVOARM_ENABLE_MOTION", "0") == "1"

    def ports(self):
        return [{"device": p.device, "description": p.description}
                for p in list_ports.comports() if p.device.startswith("/dev/cu.")]

    def connect(self, port):
        with self.lock:
            if port not in {p["device"] for p in self.ports()}:
                raise ValueError("串口不在可用设备列表")
            self.disconnect()
            arm = RoArmSerial(port)
            try:
                arm.__enter__()
                result = arm.status()
            except Exception:
                arm.__exit__(None, None, None)
                raise
            self.arm, self.port = arm, port
            self.armed_until = 0.0
            return result

    def disconnect(self):
        with self.lock:
            self.armed_until = 0.0
            if self.arm:
                self.arm.__exit__(None, None, None)
            self.arm = None
            self.port = None

    def status(self):
        with self.lock:
            if self.arm is None:
                raise ValueError("机械臂尚未连接")
            try:
                return self.arm.status()
            except Exception:
                self.armed_until = 0.0
                raise

    def health(self):
        with self.lock:
            return {
                "connected": self.arm is not None, "port": self.port,
                "motion_enabled": self.motion_enabled,
                "armed": self.arm is not None and self.motion_enabled and time.monotonic() < self.armed_until,
                "joints": [{"id": k, "label": j[0], "field": j[1], "min": j[2], "max": j[3]}
                           for k, j in JOINTS.items()],
            }

    def unlock_once(self, declaration):
        with self.lock:
            self.armed_until = 0.0
            if not self.motion_enabled:
                raise UnsafeMotion("服务端只读模式，未启用运动")
            if not self.arm:
                raise UnsafeMotion("未连接")
            if declaration != "已检查设备并清空运动区域":
                raise UnsafeMotion("必须确认检查机械臂和运动区域")
            self.arm.status()
            self.armed_until = time.monotonic() + 30
            return {"armed": True, "expires_in_seconds": 30}

    def step(self, joint, delta):
        with self.lock:
            if not self.motion_enabled or self.arm is None:
                raise UnsafeMotion("未启用运动或机械臂未连接")
            if time.monotonic() > self.armed_until:
                self.armed_until = 0.0
                raise UnsafeMotion("单次授权无效或超时")
            self.armed_until = 0.0
            if time.monotonic() - self.last_motion < 2:
                raise UnsafeMotion("请至少间隔2秒")
            status = self.arm.status()
            plan = plan_single_step(status, joint, delta)
            self.arm.send(plan.payload)
            self.last_motion = time.monotonic()
            return {"message": "仅已发送指令，尚未确认动作完成", "current_deg": plan.current_deg,
                    "target_deg": plan.target_deg, "joint": plan.joint}
