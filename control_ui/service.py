"""Mac local robot service: one serial owner, limited manual control and guided home."""
import os
import threading
import time

from serial.tools import list_ports
from evoarm.serial_arm import RoArmSerial
from .home_pose import save_pose, load_pose
from .safety import (
    JOINTS, JOG_STEPS, FACTORY_DEGREES, UnsafeMotion,
    current_degrees, checked_home_pose, plan_single_step, plan_home_step,
)

SESSION_SECONDS = 120
MIN_GAP_SECONDS = 0.8
ARRIVAL_TOLERANCE = 2.0


class ArmService:
    def __init__(self):
        self.lock = threading.RLock()
        self.arm = None
        self.port = None
        self.armed_until = 0.0
        self.last_motion_at = 0.0
        self.pending = None
        self.last_command = None

    @property
    def motion_enabled(self):
        return os.getenv("EVOARM_ENABLE_MOTION", "0") == "1"

    @property
    def allowed_joints(self):
        raw = os.getenv("EVOARM_ALLOWED_JOINTS", "1,2,3,4,5,6")
        try:
            result = {int(x.strip()) for x in raw.split(",") if x.strip()}
        except ValueError:
            return set()
        return result & set(JOINTS)

    def ports(self):
        return [{"device": p.device, "description": p.description} for p in list_ports.comports()
                if p.device.startswith("/dev/cu.")]

    def connect(self, port):
        with self.lock:
            if port not in {p["device"] for p in self.ports()}:
                raise ValueError("选择的串口不在 Mac 可用设备列表")
            self.disconnect()
            arm = RoArmSerial(port)
            try:
                arm.__enter__()
                result = arm.status()
            except Exception:
                arm.__exit__(None, None, None)
                raise
            self.arm, self.port = arm, port
            self.armed_until = 0
            self.pending = self.last_command = None
            return result

    def disconnect(self):
        with self.lock:
            self.armed_until = 0
            if self.arm is not None:
                self.arm.__exit__(None, None, None)
            self.arm = self.port = None
            self.pending = None

    def status(self):
        with self.lock:
            if self.arm is None:
                raise ValueError("尚未连接机械臂")
            try:
                result = self.arm.status()
            except Exception:
                self.armed_until = 0
                raise
            if self.pending:
                actual = current_degrees(result)[self.pending["joint"]]
                if abs(actual - self.pending["target_deg"]) <= ARRIVAL_TOLERANCE:
                    self.pending = None
            return result

    def health(self):
        with self.lock:
            remaining = max(0, int(self.armed_until - time.monotonic()))
            return {
                "connected": self.arm is not None, "port": self.port,
                "motion_enabled": self.motion_enabled,
                "allowed_joints": sorted(self.allowed_joints) if self.motion_enabled else [],
                "armed": bool(self.arm and self.motion_enabled and remaining > 0),
                "arm_seconds_left": remaining,
                "jog_steps": JOG_STEPS,
                "joints": [
                    {"id": j, "label": spec[0], "field": spec[1], "min": spec[2], "max": spec[3]}
                    for j, spec in JOINTS.items()
                ],
                "pending": self.pending,
                "last_command": self.last_command,
            }

    def unlock_once(self, declaration):
        # API name kept for backward compatibility; unlock lasts 2 minutes.
        with self.lock:
            self.armed_until = 0
            if not self.motion_enabled:
                raise UnsafeMotion("只读模式：需要显式设置 EVOARM_ENABLE_MOTION=1")
            if self.arm is None:
                raise UnsafeMotion("请先连接机械臂")
            if declaration != "已检查设备并清空运动区域":
                raise UnsafeMotion("请先确认机械臂、固定支架和工作区域安全")
            self.status()
            self.armed_until = time.monotonic() + SESSION_SECONDS
            return {"armed": True, "expires_in_seconds": SESSION_SECONDS}

    def disarm(self):
        # Intentionally does not claim to stop a command already sent.
        with self.lock:
            self.armed_until = 0
            return {"locked": True, "notice": "已锁定新的运动指令。已发出的运动不会因此立即停止；紧急情况请切断机械臂运动电源"}

    def _preflight(self, joint):
        if self.arm is None or not self.motion_enabled:
            raise UnsafeMotion("机械臂未连接，或后端仍处于只读模式")
        if joint not in self.allowed_joints:
            raise UnsafeMotion("该关节未被当前后端授权")
        if time.monotonic() >= self.armed_until:
            self.armed_until = 0
            raise UnsafeMotion("控制授权已过期，请再次确认")
        if time.monotonic() - self.last_motion_at < MIN_GAP_SECONDS:
            raise UnsafeMotion("动作间隔太短，请稍后重试")
        status = self.status()
        if self.pending is not None:
            raise UnsafeMotion("上一动作尚未确认到达目标，拒绝叠加运动指令")
        return status

    def _send_move(self, plan):
        self.arm.send(plan.payload)
        self.last_motion_at = time.monotonic()
        self.pending = {"joint": plan.joint, "target_deg": plan.target_deg}
        self.last_command = {
            "joint": plan.joint, "from_deg": plan.current_deg,
            "to_deg": plan.target_deg, "sent_at": time.time(),
            "status": "sent_not_verified",
        }
        return {"message": "已发送运动指令，待反馈验证", **self.last_command}

    def step(self, joint, delta):
        with self.lock:
            status = self._preflight(joint)
            return self._send_move(plan_single_step(status, joint, delta))

    def home_save(self):
        with self.lock:
            status = self.status()
            if self.pending is not None:
                raise UnsafeMotion("上一次动作尚未确认到位，不能把运动中姿态保存为复位点")
            degrees = checked_home_pose(status)
            return save_pose(degrees)

    def _home_targets(self, kind):
        if kind == "factory":
            return FACTORY_DEGREES
        if kind != "saved":
            raise UnsafeMotion("未知复位姿态类型")
        saved = load_pose()
        if saved is None:
            raise UnsafeMotion("还没有保存自定义复位姿态")
        return saved["degrees"]

    def home_info(self, kind):
        with self.lock:
            if kind == "saved" and load_pose() is None:
                return {"available": False, "kind": kind, "rows": []}
            targets = self._home_targets(kind)
            data = {"available": True, "kind": kind, "rows": []}
            if self.arm is None:
                for j, value in targets.items():
                    data["rows"].append({"joint": j, "label": JOINTS[j][0], "target_deg": round(value, 2)})
                return data
            current = current_degrees(self.status())
            for j, target in targets.items():
                data["rows"].append({
                    "joint": j, "label": JOINTS[j][0],
                    "current_deg": round(current[j], 2),
                    "target_deg": round(target, 2),
                    "difference_deg": round(target - current[j], 2),
                })
            return data

    def home_step(self, kind, joint):
        with self.lock:
            status = self._preflight(joint)
            target = self._home_targets(kind)
            return self._send_move(plan_home_step(status, joint, target))
