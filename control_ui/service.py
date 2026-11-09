"""Mac local robot service: one serial owner, limited manual control and guided home."""
import os
import threading
import time

from serial.tools import list_ports
from evoarm.serial_arm import RoArmSerial
from .home_pose import save_pose, load_pose
from .settling import observe_stability
from .safety import (
    JOINTS, JOG_STEPS, FACTORY_DEGREES, UnsafeMotion,
    current_degrees, checked_home_pose, plan_single_step, plan_gripper_step, plan_home_step,
)

SESSION_SECONDS = 120
MIN_GAP_SECONDS = 0.8
ARRIVAL_TOLERANCE = 2.0
# These are *classification heuristics*, not new servo commands or safety
# guarantees. A settled offset must be visible and the operator re-authorizes.
FACTORY_SETTLED_OFFSET_MAX_DEG = 5.0
JOG_SETTLED_OFFSET_MAX_DEG = 3.5
MIN_JOG_PROGRESS_DEG = 0.4
# A firmware command is not automatically treated as successful.
# If feedback never reaches target, surface a latched fault instead of leaving
# an invisible, permanent 'pending' state. New motion remains blocked.
JOG_CONFIRM_TIMEOUT_S = 10.0
GRIPPER_CONFIRM_TIMEOUT_S = 12.0
FACTORY_CONFIRM_TIMEOUT_S = 45.0
RECOVERY_SAMPLE_GAP_S = 0.75
STABLE_POSITION_TOLERANCE_DEG = 0.6


class ArmService:
    def __init__(self):
        self.lock = threading.RLock()
        self.arm = None
        self.port = None
        self.armed_until = 0.0
        self.last_motion_at = 0.0
        self.pending = None
        self.last_command = None
        self.fault = None

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
                # ESP32 can still be booting after its initial USB reset.
                result = arm.status(timeout=20)
            except Exception:
                arm.__exit__(None, None, None)
                raise
            self.arm, self.port = arm, port
            self.armed_until = 0
            self.pending = self.last_command = None  # Keep a latched fault across reconnects.
            return result

    def disconnect(self):
        with self.lock:
            self.armed_until = 0
            if self.arm is not None:
                self.arm.__exit__(None, None, None)
            self.arm = self.port = None
            self.pending = None
            # A serial reconnect must not silently erase an unverified motion fault.

    def _latch_fault(self, code, description, **details):
        self.fault = {"code": code, "message": description, **details}
        self.armed_until = 0
        self.pending = None
        if self.last_command:
            self.last_command["status"] = "unverified_fault"
        return self.fault

    def status(self):
        with self.lock:
            if self.arm is None:
                raise ValueError("尚未连接机械臂")
            pending = self.pending
            try:
                # Firmware T=100 can block other commands during reset.
                if pending and pending.get("kind") == "factory":
                    result = self.arm.status(timeout=20)
                else:
                    result = self.arm.status()
            except Exception as exc:
                self._latch_fault(
                    "feedback_timeout",
                    "状态查询失败：可能是串口超时、固件忙或供电异常；已锁定新的动作",
                    detail=str(exc),
                )
                raise
            if pending is None:
                return result
            try:
                angles = current_degrees(result)
            except UnsafeMotion as exc:
                self._latch_fault("invalid_feedback", "状态反馈不完整，已锁定新的动作", detail=str(exc))
                raise

            now = time.monotonic()
            elapsed = now - pending["since"]
            pending["elapsed_s"] = round(elapsed, 1)
            stable = observe_stability(pending, angles, now)
            is_factory = pending.get("kind") == "factory"
            if is_factory:
                differences = {
                    j: round(angles[j] - FACTORY_DEGREES[j], 2)
                    for j in FACTORY_DEGREES
                }
                worst_joint = max(differences, key=lambda j: abs(differences[j]))
                residual = abs(differences[worst_joint])
                progress = max(
                    abs(angles[j] - pending["initial_angles"][j])
                    for j in FACTORY_DEGREES
                )
                pending["max_error_deg"] = round(residual, 2)
                pending["worst_joint"] = worst_joint
                pending["joint_errors_deg"] = differences
                timeout_s = FACTORY_CONFIRM_TIMEOUT_S
                soft_limit = FACTORY_SETTLED_OFFSET_MAX_DEG
                label = "官方复位"
            else:
                joint = pending["joint"]
                actual = angles[joint]
                residual = abs(actual - pending["target_deg"])
                progress = abs(actual - pending["start_deg"])
                pending["actual_deg"] = round(actual, 2)
                pending["error_deg"] = round(residual, 2)
                timeout_s = (GRIPPER_CONFIRM_TIMEOUT_S if joint == 6 else JOG_CONFIRM_TIMEOUT_S)
                soft_limit = JOG_SETTLED_OFFSET_MAX_DEG
                label = JOINTS[joint][0]
            pending["progress_deg"] = round(progress, 2)

            # A tiny step may fall inside the 2° tolerance *before it moves*.
            # Do not claim successful arrival based on unchanged early feedback.
            enough_motion = is_factory or progress >= MIN_JOG_PROGRESS_DEG
            if residual <= ARRIVAL_TOLERANCE and enough_motion:
                self.pending = None
                if self.last_command:
                    self.last_command["status"] = "feedback_confirmed"
                    self.last_command["confirmed_at"] = time.time()
                    self.last_command["error_deg"] = round(residual, 2)
            elif stable:
                # Three stable samples mean the servo/feedback *settled*, but
                # that alone cannot prove the intended target was reached.
                if residual <= soft_limit and (is_factory or progress >= MIN_JOG_PROGRESS_DEG):
                    self.pending = None
                    self.armed_until = 0
                    if self.last_command:
                        self.last_command["status"] = "settled_with_offset"
                        self.last_command["error_deg"] = round(residual, 2)
                        self.last_command["worst_joint"] = pending.get("worst_joint")
                        self.last_command["joint_errors_deg"] = pending.get("joint_errors_deg")
                        self.last_command["settled_at"] = time.time()
                        self.last_command["message"] = (
                            f"{label}已停止变化，但与预设目标仍差{residual:.1f}°。"
                            "这是稳定但存在偏差，不等于精确到位。请目视确认姿态，再重新解锁。"
                        )
                else:
                    # A stable position far from target, or no observable motion,
                    # is a potential stall or calibration issue: fail early.
                    self._latch_fault(
                        "motion_stalled_or_offset",
                        f"{label}反馈已稳定，但未能确认到达目标（差{residual:.1f}°，"
                        f"检测到移动{progress:.1f}°）。可能是机械受阻、零位偏差或指令未执行；"
                        "请检查后再恢复",
                        expected_deg=pending.get("target_deg"),
                        actual_deg=pending.get("actual_deg"),
                        error_deg=round(residual, 2),
                        joint=pending.get("joint"),
                        joint_errors_deg=pending.get("joint_errors_deg"),
                        elapsed_s=round(elapsed, 1),
                    )
            elif elapsed >= timeout_s:
                self._latch_fault(
                    "motion_not_confirmed",
                    f"{label}超过{timeout_s:g}秒仍未反馈到达目标（误差{residual:.1f}°）。"
                    "反馈仍未稳定；请检查电源、运动空间和机械负载",
                    expected_deg=pending.get("target_deg"),
                    actual_deg=pending.get("actual_deg"),
                    error_deg=round(residual, 2),
                    joint=pending.get("joint"),
                    joint_errors_deg=pending.get("joint_errors_deg"),
                    elapsed_s=round(elapsed, 1),
                )
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
                "fault": self.fault,
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
            if self.fault:
                raise UnsafeMotion("动作异常未解除，请先检查并按“异常复核”操作")
            self.armed_until = time.monotonic() + SESSION_SECONDS
            return {"armed": True, "expires_in_seconds": SESSION_SECONDS}

    def disarm(self):
        # Intentionally does not claim to stop a command already sent.
        with self.lock:
            self.armed_until = 0
            return {"locked": True, "notice": "已锁定新的运动指令。已发出的运动不会因此立即停止；紧急情况请切断机械臂运动电源"}

    def recover_fault(self, declaration):
        """Operator-confirmed stable feedback recovery; NEVER send movement here."""
        with self.lock:
            if not self.fault:
                raise UnsafeMotion("没有需要恢复的动作异常")
            if not self.arm:
                raise UnsafeMotion("请先检查串口连接")
            if declaration != "已检查机械臂静止且运动区域安全":
                raise UnsafeMotion("需要确认机械臂已静止且无障碍")
            try:
                first = current_degrees(self.arm.status())
                time.sleep(RECOVERY_SAMPLE_GAP_S)
                second = current_degrees(self.arm.status())
            except Exception as exc:
                raise UnsafeMotion(f"恢复失败：无法连续读取有效关节状态：{exc}") from exc
            deviation = max(abs(second[j] - first[j]) for j in JOINTS)
            if deviation > STABLE_POSITION_TOLERANCE_DEG:
                raise UnsafeMotion(f"机械臂仍存在反馈变化（最大{deviation:.2f}°），不能解锁")
            previous_fault = self.fault
            self.fault = None
            self.pending = None
            self.armed_until = 0  # Reconfirm workspace before any subsequent move.
            return {"recovered": True, "previous_fault": previous_fault,
                    "message": "反馈稳定、异常已解除。需要重新点击“确认安全并解锁”"}

    def _preflight(self, joint):
        if self.arm is None or not self.motion_enabled:
            raise UnsafeMotion("机械臂未连接，或后端仍处于只读模式")
        if self.fault:
            raise UnsafeMotion("存在未解决的动作异常：" + self.fault["message"])
        if joint not in self.allowed_joints:
            raise UnsafeMotion("该关节未被当前后端授权")
        if time.monotonic() >= self.armed_until:
            self.armed_until = 0
            raise UnsafeMotion("控制授权已过期，请再次确认")
        if time.monotonic() - self.last_motion_at < MIN_GAP_SECONDS:
            raise UnsafeMotion("动作间隔太短，请稍后重试")
        status = self.status()
        if self.fault:
            raise UnsafeMotion("动作反馈异常：" + self.fault["message"])
        if self.pending is not None:
            elapsed = self.pending.get("elapsed_s", 0)
            raise UnsafeMotion(f"上一动作正在等待到位反馈（{elapsed:.1f}秒），请查看下方执行状态")
        return status

    def _send_move(self, plan):
        self.arm.send(plan.payload)
        self.last_motion_at = time.monotonic()
        self.pending = {"joint": plan.joint, "target_deg": plan.target_deg,
                        "since": time.monotonic(), "elapsed_s": 0.0,
                        "actual_deg": plan.current_deg, "start_deg": plan.current_deg,
                        "stable_intervals": 0}
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

    def gripper_step(self, delta):
        """Move gripper by a bounded angle. Uses official T=106 radians command."""
        with self.lock:
            status = self._preflight(6)
            return self._send_move(plan_gripper_step(status, delta))

    def factory_reset(self):
        """Explicit firmware T=100 reset, never a silent automatic command.

        This moves all joints, potentially along an unsafe trajectory. Requires the
        full six-joint allowlist, an active manual session, a cleared workspace
        confirmed by the operator in the UI, and no other in-flight command.
        """
        with self.lock:
            if not self.motion_enabled or self.arm is None:
                raise UnsafeMotion("机械臂未连接或运动模式未开启")
            if self.fault:
                raise UnsafeMotion("存在未解决的动作异常，不能执行官方复位")
            if set(JOINTS) - self.allowed_joints:
                raise UnsafeMotion("官方复位需要明确授权全部六个关节")
            if time.monotonic() >= self.armed_until:
                self.armed_until = 0
                raise UnsafeMotion("运动授权已过期，请重新确认")
            if time.monotonic() - self.last_motion_at < MIN_GAP_SECONDS:
                raise UnsafeMotion("动作间隔过短")
            feedback = self.status()
            if self.fault:
                raise UnsafeMotion("动作异常未解除，不能复位")
            if self.pending is not None:
                raise UnsafeMotion("上一动作尚未确认完成，不允许复位")
            initial_angles = current_degrees(feedback)  # Must be complete for reset verification.
            if any(feedback.get(key) != 1 for key in
                   ("torswitchB", "torswitchS", "torswitchE",
                    "torswitchT", "torswitchR", "torswitchG")):
                raise UnsafeMotion("一个或多个关节扭矩状态未确认，不能执行整臂复位")
            self.arm.send({"T": 100})
            self.last_motion_at = time.monotonic()
            self.pending = {"kind": "factory", "target": "official_init",
                            "since": time.monotonic(), "elapsed_s": 0.0,
                            "initial_angles": initial_angles,
                            "stable_intervals": 0}
            self.last_command = {
                "status": "factory_reset_sent_unverified",
                "command": {"T": 100},
                "sent_at": time.time(),
            }
            # Firmware may block on T=100; movement is not yet confirmed.
            return {"message": "已发送官方 T=100，等待六个关节状态确认；注意运动空间",
                    "status": "sent_not_verified"}

    def home_save(self):
        with self.lock:
            status = self.status()
            if self.fault or self.pending is not None:
                raise UnsafeMotion("动作异常或尚未到位，不能把运动中姿态保存为复位点")
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
