"""Mac local robot service: one serial owner, limited manual control and guided home."""
import os
import threading
import time
import math
import json
from pathlib import Path

from serial.tools import list_ports
from evoarm.serial_arm import RoArmSerial
from .home_pose import save_pose, load_pose
from .settling import observe_stability
from .safety import (
    JOINTS, JOG_STEPS, FACTORY_DEGREES, UnsafeMotion,
    current_degrees, checked_home_pose, plan_single_step, plan_gripper_step, plan_home_step,
)

SESSION_SECONDS = 300
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
CART_CONFIRM_TIMEOUT_S = 65.0
CART_ARRIVAL_TOLERANCE_MM = 6.0
CART_SETTLED_OFFSET_MAX_MM = 10.0
RECOVERY_SAMPLE_GAP_S = 0.75
# Send T=210 once. Retry only read-only T=105 status verification.
TORQUE_VERIFY_ATTEMPTS = 3
TORQUE_VERIFY_TIMEOUT_S = 3.5
TORQUE_VERIFY_PAUSE_S = 0.35
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
        # Teach mode never auto-enables. Keep a fail-closed flag after releasing torque.
        self.teach_active = False
        self.last_torque_status = None
        self.last_torque_result = None

    @property
    def teach_enabled(self):
        return os.getenv("EVOARM_ENABLE_TEACH", "0") == "1"

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

    @staticmethod
    def _last_port_path():
        return Path.home() / ".evoarm" / "last_serial_port.json"

    def auto_connect(self):
        """One conservative, idempotent attempt shared by all browser pages.

        Never toggle a connected port or auto-retry motion/faulted hardware.
        Auto connect requests serial STATUS only. Motion unlock is separate.
        """
        with self.lock:
            if self.arm is not None:
                return {"connected": True, "port": self.port, "reused": True}
            available = {entry["device"] for entry in self.ports()}
            preferred = os.getenv("EVOARM_SERIAL_PORT", "").strip()
            if not preferred:
                try:
                    data = json.loads(self._last_port_path().read_text(encoding="utf-8"))
                    preferred = data.get("port", "") if isinstance(data, dict) else ""
                except (OSError, ValueError):
                    pass
            candidates = sorted(
                port for port in available
                if any(token in port.lower() for token in ("usbserial", "slab_usbtouart", "usbmodem"))
            )
            # Never connect to an arbitrary unrelated serial port.
            if preferred in available:
                selected = preferred
            elif len(candidates) == 1:
                selected = candidates[0]
            else:
                return {
                    "connected": False,
                    "port": None,
                    "reason": "无法唯一确定 RoArm USB 串口；请展开连接设置并手动选择一次",
                }
            try:
                self.connect(selected)
            except Exception as exc:
                return {"connected": False, "port": selected,
                        "reason": f"自动连接失败：{exc}。请检查 USB 或供电，必要时手动重连"}
            return {"connected": True, "port": selected, "reused": False}

    def connect(self, port):
        with self.lock:
            if self.arm is not None and self.port == port:
                # Opening the same port again would reset the ESP32 and could
                # disrupt an in-flight motion or a released-torque teach pose.
                return self.arm.status()
            if self.arm is not None and (self.pending or self.teach_active):
                raise UnsafeMotion("机械臂正在运动或处于释放扭矩示教中，禁止切换串口")
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
            flags = [result.get("torswitch" + k) for k in "BSETRG"]
            self.last_torque_status = flags
            self.last_torque_result = None
            self.teach_active = not all(v == 1 for v in flags)
            # Save only an already verified working device; never persist an
            # unverified guessed port and never store private serial status.
            try:
                path = self._last_port_path()
                path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                tmp = path.with_suffix(".tmp")
                tmp.write_text(json.dumps({"port": port}) + "\n", encoding="utf-8")
                os.chmod(tmp, 0o600)
                os.replace(tmp, path)
            except OSError:
                pass  # Read-only home directories should not break a connection.
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
                elif pending and pending.get("kind") == "cartesian":
                    result = self.arm.status(timeout=30)
                else:
                    result = self.arm.status()
            except Exception as exc:
                self._latch_fault(
                    "feedback_timeout",
                    "状态查询失败：可能是串口超时、固件忙或供电异常；已锁定新的动作",
                    detail=str(exc),
                )
                raise
            self.last_torque_status = [result.get("torswitch" + k) for k in "BSETRG"]
            # A T=210 response can time out after the torque state has changed.
            # The *next genuine T=1051 reply* is authoritative: reconcile the
            # fail-closed teach flag only when all six hardware flags are valid.
            # This does not send T=210 or cause any motor motion.
            if all(v in (0, 1) for v in self.last_torque_status):
                self.teach_active = not all(v == 1 for v in self.last_torque_status)
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
            if pending.get("kind") == "cartesian":
                # Official T104 is interpolated and can block firmware processing.
                # Verify reported end-effector XYZ, not just joint-angle changes.
                def xyz(k):
                    v = result.get(k)
                    if type(v) not in (int, float) or not math.isfinite(v):
                        raise UnsafeMotion("末端XYZ反馈缺失或无效")
                    return float(v)
                try:
                    current_xyz = [xyz(k) for k in "xyz"]
                except UnsafeMotion as exc:
                    self._latch_fault("invalid_cartesian_feedback",str(exc))
                    raise
                target = pending["xyz_target"]
                residual = math.dist(current_xyz,target)
                progress = math.dist(current_xyz,pending["xyz_start"])
                pending["actual_xyz_mm"] = [round(v,2) for v in current_xyz]
                pending["position_error_mm"] = round(residual,2)
                pending["progress_mm"] = round(progress,2)
                if residual <= CART_ARRIVAL_TOLERANCE_MM and progress >= 0.8:
                    self.pending = None
                    if self.last_command:
                        self.last_command["status"] = "feedback_confirmed"
                        self.last_command["position_error_mm"] = round(residual,2)
                        self.last_command["confirmed_at"] = time.time()
                elif stable:
                    if residual <= CART_SETTLED_OFFSET_MAX_MM and progress >= 0.8:
                        self.pending = None
                        self.armed_until = 0
                        if self.last_command:
                            self.last_command["status"] = "settled_with_offset"
                            self.last_command["position_error_mm"] = round(residual,2)
                            self.last_command["message"] = (
                                f"末端XYZ已停稳但偏离目标{residual:.1f}mm，请核查姿态并重新授权")
                    else:
                        self._latch_fault("cartesian_stalled",
                            f"XYZ反馈已稳定但偏离目标{residual:.1f}mm，禁止继续运动；请检查实际位置",
                            expected_xyz_mm=target, actual_xyz_mm=current_xyz,
                            error_mm=round(residual,2))
                elif elapsed >= CART_CONFIRM_TIMEOUT_S:
                    self._latch_fault("cartesian_unverified",
                        f"XYZ反馈等待超过{CART_CONFIRM_TIMEOUT_S:g}秒，目标仍未确认；请检查供电、卡滞与串口",
                        expected_xyz_mm=target, actual_xyz_mm=current_xyz,
                        error_mm=round(residual,2))
                return result
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
                "teach_enabled": self.teach_enabled,
                "teach_active": self.teach_active,
                "torque_status": self.last_torque_status,
                "torque_last_result": self.last_torque_result,
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
        # API name kept for backward compatibility; unlock lasts 5 minutes (manual, never auto-renewed).
        with self.lock:
            self.armed_until = 0
            if not self.motion_enabled:
                raise UnsafeMotion("只读模式：需要显式设置 EVOARM_ENABLE_MOTION=1")
            if self.arm is None:
                raise UnsafeMotion("请先连接机械臂")
            if self.teach_active:
                raise UnsafeMotion("示教释放扭矩状态下，禁止解锁运动。请先支撑机械臂并确认恢复扭矩")
            if declaration != "已检查设备并清空运动区域":
                raise UnsafeMotion("请先确认机械臂、固定支架和工作区域安全")
            self.status()
            if self.fault:
                raise UnsafeMotion("动作异常未解除，请先检查并按“异常复核”操作")
            self.armed_until = time.monotonic() + SESSION_SECONDS
            return {"armed": True, "expires_in_seconds": SESSION_SECONDS}

    def set_torque(self, enabled, declaration):
        """Explicit T=210 all-servo transition; never retry the actuation itself.

        T=1051 can time out immediately after T=210 even if the servo switch
        succeeded. Retrying status reads is safe; replaying T=210 is not.
        """
        with self.lock:
            if type(enabled) is not bool:
                raise UnsafeMotion("扭矩模式无效")
            if not self.teach_enabled:
                raise UnsafeMotion("示教扭矩控制默认关闭，须显式设置 EVOARM_ENABLE_TEACH=1")
            if self.arm is None:
                raise UnsafeMotion("机械臂未连接")
            if self.pending is not None:
                raise UnsafeMotion("动作未确认完成，不能切换扭矩")
            if self.fault and not enabled:
                raise UnsafeMotion("有运动异常时不能释放扭矩")
            required = ("已托住机械臂并确认重新锁定" if enabled
                        else "已托住机械臂并确认释放全部关节")
            if declaration != required:
                raise UnsafeMotion("请先确认托住机械臂、避免重力下垂与突然转动")
            before = self.arm.status()
            flags = [before.get("torswitch" + k) for k in "BSETRG"]
            if any(v not in (0, 1) for v in flags):
                raise UnsafeMotion("扭矩反馈不完整，拒绝切换")
            if not enabled and any(v != 1 for v in flags):
                raise UnsafeMotion("一个或多个关节已释放；请读取实际状态，勿重复释放")
            self.last_torque_status = flags
            self.armed_until = 0
            target = 1 if enabled else 0
            if all(v == target for v in flags):
                # An explicit re-lock may arrive after the earlier response
                # timed out, while the physical servos are already locked.
                self.teach_active = not enabled
                result = {
                    "verified": True, "torque_enabled": enabled,
                    "torque_state": flags, "command_sent": False,
                    "verification_attempts": 0,
                    "message": ("六关节已经全部锁定，无需重复发送扭矩指令"
                                if enabled else "六关节已经全部释放，无需重复发送指令"),
                }
                self.last_torque_result = result
                return result
            self.teach_active = True  # fail closed before sending
            try:
                self.arm.send({"T": 210, "cmd": target})
            except Exception as exc:
                # Transport may fail after some bytes were written. We cannot
                # assume the command was not received. Never auto-reissue it.
                self.last_torque_status = None
                result = {
                    "verified": False, "torque_enabled": None,
                    "torque_state": None, "command_sent": False,
                    "verification_attempts": 0,
                    "message": (
                        f"指令发送异常：{exc}；可能已生效。保持托住机械臂，"
                        "请点击“读取实际扭矩状态”；当前禁止运动，不会自动重发"
                    ),
                }
                self.last_torque_result = result
                return result
            current = None
            last_error = None
            attempts = 0
            for attempt in range(TORQUE_VERIFY_ATTEMPTS):
                if attempt:
                    time.sleep(TORQUE_VERIFY_PAUSE_S)
                attempts = attempt + 1
                try:
                    observed = self.arm.status(timeout=TORQUE_VERIFY_TIMEOUT_S)
                except Exception as exc:
                    last_error = str(exc)
                    continue
                values = [observed.get("torswitch" + k) for k in "BSETRG"]
                if any(v not in (0, 1) for v in values):
                    last_error = "扭矩反馈不完整"
                    continue
                current = values
                self.last_torque_status = values
                if all(v == target for v in values):
                    self.teach_active = not enabled
                    result = {
                        "verified": True, "torque_enabled": enabled,
                        "torque_state": values, "command_sent": True,
                        "verification_attempts": attempts,
                        "message": ("六关节扭矩已锁定；运动仍需重新手动授权"
                                    if enabled else
                                    "六关节扭矩已释放；请持续托住大臂，避免下垂"),
                    }
                    self.last_torque_result = result
                    return result
                last_error = "尚未全部达到目标扭矩状态"
            # Do not report failure of actuation: only verification failed.
            # Never expose stale pre-command feedback as the latest state.
            self.teach_active = True
            if current is None:
                self.last_torque_status = None
            result = {
                "verified": False, "torque_enabled": None,
                "torque_state": current, "command_sent": True,
                "verification_attempts": attempts,
                "message": (
                    f"T=210仅发送一次，但{attempts}次状态读取仍未确认"
                    + (f"（最近反馈错误：{last_error}）" if last_error else "")
                    + "。指令可能已经生效！请托住机械臂，点击“读取实际扭矩状态”。"
                    "当前禁止新的运动指令，不会自动重发扭矩指令"
                ),
            }
            self.last_torque_result = result
            return result

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
            # Fail closed on EVERY recovery attempt, including a rejected one.
            # A fault may be raised while a prior 120s authorization remains.
            self.armed_until = 0
            if not self.arm:
                raise UnsafeMotion("请先检查串口连接")
            if declaration != "已检查机械臂静止且运动区域安全":
                raise UnsafeMotion("需要确认机械臂已静止且无障碍")
            try:
                first_status = self.arm.status()
                first = current_degrees(first_status)
                time.sleep(RECOVERY_SAMPLE_GAP_S)
                second_status = self.arm.status()
                second = current_degrees(second_status)
                for sample in (first_status, second_status):
                    if any(sample.get("torswitch" + key) != 1 for key in "BSETRG"):
                        raise UnsafeMotion("恢复失败：六关节扭矩未全部锁定，不能继续运动")
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
        if self.teach_active:
            raise UnsafeMotion("当前为释放扭矩的示教模式，禁止下发运动指令")
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
        if any(status.get("torswitch" + k) != 1 for k in "BSETRG"):
            raise UnsafeMotion("全部关节扭矩未锁定，禁止运动")
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

    def teach_move_to(self, joint, target_deg):
        """One explicitly clicked bounded joint move; never streams slider events."""
        from .safety import plan_to_angle, current_degrees
        if type(joint) is not int or type(target_deg) not in (int, float):
            raise UnsafeMotion("关节或目标角度格式错误")
        if not math.isfinite(target_deg):
            raise UnsafeMotion("目标角度必须是有限值")
        with self.lock:
            status = self._preflight(joint)
            current = current_degrees(status)[joint]
            # Absolute target via slider, but bounded displacement per command.
            # No automatic multi-command chaining.
            max_delta = 30.0
            difference = target_deg - current
            if abs(difference) > max_delta + 1e-6:
                raise UnsafeMotion(
                    f"当前{current:.1f}° → 目标{target_deg:.1f}°差{difference:+.1f}°；"
                    "单次最多30°。请先到中间角度，核对反馈后再继续")
            if abs(difference) < 0.5:
                raise UnsafeMotion("已经接近目标角度，无需重复发送")
            return self._send_move(plan_to_angle(status, joint, target_deg))

    def teach_move_relative(self, joint, delta_deg):
        """One-click relative jog based on latest hardware feedback, not slider state."""
        from .safety import plan_to_angle, current_degrees
        if type(joint) is not int or joint not in JOINTS:
            raise UnsafeMotion("关节编号无效")
        if type(delta_deg) not in (int, float) or not math.isfinite(delta_deg):
            raise UnsafeMotion("调整角度无效")
        if delta_deg not in (-10, -5, 5, 10):
            raise UnsafeMotion("快速调整仅支持 ±5° 或 ±10°")
        with self.lock:
            status = self._preflight(joint)
            current = current_degrees(status)[joint]
            return self._send_move(plan_to_angle(status, joint, current + delta_deg))

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
            if self.teach_active:
                raise UnsafeMotion("示教释放扭矩时禁止执行官方复位")
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
            if self.teach_active:
                raise UnsafeMotion("释放扭矩中不能保存正常动作待机位置")
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
