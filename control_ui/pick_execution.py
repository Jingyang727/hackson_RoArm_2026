"""Opt-in operator-supervised pick steps (one command per physical confirmation).

No unattended autoplay. Every step requires a fresh user click, a live unlocked
arm and verified success of the prior command. Cancellation only blocks future
commands; no software stop is represented as a physical emergency stop.
"""
import math
import os
import threading
import time
import uuid

from .pick_plan import build_from_saved_files, PickPlanningError


class PickExecutionError(ValueError):
    pass


class PickExecutor:
    def __init__(self):
        self.lock = threading.RLock()
        self.plan = None
        self.session_id = None
        self.next_index = 0
        self.cancelled = False
        self.rsi_batch_approved = False
        self.rsi_batch_until = 0.0

    def state(self, arm_service=None):
        with self.lock:
            progress = {
                "session_id": self.session_id,
                "has_session": self.plan is not None,
                "next_index": self.next_index,
                "total": len(self.plan["waypoints"]) if self.plan else 0,
                "next_step": None,
                "cancelled": self.cancelled,
                "hardware_execution_enabled": os.getenv("EVOARM_ENABLE_PICK", "0") == "1",
                "auto_execute": bool(
                    self.rsi_batch_approved and
                    time.monotonic()<self.rsi_batch_until and not self.cancelled
                ),
            }
            if self.plan and self.next_index < len(self.plan["waypoints"]):
                progress["next_step"] = self.plan["waypoints"][self.next_index]
            if arm_service is not None:
                h = arm_service.health()
                progress["arm_pending"] = h["pending"]
                progress["arm_fault"] = h["fault"]
                progress["arm_armed"] = h["armed"]
                progress["last_command_status"] = h["last_command"]
            if self.plan and self.next_index == len(self.plan["waypoints"]):
                last = progress.get("last_command_status") or {}
                progress["completed"] = last.get("status") == "feedback_confirmed"
            return progress

    def diagnose(self, arm_service):
        """Read-only first/next waypoint preflight; never sends a motor command."""
        with self.lock, arm_service.lock:
            if self.plan is None or self.cancelled or self.next_index >= len(self.plan["waypoints"]):
                raise PickExecutionError("没有待检查的抓取会话；请先定位黄球并创建会话")
            h = arm_service.health()
            if not h["connected"] or h["pending"] or h["fault"] or h.get("teach_active"):
                raise PickExecutionError("机械臂未连接、正在运动或存在故障；不能取得稳定的姿态诊断")
            fb = arm_service.status()
            stage = self.plan["waypoints"][self.next_index]
            torque = {k: fb.get("torswitch" + k) for k in "BSETRG"}
            result = {
                "read_only": True,
                "stage": stage["stage"],
                "step_index": self.next_index,
                "torque_locked": all(v == 1 for v in torque.values()),
                "torque_flags": torque,
                "last_command_status": (arm_service.last_command or {}).get("status"),
                "checks": [],
            }
            if stage["kind"] != "cartesian":
                result["checks"].append("下一步是夹爪动作，无需比较末端 Pitch/Roll")
                result["ready"] = result["torque_locked"]
                return result
            fields = ("x", "y", "z", "tit", "r")
            if any(type(fb.get(k)) not in (int, float) or not math.isfinite(fb[k]) for k in fields):
                raise PickExecutionError("T=1051 缺少有效XYZ/Pitch/Roll反馈")
            cur = [fb[k] for k in "xyz"]
            xyz = stage["xyz_mm"]
            dp = math.degrees(fb["tit"] - stage["t"])
            # Roll is periodic: e.g. +179° and -179° differ by only 2°.
            dr = math.degrees(math.atan2(math.sin(fb["r"] - stage["r"]),
                                          math.cos(fb["r"] - stage["r"])))
            dx = math.dist(cur[:2], xyz[:2])
            dz = abs(cur[2] - xyz[2])
            zhigh = self.plan["waypoints"][0]["xyz_mm"][2]
            issues = []
            if not result["torque_locked"]:
                issues.append("六关节扭矩并未全部锁定")
            if abs(dp) > 12 or abs(dr) > 12:
                issues.append("末端姿态超过当前单步12°门槛：请在无遮挡高位重新核对姿态；不能靠增大扭矩解决")
            if dx > 42 or dz > 25:
                issues.append("当前XYZ与目标单步距离过大；需确认安全的高位起点")
            if dx > 3 and (cur[2] < zhigh - 8 or xyz[2] < zhigh - 8):
                issues.append("当前或目标Z不满足水平移动所需的示教安全高度")
            if self.next_index == 0 and cur[2] < zhigh - 8:
                issues.append("初始末端低于示教安全高度")
            result.update({
                "current_xyz_mm": [round(v, 2) for v in cur],
                "target_xyz_mm": xyz,
                "current_pitch_deg": round(math.degrees(fb["tit"]), 2),
                "taught_pitch_deg": round(math.degrees(stage["t"]), 2),
                "pitch_delta_deg": round(dp, 2),
                "current_roll_deg": round(math.degrees(fb["r"]), 2),
                "taught_roll_deg": round(math.degrees(stage["r"]), 2),
                "roll_delta_deg": round(dr, 2),
                "xy_delta_mm": round(dx, 2),
                "z_delta_mm": round(dz, 2),
                "travel_z_mm": zhigh,
                "issues": issues,
                "ready": not issues,
            })
            return result

    def start(self, ball_board_mm, image_size, corners_px, arm_service, declaration):
        with self.lock, arm_service.lock:
            if os.getenv("EVOARM_ENABLE_PICK", "0") != "1":
                raise PickExecutionError("逐段真实执行默认关闭：请显式设置 EVOARM_ENABLE_PICK=1")
            if declaration != "已核对标定、示教参数、桌面与工作空间":
                raise PickExecutionError("启动前必须逐项核实标定结果、姿态和运动范围")
            h=arm_service.health()
            if not h["connected"] or not h["motion_enabled"] or not h["armed"]:
                raise PickExecutionError("请先连接并在主控制台解锁机械臂运动权限")
            if set(h["allowed_joints"]) != {1,2,3,4,5,6}:
                raise PickExecutionError("逐段抓放需要全部六关节得到明确授权")
            if h["pending"] or h["fault"] or h.get("teach_active"):
                raise PickExecutionError("机械臂还有待完成动作、异常或处于扭矩释放状态")
            plan=build_from_saved_files(
                ball_board_mm,image_size=image_size,corners_px=corners_px,
                execution_intent=True)
            self.plan=plan
            self.session_id=uuid.uuid4().hex
            self.next_index=0
            self.cancelled=False
            self.rsi_batch_approved=False
            self.rsi_batch_until=0.0
            return self.state(arm_service)

    def start_rsi_grasp(self, trial_id, arm_service, declaration):
        """Start first v2 Gemini grasp/lift from a *recent* real trial.

        This uses the SAME one-step-at-a-time verified PickExecutor.step
        implementation. Never automatically jogs, moves, resets or clears
        faults. It does not use the old USB camera pixel calibration.
        """
        import json
        from pathlib import Path
        from evoarm_rsi.grasp_candidate import build_grasp_candidate

        with self.lock, arm_service.lock:
            if os.getenv("EVOARM_ENABLE_PICK", "0")!="1" or os.getenv(
                    "EVOARM_ENABLE_RSI_GRASP", "0")!="1":
                raise PickExecutionError(
                    "RSI v2 hardware grasp requires explicit ENABLE_PICK and ENABLE_RSI_GRASP"
                )
            if declaration!="已检查当前Gemini黄球位置、夹爪高度和运动区域":
                raise PickExecutionError("operator confirmation of CURRENT ball/tool is required")
            if (not isinstance(trial_id,str) or len(trial_id)!=32 or
                    any(ch not in "0123456789abcdef" for ch in trial_id)):
                raise PickExecutionError("invalid real Gemini trial ID")
            home=Path.home()
            trial=home/".evoarm"/"rsi"/"experiments"/trial_id
            analysis=home/".evoarm"/"rsi"/"analysis"/trial_id
            source=trial/"manifest.json"
            qual=analysis/"ball_qualification.json"
            poses=home/".evoarm"/"pick_poses_v2.json"
            if any(not p.is_file() for p in (source,qual,poses)):
                raise PickExecutionError("missing fresh real Gemini qualification or v2 poses")
            # A stale image is not proof the ball remains in place. Require
            # a new trial and its qualification from the last 5 minutes.
            now=time.time()
            if any(not 0<=now-p.stat().st_mtime<=300 for p in (source,qual)):
                raise PickExecutionError("camera evidence is older than 5 minutes; capture new ball")
            manifest=json.loads(source.read_text(encoding="utf-8"))
            if manifest.get("simulated") is not False or manifest.get("trial_id")!=trial_id:
                raise PickExecutionError("source is not the matching real trial")
            q=json.loads(qual.read_text(encoding="utf-8"))
            if q.get("trial_id")!=trial_id:
                raise PickExecutionError("qualified ball does not match trial")
            plan=build_grasp_candidate(
                q,json.loads(poses.read_text(encoding="utf-8"))
            )
            h=arm_service.health()
            if (not h["connected"] or not h["motion_enabled"] or
                    not h["armed"] or h["pending"] or h["fault"] or
                    h.get("teach_active")):
                raise PickExecutionError("controller needs manual unlock, no fault and all torque enabled")
            if set(h["allowed_joints"])!={1,2,3,4,5,6}:
                raise PickExecutionError("all six authorized joints required")
            fb=arm_service.status()
            if any(fb.get("torswitch"+c)!=1 for c in "BSETRG"):
                raise PickExecutionError("servo torque feedback must be locked")
            from .pick_plan import _num
            xyz=[_num(fb.get(k),k) for k in "xyz"]
            hover=plan["measured_hover_xyz_mm"]
            if math.dist(xyz[:2],hover[:2])>20 or abs(xyz[2]-hover[2])>10:
                raise PickExecutionError(
                    "arm must be manually positioned at taught safe Hover "
                    "before the first complete grasp attempt; do not auto move from Home"
                )
            # The operator consent is for the scene/plan. Each actual
            # step still requires the existing explicit step declaration.
            self.plan=plan
            self.session_id=uuid.uuid4().hex
            self.next_index=0
            self.cancelled=False
            self.rsi_batch_approved=(
                os.getenv("EVOARM_ENABLE_RSI_BATCH","0")=="1"
            )
            self.rsi_batch_until=(
                time.monotonic()+180 if self.rsi_batch_approved else 0.0
            )
            progress=self.state(arm_service)
            progress["rsi_grasp_trial_id"]=trial_id
            progress["grasp_and_lift_only"]=True
            return progress

    def cancel(self, arm_service):
        with self.lock, arm_service.lock:
            self.cancelled = True
            self.rsi_batch_approved=False
            self.rsi_batch_until=0.0
            arm_service.disarm()
            return {"cancelled":True,"notice":"已禁止下一条动作。已下发的T104/T106不一定会停止；若有碰撞风险立即切断动力电源"}

    def _check_prior(self, arm_service):
        if self.next_index == 0:
            return
        last=arm_service.last_command or {}
        if last.get("pick_session_id") != self.session_id or last.get("pick_step") != self.next_index-1:
            raise PickExecutionError("上一步实际回执不匹配，禁止继续")
        if last.get("status") != "feedback_confirmed":
            raise PickExecutionError(
                "上一动作尚未收到精确到位反馈；有偏差/异常时请人工检查，不能自动叠加指令")

    def step(self, token, step_index, declaration, arm_service):
        with self.lock, arm_service.lock:
            if self.cancelled or not self.plan or token != self.session_id:
                raise PickExecutionError("抓取会话无效或已取消")
            if step_index != self.next_index:
                raise PickExecutionError("步骤序号不匹配，禁止重复发送或跳步")
            if step_index >= len(self.plan["waypoints"]):
                raise PickExecutionError("步骤已经完成，禁止重复执行")
            if os.getenv("EVOARM_ENABLE_PICK", "0") != "1":
                raise PickExecutionError("真实执行权限未打开")
            is_batch=(
                declaration=="已批准一次RSI抓取试次自动分段执行" and
                self.rsi_batch_approved and not self.cancelled and
                time.monotonic()<self.rsi_batch_until and
                self.plan.get("kind")=="rsi_v2_gemini_grasp_and_lift_candidate" and
                os.getenv("EVOARM_ENABLE_RSI_BATCH","0")=="1"
            )
            if not is_batch and declaration != "已目视确认本步的运动空间与位置":
                raise PickExecutionError("每步需要人工核查，或已有一次明确授权的有限RSI试次")
            h=arm_service.health()
            if not h["connected"] or not h["motion_enabled"] or not h["armed"]:
                raise PickExecutionError("机械臂未连接或运动解锁已过期")
            if h["fault"] or h["pending"] or h.get("teach_active"):
                raise PickExecutionError("动作未完成、存在异常或扭矩已释放，禁止继续")
            if time.monotonic()-arm_service.last_motion_at < 0.8:
                raise PickExecutionError("连续指令间隔小于0.8秒，稍后再试")
            self._check_prior(arm_service)
            if set(h["allowed_joints"]) != {1,2,3,4,5,6}:
                raise PickExecutionError("六关节授权不完整")
            fb=arm_service.status()
            if any(fb.get("torswitch"+n)!=1 for n in "BSETRG"):
                raise PickExecutionError("六关节扭矩并未全部锁定")
            stage=self.plan["waypoints"][step_index]
            if stage["kind"] == "cartesian":
                current=[fb.get(k) for k in "xyz"]
                if any(type(v) not in (int,float) or not math.isfinite(v) for v in current):
                    raise PickExecutionError("没有有效的XYZ实际反馈")
                target=stage["xyz_mm"]
                if type(fb.get("tit")) not in (int,float) or type(fb.get("r")) not in (int,float):
                    raise PickExecutionError("末端姿态反馈缺失：必须使用 tit Pitch 和 r Roll")
                pitch_diff=math.degrees(fb["tit"]-stage["t"])
                roll_diff=math.degrees(math.atan2(
                    math.sin(fb["r"]-stage["r"]),
                    math.cos(fb["r"]-stage["r"])))
                if abs(pitch_diff)>12 or abs(roll_diff)>12:
                    raise PickExecutionError(
                        "末端姿态不匹配：当前Pitch "
                        f"{math.degrees(fb['tit']):.1f}° / 示教{math.degrees(stage['t']):.1f}°"
                        f"（差{pitch_diff:+.1f}°）；当前Roll "
                        f"{math.degrees(fb['r']):.1f}° / 示教{math.degrees(stage['r']):.1f}°"
                        f"（最短角差{roll_diff:+.1f}°）。单段门槛12°。"
                        "请点「姿态诊断」检查XYZ与扭矩，确认安全后手动微调或重新示教，"
                        "不要提高扭矩或直接放宽角度限制。")
                delta_xy=math.dist(current[:2],target[:2])
                delta_z=abs(current[2]-target[2])
                if delta_xy>42 or delta_z>25:
                    raise PickExecutionError(
                        f"单次XYZ目标变化过大（水平{delta_xy:.1f}mm，垂直{delta_z:.1f}mm）。"
                        "首次执行请先人工移动到高位安全起始位置附近")
                zhigh=self.plan["waypoints"][0]["xyz_mm"][2]
                if delta_xy>3 and (current[2]<zhigh-8 or target[2]<zhigh-8):
                    raise PickExecutionError("横向移动仅允许在示教的高位安全Z进行")
                if self.next_index==0 and current[2]<zhigh-8:
                    raise PickExecutionError("请先人工将末端移动到安全高位再开始，不自动提升任意初始姿态")
                command={
                    "T":104,
                    "x":target[0],"y":target[1],"z":target[2],
                    "t":stage["t"],"r":stage["r"],"g":stage["g"],
                    "spd":min(stage["spd"],0.12)
                }
                pending={"kind":"cartesian","xyz_start":current,
                         "xyz_target":target,"since":time.monotonic(),"elapsed_s":0.0,
                         "stable_intervals":0}
            else:
                g=fb.get("g")
                if type(g) not in (int,float) or not math.isfinite(g):
                    raise PickExecutionError("夹爪反馈缺失")
                from .safety import JOINTS
                if not (math.radians(63)<=g<=math.radians(182)):
                    raise PickExecutionError("夹爪实际角度偏离官方范围，请人工检查")
                if abs(g-stage["g"])<=math.radians(2):
                    # No-op: opening a gripper already at the requested angle
                    # must not create a "pending forever" or invent motor motion.
                    arm_service.last_command={
                        "status":"feedback_confirmed","pick_session_id":self.session_id,
                        "pick_step":self.next_index,"stage":stage["stage"],
                        "command":None,"confirmed_at":time.time(),
                        "message":"夹爪已经处于目标角度，跳过重复电机指令",
                    }
                    self.next_index+=1
                    return {"sent":False,"verified":True,"step":step_index,
                            "stage":stage["stage"],"notice":"夹爪已到位，未重复下发电机命令"}
                command={"T":106,"cmd":stage["g"],"spd":stage["spd"],"acc":stage["acc"]}
                pending={"joint":6,"target_deg":math.degrees(stage["g"]),
                         "start_deg":math.degrees(g),"actual_deg":math.degrees(g),
                         "since":time.monotonic(),"elapsed_s":0.0,"stable_intervals":0}
            # Exactly one command is sent. The ordinary /api/status loop verifies
            # arrival and clears pending or enters a latched fault; never autoplay.
            arm_service.arm.send(command)
            arm_service.last_motion_at=time.monotonic()
            arm_service.pending=pending
            arm_service.last_command={
                "status":"sent_not_verified","pick_session_id":self.session_id,
                "pick_step":self.next_index,"stage":stage["stage"],
                "command":command,"sent_at":time.time(),
            }
            self.next_index+=1
            return {
                "sent":True,"verified":False,"step":step_index,
                "stage":stage["stage"],"command":command,
                "notice":"已下发单条指令，等待真实反馈。确认到位后才能执行下一步",
            }


executor=PickExecutor()
