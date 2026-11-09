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
                "auto_execute": False,
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
                ball_board_mm,image_size=image_size,corners_px=corners_px)
            self.plan=plan
            self.session_id=uuid.uuid4().hex
            self.next_index=0
            self.cancelled=False
            return self.state(arm_service)

    def cancel(self, arm_service):
        with self.lock, arm_service.lock:
            self.cancelled = True
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
            if declaration != "已目视确认本步的运动空间与位置":
                raise PickExecutionError("每步需要先核查当前运动方向与目标")
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
                if abs(fb["tit"]-stage["t"])>math.radians(12) or abs(fb["r"]-stage["r"])>math.radians(12):
                    raise PickExecutionError("末端Pitch/Roll与示教姿态差超过12°，禁止突然调整姿态")
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
