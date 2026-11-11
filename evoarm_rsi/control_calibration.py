"""Real RoArm responsiveness experiments: NO camera, yellow ball or model.

Purpose: measure joint actuation reliability using meaningful 10° motions,
not 1°/2° micro-jogs known to be unreliable on this actual arm.
Uses existing localhost control service as the ONLY serial owner.
An explicit RUN, existing manually unlocked lease, all six torques, and
normal per-joint firmware limits are always required.

An experiment is: measured starting angle -> +/-10° -> measured start.
Each commanded move collects actual T1051 joint + XYZ readings. Never
ignore a genuine controller fault, auto-enable torque, auto-rearm or
repeat an unverified command. Record partial failures on disk.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
import uuid

from control_ui.safety import JOINTS, current_degrees
from .real_probe import _base_url, _controller_get, _controller_absolute


def _measurement(status):
    angles=current_degrees(status)
    xyz=[]
    for name in "xyz":
        value=status.get(name)
        if type(value) not in (int,float) or not math.isfinite(value):
            raise RuntimeError("T1051 has no valid XYZ feedback")
        xyz.append(round(float(value),3))
    return {"host_monotonic_ns":time.monotonic_ns(),
            "joint_deg":{str(j):round(angles[j],4) for j in range(1,7)},
            "xyz_mm":xyz,
            "torque":[status.get("torswitch"+f) for f in "BSETRG"]}


def _readiness(base):
    raw=_controller_get(base,"/api/status")
    health=_controller_get(base,"/api/health")
    snapshot=_measurement(raw)
    blockers=[]
    for flag in ("connected","motion_enabled","armed"):
        if health.get(flag) is not True:
            blockers.append(flag)
    if health.get("fault"):
        blockers.append("existing_fault")
    if health.get("pending"):
        blockers.append("pending_motion")
    if health.get("teach_active"):
        blockers.append("torque_released")
    if snapshot["torque"]!=[1]*6:
        blockers.append("torque_not_locked")
    return health,snapshot,blockers


def _direction(current, joint, movement, preferred):
    lo,hi=JOINTS[joint][2:]
    for sign in (preferred,-preferred):
        if lo+1.0<=current+sign*movement<=hi-1.0:
            return sign
    return None


def _wait_real(base, joint, target, *, timeout_s=17.0, poll_s=.35):
    """Do not equate HTTP acknowledgement with actual joint movement."""
    deadline=time.monotonic()+timeout_s
    history=[]
    while time.monotonic()<deadline:
        raw=_controller_get(base,"/api/status")
        health=_controller_get(base,"/api/health")
        sample=_measurement(raw)
        history.append(sample)
        if health.get("fault"):
            return {"outcome":"fault","feedback":history,
                    "fault":health["fault"],"last_command":health.get("last_command")}
        if health.get("pending") is None:
            last=health.get("last_command") or {}
            actual=sample["joint_deg"][str(joint)]
            return {"outcome":str(last.get("status") or "unknown"),
                    "feedback":history,
                    "actual_deg":actual,
                    "residual_deg":round(target-actual,4),
                    "last_command":last}
        time.sleep(poll_s)
    return {"outcome":"timeout","feedback":history,
            "message":"pending or unverified motion; no automatic retry"}


def _persist(report,output_file):
    file=Path(output_file).expanduser()
    file.parent.mkdir(parents=True,exist_ok=True)
    tmp=file.with_suffix(file.suffix+".tmp")
    tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    tmp.replace(file)


def run_control_experiments(*,controller="http://127.0.0.1:8765",
                            joints=(1,2,3),degrees=10,cycles=1,
                            execute=False,consent=False,output_file=None):
    """A single manually authorized measured motion sweep; defaults READ ONLY."""
    if isinstance(joints,list):
        joints=tuple(joints)
    if (not isinstance(joints,tuple) or not joints or
            len(set(joints))!=len(joints) or
            any(type(j)!=int or j not in (1,2,3,4,5) for j in joints)):
        raise ValueError("joints must be distinct numbers in 1..5")
    if type(degrees)!=int or degrees not in (5,10):
        raise ValueError("actual meaningful test amplitudes are 5° or 10°")
    if type(cycles)!=int or not 1<=cycles<=10:
        raise ValueError("initial experimental batch supports 1..10 cycles")
    base=_base_url(controller)
    health,start,issues=_readiness(base)
    trial_id=uuid.uuid4().hex
    out=(Path(output_file).expanduser() if output_file else
         Path.home()/".evoarm"/"rsi"/"control_experiments"/(trial_id+".json"))
    plan=[]
    for j in joints:
        cur=start["joint_deg"][str(j)]
        sign=_direction(cur,j,degrees,1)
        plan.append({"joint":j,"start_deg":cur,
                     "move_deg":sign*degrees if sign is not None else None,
                     "supported":sign is not None})
    report={
        "kind":"roarm_joint_responsiveness_v1",
        "trial_id":trial_id,
        "started_at_unix":time.time(),
        "controller":base,"degrees":degrees,"cycles_requested":cycles,
        "joints":list(joints),"starting_measurement":start,
        "planned_initial_movements":plan,
        "preflight_issues":issues,
        "motor_execution_requested":bool(execute),
        "actions":[],"completed_actions":0,"stop_reason":None,
        "factory_reset_sent":False,
        "camera_required":False,
        "source":"real T1051 feedback, not visual estimates",
        "output_file":str(out),
    }
    if not execute:
        report["status"]="read_only_preview"
        return report
    if not consent:
        raise RuntimeError("type RUN locally before any physical motor actuation")
    if issues:
        report["status"]="blocked"
        report["stop_reason"]="; ".join(issues)
        _persist(report,out)
        return report
    if not set(joints).issubset(set(health.get("allowed_joints",[]))):
        report["status"]="blocked"
        report["stop_reason"]="requested joints not enabled in controller allowlist"
        _persist(report,out)
        return report
    report["status"]="running"
    _persist(report,out)
    stop=False
    for cycle in range(cycles):
        for j in joints:
            # Two directions around the same measured baseline so
            # backlash / hysteresis is recorded instead of masked.
            for preferred in (1,-1):
                try:
                    h,s,issues=_readiness(base)
                    if issues:
                        raise RuntimeError("controller not ready: "+",".join(issues))
                    begin=s["joint_deg"][str(j)]
                    sign=_direction(begin,j,degrees,preferred)
                    if sign is None:
                        raise RuntimeError("10° target outside validated joint travel")
                    goal=round(begin+sign*degrees,2)
                    record={"cycle":cycle+1,"joint":j,
                            "direction":sign,"amplitude_deg":degrees,
                            "start":s,"forward_target_deg":goal,
                            "stages":[]}
                    report["actions"].append(record)
                    _persist(report,out)
                    # Minimum 0.8s service gap. Never resend on an
                    # unknown HTTP timeout: the motor may already have acted.
                    time.sleep(.95)
                    ack=_controller_absolute(base,j,goal)
                    forward=_wait_real(base,j,goal)
                    record["stages"].append({"kind":"forward","ack":ack,**forward})
                    _persist(report,out)
                    if forward["outcome"]!="feedback_confirmed":
                        raise RuntimeError("forward not confirmed: "+forward["outcome"])
                    observed=forward["actual_deg"]-begin
                    record["observed_forward_deg"]=round(observed,3)
                    record["response_ratio"]=round(observed/(sign*degrees),4)
                    record["forward_error_deg"]=round(goal-forward["actual_deg"],3)
                    # Return to ORIGINAL measured starting angle, not
                    # blindly -10° from a partially reached motor pose.
                    # A <0.5° return request would be a bad micro-move.
                    if abs(forward["actual_deg"]-begin)<.5:
                        raise RuntimeError("forward response too small to return reliably")
                    h,s,issues=_readiness(base)
                    if issues:
                        raise RuntimeError("before return: "+",".join(issues))
                    time.sleep(.95)
                    back_target=round(begin,2)
                    ack=_controller_absolute(base,j,back_target)
                    backward=_wait_real(base,j,back_target)
                    record["stages"].append({"kind":"return","ack":ack,**backward})
                    _persist(report,out)
                    if backward["outcome"]!="feedback_confirmed":
                        raise RuntimeError("return not confirmed: "+backward["outcome"])
                    record["return_error_deg"]=round(
                        backward["actual_deg"]-begin,3)
                    record["complete"]=True
                    report["completed_actions"]+=1
                    _persist(report,out)
                except Exception as exc:
                    # Errors are data. Never attempt recovery, re-arm,
                    # factory reset or another motor command after a fault.
                    report["stop_reason"]=type(exc).__name__+": "+str(exc)[:500]
                    stop=True
                    _persist(report,out)
                    break
            if stop:
                break
        if stop:
            break
    report["status"]="stopped" if stop else "completed"
    report["ended_at_unix"]=time.time()
    _persist(report,out)
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description="RoArm meaningful 10°/5° joint response experiments")
    p.add_argument("--controller",default="http://127.0.0.1:8765")
    p.add_argument("--joints",default="1,2,3",help="comma-separated joint IDs, 1..5")
    p.add_argument("--degrees",type=int,choices=(5,10),default=10)
    p.add_argument("--cycles",type=int,default=1)
    p.add_argument("--output")
    p.add_argument("--execute",action="store_true")
    a=p.parse_args(argv)
    try:
        joints=tuple(int(x.strip()) for x in a.joints.split(","))
    except ValueError as exc:
        p.error("invalid comma-separated joints: "+str(exc))
    consent=False
    if a.execute:
        print("现场确认当前各关节整个+/-10°运动范围无碰撞，且能立即断电。")
        consent=input("输入 RUN 执行一组真实关节响应实验：").strip()=="RUN"
        if not consent:
            raise SystemExit("没有RUN授权，未发送电机命令")
    result=run_control_experiments(
        controller=a.controller,joints=joints,degrees=a.degrees,
        cycles=a.cycles,execute=a.execute,consent=consent,output_file=a.output)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
