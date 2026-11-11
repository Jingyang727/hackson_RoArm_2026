"""One operator-approved real v2 Gemini grasp-and-lift attempt.

Not simulated. Uses the existing PickExecutor's T104/T106 endpoint, which
checks tool posture, joint torque, geometry, no pending fault, and actual
T=1051 feedback BEFORE each waypoint. The model never emits motor payloads.
All motion requires explicit manual arm lease and opt-in ENABLE_RSI_BATCH=1.

The first experiment is one fixed plan: hover, open, descend, close, lift.
It stops immediately on incomplete feedback, without automatic rollback.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import threading
import time

from .camera_service import client_request
from .real_probe import _controller_get,_controller_post,_base_url


def run_grasp_attempt(trial_id,*,controller="http://127.0.0.1:8765",
                      execute=False, consent=False, camera_home=None):
    if not (isinstance(trial_id,str) and len(trial_id)==32 and
            all(ch in "0123456789abcdef" for ch in trial_id)):
        raise ValueError("invalid real Gemini trial ID")
    base=_base_url(controller)
    h=_controller_get(base,"/api/health")
    s=_controller_get(base,"/api/status")
    if not h.get("connected") or h.get("fault") or h.get("pending"):
        raise RuntimeError("robot not connected or has an unresolved motion")
    if not execute:
        return {
            "trial_id":trial_id,"status":"read_only",
            "robot_connected":True,"armed":h.get("armed"),
            "torque_status":h.get("torque_status"),
            "robot_xyz":[s.get(k) for k in "xyz"],
            "note":"A full grasp starts from live OFFICIAL factory joint pose (not old taught Home XYZ), with fresh Gemini ball; no empty-board recapture",
            "robot_moved":False,
        }
    if not consent:
        raise RuntimeError("operator must explicitly approve this full attempt")
    if not h.get("armed") or h.get("torque_status")!=[1]*6:
        raise RuntimeError("manual lease and six real locked torques required")
    # The human already typed RUN. Attach a ONE-SESSION RSI capability
    # to the existing manual motor lease, without stopping the serial
    # service and losing the 30-minute unlock.
    _controller_post(base,"/api/rsi/grasp/enable-once",{
        "declaration":"已在现场确认本次RSI抓取、运动范围和可断电"
    })
    home=Path(camera_home) if camera_home else Path.home()
    cam=client_request(home,"GET","/health")
    if cam.get("camera_service")!="ready" or cam.get("robot_control") is not False:
        raise RuntimeError("camera-only Gemini service is not ready")
    if cam.get("recording"):
        raise RuntimeError("camera service is already recording")
    active=threading.Event()
    stop=threading.Event()
    camera_trials=[]
    camera_errors=[]
    # Continuous recording in chunks of <=20s; this retains raw RGB-D,
    # rather than only a before/after still frame.
    def recorder():
        while not stop.is_set():
            try:
                item=client_request(home,"POST","/record",{"seconds":20.0})
                camera_trials.append(item)
            except Exception as exc:
                camera_errors.append(f"{type(exc).__name__}: {str(exc)[:200]}")
                return
    video=threading.Thread(target=recorder,daemon=True)
    video.start()
    t0=time.monotonic()
    while time.monotonic()-t0<14:
        if camera_errors:
            break
        state=client_request(home,"GET","/health")
        if state.get("phase")=="recording" and state.get("recording_started_monotonic_ns"):
            active.set()
            break
        time.sleep(0.2)
    if not active.is_set():
        stop.set()
        video.join(timeout=30)
        raise RuntimeError("Gemini did not start recording: no robot motion")
    steps=[]
    failure=None
    stall_recovery=None
    session_id=None
    total=0
    try:
        session=_controller_post(base,"/api/rsi/grasp/start",{
            "trial_id":trial_id,
            "declaration":"已检查当前Gemini黄球位置、夹爪高度和运动区域",
        })
        session_id=session["session_id"]
        if not session.get("auto_execute"):
            raise RuntimeError("controller RSI batch capability not explicitly enabled")
        total=int(session["total"])
        if not 5<=total<=40:
            raise RuntimeError("unexpected number of real grasp stages")
        for i in range(total):
            if camera_errors:
                raise RuntimeError("camera stopped recording during grasp")
            state=_controller_get(base,"/api/pick/execution")
            if (state.get("session_id")!=session_id or
                    state.get("next_index")!=i or
                    state.get("cancelled")):
                raise RuntimeError("pick session advanced unexpectedly")
            waypoint=state.get("next_step") or {}
            record={
                "step_index":i,
                "stage":waypoint.get("stage"),
                "target_xyz_mm":waypoint.get("xyz_mm"),
                "target_gripper_rad":waypoint.get("g"),
                "host_monotonic_ns":time.monotonic_ns(),
            }
            steps.append(record)
            ack=_controller_post(base,"/api/pick/step",{
                "session_id":session_id,"step_index":i,
                "declaration":"已批准一次RSI抓取试次自动分段执行",
            })
            record["ack"]=ack
            deadline=time.monotonic()+(70 if waypoint.get("kind")=="cartesian" else 15)
            while time.monotonic()<deadline:
                status=_controller_get(base,"/api/status")
                health=_controller_get(base,"/api/health")
                if health.get("fault"):
                    record["fault_snapshot"]=health["fault"]
                    raise RuntimeError("real controller fault: "+str(health["fault"]))
                if health.get("pending") is None:
                    result=health.get("last_command") or {}
                    record["last_command"]=result
                    record["actual_xyz_mm"]=[status.get(k) for k in "xyz"]
                    record["actual_gripper_rad"]=status.get("g")
                    record["finished_monotonic_ns"]=time.monotonic_ns()
                    if (result.get("status")!="feedback_confirmed" or
                            result.get("pick_session_id")!=session_id or
                            result.get("pick_step")!=i):
                        raise RuntimeError(
                            "real step not confirmed: "+str(result.get("status"))
                        )
                    record["feedback_confirmed"]=True
                    break
                time.sleep(0.45)
            else:
                raise RuntimeError("real feedback timeout; no new motor command")
            # Preserve existing minimum gap between physical commands.
            time.sleep(0.9)
    except Exception as exc:
        failure=f"{type(exc).__name__}: {str(exc)[:500]}"
        if session_id:
            # Local operator RUN approved this one session. If a T104 stage
            # generated a *software-latched* Cartesian stall, investigate
            # stable T=1051 readings and clear the latch automatically.
            # Never re-arm or re-send the failed stage, even after success.
            try:
                last_health=_controller_get(base,"/api/health")
                active_fault=last_health.get("fault")
                if (isinstance(active_fault,dict) and
                        active_fault.get("code")=="cartesian_stalled"):
                    stall_recovery=_controller_post(
                        base,"/api/rsi/grasp/recover-stall",
                        {"session_id":session_id}
                    )
                else:
                    stall_recovery={
                        "attempted":False,
                        "reason":"no matching recoverable cartesian_stalled fault",
                        "fault":active_fault,
                    }
            except Exception as recovery_exc:
                stall_recovery={
                    "attempted":True,
                    "software_fault_cleared":False,
                    "error":f"{type(recovery_exc).__name__}: {str(recovery_exc)[:300]}",
                }
            try:
                _controller_post(base,"/api/pick/cancel",{})
            except Exception:
                pass
    finally:
        stop.set()
        video.join(timeout=35)
    if video.is_alive():
        camera_errors.append("camera recording thread did not exit")
    completed=bool(steps and len(steps)==total and
                   all(x.get("feedback_confirmed") for x in steps))
    report={
        "kind":"rsi_v2_physical_grasp_and_lift",
        "source_ball_trial_id":trial_id,
        "session_id":session_id,
        "grasp_attempted":any(
            x.get("stage")=="close_ball" and x.get("feedback_confirmed")
            for x in steps
        ),
        "lift_completed":bool(completed and any(
            x.get("stage")=="lift_ball" for x in steps
        )),
        "grasp_success_verified":False,
        "steps_completed":sum(bool(x.get("feedback_confirmed")) for x in steps),
        "total_steps":total,
        "steps":steps,
        "motion_error":failure,
        "automatic_stall_recovery":stall_recovery,
        "motor_retried_after_fault":False,
        "capture_errors":camera_errors,
        "camera_trials":camera_trials,
        "unattended":False,
        "robot_returned_to_home":False,
    }
    basepath=Path.home()/".evoarm"/"rsi"/"analysis"/trial_id
    basepath.mkdir(parents=True,exist_ok=True)
    output=basepath/"grasp_run.json"
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    report["report_file"]=str(output)
    for item in camera_trials:
        ident=item.get("trial_id")
        if ident:
            path=Path.home()/".evoarm"/"rsi"/"analysis"/ident
            path.mkdir(parents=True,exist_ok=True)
            (path/"grasp_attempt.json").write_text(
                json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    # One *retrospective* Grok review after motion has stopped. Company-only,
    # no automatic next motor command even if the model proposes changes.
    report["grok_review_file"]=None
    report["grok_review_error"]=None
    if camera_trials:
        try:
            from .gateway import CompanyModelGateway
            from .vision_review import review_trial
            gateway=CompanyModelGateway()
            if gateway.configured():
                last=camera_trials[-1]
                last_trial=Path(last["folder"])
                judged=review_trial(last_trial,gateway=gateway,frames=5)
                target=(Path.home()/".evoarm"/"rsi"/"analysis"/last_trial.name/
                        "visual_review.json")
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_text(
                    json.dumps(judged,ensure_ascii=False,indent=2),encoding="utf-8"
                )
                report["grok_review_file"]=str(target)
            else:
                report["grok_review_error"]="company Grok token not configured"
        except Exception as exc:
            report["grok_review_error"]=f"{type(exc).__name__}: {str(exc)[:200]}"
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    if failure:
        raise RuntimeError(
            f"grasp trial stopped: {failure}. No retry or automatic return. "
            f"Report: {output}"
        )
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description="Full v2 Gemini grasp-and-lift trial, approved one time")
    p.add_argument("trial_id")
    p.add_argument("--controller",default="http://127.0.0.1:8765")
    p.add_argument("--execute",action="store_true")
    a=p.parse_args(argv)
    if a.execute:
        answer=input(
            "现场确认机械臂处于官方初始姿态（经现有归位UI及反馈检查），黄球在当前Gemini位置，"
            "区域清空并可随时断电。输入 RUN 执行一次完整抓取/提升："
        )
        if answer.strip()!="RUN":
            raise SystemExit("operator did not approve physical grasp")
    result=run_grasp_attempt(
        a.trial_id,controller=a.controller,
        execute=a.execute,consent=a.execute
    )
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
