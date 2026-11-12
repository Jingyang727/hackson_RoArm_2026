"""One scene-derived grasp/lift trial, through the existing serial owner."""
from __future__ import annotations

import math
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ConfigDict, Field

from . import bench_ui
from .safety import GRIPPER_SPEED_STEPS_S, GRIPPER_ACCEL_100_STEPS_S2
from evoarm_rsi.ball_trial import observe_ball, build_plan, preview, check_segment, SPEED
from evoarm_rsi.bench_calibration import load_session, _digest
from evoarm_rsi.bench_start import _ready
from evoarm_rsi.camera_service import client_request, preview_frame
from evoarm_rsi.task_agent import _write
from .vision import find_yellow_ball


def observe_pending(service,pending,status,now):
    """Read-only trial feedback classification. NEVER writes a motor command."""
    kind=pending["stage_kind"];elapsed=now-pending["since"]
    pending["elapsed_s"]=round(elapsed,3)
    def finish(**fields):
        service.pending=None
        service.last_command.update(status="feedback_confirmed",confirmed_at=time.time(),**fields)
    if any(status.get("torswitch"+k)!=1 for k in "BSETRG"):
        service._latch_fault("trial_torque_changed","抓取试验中扭矩状态改变，停止后续动作")
        return
    values=[status.get(k) for k in ("x","y","z","tit","r","g")]
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        service._latch_fault("trial_invalid_feedback","抓取试验反馈字段无效")
        return
    if kind=="cartesian":
        residual=math.dist(values[:3],pending["xyz_target"])
        position_progress=math.dist(values[:3],pending["xyz_start"])
        attitude_error=abs(status["tit"]-pending["pitch_target"])
        roll_error=abs(math.atan2(math.sin(status["r"]-pending["roll_target"]),math.cos(status["r"]-pending["roll_target"])))
        grip_error=abs(status["g"]-pending["grip_target"])
        progress=max(position_progress,math.degrees(abs(status["g"]-pending["grip_start"])),
                     math.degrees(abs(status["tit"]-pending["pitch_start"])))
        pending.update(position_error_mm=residual,progress=progress)
        if residual<=6 and attitude_error<=math.radians(2) and roll_error<=math.radians(2) and grip_error<=math.radians(2) and progress>=.4:
            finish(position_error_mm=residual)
        elif elapsed>8:
            service._latch_fault("trial_cartesian_unverified","本轮笛卡尔动作未在录像窗口内确认，不重发")
        return
    g=status["g"];target=pending["grip_target"]
    residual=abs(math.degrees(g-target));progress=math.degrees(g-pending["grip_start"])
    pending.update(gripper_error_deg=residual,gripper_progress_deg=progress)
    if kind=="hold":
        if residual<=2 and elapsed>=.2:finish(grip_closure="contact_candidate_position_consistent_after_hold_request")
        elif elapsed>1.5:service._latch_fault("trial_hold_unverified","夹爪保持角度未确认")
        return
    if residual<=2 and progress>=.4:
        finish(grip_closure="angle_reached_not_visual_grasp_proof")
        return
    samples=pending.setdefault("grip_samples",[])
    if not samples or now-samples[-1][0]>=.12:samples.append([now,g])
    while samples and now-samples[0][0]>.6:samples.pop(0)
    load=status.get("tG")
    load_valid=type(load) in (int,float) and math.isfinite(load)
    stable=(len(samples)>=3 and samples[-1][0]-samples[0][0]>=.35 and
            math.degrees(max(s[1] for s in samples)-min(s[1] for s in samples))<=.6)
    baseline=pending.get("initial_load")
    baseline=baseline if type(baseline) in (int,float) and math.isfinite(baseline) else 0
    if elapsed>=.8 and stable and progress>=2 and g<target-math.radians(2) and load_valid and abs(load-baseline)>=5:
        # This is only a candidate contact. A separate POST must set a
        # measured hold target; no automatic write occurs inside GET/status.
        pending["contact_candidate"]=True
        pending["contact_angle_rad"]=g
    if elapsed>3:service._latch_fault("trial_grip_unverified","闭爪到位或接触未及时确认，不继续提升")


class Empty(BaseModel):
    model_config=ConfigDict(extra="forbid")


class PlanID(BaseModel):
    model_config=ConfigDict(extra="forbid")
    plan_id:str=Field(pattern=r"^[a-f0-9]{32}$")


class Begin(PlanID):
    camera_request_id:str=Field(pattern=r"^[a-f0-9]{32}$")


class Step(PlanID):
    step_index:int=Field(ge=0,le=3)


def install(app,service_provider):
    router=APIRouter(prefix="/api/bench/grasp")
    plans={};prepare_lock=threading.Lock()
    def folder(key):return Path.home()/".evoarm/rsi/grasp_plans"/key
    def calibration():
        _,session=load_session(bench_ui.session_path())
        if session["setup_id"]!=bench_ui.setup_id():raise ValueError("current layout differs from calibration")
        fingerprint=_digest({k:session[k] for k in ("session_id","reference_sha256","records")})
        return session,fingerprint
    def camera_active(recording_id):
        camera=client_request(Path.home(),"GET","/health")
        stamp=camera.get("recording_started_monotonic_ns")
        age=(time.monotonic_ns()-stamp)/1e9 if type(stamp) is int else 999
        if (camera.get("recording") is not True or camera.get("robot_control") is not False or
                camera.get("phase")!="recording" or camera.get("recording_request_id")!=recording_id or not 0<=age<9.5):
            raise ValueError("this trial requires its own active ten-second recording")
        return age
    def load(key):
        if key not in plans:raise ValueError("plan expired or controller restarted; prepare a new live plan")
        state=plans[key]
        if time.monotonic()-state["created"]>180:raise ValueError("live plan expired; prepare again without moving")
        return state
    def persist(key,state):
        _write(folder(key)/"state.json",{k:v for k,v in state.items() if k!="created"})
    def validate_ball(plan):
        frame=preview_frame(Path.home())
        image=cv2.imdecode(np.frombuffer(frame["jpeg"],np.uint8),cv2.IMREAD_COLOR)
        if image is None or list(image.shape[1::-1])!=plan["image_size"]:
            raise ValueError("live camera dimensions changed since scene review")
        ball=find_yellow_ball(image) if image is not None else None
        expected=plan["ball"]
        if (not ball or ball["candidate_count"]!=1 or
                math.dist(ball["center_px"],expected["pixel_center"])>4 or
                abs(ball["radius_px"]/expected["radius_px"]-1)>.2):
            raise ValueError("current ball does not match the reviewed plan; no command sent")
    def confirmed(service,state,index):
        last=service.last_command or {}
        if (service.pending is not None or last.get("grasp_plan_id")!=state["id"] or
                last.get("grasp_step")!=index or last.get("status")!="feedback_confirmed"):
            raise ValueError("previous actual action has not been feedback-confirmed")
    def dispatch(service,state,index,command,pending):
        # Persist consumed step BEFORE a potentially uncertain serial write.
        state["next_step"]=index+1;state["dispatches"].append({"step":index,"command":command,"at_ns":time.monotonic_ns()})
        persist(state["id"],state)
        service._send_motion(command)
        service.last_motion_at=time.monotonic()
        service.pending={"kind":"ball_trial","since":time.monotonic(),**pending}
        service.last_command={"status":"sent_not_verified","grasp_plan_id":state["id"],
                              "grasp_step":index,"command":command,"sent_at":time.time()}
        return {"sent":True,"plan_id":state["id"],"step_index":index,"next_step":state["next_step"]}
    def reject(exc):raise HTTPException(status_code=409,detail=str(exc)) from exc

    @router.post("/prepare")
    def prepare(req:Empty):
        if not prepare_lock.acquire(blocking=False):raise HTTPException(status_code=409,detail="a live preparation is already running")
        try:
            service=service_provider()
            for key in list(plans):
                if time.monotonic()-plans[key]["created"]>180:plans.pop(key)
            if len(plans)>=32:raise ValueError("too many unexpired plans; inspect the current attempt first")
            with service.lock:
                status=service.status();_ready(service.health())
                session,fingerprint=calibration()
            cam=client_request(Path.home(),"GET","/health")
            if cam.get("recording"):raise ValueError("camera is recording another trial")
            capture=client_request(Path.home(),"POST","/record",{"seconds":2})
            trial_id=capture["trial_id"]
            if not isinstance(trial_id,str) or len(trial_id)!=32 or any(c not in "0123456789abcdef" for c in trial_id):
                raise ValueError("invalid scene capture identifier")
            ball,geometry,image=observe_ball(bench_ui.experiment_root()/trial_id,session)
            with service.lock:
                fresh=service.status();_ready(service.health())
                if any(abs(fresh[k]-status[k])>math.radians(.6) for k in "bsetrg"):
                    raise ValueError("robot moved during live scene capture")
                plan=build_plan(session,fresh,ball,geometry)
            key=uuid.uuid4().hex
            root=folder(key);root.mkdir(parents=True,exist_ok=False,mode=0o700)
            if not cv2.imwrite(str(root/"preview.jpg"),preview(image,plan,geometry)):raise ValueError("cannot save scene review image")
            state={"id":key,"created":time.monotonic(),"status":"prepared","next_step":0,"plan":plan,
                   "fingerprint":fingerprint,"dispatches":[],"hold_attempted":False}
            plans[key]=state;persist(key,state)
            return {"plan_id":key,"plan":plan,"preview_file":str(root/"preview.jpg"),
                    "plan_file":str(root/"state.json"),"next_action":"review image then execute this plan_id; prepare alone never moves"}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:reject(exc)
        finally:prepare_lock.release()

    @router.post("/begin")
    def begin(req:Begin):
        service=service_provider()
        try:
            with service.lock:
                state=load(req.plan_id)
                if state["status"]!="prepared":raise ValueError("plan already begun or consumed")
                actual=service._preflight(1)
                if service.allowed_joints!=set(range(1,7)):raise ValueError("all six joints must be authorized")
                _,fingerprint=calibration()
                if fingerprint!=state["fingerprint"]:raise ValueError("calibration changed after preparation")
                initial=state["plan"]["initial_status"]
                if any(abs(actual[k]-initial[k])>math.radians(1) for k in "bsetrg"):
                    raise ValueError("starting joints changed since scene review")
                if camera_active(req.camera_request_id)>1.5:raise ValueError("trial recording is not at its beginning")
                validate_ball(state["plan"])
                state.update(status="active",camera_request_id=req.camera_request_id)
                persist(req.plan_id,state)
                return {"plan_id":req.plan_id,"steps":len(state["plan"]["steps"]),"active":True,
                        "setup_id":state["plan"]["setup_id"]}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:reject(exc)

    @router.post("/step")
    def step(req:Step):
        service=service_provider()
        try:
            with service.lock:
                state=load(req.plan_id)
                if state["status"]!="active" or state["next_step"]!=req.step_index:
                    raise ValueError("step is not the next unconsumed action; never replay")
                stage=state["plan"]["steps"][req.step_index]
                actual=service._preflight(6 if stage["kind"]=="gripper" else 1)
                if service.allowed_joints!=set(range(1,7)):raise ValueError("all six joints must be authorized")
                if req.step_index:confirmed(service,state,req.step_index-1)
                _,fingerprint=calibration()
                if fingerprint!=state["fingerprint"]:raise ValueError("calibration changed during trial")
                camera_active(state["camera_request_id"])
                if stage["stage"]=="descend":validate_ball(state["plan"])
                if stage["kind"]=="cartesian":
                    target={**stage}
                    if stage["stage"]=="lift":target["gripper_rad"]=actual["g"]
                    check_segment(actual,target,state["plan"]["table_plane"])
                    x,y,z=target["xyz_mm"]
                    if (stage["stage"]=="align_open" and math.dist([actual[k] for k in "xyz"],[x,y,z])<=1
                            and abs(actual["tit"]-target["pitch_rad"])<=math.radians(.5)
                            and abs(actual["r"]-target["roll_rad"])<=math.radians(.5)
                            and abs(actual["g"]-target["gripper_rad"])<=math.radians(.5)):
                        state["next_step"]+=1
                        state["dispatches"].append({"step":req.step_index,"command":None,"at_ns":time.monotonic_ns(),
                                                   "reason":"live high alignment and opening already within tolerance"})
                        persist(req.plan_id,state)
                        service.last_command={"status":"feedback_confirmed","grasp_plan_id":req.plan_id,
                                              "grasp_step":req.step_index,"command":None,"verified_current_pose":True}
                        return {"sent":False,"plan_id":req.plan_id,"step_index":req.step_index,"next_step":state["next_step"]}
                    command={"T":104,"x":x,"y":y,"z":z,"t":target["pitch_rad"],
                             "r":target["roll_rad"],"g":target["gripper_rad"],"spd":SPEED}
                    pending={"stage_kind":"cartesian","xyz_start":[actual[k] for k in "xyz"],"xyz_target":[x,y,z],
                             "pitch_start":actual["tit"],"pitch_target":target["pitch_rad"],
                             "roll_target":target["roll_rad"],"grip_start":actual["g"],"grip_target":target["gripper_rad"]}
                else:
                    expected=state["plan"]["steps"][1]
                    if (math.dist([actual[k] for k in "xyz"],expected["xyz_mm"])>10 or
                            abs(actual["tit"]-expected["pitch_rad"])>math.radians(3)):
                        raise ValueError("tool left the reviewed grasp pose before closing")
                    command={"T":106,"cmd":stage["gripper_rad"],"spd":GRIPPER_SPEED_STEPS_S,"acc":GRIPPER_ACCEL_100_STEPS_S2}
                    pending={"stage_kind":"close","grip_start":actual["g"],"grip_target":stage["gripper_rad"],
                             "initial_load":actual.get("tG",0)}
                return dispatch(service,state,req.step_index,command,pending)
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:reject(exc)

    @router.post("/hold-contact")
    def hold(req:PlanID):
        service=service_provider()
        try:
            with service.lock:
                state=load(req.plan_id)
                if state["status"]!="active" or state["next_step"]!=3 or state["hold_attempted"]:
                    raise ValueError("no unconsumed grasp contact candidate")
                actual=service.status();health=service.health();pending=service.pending or {}
                if (not health["armed"] or health["fault"] or pending.get("kind")!="ball_trial" or
                        pending.get("stage_kind")!="close" or not pending.get("contact_candidate") or
                        (service.last_command or {}).get("grasp_plan_id")!=req.plan_id):
                    raise ValueError("no live, stable loaded grip-contact candidate")
                camera_active(state["camera_request_id"])
                target=actual["g"]
                if not pending["grip_start"]+math.radians(2)<=target<pending["grip_target"]:
                    raise ValueError("contact hold target is outside the measured closing interval")
                state["hold_attempted"]=True;persist(req.plan_id,state)
                service._send_motion({"T":106,"cmd":target,"spd":GRIPPER_SPEED_STEPS_S,"acc":GRIPPER_ACCEL_100_STEPS_S2})
                service.last_motion_at=time.monotonic()
                service.pending={"kind":"ball_trial","stage_kind":"hold","since":time.monotonic(),
                                 "grip_start":target,"grip_target":target}
                service.last_command={"status":"sent_not_verified","grasp_plan_id":req.plan_id,"grasp_step":2,
                                      "contact_candidate":True,"sent_at":time.time()}
                return {"hold_sent":True,"contact_candidate_only":True}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:reject(exc)

    @router.post("/end")
    def end(req:PlanID):
        service=service_provider()
        try:
            with service.lock:
                state=load(req.plan_id)
                complete=False
                if state["next_step"]==4:
                    try:confirmed(service,state,3);complete=True
                    except ValueError:pass
                state["status"]="completed_motion" if complete else "aborted"
                persist(req.plan_id,state);service.disarm()
                return {"completed_motion":complete,"visual_grasp_success":None,"placement_attempted":False}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:reject(exc)

    @router.get("/plans/{plan_id}")
    def plan_state(plan_id:str):
        try:
            with service_provider().lock:
                state=load(plan_id)
                return {"plan_id":plan_id,"setup_id":state["plan"]["setup_id"],"status":state["status"],
                        "next_step":state["next_step"],"plan":state["plan"],"dispatches":state["dispatches"]}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:reject(exc)

    app.include_router(router)
