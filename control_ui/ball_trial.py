"""One scene-derived grasp/lift trial, through the existing serial owner."""
from __future__ import annotations

import math
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np
from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel, ConfigDict, Field

from . import bench_ui
from .safety import GRIPPER_SPEED_STEPS_S, GRIPPER_ACCEL_100_STEPS_S2
from evoarm_rsi.ball_trial import observe_ball, build_plan, preview, check_segment, SPEED, ObservationRejected
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
    step_index:int=Field(ge=0,le=4)


UI_DECLARATION="已核对黄球和路径，现场有人看护且双手离开运动范围"


class UIRun(PlanID):
    declaration:str


def install(app,service_provider):
    router=APIRouter(prefix="/api/bench/grasp")
    plans={};prepare_lock=threading.Lock();run_lock=threading.RLock()
    ui={"status":"idle","phase":"idle","last_plan_id":None,"last_error":None,"stop_requested":False}
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
        reference_radius=expected["size_evidence"]["observed_area_diameter_px"]/2
        if (not ball or ball["candidate_count"]!=1 or
                math.dist(ball["center_px"],expected["pixel_center"])>4 or
                abs(ball["area_radius_px"]/reference_radius-1)>.2):
            raise ValueError("current ball does not match the reviewed plan; no command sent")
    def confirmed(service,state,index):
        last=service.last_command or {}
        if (service.pending is not None or last.get("grasp_plan_id")!=state["id"] or
                last.get("grasp_step")!=index or last.get("status")!="feedback_confirmed"):
            raise ValueError("previous actual action has not been feedback-confirmed")
    def dispatch(service,state,index,command,pending):
        if state.get("stop_requested"):raise ValueError("user stopped subsequent actions")
        # Persist consumed step BEFORE a potentially uncertain serial write.
        state["next_step"]=index+1;state["dispatches"].append({"step":index,"command":command,"at_ns":time.monotonic_ns()})
        persist(state["id"],state)
        if state.get("stop_requested"):raise ValueError("user stopped subsequent actions before transmission")
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
            with run_lock:
                if ui["status"]=="running":raise ValueError("a grasp trial is running")
                old=plans.get(ui.get("last_plan_id"))
                if old and old["status"]=="prepared":old["status"]="superseded"
                ui.update(last_plan_id=None,last_error=None,status="idle",phase="idle",result=None,error=None,
                          stop_requested=False,plan_id=None)
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
            with run_lock:ui.update(last_plan_id=key,last_error=None)
            return {"plan_id":key,"plan":plan,"preview_file":str(root/"preview.jpg"),
                    "preview_url":f"/api/bench/grasp/plans/{key}/preview",
                    "plan_file":str(root/"state.json"),"next_action":"review image then execute this plan_id; prepare alone never moves"}
        except ObservationRejected as exc:
            trial=exc.evidence["trial_id"]
            detail={"code":exc.evidence["rejection_code"],"message":str(exc),"observation":exc.evidence,
                    "image_url":f"/api/bench/grasp/diagnostics/{trial}/observation"}
            with run_lock:ui["last_error"]=detail
            raise HTTPException(status_code=409,detail=detail) from exc
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:
            with run_lock:ui["last_error"]={"message":str(exc)}
            reject(exc)
        finally:prepare_lock.release()

    @router.post("/begin")
    def begin(req:Begin):
        service=service_provider()
        try:
            with service.lock:
                state=load(req.plan_id)
                if state["status"]!="prepared" or state.get("stop_requested"):raise ValueError("plan already begun, stopped or consumed")
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
                if state["status"]!="active" or state.get("stop_requested") or state["next_step"]!=req.step_index:
                    raise ValueError("step is not the next unconsumed action; never replay")
                stage=state["plan"]["steps"][req.step_index]
                actual=service._preflight(6 if stage["kind"]=="gripper" else 1)
                if service.allowed_joints!=set(range(1,7)):raise ValueError("all six joints must be authorized")
                if req.step_index:confirmed(service,state,req.step_index-1)
                _,fingerprint=calibration()
                if fingerprint!=state["fingerprint"]:raise ValueError("calibration changed during trial")
                camera_active(state["camera_request_id"])
                if stage["stage"]=="hover":validate_ball(state["plan"])
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
                    expected=state["plan"]["steps"][2]
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
                if state["status"]!="active" or state.get("stop_requested") or state["next_step"]!=4 or state["hold_attempted"]:
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
                if state.get("stop_requested"):raise ValueError("user stopped subsequent actions")
                service._send_motion({"T":106,"cmd":target,"spd":GRIPPER_SPEED_STEPS_S,"acc":GRIPPER_ACCEL_100_STEPS_S2})
                service.last_motion_at=time.monotonic()
                service.pending={"kind":"ball_trial","stage_kind":"hold","since":time.monotonic(),
                                 "grip_start":target,"grip_target":target}
                service.last_command={"status":"sent_not_verified","grasp_plan_id":req.plan_id,"grasp_step":3,
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
                if state["next_step"]==5:
                    try:confirmed(service,state,4);complete=True
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

    def image_file(identifier,path,media_type):
        if len(identifier)!=32 or any(c not in "0123456789abcdef" for c in identifier) or not path.is_file() or path.is_symlink():
            raise HTTPException(status_code=404,detail="image is unavailable")
        return FileResponse(path,media_type=media_type,headers={"Cache-Control":"no-store","X-Content-Type-Options":"nosniff"})

    @router.get("/plans/{plan_id}/preview")
    def plan_image(plan_id:str):return image_file(plan_id,folder(plan_id)/"preview.jpg","image/jpeg")

    @router.get("/diagnostics/{trial_id}/{kind}")
    def diagnostic_image(trial_id:str,kind:str):
        names={"image":"ball-size-overlay.jpg","mask":"ball-size-mask.png","observation":"grasp-observation.jpg"}
        if kind not in names:raise HTTPException(status_code=404,detail="unknown image")
        return image_file(trial_id,bench_ui.experiment_root()/trial_id/names[kind],"image/png" if kind=="mask" else "image/jpeg")

    @router.get("/evidence/{trial_id}/keyframes")
    def keyframes(trial_id:str):return image_file(trial_id,bench_ui.experiment_root()/trial_id/"grasp-keyframes.jpg","image/jpeg")

    @router.get("/ui-state")
    def ui_state():
        with run_lock:
            job={k:ui.get(k) for k in ("status","phase","plan_id","last_error","stop_requested","result","error")}
            cached_health=ui.get("health")
            latest=plans.get(ui.get("last_plan_id"));plan=None
            if latest:
                age=time.monotonic()-latest["created"]
                plan={"plan_id":latest["id"],"status":latest["status"],"seconds_left":max(0,180-age),
                      "image_size":latest["plan"]["image_size"],
                      "preview_url":f"/api/bench/grasp/plans/{latest['id']}/preview",
                      "warnings":latest["plan"].get("observation_warnings",[]),
                      "mask_url":f"/api/bench/grasp/diagnostics/{latest['plan']['ball']['source_trial_id']}/mask",
                      "size":latest["plan"]["ball"].get("size_evidence"),"expired":age>=180}
        health=cached_health if job["status"]=="running" and cached_health else service_provider().health()
        return {"job":job,"plan":plan,"health":health,"setup_id":bench_ui.setup_id()}

    @router.post("/run",status_code=202)
    def run_from_ui(req:UIRun,request:Request):
        service=service_provider()
        try:
            with run_lock:
                if ui["status"]=="running" or prepare_lock.locked():raise ValueError("a preparation or execution is already running")
                if req.declaration!=UI_DECLARATION:raise ValueError("review the image and confirm onsite clearance first")
                state=load(req.plan_id)
                if state["status"]!="prepared" or state.get("stop_requested"):raise ValueError("prepare a new unconsumed plan")
                if (Path.home()/".evoarm/rsi/grasp_runs"/req.plan_id/"attempt.json").exists():raise ValueError("this execution has a receipt; do not replay it")
                if any(s["status"]=="active" for s in plans.values()):raise ValueError("another trial is active")
                port=(request.scope.get("server") or (None,None))[1]
                if type(port) is not int or not 1024<=port<=65535:raise ValueError("UI execution requires a local TCP controller")
                with service.lock:
                    service.status();_ready(service.health())
                    cam=client_request(Path.home(),"GET","/health")
                    if cam.get("recording"):raise ValueError("camera is recording another observation")
                    service.unlock_once("已检查设备并清空运动区域")
                    ui.update(status="running",phase="recording_warmup",plan_id=req.plan_id,stop_requested=False,
                              result=None,error=None,health=service.health())
                home=Path.home()
            def publish(values):
                with run_lock:
                    if ui.get("plan_id")==req.plan_id:
                        if not ui["stop_requested"]:ui["phase"]=values["phase"]
                        if "health" in values:ui["health"]=values["health"]
            def worker():
                from evoarm_rsi.grasp_once import execute
                outcome={}
                try:
                    result=execute(req.plan_id,controller=f"http://127.0.0.1:{port}",home=home,progress=publish)
                    trial=result.get("capture_trial_id")
                    summary={k:result.get(k) for k in ("motion_completed","grasp_attempted","motion_error","camera_error",
                             "physical_elapsed_s","within_10s","recording_covers_motion","visual_grasp_success","capture_trial_id")}
                    if result.get("keyframes_file") and trial and len(trial)==32 and all(c in "0123456789abcdef" for c in trial):
                        summary["keyframes_url"]=f"/api/bench/grasp/evidence/{trial}/keyframes"
                    ok=result["motion_completed"] and result.get("recording_covers_motion") and not result.get("camera_error") and not result.get("disarm_error")
                    outcome=dict(status="completed" if ok else "failed",result=summary,
                                 error=result.get("motion_error") or result.get("camera_error") or result.get("disarm_error"))
                except Exception as exc:
                    outcome=dict(status="failed",error=f"{type(exc).__name__}: {str(exc)[:500]}")
                finally:
                    service.disarm()
                    with run_lock:
                        if ui["stop_requested"]:outcome["status"]="stopped"
                        ui.update(phase="finished",**outcome)
            try:threading.Thread(target=worker,name="evoarm-ui-grasp",daemon=True).start()
            except RuntimeError:
                service.disarm()
                with run_lock:ui.update(status="failed",phase="finished",error="执行线程未启动；未发送运动")
                raise
            return {"started":True,"plan_id":req.plan_id}
        except (OSError,RuntimeError,ValueError,KeyError,TypeError) as exc:reject(exc)

    @router.post("/stop")
    def stop_from_ui(req:PlanID):
        with run_lock:
            if ui["status"]!="running" or ui.get("plan_id")!=req.plan_id:
                raise HTTPException(status_code=409,detail="no matching running UI trial")
            state=plans.get(req.plan_id)
            if state:state["stop_requested"]=True
            ui.update(stop_requested=True,phase="stopping")
        threading.Thread(target=service_provider().disarm,name="evoarm-stop-new-actions",daemon=True).start()
        return {"stop_requested":True,"notice":"停止后续指令；已接受或已发出的动作不会立即停止"}

    app.include_router(router)
