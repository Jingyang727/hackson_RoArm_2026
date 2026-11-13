"""Prepare and EXECUTE one current-scene grasp/lift through the local controller.

`prepare` never moves. `execute --plan-id ...` opens, approaches, closes and
lifts, with one identified ten-second RGB-D recording. Company OpenCode/Grok
reviews the preview and exported key frames; no second LLM API is required.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import uuid

from .bench_start import _one_attempt
from .camera_service import client_request
from .real_probe import _base_url, _controller_get, _controller_post
from .task_agent import _write


def export_keyframes(folder,events,*,stages=('close','lift','finish','fault'),sample_count=0):
    import cv2
    import numpy as np
    folder=Path(folder)
    rows=[json.loads(r) for r in (folder/"frames.jsonl").read_text().splitlines() if r]
    if not rows:return None
    stamps=np.array([r["host_monotonic_ns"] for r in rows],dtype=np.int64)
    wanted=[("before",int(stamps[0]))]
    wanted.extend((e["stage"],e["host_monotonic_ns"]) for e in events if e["stage"] in stages)
    wanted.extend(("sample",int(stamps[i])) for i in
                  np.linspace(0,len(stamps)-1,sample_count,dtype=int))
    wanted.append(("last",int(stamps[-1])))
    # Preserve event labels when a uniform sample selects the same frame.
    selected={}
    for label,stamp in wanted:
        index=int(np.argmin(np.abs(stamps-stamp)))
        labels=selected.setdefault(index,[])
        if label not in labels:labels.append(label)
    cap=cv2.VideoCapture(str(folder/"rgb.mp4"));tiles=[];evidence=[]
    try:
        for index,labels in sorted(selected.items()):
            label='/'.join(labels)
            cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,frame=cap.read()
            if not ok:continue
            tile=cv2.resize(frame,(480,270))
            cv2.putText(tile,f"{label}: {(int(stamps[index])-int(stamps[0]))/1e9:.2f}s",(10,25),
                        cv2.FONT_HERSHEY_SIMPLEX,.6,(30,30,240),2)
            cv2.imwrite(str(folder/f"evidence-{len(tiles):02d}.jpg"),frame)
            evidence.append({'label':label,'labels':labels,'frame_index':index,
                             'elapsed_s':(int(stamps[index])-int(stamps[0]))/1e9,
                             'file':f'evidence-{len(tiles):02d}.jpg'})
            tiles.append(tile)
    finally:cap.release()
    if not tiles:return None
    # A chronological strip contains only actual frames: never pad with black.
    sheet=np.vstack(tiles)
    output=folder/"grasp-keyframes.jpg";cv2.imwrite(str(output),sheet)
    _write(folder/'keyframes.json',{'frames':evidence,'padding_frames':0})
    return str(output)


def execute(plan_id,*,controller="http://127.0.0.1:8765",home=None,progress=None):
    if len(plan_id)!=32 or any(c not in "0123456789abcdef" for c in plan_id):raise ValueError("invalid plan ID")
    base=_base_url(controller);home=Path(home) if home is not None else Path.home()
    root=home/".evoarm/rsi/grasp_runs"/plan_id
    with _one_attempt(root):
        receipt=root/"attempt.json"
        if receipt.exists():raise RuntimeError("this plan already has an execution receipt; never replay it")
        prepared=_controller_get(base,"/api/bench/grasp/plans/"+plan_id)
        if prepared.get("status")!="prepared":raise RuntimeError("this plan is no longer unconsumed; prepare a new live plan")
        h=_controller_get(base,"/api/health")
        if (not h.get("connected") or not h.get("armed") or h.get("fault") or h.get("pending")
                or h.get("torque_status")!=[1]*6):
            raise RuntimeError("connect and explicitly unlock the existing motion controller before execute")
        cam=client_request(home,"GET","/health")
        if cam.get("recording") or cam.get("recording_request_ids") is not True:raise RuntimeError("identified camera recording is unavailable")
        request_id=uuid.uuid4().hex
        report={"kind":"first_ball_grasp_execution_v1","plan_id":plan_id,"camera_request_id":request_id,
                "setup_id":prepared["setup_id"],"created_at_unix":time.time(),
                "perception_policy":prepared["plan"].get("perception_policy"),
                "observation_warnings":prepared["plan"].get("observation_warnings",[]),
                "events":[],"steps":[],"feedback":[],"motion_error":None,
                "grasp_attempted":False,"grasp_command_attempted":False,"placement_attempted":False,
                "motion_completed":False,"visual_grasp_success":None,"robot_moved":False}
        def event(stage):report["events"].append({"stage":stage,"host_monotonic_ns":time.monotonic_ns()})
        def publish(phase,**values):
            if progress is not None:progress({"phase":phase,**values})
        _write(receipt,report)
        last_dispatch=0.
        with ThreadPoolExecutor(max_workers=1) as pool:
            publish("recording_warmup")
            video=pool.submit(client_request,home,"POST","/record",{"seconds":10,"request_id":request_id})
            try:
                until=time.monotonic()+14
                while True:
                    if video.done():video.result();raise RuntimeError("camera ended before execution")
                    cam=client_request(home,"GET","/health")
                    if cam.get("recording_request_id")==request_id and cam.get("phase")=="recording" and cam.get("recording_started_monotonic_ns"):
                        capture_start=cam["recording_started_monotonic_ns"];break
                    if time.monotonic()>until:raise RuntimeError("camera never started this recording; no motion sent")
                    time.sleep(.05)
                session=_controller_post(base,"/api/bench/grasp/begin",{"plan_id":plan_id,"camera_request_id":request_id})
                if session.get("steps")!=5:raise RuntimeError("unexpected grasp execution contract")
                for index,stage in enumerate(("approach","approach","approach","close","lift")):
                    pause=.8-(time.monotonic()-last_dispatch)
                    if pause>0:time.sleep(pause)
                    if time.monotonic_ns()-capture_start>=9_250_000_000:raise RuntimeError("ten-second budget reached; no further command")
                    if video.done():video.result();raise RuntimeError("recording ended before next action")
                    if index==0:event("start")
                    event(stage)
                    if index==3:report["grasp_command_attempted"]=True
                    if report["robot_moved"] is False:report["robot_moved"]=None
                    _write(receipt,report)
                    ack=_controller_post(base,"/api/bench/grasp/step",{"plan_id":plan_id,"step_index":index})
                    publish(prepared["plan"]["steps"][index]["stage"],step_index=index)
                    report["steps"].append({"index":index,"stage":stage,"ack":ack,"sent_ns":time.monotonic_ns()})
                    if ack.get("sent"):last_dispatch=time.monotonic()
                    elif report["robot_moved"] is None:report["robot_moved"]=False
                    if index==3:report["grasp_attempted"]=True
                    held=False
                    while True:
                        status=_controller_get(base,"/api/status");h=_controller_get(base,"/api/health")
                        publish(prepared["plan"]["steps"][index]["stage"],step_index=index,health=h)
                        report["feedback"].append({"host_monotonic_ns":time.monotonic_ns(),"status":status,"health":h})
                        if h.get("fault"):raise RuntimeError("controller fault: "+str(h["fault"]))
                        pending=h.get("pending")
                        if pending is None:
                            last=h.get("last_command") or {}
                            if last.get("status")!="feedback_confirmed" or last.get("grasp_plan_id")!=plan_id or last.get("grasp_step")!=index:
                                raise RuntimeError("actual step was not feedback-confirmed")
                            if ack.get("sent"):report["robot_moved"]=True
                            break
                        if pending.get("contact_candidate") and not held:
                            held=True  # no replay even if a response is lost
                            report["contact_hold"]=_controller_post(base,"/api/bench/grasp/hold-contact",{"plan_id":plan_id})
                            last_dispatch=time.monotonic()
                        if time.monotonic_ns()-capture_start>=9_750_000_000:
                            raise RuntimeError("physical action not confirmed within the ten-second video")
                        if video.done():video.result();raise RuntimeError("camera stopped before arrival")
                        time.sleep(.08)
                event("finish");report["motion_completed"]=True
                report["physical_elapsed_s"]=(report["events"][-1]["host_monotonic_ns"]-report["events"][0]["host_monotonic_ns"])/1e9
                report["within_10s"]=report["physical_elapsed_s"]<=10
            except Exception as exc:
                report["motion_error"]=f"{type(exc).__name__}: {str(exc)[:600]}";event("fault")
            finally:
                publish("saving_recording")
                try:report["controller_end"]=_controller_post(base,"/api/bench/grasp/end",{"plan_id":plan_id})
                except Exception as exc:report["end_error"]=type(exc).__name__
                try:_controller_post(base,"/api/disarm",{})
                except Exception as exc:report["disarm_error"]=type(exc).__name__
                try:
                    state=_controller_get(base,"/api/bench/grasp/plans/"+plan_id)
                    report["controller_dispatches"]=state["dispatches"]
                    if any(d["step"]==3 for d in state["dispatches"]):report["grasp_attempted"]=True
                except Exception as exc:report["dispatch_evidence_error"]=type(exc).__name__
                try:
                    capture=video.result(timeout=35)
                    trial_id=capture["trial_id"]
                    if capture.get("request_id")!=request_id or len(trial_id)!=32 or any(c not in "0123456789abcdef" for c in trial_id):
                        raise ValueError("camera result does not match this execution")
                    report["capture_trial_id"]=trial_id
                    trial=home/".evoarm/rsi/experiments"/trial_id
                    manifest=json.loads((trial/"manifest.json").read_text())
                    manifest.update(kind="real_first_ball_grasp_lift",robot_motion_executed=report["robot_moved"],
                                    robot_feedback_recorded=bool(report["feedback"]),grasp_attempted=report["grasp_attempted"],
                                    placement_attempted=False,motion_report_file="grasp-trial.json")
                    _write(trial/"manifest.json",manifest)
                    frames=[json.loads(r)["host_monotonic_ns"] for r in (trial/"frames.jsonl").read_text().splitlines() if r]
                    if len(frames)<3 or any(b<=a for a,b in zip(frames,frames[1:])) or (frames[-1]-frames[0])/1e9<9:
                        raise ValueError("recording did not retain the requested ten-second evidence")
                    report["recording_observed_span_s"]=(frames[-1]-frames[0])/1e9
                    report["recording_covers_motion"]=bool(report["events"] and frames and
                        frames[0]<=report["events"][0]["host_monotonic_ns"] and report["events"][-1]["host_monotonic_ns"]<=frames[-1])
                    report["keyframes_file"]=export_keyframes(trial,report["events"])
                except Exception as exc:report["camera_error"]=f"{type(exc).__name__}: {str(exc)[:400]}"
                if report.get("capture_trial_id"):
                    trial=home/".evoarm/rsi/experiments"/report["capture_trial_id"]
                    _write(trial/"grasp-trial.json",report)
                    _write(trial/"grasp-plan.json",prepared["plan"])
                    motion={"kind":"pick_place_trial_v1","simulated":False,"capture_trial_id":report["capture_trial_id"],
                            "setup_id":report["setup_id"],"events":report["events"],"grasp_attempted":report["grasp_attempted"],
                            "placement_attempted":False,"motion_error":report["motion_error"] or report.get("camera_error")}
                    _write(trial/"motion-report.json",motion)
                _write(receipt,report)
        return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=("prepare","execute"))
    parser.add_argument("--controller",default="http://127.0.0.1:8765")
    parser.add_argument("--plan-id")
    parser.add_argument("--plan-file",help="prepare JSON; only its server-issued plan_id is read")
    parser.add_argument("--output")
    args=parser.parse_args(argv)
    if args.action=="prepare":result=_controller_post(_base_url(args.controller),"/api/bench/grasp/prepare",{})
    else:
        if args.plan_file:
            if args.plan_id:parser.error("use --plan-id or --plan-file, not both")
            args.plan_id=json.loads(Path(args.plan_file).read_text())["plan_id"]
        if not args.plan_id:parser.error("execute requires --plan-id from a reviewed live prepare result")
        result=execute(args.plan_id,controller=args.controller)
    if args.output:_write(args.output,result)
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    if args.action=="execute" and (not result["motion_completed"] or result.get("camera_error") or
                                 result.get("disarm_error") or not result.get("recording_covers_motion")):
        raise SystemExit(2)


if __name__=="__main__":main()
