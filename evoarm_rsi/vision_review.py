"""Review real RSI footage with the explicitly configured COMPANY model.

This is a VISION capability probe + retrospective reviewer, NOT a robot driver.
Send small ordered RGB keyframes and factual depth/trajectory metadata.
OpenAI-compatible chat image_url arrays are attempted; actual company model
support is determined from the response, never assumed.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
from pathlib import Path
from urllib import request, error
from urllib.parse import urlparse

from .gateway import CompanyModelGateway, MAX_RESPONSE_BYTES


ALLOWED_OUTCOMES = {"unknown", "not_attempted", "failed", "picked_up", "dropped"}
ALLOWED_FAILURES = {"none", "unknown", "missed_ball", "gripper_did_not_close",
                    "ball_slipped", "insufficient_lift", "motion_error", "no_attempt"}
ALLOWED_CHANGES = {"offset_x_mm", "offset_y_mm", "grasp_z_offset_mm",
                   "gripper_close_deg", "approach_speed"}


def build_keyframes(trial_dir, count=5, max_side_px=768, jpeg_quality=72):
    import cv2
    import numpy as np

    if not 2 <= count <= 8:
        raise ValueError("use 2..8 frames per review")
    if not 320 <= max_side_px <= 1280:
        raise ValueError("invalid resize bound")
    folder = Path(trial_dir)
    stamps = [json.loads(s) for s in
              (folder/"frames.jsonl").read_text(encoding="utf-8").splitlines() if s.strip()]
    if len(stamps) < 2:
        raise ValueError("too few recorded frames")
    cap = cv2.VideoCapture(str(folder/"rgb.mp4"))
    if not cap.isOpened():
        raise RuntimeError("unable to open trial RGB video")
    selected = np.unique(np.rint(np.linspace(0, len(stamps)-1, count)).astype(int))
    first = stamps[0]["host_monotonic_ns"]
    output = []
    try:
        for index in selected:
            index = int(index)
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, bgr = cap.read()
            if not ok:
                raise RuntimeError(f"unable to read recorded frame {index}")
            h, w = bgr.shape[:2]
            factor = min(1.0, max_side_px/max(w,h))
            if factor < 1:
                bgr = cv2.resize(bgr, (round(w*factor),round(h*factor)),
                                 interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg",bgr,
                                      [cv2.IMWRITE_JPEG_QUALITY,jpeg_quality])
            if not ok:
                raise RuntimeError("cannot JPEG encode trial keyframe")
            output.append({
                "frame_index":index,
                "elapsed_s":round((stamps[index]["host_monotonic_ns"]-first)/1e9,3),
                "data_url":"data:image/jpeg;base64,"+
                           base64.b64encode(encoded.tobytes()).decode("ascii"),
            })
    finally:
        cap.release()
    return output


def _read_json(path, default=None):
    path = Path(path)
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else default


def describe_evidence(trial_dir):
    """Provide truthful evidence flags; observations alone cannot prove grasp."""
    folder=Path(trial_dir)
    manifest=_read_json(folder/"manifest.json",{})
    if manifest.get("simulated") is not False:
        raise ValueError("vision review requires real camera evidence")
    observation=_read_json(folder/"observation.json",{})
    evaluation=_read_json(folder/"evaluation.json",None)
    trajectory=folder/"trajectory.jsonl"
    commands=folder/"commands.jsonl"
    depth=folder/"depth_samples.npz"
    physical_probe=_read_json(
        Path.home()/".evoarm"/"rsi"/"analysis"/str(manifest.get("trial_id"))/"physical_probe.json",
        None
    )
    linked_real_motion=bool(
        isinstance(physical_probe,dict) and
        physical_probe.get("kind")=="physical_single_joint_probe" and
        physical_probe.get("robot_moved") is True and
        isinstance(physical_probe.get("controller_feedback"),list) and
        any(row.get("kind")=="move_command"
            for row in physical_probe["controller_feedback"])
    )
    info={
        "trial_id":manifest.get("trial_id"),
        "trial_kind":manifest.get("kind"),
        "real_video":True,
        "robot_motion_executed":bool(manifest.get("robot_motion_executed",False)) or linked_real_motion,
        "grasp_attempted":bool(manifest.get("grasp_attempted",False)),
        "linked_physical_probe":(
            {"robot_moved":physical_probe["robot_moved"],
             "reverse_confirmed":physical_probe["reverse_confirmed"],
             "motion_error":physical_probe.get("motion_error"),
             "controller_feedback":physical_probe.get("controller_feedback",[])}
            if linked_real_motion else None
        ),
        "robot_feedback_recorded":bool(manifest.get("robot_feedback_recorded",False)),
        "robot_feedback_samples":sum(1 for row in trajectory.open(encoding="utf-8")
                                      if row.strip()) if trajectory.exists() else 0,
        "command_events":sum(1 for row in commands.open(encoding="utf-8")
                              if row.strip()) if commands.exists() else 0,
        "depth_samples_available":depth.exists(),
        "capture":manifest.get("capture"),
        "evaluation":evaluation,
        "observation":observation,
    }
    if depth.exists():
        import numpy as np
        with np.load(depth) as saved:
            values=saved["depth_mm"]
            valid=values[values>0]
            info["depth_statistics"]={
                "frames":int(len(values)),
                "valid_fraction":round(float(len(valid)/values.size),4),
                "median_valid_mm":float(np.median(valid)) if len(valid) else None,
                "note":"depth statistics are for whole scene, not ball-specific",
            }
    return info


def _validate_review(value, *, motion_recorded, grasp_attempted=True):
    if not isinstance(value,dict):
        raise ValueError("model review must be a JSON object")
    outcome=value.get("observed_outcome")
    if outcome not in ALLOWED_OUTCOMES:
        raise ValueError("invalid observed_outcome")
    conf=value.get("confidence")
    if type(conf) not in (int,float) or not math.isfinite(conf) or not 0 <= conf <= 1:
        raise ValueError("confidence must be 0..1")
    failure=value.get("failure_type")
    if failure not in ALLOWED_FAILURES:
        raise ValueError("invalid failure_type")
    notes=value.get("evidence",[])
    if not isinstance(notes,list) or len(notes)>10 or any(
        not isinstance(n,str) or len(n)>400 for n in notes):
        raise ValueError("invalid evidence")
    changes=value.get("changes",{})
    if not isinstance(changes,dict) or set(changes)-ALLOWED_CHANGES:
        raise ValueError("invalid next-trial changes")
    if any(type(v) not in (int,float) or not math.isfinite(v)
           for v in changes.values()):
        raise ValueError("nonfinite proposed changes")
    if not motion_recorded or not grasp_attempted:
        # A 1-degree exploration is REAL motion, but NOT a grasp attempt.
        # In neither case may the model hallucinate pick-up or drop.
        outcome="not_attempted"
        conf=1.0
        failure="no_attempt"
        # Keep policy suggestions advisory for the next micro-exploration
        # only when real movement occurred. No physical actuation here.
        if not motion_recorded:
            changes={}
    return {
        "observed_outcome":outcome, "confidence":float(conf),
        "failure_type":failure, "evidence":notes,
        "changes":changes, "motion_recorded":motion_recorded,
        "model_output_is_advisory":True,
    }


def review_trial(trial_dir, *, gateway=None, frames=5):
    gateway=gateway or CompanyModelGateway()
    if not gateway.configured():
        raise RuntimeError("company model is not configured; no public fallback")
    url=urlparse(gateway.base_url)
    if url.scheme!="https" or not url.hostname or url.username or url.password or url.fragment or url.query:
        raise ValueError("company gateway base URL must be HTTPS")
    evidence=describe_evidence(trial_dir)
    keyframes=build_keyframes(trial_dir,count=frames)
    system=(
        "You review short robotics experiments. Use the chronological RGB images "
        "and quantitative depth/robot facts. Never fabricate movement or ball grasp "
        "if robot_motion_executed is false. Return ONLY one JSON object with keys "
        "observed_outcome (unknown/not_attempted/failed/picked_up/dropped), "
        "confidence (0..1), failure_type "
        "(none/unknown/missed_ball/gripper_did_not_close/ball_slipped/"
        "insufficient_lift/motion_error/no_attempt), evidence (list of short "
        "observed facts), changes (optional single small numerical policy change). "
        "Any changes are suggestions for a LATER trial, not motor commands."
    )
    content=[{"type":"text","text":json.dumps({
        "task":"Interpret ordered frames and evidence; abstain when evidence is insufficient.",
        "frame_timestamps":[{"index":k["frame_index"],"elapsed_s":k["elapsed_s"]}
                            for k in keyframes],
        "metadata":evidence,
    },ensure_ascii=False)}]
    for frame in keyframes:
        content.append({"type":"image_url","image_url":{
            "url":frame["data_url"],"detail":"low"
        }})
    payload={"model":gateway.model,"temperature":0.1,"messages":[
        {"role":"system","content":system},
        {"role":"user","content":content},
    ]}
    req=request.Request(
        gateway.base_url+"/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type":"application/json",
                 "Authorization":"Bearer "+os.environ["EVOARM_COMPANY_LLM_TOKEN"]},
    )
    try:
        with request.urlopen(req,timeout=gateway.timeout_s) as response:
            if response.status!=200:
                raise RuntimeError(f"company vision gateway HTTP {response.status}")
            raw=response.read(MAX_RESPONSE_BYTES+1)
    except error.HTTPError as exc:
        raise RuntimeError(
            f"company vision gateway HTTP {exc.code}; "
            "this model/endpoint may not support image_url inputs"
        ) from exc
    except error.URLError as exc:
        raise RuntimeError(
            f"company vision gateway network failure: {type(exc.reason).__name__}"
        ) from exc
    if len(raw)>MAX_RESPONSE_BYTES:
        raise RuntimeError("company vision response too large")
    answer=json.loads(raw.decode("utf-8"))["choices"][0]["message"]["content"]
    if not isinstance(answer,str):
        raise ValueError("company vision final answer is not text JSON")
    # May include JSON markdown fences; strip only a single enclosing fence.
    answer=answer.strip()
    if answer.startswith("```"):
        lines=answer.splitlines()
        answer="\n".join(lines[1:-1]).strip()
    reviewed=_validate_review(json.loads(answer),
                             motion_recorded=evidence["robot_motion_executed"],
                             grasp_attempted=evidence.get("grasp_attempted",False))
    reviewed.update({
        "trial_id":evidence["trial_id"],
        "model":gateway.model,
        "number_of_rgb_keyframes":len(keyframes),
        "input_modality":"timestamped_rgb_keyframes_plus_structured_depth_and_telemetry",
        "company_gateway_vision_accepted":True,
        "no_public_fallback":True,
    })
    return reviewed


def main(argv=None):
    parser=argparse.ArgumentParser(description="Review real RSI RGB-D trial with COMPANY vision model (no robot motion)")
    parser.add_argument("trial")
    parser.add_argument("--frames",type=int,default=5)
    parser.add_argument("--output")
    args=parser.parse_args(argv)
    trial=Path(args.trial).expanduser()
    result=review_trial(trial,frames=args.frames)
    output=(Path(args.output).expanduser() if args.output else
            Path.home()/".evoarm"/"rsi"/"analysis"/trial.name/"visual_review.json")
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({**result,"output_file":str(output)},ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
