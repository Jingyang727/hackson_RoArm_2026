"""Read-only audit for actual J1 motion trials, including older Trial 004.

The camera and model can be imperfect observers of a 1-degree joint offset.
Motor T=1051 status is the primary quantitative evidence. Never silently
equate a controller acknowledgement with arrival.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _joint1_deg(status):
    v=(status or {}).get("b")
    if not isinstance(v,(int,float)) or not math.isfinite(v):
        return None
    return math.degrees(v)


def audit_physical_probe(probe):
    if probe.get("kind")!="physical_single_joint_probe":
        raise ValueError("not a saved physical RSI probe")
    history=probe.get("controller_feedback") or []
    if not isinstance(history,list):
        raise ValueError("invalid controller history")
    requested=probe.get("requested_step_deg")
    if not isinstance(requested,(int,float)) or not math.isfinite(requested):
        raise ValueError("missing finite J1 requested step")
    starts=[_joint1_deg(row.get("status")) for row in history
            if row.get("kind")=="pre_move"]
    starts=[v for v in starts if v is not None]
    # Earlier runs recorded only a pre_move T=1051 snapshot before
    # camera warmup; the subsequent start can drift slightly.
    start=probe.get("start_j1_deg")
    if start is None and starts:
        start=starts[-1]
    feedback=[]
    for row in history:
        status=row.get("robot_status")
        joint=_joint1_deg(status)
        if joint is not None:
            feedback.append({
                "host_monotonic_ns":row.get("host_monotonic_ns"),
                "angle_deg":round(joint,4),
                "pending":row.get("pending") is not None,
            })
    last=feedback[-1]["angle_deg"] if feedback else None
    target=start+requested if start is not None else None
    delta=last-start if last is not None and start is not None else None
    remaining=target-last if target is not None and last is not None else None
    confirm=bool(probe.get("command_acknowledged"))
    controller_reported=any(
        row.get("kind")=="forward_motion_confirmed" for row in history
    )
    fault_text=probe.get("motion_error")
    response={
        "source":"saved_real_robot_T1051",
        "requested_joint":"J1",
        "requested_delta_deg":requested,
        "reported_initial_deg":round(start,3) if start is not None else None,
        "requested_target_deg":round(target,3) if target is not None else None,
        "last_feedback_deg":round(last,3) if last is not None else None,
        "observed_delta_deg":round(delta,3) if delta is not None else None,
        "remaining_target_error_deg":round(remaining,3) if remaining is not None else None,
        "absolute_error_within_0_75deg":(
            abs(remaining)<=.75 if remaining is not None else None
        ),
        "command_acknowledged":confirm,
        "controller_forward_reported_confirmed":controller_reported,
        "reverse_attempted":bool(probe.get("reverse_attempted")),
        "return_confirmed":bool(probe.get("reverse_confirmed")),
        "returned_to_initial_position":(
            bool(probe.get("reverse_confirmed")) and
            abs(last-start)<=.75 if start is not None and last is not None
            else False
        ),
        "motion_error":fault_text,
        "frame_count_with_robot_angle":len(feedback),
        "original_HTTP_409_cause_recoverable_from_old_log":(
            bool(fault_text) and "HTTP Error 409" not in fault_text
        ),
        "notes":[
            "Observed robot movement and precise arrival are distinct outcomes",
            "The last feedback may be after a return attempt: inspect per-step samples to separate forward and reverse",
            "Older HTTPError 409 without FastAPI detail cannot be reconstructed from status alone",
            "No motion is performed by this audit",
        ],
    }
    return response


def main(argv=None):
    ap=argparse.ArgumentParser(description="Offline J1 trial outcome audit; no robot API or camera")
    ap.add_argument("trial_id")
    ap.add_argument("--report")
    ap.add_argument("--output")
    a=ap.parse_args(argv)
    path=Path(a.report).expanduser() if a.report else (
        Path.home()/".evoarm"/"rsi"/"analysis"/a.trial_id/"physical_probe.json"
    )
    result=audit_physical_probe(json.loads(path.read_text(encoding="utf-8")))
    out=Path(a.output).expanduser() if a.output else (
        path.with_name("motion_audit.json")
    )
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({**result,"output_file":str(out)},ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
