"""Read-only T1051 diagnosis for a real Cartesian grasp stage.

Never actuates a motor. Decompose error into X/Y/Z and quantify
which reported joints actually moved. Robot XYZ is the arm firmware's
forward-kinematic feedback, not an external metrology measurement.
"""
from __future__ import annotations
import json
import math
from pathlib import Path

XYZ=("x","y","z")
JOINTS={"b":"J1_base","s":"J2_shoulder","e":"J3_elbow",
        "t":"J4_wrist_pitch","r":"J5_wrist_roll","g":"J6_gripper"}
TELEMETRY=("b","s","e","t","r","g","tit","v",
           "tB","tS","tE","tT","tR","tG")


def _finite(v):
    return type(v) in (int,float) and math.isfinite(v)


def summarize_grasp_step(step):
    """Pure numeric summary; no API calls and no guesses about collisions."""
    target=step.get("target_xyz_mm")
    trace=[s for s in step.get("feedback_trace",[]) if isinstance(s,dict)]
    usable=[s for s in trace if all(_finite(s.get(k)) for k in XYZ)]
    out={"stage":step.get("stage"),"sample_count":len(trace),
         "valid_xyz_samples":len(usable),"reached_target":None,
         "observed_fault":step.get("fault_snapshot"),
         "action_confirmed":bool(step.get("feedback_confirmed",False)),
         "motor_commands_sent":None}
    if not isinstance(target,list) or len(target)!=3 or not all(
        _finite(v) for v in target
    ):
        out["diagnosis"]="no valid saved Cartesian target"
        return out
    if not usable:
        # PREVIOUS trials did not record full joint traces. Their
        # cartesian_stalled snapshots do contain authoritative target
        # and final XYZ from the actual controller. Recover useful
        # residual metrics without inventing J1/J2 movement samples.
        old_fault=step.get("fault_snapshot") or {}
        last=old_fault.get("actual_xyz_mm")
        if last is None:
            last=step.get("actual_xyz_mm")
        if isinstance(last,list) and len(last)==3 and all(_finite(n) for n in last):
            remaining=[target[i]-last[i] for i in range(3)]
            dist=math.sqrt(sum(n*n for n in remaining))
            out.update({
                "last_xyz_mm":[round(n,3) for n in last],
                "target_xyz_mm":[round(n,3) for n in target],
                "residual_target_minus_actual_xyz_mm":[round(n,3) for n in remaining],
                "euclidean_residual_mm":round(dist,3),
                "reached_target":dist<=6.0,
                "angular_y_difference_deg_at_xy_radius":round(
                    math.degrees(
                        math.atan2(target[1],target[0])-
                        math.atan2(last[1],last[0])
                    ),3
                ) if min(math.hypot(*target[:2]),math.hypot(*last[:2]))>50 else None,
                "diagnosis":"last XYZ is from real controller fault snapshot; "
                            "J1/J2 progression, voltage, load and initial pose "
                            "are not available in this older trial",
            })
        else:
            out["diagnosis"]="insufficient saved real controller measurements"
        return out
    start,end=usable[0],usable[-1]
    v0=[start[k] for k in XYZ]; v1=[end[k] for k in XYZ]
    error=[target[i]-v1[i] for i in range(3)]
    move=[v1[i]-v0[i] for i in range(3)]
    out.update({
        "start_xyz_mm":[round(n,3) for n in v0],
        "last_xyz_mm":[round(n,3) for n in v1],
        "target_xyz_mm":[round(n,3) for n in target],
        "observed_delta_xyz_mm":[round(n,3) for n in move],
        "residual_target_minus_actual_xyz_mm":[round(n,3) for n in error],
        "euclidean_residual_mm":round(math.sqrt(sum(n*n for n in error)),3),
        "reached_target":math.sqrt(sum(n*n for n in error))<=6.0,
        "joint_feedback_delta_deg":{
            label:(round(math.degrees(end[k]-start[k]),3)
                    if _finite(start.get(k)) and _finite(end.get(k))
                    else None)
            for k,label in JOINTS.items()
        },
        "start_pitch_deg":(
            round(math.degrees(start["tit"]),3)
            if _finite(start.get("tit")) else None
        ),
        "end_pitch_deg":(
            round(math.degrees(end["tit"]),3)
            if _finite(end.get("tit")) else None
        ),
        "min_voltage_v":(
            round(min(s["v"] for s in usable if _finite(s.get("v")))/100.0,2)
            if any(_finite(s.get("v")) for s in usable) else None
        ),
        "max_abs_servo_load_raw":{
            k:max(abs(s[k]) for s in usable if _finite(s.get(k)))
            for k in ("tB","tS","tE","tT","tR","tG")
            if any(_finite(s.get(k)) for s in usable)
        },
        "angular_y_difference_deg_at_xy_radius":round(
            math.degrees(
                math.atan2(target[1],target[0])-
                math.atan2(v1[1],v1[0])
            ),3
        ) if math.hypot(target[0],target[1])>50 and
             math.hypot(v1[0],v1[1])>50 else None,
        "diagnosis":"motion evidence only; IK, power, physical obstruction "
                    "and firmware behavior are not uniquely identifiable from this trace",
    })
    return out


def diagnose_report(report):
    if report.get("kind")!="rsi_v2_physical_grasp_and_lift":
        raise ValueError("not a real RSI grasp report")
    steps=report.get("steps",[])
    return {
        "trial":report.get("source_ball_trial_id"),
        "physical_grasp_attempted":bool(report.get("grasp_attempted")),
        "motion_error":report.get("motion_error"),
        "steps":[summarize_grasp_step(s) for s in steps],
        "read_only":True,
        "evidence_source":"T1051 controller feedback and saved grasp_run.json",
        "limitations":[
            "T1051 XYZ is firmware forward kinematics, not external measured TCP",
            "A stable software fault is not proof of physical jam",
            "Old reports may have no continuous telemetry; do not fabricate it",
        ],
    }


def main(argv=None):
    import argparse
    p=argparse.ArgumentParser(description="Offline hardware-free grasp stage diagnostics")
    p.add_argument("report",type=Path)
    p.add_argument("--output",type=Path)
    a=p.parse_args(argv)
    report=json.loads(a.report.expanduser().read_text(encoding="utf-8"))
    result=diagnose_report(report)
    out=(a.output.expanduser() if a.output
         else a.report.expanduser().with_name("grasp_motion_diagnosis.json"))
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({"summary":result,"output_file":str(out)},
                     ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
