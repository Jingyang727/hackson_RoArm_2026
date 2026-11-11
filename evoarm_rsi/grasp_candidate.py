"""Plan a first REAL RSI grasp-and-lift attempt from Gemini + v2 teaching.

The output is a full grasp/lift sequence, never a command or implicit motion
authorization. Execution uses the existing PickExecutor's T104/T106 status
verification once the operator approves the actual current scene and tool.

No arbitrary XYZ hallucination: the measured ball XY comes from a stable
on-board camera localization and the previously validated 2D transform;
all Z/orientation/gripper values are the user's own measured v2 teach poses.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


class GraspCandidateError(ValueError):
    pass


def build_grasp_candidate(qualification, poses, *, max_xy_shift_mm=20.0):
    if qualification.get("read_only") is not True or qualification.get("motion_authorized") is not False:
        raise GraspCandidateError("requires real read-only ball localization")
    if not qualification.get("stable_ball") or not qualification.get("latest_observation_eligible"):
        raise GraspCandidateError("real yellow ball has no current stable board position")
    xy=qualification.get("robot_xy_preview_mm")
    if (not isinstance(xy,list) or len(xy)!=2 or
        any(type(q) not in (int,float) or not math.isfinite(q) for q in xy)):
        raise GraspCandidateError("robot XY candidate is missing")
    items=poses.get("poses",poses)
    required=("home","hover","grasp","place","gripper_open","gripper_close")
    if not isinstance(items,dict) or any(k not in items for k in required):
        raise GraspCandidateError("six measured v2 teach snapshots required")
    def pos(k):
        x=items[k].get("xyz_mm")
        if (not isinstance(xy,list) or not isinstance(x,list) or len(x)!=3 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in x)):
            raise GraspCandidateError("missing measured XYZ for "+k)
        return x
    hover=pos("hover")
    grasp=pos("grasp")
    pos("home"); pos("place")
    if math.dist(hover[:2],grasp[:2])>15:
        raise GraspCandidateError("taught hover and grasp XY are inconsistent")
    if (math.dist(xy,hover[:2])>max_xy_shift_mm or
        math.dist(xy,grasp[:2])>max_xy_shift_mm):
        raise GraspCandidateError("ball differs from measured grasp by >20mm; re-align the tool before trying")
    if hover[2]-grasp[2]<30 or hover[2]-grasp[2]>150:
        raise GraspCandidateError("taught safe vertical travel is not plausible")
    pitch_h=items["hover"].get("pitch_rad")
    pitch_g=items["grasp"].get("pitch_rad")
    roll_h=items["hover"].get("roll_rad")
    roll_g=items["grasp"].get("roll_rad")
    if any(type(v) not in (int,float) or not math.isfinite(v)
           for v in (pitch_h,pitch_g,roll_h,roll_g)):
        raise GraspCandidateError("measured tool attitude is missing")
    if abs(math.degrees(pitch_h-pitch_g))>12:
        raise GraspCandidateError("hover and grasp pitch mismatch")
    if abs(math.degrees(math.atan2(math.sin(roll_h-roll_g),
                                  math.cos(roll_h-roll_g))))>12:
        raise GraspCandidateError("hover and grasp roll mismatch")
    open_g=items["gripper_open"].get("gripper_rad")
    close_g=items["gripper_close"].get("gripper_rad")
    if (type(open_g) not in (float,int) or type(close_g) not in (float,int)
        or not 1.08<=open_g<close_g<=3.14
        or close_g-open_g<math.radians(8)):
        raise GraspCandidateError("measured gripper open/close poses not valid")
    x,y=xy
    steps=[]
    def go(name,z,gripper,reason):
        steps.append({
            "stage":name,"kind":"cartesian","xyz_mm":[round(x,2),round(y,2),round(z,2)],
            "t":round(pitch_h,5),"r":round(roll_h,5),"g":round(gripper,5),
            "spd":0.10,"reason":reason,"requires_operator_confirm":True,
        })
    def hand(name,g,reason):
        steps.append({
            "stage":name,"kind":"gripper","g":round(g,5),"spd":100,"acc":10,
            "reason":reason,"requires_operator_confirm":True,
        })
    go("hover_ball",hover[2],open_g,"仅在机械臂已经位于实测高位Hover附近且桌面空间安全时定位")
    hand("open_before_pick",open_g,"张开夹爪，准备接近黄球")
    n=max(1,math.ceil((hover[2]-grasp[2])/20))
    for i in range(1,n+1):
        z=hover[2]+(grasp[2]-hover[2])*i/n
        go("descend_pick" if i==n else f"descend_pick_{i}",z,open_g,
           "基于真实示教的20mm内逐段垂直下降")
    hand("close_ball",close_g,"尝试夹取黄球；夹爪角度本身无法证明夹住")
    for i in range(1,n+1):
        z=grasp[2]+(hover[2]-grasp[2])*i/n
        go("lift_ball" if i==n else f"lift_ball_{i}",z,close_g,
           "保持夹爪闭合并分段提升，随后以RGB-D判定黄球是否随动")
    return {
        "kind":"rsi_v2_gemini_grasp_and_lift_candidate",
        "trial_id":qualification.get("trial_id"),
        "read_only":True,"motion_authorized":False,
        "auto_execute":False,
        "grasp_attempt_included":True,
        "place_attempt_included":False,
        "ball_board_xy_mm":qualification.get("ball_board_xy_mm"),
        "ball_robot_xy_mm":[round(x,2),round(y,2)],
        "measured_hover_xyz_mm":hover,
        "measured_grasp_xyz_mm":grasp,
        "taught_grasp_xy_gap_mm":round(math.dist(xy,grasp[:2]),2),
        "waypoints":steps,
        "number_of_steps":len(steps),
        "requirements":[
            "This full grasp-and-lift candidate is NOT a current-scene collision-proof plan",
            "The camera board frame must still match the saved robot 2D board fit",
            "Robot must already be near physically measured hover before first T104",
            "Tool TCP and tabletop clearance must be physically checked",
            "One actual action at a time, require real T1051 feedback and stop on faults",
            "Always evaluate the lifted ball in RGB-D before another grasp attempt",
        ],
    }


def main(argv=None):
    p=argparse.ArgumentParser(description="Build a full v2 grasp/lift candidate; NEVER actuates robot")
    p.add_argument("trial_id")
    p.add_argument("--qualification")
    p.add_argument("--poses")
    p.add_argument("--output")
    a=p.parse_args(argv)
    home=Path.home()
    root=home/".evoarm"/"rsi"/"analysis"/a.trial_id
    q=Path(a.qualification).expanduser() if a.qualification else root/"ball_qualification.json"
    poses=Path(a.poses).expanduser() if a.poses else home/".evoarm"/"pick_poses_v2.json"
    result=build_grasp_candidate(
        json.loads(q.read_text(encoding="utf-8")),
        json.loads(poses.read_text(encoding="utf-8")),
    )
    output=Path(a.output).expanduser() if a.output else root/"grasp_candidate.json"
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({**result,"output_file":str(output)},ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
