"""Read-only preflight for EvoArm's first real yellow-ball grasp experiment.

Consumes *observed* qualified camera coordinates, a saved planar robot
calibration, and the six taught poses. Produces a comparison, not motor
commands or a fabricated grasp Z. Never opens camera, serial or network.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _pose_xy(poses, name):
    row=poses["poses"][name]
    xyz=np.asarray(row["xyz_mm"], dtype=float)
    if xyz.shape!=(3,) or not np.isfinite(xyz).all():
        raise ValueError(f"invalid taught pose: {name}")
    return xyz


def inspect_grasp_preflight(qualification, calibration, poses):
    if qualification.get("motion_authorized") is not False or qualification.get("read_only") is not True:
        raise ValueError("qualification must remain read-only")
    fit=calibration.get("fit") or {}
    if not fit.get("validated"):
        raise ValueError("previous robot XY fit is not validated")
    if fit.get("z_calibrated") or fit.get("tcp_calibrated"):
        # This program does not know how either was validated; do not silently
        # inherit a boolean as evidence of Gemini's 3D tool extrinsics.
        pass
    if not qualification.get("stable_ball"):
        raise ValueError("no stable on-board ball cluster")
    if qualification.get("robot_target_xyz_mm") is not None:
        raise ValueError("unexpected robot motion target in qualification")
    preview=np.asarray(qualification.get("robot_xy_preview_mm"),dtype=float)
    if preview.shape!=(2,) or not np.isfinite(preview).all():
        raise ValueError("invalid robot XY preview")
    grasp=_pose_xy(poses,"grasp")
    hover=_pose_xy(poses,"hover")
    home=_pose_xy(poses,"home")
    place=_pose_xy(poses,"place")
    gripper=poses["poses"]
    if not (gripper["gripper_open"]["gripper_rad"] <
            gripper["gripper_close"]["gripper_rad"]):
        raise ValueError("unexpected gripper direction: revisit teach snapshot")
    planar_errors={
        "grasp_to_ball_xy_mm":round(float(np.linalg.norm(grasp[:2]-preview)),3),
        "hover_to_ball_xy_mm":round(float(np.linalg.norm(hover[:2]-preview)),3),
        "hover_to_grasp_xy_mm":round(float(np.linalg.norm(hover[:2]-grasp[:2])),3),
    }
    validation=float(fit.get("validation_error_mm",float("inf")))
    if not np.isfinite(validation) or validation>10:
        raise ValueError("invalid held-out planar calibration")
    result={
        "trial_id":qualification.get("trial_id"),
        "read_only":True,
        "motion_authorized":False,
        "robot_target_xyz_mm":None,
        "ball_board_xy_mm":qualification.get("ball_board_xy_mm"),
        "ball_robot_xy_preview_mm":[round(float(x),3) for x in preview],
        "fit_holdout_error_mm":validation,
        "taught_pose_xyz_mm":{
            name:[round(float(v),3) for v in xyz]
            for name,xyz in (("home",home),("hover",hover),("grasp",grasp),("place",place))
        },
        "xy_comparisons":planar_errors,
        "readiness":{
            "real_ball_observations":True,
            "xy_fit_heldout_test":True,
            "gemini_board_origin_and_orientation_revalidated":False,
            "gemini_to_robot_3d_extrinsics_verified":False,
            "ball_center_height_and_tcp_verified":False,
            "joint_path_and_clearance_verified":False,
            "robot_motion_allowed":False,
        },
        "next_experiment":"verify two known board points against stationary robot pose and independently check tool Z",
        "notes":[
            "Robot XY is valid only if the physical chessboard has the same coordinate origin and orientation as the saved planar calibration.",
            "Ball surface depth is a camera optical-axis quantity; it is NOT the robot's grasp Z.",
            "Difference between grasp and ball XY is observed, not a recommended motor offset.",
            "Saved teach poses may be useful for an eventual bounded trial only after actual geometry checks.",
        ],
    }
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description="Read-only RSI grasp preflight; NO robot control")
    p.add_argument("trial", help="real trial directory")
    p.add_argument("--qualification",default=None)
    p.add_argument("--calibration",default=None)
    p.add_argument("--poses",default=None)
    p.add_argument("--output",default=None)
    args=p.parse_args(argv)
    trial=Path(args.trial).expanduser()
    analysis=Path.home()/".evoarm"/"rsi"/"analysis"/trial.name
    qualification=Path(args.qualification).expanduser() if args.qualification else analysis/"ball_qualification.json"
    calibration=Path(args.calibration).expanduser() if args.calibration else Path.home()/".evoarm"/"robot_xy_calibration.json"
    poses=Path(args.poses).expanduser() if args.poses else Path.home()/".evoarm"/"pick_poses_v2.json"
    def load(path):
        return json.loads(path.read_text(encoding="utf-8"))
    report=inspect_grasp_preflight(load(qualification),load(calibration),load(poses))
    output=Path(args.output).expanduser() if args.output else analysis/"grasp_preflight.json"
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({**report,"output_file":str(output)},ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
