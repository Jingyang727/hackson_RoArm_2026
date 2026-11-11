"""Qualify a real camera-ball localization for *read-only* robot XY preview.

Uses saved localize results and existing board->robot planar fit; never
connects to a camera, opens a serial port, or generates motion commands.
False yellow objects outside the physical checkerboard are rejected.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def qualify_trial(localization, calibration, intrinsics=None, *,
                  min_depth_contrast_mm=10.0, min_stable_frames=5):
    if not localization.get("read_only") or localization.get("motion_authorized") is not False:
        raise ValueError("requires a read-only localization report")
    fit = calibration.get("fit", {})
    if fit.get("validated") is not True:
        raise ValueError("independently validated board XY -> robot XY fit required")
    if float(fit.get("validation_error_mm", float("inf"))) > 10:
        raise ValueError("board XY fit has excessive held-out validation error")
    square = float(calibration.get("square_mm", 24))
    # 9x6 *inner corners*, eight by five inner-corner intervals.
    board_limits = [8*square, 5*square]
    matrix = np.asarray(fit.get("matrix_2x2"), dtype=np.float64)
    offset = np.asarray(fit.get("offset_mm"), dtype=np.float64)
    if (matrix.shape != (2,2) or offset.shape != (2,)
            or not np.isfinite(matrix).all() or not np.isfinite(offset).all()):
        raise ValueError("invalid calibrated planar transform")
    eligible = []
    rejected = {}
    for observation in localization.get("observations", []):
        reason = None
        xy = observation.get("ball_board_xy_mm")
        if not isinstance(xy, list) or len(xy) != 2:
            reason = "missing_board_xy"
        elif not np.isfinite(np.asarray(xy, dtype=float)).all():
            reason = "invalid_board_xy"
        elif not (0 <= xy[0] <= board_limits[0] and
                  0 <= xy[1] <= board_limits[1]):
            reason = "outside_inner_corner_board"
        elif (observation.get("ball_surface_depth_mm") is None or
              observation.get("apparent_depth_difference_mm") is None or
              float(observation["apparent_depth_difference_mm"]) <
              min_depth_contrast_mm):
            reason = "insufficient_ball_depth_contrast"
        if reason:
            rejected[reason] = rejected.get(reason, 0) + 1
        else:
            eligible.append(observation)
    # A stable cluster must be the *latest* observations, not an arbitrary
    # group of earlier detections or multiple balls.
    stable = eligible[-max(min_stable_frames, 12):]
    is_stable = False
    spatial_spread = None
    if len(stable) >= min_stable_frames:
        pts = np.asarray([v["ball_board_xy_mm"] for v in stable], dtype=float)
        med = np.median(pts, axis=0)
        spatial_spread = float(np.max(np.linalg.norm(pts-med, axis=1)))
        is_stable = spatial_spread <= 3.0
    else:
        med = None
    board_xy = [round(float(v), 3) for v in med] if is_stable else None
    robot_xy = (matrix @ med + offset).tolist() if is_stable else None
    result = {
        "trial_id": localization.get("trial_id"),
        "source": "real_camera_ball_board_xy_qualification",
        "read_only": True,
        "motion_authorized": False,
        "robot_target_xyz_mm": None,
        "board_limits_inner_corners_mm": board_limits,
        "raw_yellow_detections": len(localization.get("observations", [])),
        "eligible_inside_board_with_depth": len(eligible),
        "rejected_counts": rejected,
        "last_cluster_frames": len(stable),
        "max_cluster_deviation_mm": (round(spatial_spread, 3)
                                      if spatial_spread is not None else None),
        "stable_ball": is_stable,
        "ball_board_xy_mm": board_xy,
        "robot_xy_preview_mm": ([round(float(v), 3) for v in robot_xy]
                                if robot_xy is not None else None),
        "assumptions": [
            "The same physical chessboard, direction and origin as the previous robot XY calibration are preserved",
            "The camera and board did not move between the empty-board reference and ball placement",
            "A planar XY preview is not TCP, robot Z or collision validation",
        ],
        "warnings": [],
    }
    if intrinsics:
        values = [intrinsics.get(k) for k in ("fx","fy","cx","cy")]
        if all(isinstance(v,(int,float)) and np.isfinite(v) for v in values) and (
                intrinsics["fx"]>0 and intrinsics["fy"]>0 and
                intrinsics.get("aligned_to")=="color"):
            recent = stable[-min_stable_frames:]
            if len(recent)>=min_stable_frames and is_stable:
                u,v = np.median(np.asarray([x["ball_center_px"] for x in recent]),axis=0)
                z = float(np.median([x["ball_surface_depth_mm"] for x in recent]))
                xyz = [(u-intrinsics["cx"])*z/intrinsics["fx"],
                       (v-intrinsics["cy"])*z/intrinsics["fy"],z]
                result["ball_camera_surface_xyz_candidate_mm"] = [
                    round(float(q),3) for q in xyz]
                result["intrinsics_verified"] = bool(intrinsics.get("verified", False))
    if not is_stable:
        result["warnings"].append("Not enough spatially stable on-board yellow-ball observations")
    result["warnings"].append(
        "Robot XY is a planar preview only; no 3D base extrinsics, TCP or robot motion permission"
    )
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description="Read-only board-ball qualification and robot XY preview")
    p.add_argument("trial", help="trial directory containing saved localization and calibration data")
    p.add_argument("--localization",help="explicit localize report path")
    p.add_argument("--calibration",help="saved robot XY calibration JSON")
    p.add_argument("--intrinsics",help="saved Gemini intrinsics JSON")
    p.add_argument("--output",help="user-writable output path")
    args=p.parse_args(argv)
    trial=Path(args.trial).expanduser()
    loc=Path(args.localization).expanduser() if args.localization else (
        Path.home()/".evoarm"/"rsi"/"analysis"/trial.name/"rgbd_localization.json")
    calib=Path(args.calibration).expanduser() if args.calibration else (
        Path.home()/".evoarm"/"robot_xy_calibration.json")
    intr=Path(args.intrinsics).expanduser() if args.intrinsics else (
        trial/"gemini_color_intrinsics.json")
    report=qualify_trial(
        json.loads(loc.read_text(encoding="utf-8")),
        json.loads(calib.read_text(encoding="utf-8")),
        json.loads(intr.read_text(encoding="utf-8")) if intr.exists() else None,
    )
    output=Path(args.output).expanduser() if args.output else (
        Path.home()/".evoarm"/"rsi"/"analysis"/trial.name/"ball_qualification.json")
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({**report,"output_file":str(output)},ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
