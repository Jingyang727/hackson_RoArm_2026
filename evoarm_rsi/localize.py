"""Offline, read-only RGB-D object localization from a saved EvoArm Trial.

No camera opening, model calls or arm access. For metric 3D in camera frame,
explicit color-aligned depth intrinsics are required. A robot-base XYZ is NOT
returned until an independently validated camera-to-robot transform exists.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def unproject_depth(u, v, depth_mm, intrinsics):
    """Aligned depth pixel -> optical camera XYZ, in mm (X right, Y down, Z out)."""
    if not isinstance(intrinsics, dict):
        raise ValueError("camera intrinsics required")
    fx = _number(intrinsics.get("fx"), "fx")
    fy = _number(intrinsics.get("fy"), "fy")
    cx = _number(intrinsics.get("cx"), "cx")
    cy = _number(intrinsics.get("cy"), "cy")
    z = _number(depth_mm, "depth_mm")
    if min(fx, fy, z) <= 0:
        raise ValueError("fx, fy, depth must be positive")
    return [round((u-cx)*z/fx, 3), round((v-cy)*z/fy, 3), round(z, 3)]


def locate_saved_trial(folder, *, intrinsics_file=None, square_mm=24.0,
                       max_depth_age_ms=180.0):
    import cv2
    from control_ui.vision import find_chessboard, find_yellow_ball

    folder = Path(folder)
    frames_path = folder/"frames.jsonl"
    video = folder/"rgb.mp4"
    depth_path = folder/"depth_samples.npz"
    if not frames_path.is_file() or not video.is_file() or not depth_path.is_file():
        raise ValueError("trial must contain rgb.mp4, frames.jsonl and depth_samples.npz")
    manifest = _read_json(folder/"manifest.json")
    if manifest.get("simulated", True):
        raise ValueError("simulated trial cannot be treated as real RGB-D")
    frames = [json.loads(row) for row in frames_path.read_text().splitlines() if row.strip()]
    if not frames:
        raise ValueError("missing frame timestamps")
    timestamps = np.asarray([int(r["host_monotonic_ns"]) for r in frames], dtype=np.int64)
    if len(timestamps)>1 and not bool(np.all(np.diff(timestamps)>0)):
        raise ValueError("RGB timestamp sequence is not increasing")
    with np.load(depth_path) as npz:
        depths = npz["depth_mm"]
        stamps = np.asarray(npz["monotonic_ns"], dtype=np.int64)
    if not len(stamps) or len(stamps)!=len(depths):
        raise ValueError("depth timestamps missing or out of sync")
    if not bool(np.all(np.diff(stamps)>0)):
        raise ValueError("depth timestamps are not strictly increasing")
    intrinsics = _read_json(intrinsics_file) if intrinsics_file else None
    if intrinsics is not None:
        if intrinsics.get("aligned_to") != "color":
            raise ValueError("expected calibrated intrinsics for COLOR-aligned depth")
        if intrinsics.get("verified") is not True:
            raise ValueError("intrinsics file is not marked verified")
    square = _number(square_mm,"square_mm")
    if not 0 < square <= 200:
        raise ValueError("invalid chessboard square size")
    max_age = _number(max_depth_age_ms, "max_depth_age_ms")
    if not 0 < max_age <= 1000:
        raise ValueError("max depth age must be 0..1000 ms")

    # Decode only RGB frames corresponding to stored depth samples. With
    # irregular camera fps, timestamps (not MP4 fps) choose the frame indices.
    selected = []
    for depth_i, stamp in enumerate(stamps):
        frame_i = int(np.argmin(np.abs(timestamps-stamp)))
        delta_ms = abs(int(timestamps[frame_i])-int(stamp))/1e6
        if delta_ms <= max_age:
            selected.append((depth_i, frame_i, round(delta_ms, 3)))
    if not selected:
        raise ValueError("no depth sample aligned to RGB timestamps")
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError("cannot decode saved RGB video")
    results = []
    board_detections = 0
    try:
        for depth_i, frame_i, delta_ms in selected:
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_i)
            ok, rgb = cap.read()
            if not ok:
                continue
            depth = depths[depth_i]
            if depth.shape != rgb.shape[:2]:
                raise ValueError("saved depth shape is not aligned with RGB")
            ball = find_yellow_ball(rgb)
            grid, H = find_chessboard(rgb)
            if grid is not None:
                board_detections += 1
            if ball is None:
                continue
            u, v = ball["center_px"]
            px, py = int(round(u)), int(round(v))
            radius = max(2, int(round(ball["radius_px"])))
            height, width = depth.shape
            outer = int(math.ceil(2.25*radius))
            x0, x1 = max(0, px-outer), min(width, px+outer+1)
            y0, y1 = max(0, py-outer), min(height, py+outer+1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            rr = (xx-px)**2+(yy-py)**2
            disk = rr <= (0.45*radius)**2
            crop = depth[y0:y1, x0:x1]
            valid = crop[disk & (crop>0)]
            ball_surface_mm = float(np.median(valid)) if len(valid)>=5 else None

            # A ring near the ball is only a local background-depth reference.
            # It is NOT a fitted tabletop plane or a calibrated object height.
            ring = (rr >= (1.3*radius)**2) & (rr <= (2.2*radius)**2)
            background = crop[ring & (crop>0)]
            background_mm = float(np.median(background)) if len(background)>=10 else None

            xy = None
            if H is not None:
                point = cv2.perspectiveTransform(
                    np.asarray([[[u,v]]],dtype=np.float32),H
                )[0,0]
                xy = [round(float(point[0]*square),2),
                      round(float(point[1]*square),2)]
            camera_xyz = None
            if ball_surface_mm is not None and intrinsics is not None:
                camera_xyz = unproject_depth(u,v,ball_surface_mm,intrinsics)
            results.append({
                "rgb_frame_index": frame_i,
                "depth_sample_index": depth_i,
                "rgb_depth_time_delta_ms": delta_ms,
                "ball_center_px": [u,v],
                "ball_radius_px": ball["radius_px"],
                "ball_surface_depth_mm": round(ball_surface_mm,2) if ball_surface_mm else None,
                "nearby_background_depth_mm": round(background_mm,2) if background_mm else None,
                "apparent_depth_difference_mm": (
                    round(background_mm-ball_surface_mm,2)
                    if background_mm and ball_surface_mm else None
                ),
                "ball_board_xy_mm": xy,
                "ball_camera_surface_xyz_mm": camera_xyz,
            })
    finally:
        cap.release()

    with_depth = [r for r in results if r["ball_surface_depth_mm"] is not None]
    with_board = [r for r in results if r["ball_board_xy_mm"] is not None]
    report = {
        "trial_id": manifest.get("trial_id"),
        "source": "real_rgbd_recording",
        "read_only": True,
        "motion_authorized": False,
        "camera_intrinsics_verified": intrinsics is not None,
        "robot_extrinsics_verified": False,
        "robot_target_xyz_mm": None,
        "rgb_frames_in_file": len(frames),
        "depth_samples_in_file": len(stamps),
        "rgb_depth_pairs_near_time": len(selected),
        "board_frames_detected": board_detections,
        "ball_frames_detected": len(results),
        "ball_frames_with_valid_depth": len(with_depth),
        "ball_frames_with_live_board_xy": len(with_board),
        "median_ball_surface_depth_mm": (
            round(float(np.median([r["ball_surface_depth_mm"] for r in with_depth])),2)
            if with_depth else None
        ),
        "observations": results,
        "warnings": [],
    }
    if not intrinsics:
        report["warnings"].append(
            "No VERIFIED color-aligned depth intrinsics: cannot unproject camera XYZ"
        )
    report["warnings"].append(
        "Camera-to-robot 3D extrinsics/TCP not verified; never convert this into a robot motion target"
    )
    if not with_board:
        report["warnings"].append(
            "No simultaneous full 9x6 checkerboard detection; board XY unavailable"
        )
    if not with_depth:
        report["warnings"].append(
            "No reliable depth inside detected ball; check alignment and occlusion"
        )
    # Persist an explicit read-only report rather than replacing raw evidence.
    out = folder/"rgbd_localization.json"
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),
                   encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline Gemini RGB-D object localization; NO MOTOR COMMANDS")
    parser.add_argument("folder")
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--square-mm",type=float,default=24.0)
    parser.add_argument("--max-depth-age-ms",type=float,default=180.0)
    args = parser.parse_args(argv)
    report = locate_saved_trial(args.folder,intrinsics_file=args.intrinsics,
                                square_mm=args.square_mm,max_depth_age_ms=args.max_depth_age_ms)
    print(json.dumps({
        k:v for k,v in report.items() if k!="observations"
    },ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
