"""Current-frame object/depth evidence for a movable ball and black funnel.

No motor targets are inferred from unverified camera-to-base geometry.
Depth holes remain unknown. All returned coordinates describe visible
surfaces, not ball centers, funnel rims, grasp Z or release Z.
"""
from __future__ import annotations

import base64
import json
import math
from pathlib import Path

import cv2
import numpy as np


def image_part(bgr):
    height, width = bgr.shape[:2]
    ratio = min(1.0, 768 / max(height, width))
    small = cv2.resize(bgr, (round(width * ratio), round(height * ratio)))
    ok, encoded = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise ValueError("cannot encode camera evidence")
    return {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," +
            base64.b64encode(encoded.tobytes()).decode(), "detail": "high"}}


def validate_scene(value):
    result = {}
    for name in ("ball", "funnel_mouth"):
        obj = value.get(name)
        if obj is None:
            result[name] = None
            continue
        if not isinstance(obj, dict):
            raise ValueError("object must be null or a bounding-box object")
        box = obj.get("bbox_norm")
        if (not isinstance(box, list) or len(box) != 4 or any(
                type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1
                for v in box) or box[0] >= box[2] or box[1] >= box[3]):
            raise ValueError("invalid normalized object box")
        confidence = obj.get("confidence")
        if type(confidence) not in (int, float) or not 0 <= confidence <= 1:
            raise ValueError("invalid detection confidence")
        result[name] = {"bbox_norm": box, "confidence": confidence}
    return result


def load_rgbd(folder, frame_index=None, *, max_pairing_ms=180):
    folder = Path(folder)
    manifest = json.loads((folder / "manifest.json").read_text())
    if manifest.get("simulated") is not False or manifest.get("status") != "recorded":
        raise ValueError("complete real camera recording required")
    frames = [json.loads(row) for row in (folder / "frames.jsonl").read_text().splitlines() if row]
    stamps = np.asarray([row["host_monotonic_ns"] for row in frames], dtype=np.int64)
    if not len(stamps) or np.any(np.diff(stamps) <= 0):
        raise ValueError("invalid RGB timestamps")
    with np.load(folder / "depth_samples.npz", allow_pickle=False) as saved:
        depths = saved["depth_mm"]
        times = saved["monotonic_ns"]
    if len(times) != len(depths) or not len(times) or np.any(np.diff(times) <= 0):
        raise ValueError("invalid depth timestamps")
    # Last RGB image paired to the last saved depth: do not assume MP4 fps.
    if frame_index is None:
        frame_index = int(np.argmin(np.abs(stamps - times[-1])))
    if not 0 <= frame_index < len(frames):
        raise ValueError("frame outside recording")
    depth_i = int(np.argmin(np.abs(times - stamps[frame_index])))
    pairing_ms = abs(int(times[depth_i]) - int(stamps[frame_index])) / 1e6
    if pairing_ms > max_pairing_ms:
        raise ValueError("RGB/depth pair too far apart in time")
    cap = cv2.VideoCapture(str(folder / "rgb.mp4"))
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, bgr = cap.read()
    finally:
        cap.release()
    if not ok or depths[depth_i].shape != bgr.shape[:2]:
        raise ValueError("RGB/depth image unavailable or not aligned")
    return {"trial_id": manifest["trial_id"], "bgr": bgr, "depth_mm": depths[depth_i],
            "frame_index": frame_index, "host_monotonic_ns": int(stamps[frame_index]),
            "pairing_ms": round(pairing_ms, 3),
            "elapsed_s": (int(stamps[frame_index]) - int(stamps[0])) / 1e9}


def object_depth(depth, box, *, intrinsics=None):
    height, width = depth.shape
    x0, y0, x1, y1 = box
    # Use the central half of the detection to limit surrounding-table pixels.
    xa, xb = int((.75*x0+.25*x1)*width), math.ceil((.25*x0+.75*x1)*width)
    ya, yb = int((.75*y0+.25*y1)*height), math.ceil((.25*y0+.75*y1)*height)
    patch = depth[ya:yb, xa:xb]
    mask = np.isfinite(patch) & (patch > 0)
    valid = patch[mask]
    fraction = len(valid) / max(1, patch.size)
    result = {"valid_fraction": round(fraction, 3), "samples": len(valid),
              "status": "measured" if len(valid) >= 9 and fraction >= .35 else "unknown",
              "surface_depth_mm": None, "camera_surface_xyz_candidate_mm": None,
              "robot_target_xyz_mm": None,
              "meaning": "Visible central surface only; a funnel hole can measure its interior/table, not its rim."}
    if result["status"] == "unknown":
        return result
    z = float(np.median(valid))
    result.update(surface_depth_mm=round(z, 1),
                  depth_p10_p90_mm=[round(float(v), 1) for v in np.percentile(valid, [10, 90])])
    if intrinsics and intrinsics.get("verified") is True and intrinsics.get("aligned_to") == "color":
        values = [intrinsics.get(k) for k in ("fx", "fy", "cx", "cy")]
        if (all(type(v) in (int, float) and math.isfinite(v) for v in values)
                and values[0] > 0 and values[1] > 0
                and intrinsics.get("width") == width and intrinsics.get("height") == height):
            # Use an ACTUALLY sampled pixel; do not pair box center with
            # the depth of an unrelated surface somewhere else in the ROI.
            ys, xs = np.nonzero(mask)
            idx = int(np.argmin(np.abs(valid - z)))
            u, v = int(xs[idx] + xa), int(ys[idx] + ya)
            z = float(valid[idx])
            result["sample_pixel"] = [u, v]
            result["camera_surface_xyz_candidate_mm"] = [round(q, 2) for q in (
                (u-values[2])*z/values[0], (v-values[3])*z/values[1], z)]
    return result


def depth_grid(depth, rows=6, cols=8):
    """Compact spatial depth facts aligned with the accompanying RGB frame."""
    grid = []
    for parts in np.array_split(depth, rows, axis=0):
        row = []
        for patch in np.array_split(parts, cols, axis=1):
            values = patch[np.isfinite(patch) & (patch > 0)]
            row.append(round(float(np.median(values))) if len(values) >= patch.size*.35 and len(values) else None)
        grid.append(row)
    return {"median_depth_mm": grid, "null_means": "insufficient depth, never free space",
            "layout": "6 rows x 8 columns in the same RGB image; left-to-right, top-to-bottom"}
