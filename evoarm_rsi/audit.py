"""Audit recorded EvoArm camera/robot evidence before model evaluation.

Never infer grasp success from camera-only data; report factual coverage
and record gaps. No network calls, models, robot or camera access.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics


def audit_trial(folder):
    import numpy as np

    folder = Path(folder)
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    rows = [json.loads(line) for line in
            (folder / "frames.jsonl").read_text(encoding="utf-8").splitlines()
            if line.strip()]
    if not rows:
        raise ValueError("no RGB frame timestamps")
    timestamps = [int(row["host_monotonic_ns"]) for row in rows]
    monotonic = all(b > a for a, b in zip(timestamps, timestamps[1:]))
    span = ((timestamps[-1] - timestamps[0]) / 1e9) if len(timestamps) > 1 else 0.0
    gaps = [(b - a)/1e9 for a, b in zip(timestamps, timestamps[1:])]
    report = {
        "trial_id": manifest.get("trial_id"),
        "camera": manifest.get("camera_type"),
        "real_camera": not manifest.get("simulated", False),
        "rgb_frames": len(rows),
        "rgb_span_s": round(span, 3),
        "rgb_effective_fps": round((len(rows)-1)/span, 2) if span > 0 else 0,
        "rgb_max_gap_ms": round(max(gaps)*1000, 2) if gaps else None,
        "rgb_timestamps_strictly_increasing": monotonic,
        "rgb_device_timestamps_available": all(
            row.get("device_timestamp_ms") is not None for row in rows
        ),
        "robot_feedback_declared": bool(manifest.get("robot_feedback_recorded")),
        "robot_feedback_samples": 0,
        "grasp_outcome_observed": False,
    }
    depth_file = folder / "depth_samples.npz"
    if depth_file.exists():
        with np.load(depth_file) as depth:
            values = depth["depth_mm"]
            stamps = depth["monotonic_ns"]
            report["depth_samples"] = len(values)
            report["depth_shape"] = list(values.shape[1:])
            report["depth_valid_ratio"] = round(float(np.count_nonzero(values)) / values.size, 4)
            report["depth_sample_span_s"] = (
                round((int(stamps[-1])-int(stamps[0]))/1e9, 3)
                if len(stamps) > 1 else 0
            )
    else:
        report["depth_samples"] = 0
    trajectory = folder / "trajectory.jsonl"
    if trajectory.exists():
        samples = [json.loads(line) for line in trajectory.read_text().splitlines()
                   if line.strip()]
        real = [r for r in samples if r.get("source") == "real_roarm_readonly_api"]
        report["robot_feedback_samples"] = len(real)
        if real:
            report["robot_feedback_span_s"] = round((
                real[-1]["sample_end_monotonic_ns"] -
                real[0]["sample_end_monotonic_ns"]) / 1e9, 3)
    report["warnings"] = []
    if not monotonic:
        report["warnings"].append("RGB frame timestamps are not strictly increasing")
    if not report["rgb_device_timestamps_available"]:
        report["warnings"].append("Only host monotonic frame timestamps available")
    if not report["robot_feedback_samples"]:
        report["warnings"].append("No real robot state captured in this trial")
    if report["rgb_effective_fps"] < 15:
        report["warnings"].append("Effective RGB fps below 15; use frame timestamps, not configured fps")
    if manifest.get("kind") == "real_camera_observation_only":
        report["warnings"].append("Camera-only trial: cannot evaluate grasp success or RSI improvement")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Audit local RSI experiment evidence")
    parser.add_argument("folder", help="path to one trial folder")
    args = parser.parse_args(argv)
    print(json.dumps(audit_trial(args.folder), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
