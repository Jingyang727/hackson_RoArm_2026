"""Preserve frame order while matching MP4 playback rate to capture time.

Per-frame monotonic timestamps remain authoritative for irregular sampling.
This is an offline media operation, with no camera or motor access.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def correct_video_timing(folder, *, output=None):
    import cv2
    folder = Path(folder).expanduser()
    source = folder / "rgb.mp4"
    dest = Path(output).expanduser() if output else folder / "rgb-realtime.mp4"
    if source.resolve() == dest.resolve():
        raise ValueError("timebase correction must use a separate output file")
    rows = [json.loads(row) for row in (folder/"frames.jsonl").read_text().splitlines() if row]
    stamps = [r["host_monotonic_ns"] for r in rows]
    if (len(stamps) < 2 or any(type(v) is not int for v in stamps)
            or any(b <= a for a, b in zip(stamps, stamps[1:]))):
        raise ValueError("at least two increasing capture timestamps required")
    span = (stamps[-1]-stamps[0])/1e9
    fps = (len(stamps)-1)/span
    if not .1 <= fps <= 240:
        raise ValueError("capture timing is not a supported video frame rate")
    cap = cv2.VideoCapture(str(source))
    writer = None
    count = 0
    original_fps = cap.get(cv2.CAP_PROP_FPS)
    if not cap.isOpened():
        cap.release()
        raise ValueError("source MP4 cannot be opened")
    if dest.exists():
        cap.release()
        raise ValueError("refusing to overwrite an existing corrected recording")
    dest.parent.mkdir(parents=True, exist_ok=True)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(str(dest), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
                if not writer.isOpened():
                    raise RuntimeError("corrected MP4 encoder could not be opened")
            writer.write(frame)
            count += 1
        if count != len(stamps):
            raise ValueError("video frames do not match timestamp count")
    except Exception:
        if writer is not None:
            writer.release()
            writer = None
        dest.unlink(missing_ok=True)
        raise
    finally:
        cap.release()
        if writer is not None:
            writer.release()
    return {"source_file": str(source), "output_file": str(dest), "frames": count,
            "source_nominal_fps": original_fps, "corrected_nominal_fps": fps,
            "capture_span_s": span, "estimated_playback_duration_s": count/fps,
            "per_frame_timestamps_authoritative": True,
            "note": "Average-rate correction; one terminal frame interval and frame jitter remain. "
                    "Do not infer physical execution time from MP4 duration."}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline video timebase correction; originals retained")
    parser.add_argument("folder")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    print(json.dumps(correct_video_timing(args.folder, output=args.output), indent=2))


if __name__ == "__main__":
    main()
