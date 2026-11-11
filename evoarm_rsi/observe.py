"""Record a REAL camera-only RSI observation trial (no motor commands).

Example:
  python -m evoarm_rsi.observe --camera gemini --seconds 10
Optional:
  python -m evoarm_rsi.observe --camera usb --seconds 10 --device 0

No LLM or telemetry uploads. The output is local and usable as P1 training
evidence once timestamped robot feedback and grasp events are added.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import uuid

from .camera import OrbbecRGBDAdapter, TrialVideoRecorder, USBCameraAdapter
from .core import TrialStore


def record_observation(*, camera="gemini", seconds=10.0, device=0,
                       store=None, source=None, robot_status_url=None):
    if not 0 < seconds <= 20:
        raise ValueError("observation duration must be in (0,20] seconds")
    store = store if store is not None else TrialStore()
    if source is None:
        if camera == "gemini":
            from .orbbec import Gemini335LProvider
            source = OrbbecRGBDAdapter(Gemini335LProvider())
        elif camera == "usb":
            source = USBCameraAdapter(device)
        else:
            raise ValueError("unsupported camera type")
    trial_id = uuid.uuid4().hex
    manifest = {
        "trial_id": trial_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "kind": "real_camera_observation_only",
        "camera_type": camera, "duration_s": seconds,
        "simulated": False,
        "robot_motion_executed": False,
        "robot_feedback_recorded": False,
        "agent_called": False,
        "status": "created",
        "robot_status_source": robot_status_url or None,
    }
    folder = store.create(trial_id, manifest)
    try:
        source.start()
        try:
            manifest["status"] = "recording"
            store.write(trial_id, "manifest.json", manifest)
            # Start optional robot read-only capture *concurrently* with the
            # camera. This does not own serial and never sends motion.
            sampler = None
            telemetry = None
            try:
                if robot_status_url:
                    from .robot_telemetry import ReadOnlyRobotSampler
                    sampler = ReadOnlyRobotSampler(robot_status_url)
                    sampler.start(store, trial_id)
                result = TrialVideoRecorder(source, fps=20).capture(folder, seconds)
                # Preserve SDK-supplied intrinsics from the SAME stream,
                # without launching a second camera process.
                provider = getattr(source, "frame_provider", None)
                intrinsics = getattr(provider, "color_intrinsics", None)
                if isinstance(intrinsics, dict):
                    (folder / "gemini_color_intrinsics.json").write_text(
                        json.dumps(intrinsics, ensure_ascii=False, indent=2),
                        encoding="utf-8",
                    )
                    result["intrinsics_file"] = "gemini_color_intrinsics.json"
                    result["intrinsics_verified"] = False
                if getattr(provider, "intrinsics_error", None):
                    result["intrinsics_error_type"] = provider.intrinsics_error
            finally:
                if sampler is not None:
                    telemetry = sampler.stop()
        finally:
            source.stop()
        manifest.update({
            "status": "recorded", "capture": result,
            "robot_feedback_recorded": bool(telemetry and telemetry["real_samples"]),
            "robot_telemetry": telemetry,
            "finished_at": datetime.now(timezone.utc).isoformat()
        })
        store.write(trial_id, "manifest.json", manifest)
        store.write(trial_id, "observation.json", {
            "camera": camera, "capture": result, "robot_telemetry": telemetry,
            "note": ("real RGB-D + read-only robot state; no motor commands or grasp outcome"
                     if telemetry and telemetry["real_samples"] else
                     "real RGB-D observation only; no motor commands or grasp outcome"),
        })
        store.event(trial_id, "events", {
            "kind": "camera_observation_completed",
            "simulated": False,
            "motor_command_sent": False,
        })
    except Exception as exc:
        manifest.update({"status": "failed", "error": str(exc),
                         "finished_at": datetime.now(timezone.utc).isoformat()})
        store.write(trial_id, "manifest.json", manifest)
        raise
    return {"trial_id": trial_id, "folder": str(folder), "capture": result}


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="EvoArm P1: record real Gemini RGB-D evidence (NO robot movement)"
    )
    parser.add_argument("--camera", choices=["gemini", "usb"], default="gemini")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--store", default=None,
                        help="experiment folder (default ~/.evoarm/rsi/experiments)")
    parser.add_argument("--robot-status-url", default=None,
                        help="optional read-only localhost /api/status endpoint; never sends motion")
    args = parser.parse_args(argv)
    outcome = record_observation(
        camera=args.camera, seconds=args.seconds, device=args.device,
        store=TrialStore(args.store) if args.store else None,
        robot_status_url=args.robot_status_url
    )
    print(json.dumps(outcome, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
