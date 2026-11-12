"""One recorded automatic lift from the five-point calibration contact pose.

Uses only the existing localhost controller and dedicated camera daemon.
No old taught poses, raw serial, direct SDK, torque commands or reset.
It is explicitly a pregrasp action, not a completed grasp/place attempt.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import fcntl
import json
import math
from pathlib import Path
import time
import uuid

from control_ui.safety import plan_single_step
from .bench_calibration import load_session, fit_points
from .camera_service import client_request
from .gateway import CompanyModelGateway
from .real_probe import _base_url, _controller_get, _controller_post
from .roarm_geometry import forward, height_above_plane
from .task_agent import _write
from .task_memory import valid_id


def plan_lift(session, status):
    """A single -5 degree elbow step, only if the measured tip rises clear."""
    fit = fit_points(session["records"])
    if not fit["planar_fit_validated"]:
        raise ValueError("five-point calibration failed its held-out test")
    plane = fit["table_plane_robot"]
    xyz = [status.get(k) for k in "xyz"]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in xyz):
        raise ValueError("finite measured XYZ required")
    model = forward(status)
    if math.dist(xyz, model["xyz_mm"]) > 3.:
        raise ValueError("firmware XYZ does not match the default M3 edge-tool model")
    pitch = status.get("tit")
    if type(pitch) not in (int, float) or not math.isfinite(pitch):
        raise ValueError("finite measured tool pitch required")
    if abs(pitch-model["pitch_rad"]) > math.radians(1):
        raise ValueError("firmware pitch and joint feedback disagree")
    if not math.radians(75) <= pitch <= math.radians(100):
        raise ValueError("this contact-pose lift requires a near-vertical closed tool")
    # Closed tool only: opening a finger against the paper is a different,
    # unchecked swept volume. Do not open or restore a saved gripper target.
    if not math.radians(175) <= status["g"] <= math.pi+.02:
        raise ValueError("gripper is not closed; do not open/close it against the board")
    baseline = session["records"]["P1"]["tool_pose_rad"]
    for name, field in (("roll", "r"), ("gripper", "g")):
        delta = math.atan2(math.sin(status[field]-baseline[name]), math.cos(status[field]-baseline[name]))
        if abs(delta) > math.radians(5):
            raise ValueError("contact-point roll/gripper differs from the five-point samples")
    if any(status.get("torswitch"+key) != 1 for key in "BSETRG"):
        raise ValueError("all six real torque flags must equal 1")
    initial_height = height_above_plane(xyz, plane)
    if not -3 <= initial_height <= 6:
        raise ValueError("tip is not at the calibrated contact pose; this lift must be replanned")
    # The existing controller's unchanged joint limits validate the moving
    # elbow. A static shoulder offset is retained and never driven farther.
    movement = plan_single_step(status, 3, -5)
    path = []
    for step in range(21):
        sample = {**status, "e": status["e"]-math.radians(5)*step/20}
        pose = forward(sample)
        height = height_above_plane(pose["xyz_mm"], plane)
        if height < initial_height-1 or math.dist(xyz[:2], pose["xyz_mm"][:2]) > 25:
            raise ValueError("elbow step does not stay in the bounded upward corridor")
        # The wrist and elbow link endpoints must already be clear of table.
        if any(height_above_plane(p, plane) < 30 for p in pose["joints_xyz_mm"][1:3]):
            raise ValueError("an upstream link is too close to the measured table plane")
        path.append({"xyz_mm": pose["xyz_mm"], "height_mm": height})
    gain = path[-1]["height_mm"]-initial_height
    if not 8 <= gain <= 25:
        raise ValueError("predicted lift is not within the 8–25mm first-step envelope")
    return {"kind": "bench_contact_lift_plan_v1", "setup_id": session["setup_id"],
            "calibration_session_id": session["session_id"],
            "reference_sha256": session["reference_sha256"],
            "table_plane": plane, "initial_status": status,
            "joint": 3, "delta_deg": -5, "target_deg": movement.target_deg,
            "predicted_path": path, "predicted_lift_mm": gain,
            "initial_height_mm": initial_height, "taught_positions_used": False,
            "grasp_attempted": False, "placement_attempted": False}


def _ready(health):
    if (health.get("connected") is not True or health.get("fault") is not None or health.get("pending") is not None
            or health.get("teach_active") is not False or health.get("torque_status") != [1]*6):
        raise RuntimeError("controller requires connected, locked, idle feedback without faults")


def _check_recording(trial, report):
    manifest = json.loads((trial/"manifest.json").read_text())
    if manifest.get("trial_id") != trial.name:
        raise RuntimeError("recording manifest belongs to another trial")
    if manifest.get("simulated") is False:
        manifest.update(kind="real_pregrasp_lift", robot_motion_executed=report["robot_moved"],
                        motion_transmission_attempted=report["command_attempted"],
                        robot_feedback_recorded=bool(report["feedback"]), grasp_attempted=False,
                        placement_attempted=False, motion_report_file="pregrasp-lift.json")
        _write(trial/"manifest.json", manifest)
    if manifest.get("simulated") is not False or manifest.get("status") != "recorded":
        raise RuntimeError("recording did not retain complete real evidence")
    frames = [json.loads(row)["host_monotonic_ns"] for row in (trial/"frames.jsonl").read_text().splitlines() if row]
    if len(frames) < 3 or any(b <= a for a, b in zip(frames, frames[1:])) or (frames[-1]-frames[0])/1e9 < 9:
        raise RuntimeError("camera timestamps do not cover the requested ten-second observation")
    if not (trial/"depth_samples.npz").is_file() or not (trial/"rgb.mp4").is_file():
        raise RuntimeError("recorded RGB-D artifacts are missing")
    if report.get("motion_feedback_verified") and not (
            frames[0] <= report["command_start_monotonic_ns"] <= report["motion_finish_monotonic_ns"] <= frames[-1]):
        raise RuntimeError("recording does not cover the physical action and arrival feedback")
    report["recording_observed_span_s"] = (frames[-1]-frames[0])/1e9
    report["movement_covered_by_recording"] = bool(report.get("motion_feedback_verified"))


@contextmanager
def _one_attempt(folder):
    folder.mkdir(parents=True, exist_ok=True)
    with (folder/"lift.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("another preparation is running") from exc
        yield


def run(*, setup_id, controller="http://127.0.0.1:8765", execute=False, home=None):
    valid_id(setup_id)
    base = _base_url(controller)
    home = Path(home) if home is not None else Path.home()
    path, session = load_session(home/".evoarm/rsi/calibrations"/setup_id/"session.json")
    if session["setup_id"] != setup_id:
        raise ValueError("calibration layout differs from requested setup")
    folder = path.parent/"first-motion"
    with _one_attempt(folder):
        health = _controller_get(base, "/api/health")
        status = _controller_get(base, "/api/status")
        health = _controller_get(base, "/api/health")
        _ready(health)
        plan = plan_lift(session, status)
        config = CompanyModelGateway()
        report = {"kind": "bench_pregrasp_lift_v1", "setup_id": setup_id,
                  "simulated": False, "plan": plan, "feedback": [],
                  "company_grok_configured": config.configured(),
                  "grasp_attempted": False, "placement_attempted": False,
                  "command_attempted": False, "command_acknowledged": False,
                  "robot_moved": False,
                  "motion_feedback_verified": False, "motion_error": None,
                  "next_stage": "observe lifted tool, then plan scene-derived approach/grasp/place"}
        if not execute:
            return {**report, "mode": "read_only", "robot_moved": False}
        if health.get("motion_enabled") is not True or health.get("armed") is not True:
            raise RuntimeError("enable motion and explicitly unlock the existing controller before execution")
        receipt = folder/"lift-attempt.json"
        if receipt.exists():
            raise RuntimeError("this preparation already has an attempt receipt; inspect it, never blindly repeat")
        camera = client_request(home, "GET", "/health")
        if (camera.get("camera_service") != "ready" or camera.get("robot_control") is not False
                or camera.get("recording") or not camera.get("recording_request_ids")):
            raise RuntimeError("restart the updated camera-only service; require idle recording and request ID support")
        request_id = uuid.uuid4().hex
        report["camera_request_id"] = request_id
        with ThreadPoolExecutor(max_workers=1) as pool:
            video = pool.submit(client_request, home, "POST", "/record", {"seconds": 10, "request_id": request_id})
            try:
                warmup_deadline = time.monotonic()+14
                while True:
                    if video.done():
                        video.result()  # retain an actual capture failure
                        raise RuntimeError("recording ended before any motion")
                    camera = client_request(home, "GET", "/health")
                    if (camera.get("recording_request_id") == request_id and camera.get("phase") == "recording"
                            and camera.get("recording_started_monotonic_ns")):
                        capture_start = camera["recording_started_monotonic_ns"]
                        break
                    if time.monotonic() >= warmup_deadline:
                        raise RuntimeError("camera did not start this request; no motion sent")
                    time.sleep(.05)
                # Recheck after warmup; neither a queued request nor an old
                # capture's health is authority to send the arm command.
                fresh = _controller_get(base, "/api/status")
                fresh_health = _controller_get(base, "/api/health")
                _ready(fresh_health)
                if math.dist([fresh[k] for k in "xyz"], [status[k] for k in "xyz"]) > 2:
                    raise RuntimeError("contact pose drifted during camera warmup")
                if any(abs(fresh[k]-status[k]) > math.radians(.6) for k in "bsetrg"):
                    raise RuntimeError("joints drifted during camera warmup")
                plan_lift(session, fresh)
                if not 0 <= (time.monotonic_ns()-capture_start)/1e9 < 1.5:
                    raise RuntimeError("capture is no longer at its beginning; no command sent")
                if video.done():
                    video.result()
                    raise RuntimeError("camera ended before command")
                report["command_attempted"] = True
                report["robot_moved"] = None  # unknown until actual feedback verifies progress
                report["command_start_monotonic_ns"] = time.monotonic_ns()
                _write(receipt, report)  # persistent no-replay boundary BEFORE HTTP write
                report["ack"] = _controller_post(base, "/api/step", {"joint": 3, "delta": -5})
                report["command_acknowledged"] = True
                deadline = capture_start/1e9+9
                while time.monotonic() < deadline:
                    feedback = _controller_get(base, "/api/status")
                    h = _controller_get(base, "/api/health")
                    report["feedback"].append({"host_monotonic_ns": time.monotonic_ns(),
                                                "status": feedback, "health": h})
                    if h.get("fault") or h.get("teach_active") or h.get("torque_status") != [1]*6:
                        raise RuntimeError("controller fault or torque change; no further command")
                    if h.get("pending") is None:
                        actual = height_above_plane([feedback[k] for k in "xyz"], plan["table_plane"])
                        gain = actual-plan["initial_height_mm"]
                        last = h.get("last_command") or {}
                        if (last.get("status") != "feedback_confirmed" or last.get("joint") != 3
                                or abs(last.get("to_deg", 999)-plan["target_deg"]) > .7
                                or abs(math.degrees(feedback["e"])-plan["target_deg"]) > 2
                                or not 6 <= gain <= 30
                                or any(abs(feedback[k]-fresh[k]) > math.radians(1.) for k in "bstrg")):
                            raise RuntimeError("the recorded feedback did not verify the bounded lift")
                        report.update(motion_feedback_verified=True, robot_moved=True, measured_lift_mm=gain,
                                      final_status=feedback, motion_finish_monotonic_ns=time.monotonic_ns())
                        report["physical_action_elapsed_s"] = (
                            report["motion_finish_monotonic_ns"]-report["command_start_monotonic_ns"])/1e9
                        break
                    if video.done():
                        video.result()
                        raise RuntimeError("camera ended before arrival confirmation")
                    time.sleep(.1)
                else:
                    raise RuntimeError("lift was not confirmed within the 10-second recording")
            except Exception as exc:
                report["motion_error"] = f"{type(exc).__name__}: {str(exc)[:500]}"
            finally:
                # Revoke the lease at arrival/failure, while the camera
                # finishes its ten seconds. This does not cut torque or
                # cancel an uncertain physical trajectory.
                try:
                    _controller_post(base, "/api/disarm", {})
                except Exception as exc:
                    report["disarm_error"] = type(exc).__name__
                try:
                    capture = video.result(timeout=30)
                    trial_id = capture["trial_id"]
                    if not isinstance(trial_id, str) or len(trial_id) != 32 or any(c not in "0123456789abcdef" for c in trial_id):
                        raise RuntimeError("invalid camera trial ID")
                    if capture.get("request_id") != request_id:
                        raise RuntimeError("camera result does not match this motion request")
                    report["capture_trial_id"] = trial_id
                    trial = home/".evoarm/rsi/experiments"/trial_id
                    _check_recording(trial, report)
                except Exception as exc:
                    report["camera_error"] = f"{type(exc).__name__}: {str(exc)[:300]}"
                if report.get("capture_trial_id"):
                    _write(home/".evoarm/rsi/experiments"/report["capture_trial_id"]/"pregrasp-lift.json", report)
                if report["command_attempted"]:
                    _write(receipt, report)
                _write(folder/"latest-report.json", report)
        return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--setup-id", default="bench-layout-v2")
    parser.add_argument("--controller", default="http://127.0.0.1:8765")
    parser.add_argument("--execute", action="store_true", help="one real elbow lift with a 10s RGB-D recording")
    args = parser.parse_args(argv)
    result = run(setup_id=args.setup_id, controller=args.controller, execute=args.execute)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    if args.execute and (not result["motion_feedback_verified"] or result.get("camera_error") or result.get("disarm_error")):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
