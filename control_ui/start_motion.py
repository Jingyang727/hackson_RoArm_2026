"""Narrow high-pose wrist recovery; existing ArmService owns all serial I/O.

No user-selected angles or limit changes. Recompute the measured trajectory
inside the serial owner's lock and require this identified recording.
"""
import math
from pathlib import Path
import time

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, ConfigDict

from .safety import Movement, SPEED_DEG_S, ACC_DEG_S2
from . import bench_ui
from evoarm_rsi.bench_calibration import load_session
from evoarm_rsi.bench_start import plan_start
from evoarm_rsi.camera_service import client_request


class StartOnce(BaseModel):
    model_config = ConfigDict(extra="forbid")
    reference_sha256: str = Field(pattern=r"^[a-f0-9]{64}$")
    expected_joint_rad: list[float] = Field(min_length=6, max_length=6)
    camera_request_id: str = Field(pattern=r"^[a-f0-9]{32}$")


def install(app, service_provider):
    router = APIRouter(prefix="/api/bench/motion")
    used = set()

    @router.get("/capabilities")
    def capabilities():
        return {"high_wrist_recovery": True, "training_limits_changed": False,
                "requires_motion_lease": True, "requires_identified_recording": True}

    @router.post("/start-once")
    def start_once(req: StartOnce):
        service = service_provider()
        try:
            with service.lock:
                if req.camera_request_id in used:
                    raise ValueError("this camera request already consumed its one start action")
                if len(used) >= 256:
                    raise ValueError("start action capacity reached; inspect existing trials")
                status = service._preflight(4)
                if any(not math.isfinite(a) or abs(status[k]-a) > math.radians(.6)
                       for k, a in zip("bsetrg", req.expected_joint_rad)):
                    raise ValueError("actual joints changed since start prediction")
                _, session = load_session(bench_ui.session_path())
                if session["setup_id"] != bench_ui.setup_id() or session["reference_sha256"] != req.reference_sha256:
                    raise ValueError("calibration changed since start prediction")
                plan = plan_start(session, status)
                if plan["start_mode"] != "high_wrist_recovery":
                    raise ValueError("this endpoint only supports measured high wrist recovery")
                camera = client_request(Path.home(), "GET", "/health")
                stamp = camera.get("recording_started_monotonic_ns")
                if (camera.get("robot_control") is not False or camera.get("recording") is not True or
                        camera.get("phase") != "recording" or camera.get("recording_request_id") != req.camera_request_id or
                        type(stamp) is not int or not 0 <= (time.monotonic_ns()-stamp)/1e9 < 1.5):
                    raise ValueError("this start action requires its own active fresh recording")
                current = math.degrees(status["t"])
                # plan_start proves a <=30deg inward-only path, >=100mm
                # clearance and a destination inside the unchanged +/-60
                # interval. Vendor wrist control clamps to +/-90 degrees.
                movement = Movement(4, round(current, 2), plan["target_deg"], {
                    "T": 121, "joint": 4, "angle": plan["target_deg"],
                    "spd": SPEED_DEG_S, "acc": ACC_DEG_S2})
                used.add(req.camera_request_id)  # BEFORE any potentially uncertain write
                result = service._send_move(movement)
                service.last_command["bench_camera_request_id"] = req.camera_request_id
                return {**result, "bench_camera_request_id": req.camera_request_id,
                        "start_mode": plan["start_mode"]}
        except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    app.include_router(router)
