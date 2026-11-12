"""Focused manual calibration UI API; no taught task locations or joint jogs."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import time
from typing import Literal

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field

from evoarm_rsi import bench_calibration as bench
from evoarm_rsi.task_memory import valid_id, summary


def setup_id():
    return valid_id(os.environ.get("EVOARM_SETUP_ID", "bench-layout-v2"))


def experiment_root():
    return Path.home()/".evoarm/rsi/experiments"


def session_path():
    return Path.home()/".evoarm/rsi/calibrations"/setup_id()/"session.json"


class NewSession(BaseModel):
    trial_id: str = Field(pattern=r"^[a-f0-9]{32}$")
    square_mm: float = Field(gt=0, le=200)


class CaptureRequest(BaseModel):
    point_id: Literal["P1", "P2", "P3", "P4", "V1"]
    aligned_and_supported: bool
    replace: bool = False


def _call(function, *args, **kwargs):
    try:
        return function(*args, **kwargs)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


def state(service):
    path = session_path()
    result = {"setup_id": setup_id(), "session_exists": path.is_file(), "health": service.health(),
              "current_image_available": path.is_file() and path.with_name("current.png").is_file()}
    if path.is_file():
        _, session = bench.load_session(path)
        if session["setup_id"] != setup_id():
            raise ValueError("标定会话与当前布置不匹配")
        result.update(bench.session_status(path))
        result["square_mm"] = session["reference"]["square_mm"]
        result["image_size"] = session["reference"]["image_size"]
        result["overlay_points"] = [{"id": key, "grid": grid,
                                     "pixel_xy": session["reference"]["corners_px"][grid[1]*9+grid[0]]}
                                    for key, grid in session["points"].items()]
        result["reference_tool_pose_rad"] = next((session["records"][key]["tool_pose_rad"]
                                                  for key in bench.POINTS if key in session["records"]), None)
    return result


def install(app, service_provider):
    router = APIRouter(prefix="/api/bench")

    @router.get("/status")
    def status():
        return _call(state, service_provider())

    @router.get("/sources")
    def sources():
        rows = []
        root = experiment_root()
        for folder in sorted(root.glob("*"), key=lambda p: p.stat().st_mtime, reverse=True):
            if not re.fullmatch(r"[a-f0-9]{32}", folder.name):
                continue
            try:
                value = json.loads((folder/"manifest.json").read_text())
                if (value.get("simulated") is not False or value.get("status") != "recorded"
                        or not (folder/"gemini_color_intrinsics.json").is_file()):
                    continue
                rows.append({"trial_id": folder.name})
            except (OSError, ValueError):
                continue
            if len(rows) == 12:
                break
        return {"sources": rows}

    @router.post("/start")
    def start(req: NewSession):
        if not re.fullmatch(r"[a-f0-9]{32}", req.trial_id):
            raise HTTPException(status_code=422, detail="录像ID无效")
        return _call(bench.start_session, experiment_root()/req.trial_id, setup_id=setup_id(),
                     square_mm=req.square_mm, output=session_path())

    @router.get("/robot")
    def robot():
        service = service_provider()
        raw = _call(service.status)
        return {"xyz_mm": [raw.get(k) for k in "xyz"],
                "pitch_rad": raw.get("tit"), "roll_rad": raw.get("r"), "gripper_rad": raw.get("g"),
                "health": service.health()}

    @router.post("/capture")
    def capture(req: CaptureRequest):
        if req.aligned_and_supported is not True:
            raise HTTPException(status_code=409, detail="请持续支撑机械臂，确认触点对准角点且静止，再采点")
        service = service_provider()
        # Lock acquisition order is session -> serial, matching the CLI.
        result = _call(bench.capture_point, session_path(), req.point_id, aligned=True,
                       replace=req.replace, arm_service=service, allow_manual=True)
        if result["next_point"] is None:
            result = _call(bench.fit_session, session_path())
        return result

    @router.get("/frame")
    def frame(mode: Literal["reference", "current"] = "reference"):
        path = session_path().with_name("current.png") if mode == "current" else session_path().with_suffix(".png")
        if not path.is_file():
            raise HTTPException(status_code=404, detail="尚未生成此画面")
        return FileResponse(path, media_type="image/png", headers={"Cache-Control": "no-store"})

    @router.post("/live/start")
    def live_start():
        from evoarm_rsi.camera_service import client_request
        _call(bench.load_session, session_path())
        try:
            return client_request(Path.home(), "POST", "/preview/start", {})
        except (OSError, RuntimeError, ValueError) as exc:
            raise HTTPException(status_code=503, detail="实时预览未启动。请确认独立相机服务已从新版代码启动；不需要连接或操作机械臂。") from exc

    @router.get("/live-frame")
    def live_frame(after: int = Query(0, ge=0, le=2**53)):
        from evoarm_rsi.camera_service import preview_frame
        _, session = _call(bench.load_session, session_path())
        try:
            frame = preview_frame(Path.home(), after)
        except (OSError, RuntimeError, ValueError, TypeError) as exc:
            raise HTTPException(status_code=503, detail="相机没有返回新画面，实时预览已暂停。请检查相机服务和USB连接。") from exc
        if [frame["width"], frame["height"]] != session["reference"]["image_size"]:
            raise HTTPException(status_code=409, detail="实时画面分辨率与标定参考不同，不能叠加旧标记；请核对相机配置。")
        local_age = (time.monotonic_ns()-frame["stamp_ns"])/1e6
        age = max(frame["age_ms"], local_age)
        if local_age < 0 or not 0 <= age <= 1500:
            raise HTTPException(status_code=503, detail="相机画面已过期，不能作为实时对准依据。")
        return Response(frame["jpeg"], media_type="image/jpeg", headers={
            "Cache-Control": "no-store", "X-Frame-Sequence": str(frame["sequence"]),
            "X-Frame-Age-Ms": str(round(age, 2)), "X-Frame-Width": str(frame["width"]),
            "X-Frame-Height": str(frame["height"]), "X-Content-Type-Options": "nosniff"})

    @router.post("/snapshot")
    def snapshot():
        from evoarm_rsi.camera_service import client_request
        from evoarm_rsi.task_scene import load_rgbd
        path, session = _call(bench.load_session, session_path())
        # Uses the existing privileged camera service as ordinary user; no
        # SDK, sudo, USB reopening or robot command in this process.
        capture = _call(client_request, Path.home(), "POST", "/record", {"seconds": 1})
        trial_id = capture.get("trial_id")
        if not isinstance(trial_id, str) or not re.fullmatch(r"[a-f0-9]{32}", trial_id):
            raise HTTPException(status_code=409, detail="相机服务未返回有效录像ID")
        folder = experiment_root()/trial_id
        rgbd = _call(load_rgbd, folder)
        if list(rgbd["bgr"].shape[1::-1]) != session["reference"]["image_size"]:
            raise HTTPException(status_code=409, detail="相机分辨率已变化；需要重新标定")
        import cv2
        if not cv2.imwrite(str(path.with_name("current.png")), rgbd["bgr"]):
            raise HTTPException(status_code=409, detail="当前画面保存失败")
        return {"captured_at": datetime.now(timezone.utc).isoformat(), "trial_id": trial_id,
                "motion_commands_sent": False}

    app.include_router(router)

    @app.get("/api/task/trials")
    def task_trials():
        rows = []
        for file in (Path.home()/".evoarm/rsi/task_memory").glob("*.json"):
            try:
                value = summary(json.loads(file.read_text()))
                if value["setup_id"] == setup_id():
                    rows.append(value)
            except (OSError, ValueError, KeyError, TypeError):
                continue
        known={row["trial_id"] for row in rows}
        for file in experiment_root().glob("*/grasp-trial.json"):
            try:
                value=json.loads(file.read_text())
                trial_id=valid_id(value["capture_trial_id"])
                if (value.get("kind")!="first_ball_grasp_execution_v1" or value.get("setup_id")!=setup_id()
                        or trial_id!=file.parent.name or trial_id in known):continue
                completed=value.get("motion_completed") is True
                evidence=["已进入闭爪尝试；是否抓起需看录像" if value.get("grasp_attempted") else "闭爪动作尚未执行",
                          "抓取与提升动作序列已完成，视觉结果待确认" if completed else "动作中止："+str(value.get("motion_error") or value.get("camera_error") or "待核对")[:150]]
                rows.append({"trial_id":trial_id,"setup_id":setup_id(),"phase":"grasp_lift",
                             "pickup_observed":None,"ball_entered_funnel":None,
                             "within_10s":value.get("within_10s") if completed else None,
                             "failure_type":"motion_error" if value.get("motion_error") else "unknown",
                             "evidence":evidence,"policy":{},"created_at_unix":float(value["created_at_unix"])})
            except (OSError, ValueError, KeyError, TypeError):continue
        return {"setup_id": setup_id(), "trials": sorted(rows, key=lambda r: r["created_at_unix"], reverse=True)[:50],
                "execution_available": True,"execution_scope":"single_grasp_lift","placement_available":False}
