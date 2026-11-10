"""Mac-local safe control UI. Bind only to 127.0.0.1."""
from pathlib import Path
import json

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, Response
from pydantic import BaseModel

from .service import ArmService
from .safety import UnsafeMotion
from .robot_calibration import CalibrationError, calibration, default_path as xy_calibration_path
from .pick_plan import PickPlanningError, plan_pick_place, build_from_saved_files, save_pick_profile, save_pick_draft, load_pick_progress
from .pick_teach import observe_tool_pose
from .pick_execution import executor

app = FastAPI(title="EvoArm 中文控制台", docs_url=None, redoc_url=None)
service = ArmService()


class Connect(BaseModel):
    port: str


class Unlock(BaseModel):
    declaration: str


class Step(BaseModel):
    joint: int
    delta: float


class GripperMove(BaseModel):
    delta: float


class HomeMove(BaseModel):
    kind: str
    joint: int


class CalibrationStart(BaseModel):
    square_mm: float
    image_size: list[int]
    corners_px: list[list[float]] | None = None


class CalibrationCapture(BaseModel):
    session_id: str
    point_id: str
    col: int
    row: int


class CalibrationSave(BaseModel):
    session_id: str


class TeachTorque(BaseModel):
    enabled: bool
    declaration: str


class PickProfile(BaseModel):
    grip_z_mm: float
    place_z_mm: float
    travel_z_mm: float
    place_xy_mm: list[float]
    pitch_rad: float
    roll_rad: float
    open_g_rad: float
    close_g_rad: float
    ball_diameter_mm: float


class PickDraft(BaseModel):
    values: dict


class PickPlan(BaseModel):
    ball_board_mm: list[float]
    image_size: list[int]
    corners_px: list[list[float]]


class PickStart(PickPlan):
    declaration: str


class PickStepRequest(BaseModel):
    session_id: str
    step_index: int
    declaration: str



def invoke(fn, *args):
    try:
        return fn(*args)
    except (UnsafeMotion, ValueError, RuntimeError, OSError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.get("/assets/theme.css")
def shared_theme():
    return Response(
        content=Path(__file__).with_name("theme.css").read_text(encoding="utf-8"),
        media_type="text/css",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@app.get("/", response_class=HTMLResponse)
def home():
    return Path(__file__).with_name("index.html").read_text(encoding="utf-8")


@app.get("/camera", response_class=HTMLResponse)
def camera_page():
    return Path(__file__).with_name("camera.html").read_text(encoding="utf-8")


@app.post("/api/vision/analyze")
async def vision_analyze(request: Request, square_mm: float | None = None, use_reference: bool = False):
    """Analyze ONE browser-uploaded PNG/JPEG frame; never accesses serial robot APIs."""
    try:
        from .vision import VisionError, analyze_image
    except ImportError as exc:
        raise HTTPException(
            status_code=503,
            detail="缺少视觉依赖。请先在 Mac 执行 pip install -r requirements-vision.txt",
        ) from exc
    if request.headers.get("content-type", "").split(";")[0].strip() not in (
        "image/png", "image/jpeg", "image/jpg"
    ):
        raise HTTPException(status_code=415, detail="请以 PNG 或 JPEG 格式上传单帧")
    if int(request.headers.get("content-length", "0") or "0") > 12_000_000:
        raise HTTPException(status_code=413, detail="图像超过12MB")
    raw = await request.body()
    reference = None
    if use_reference:
        # The reference comes from a verified, persisted 4+1 map, never an
        # arbitrary browser-supplied set of corners. New unfinished sessions
        # must not silently reuse a previously saved map.
        try:
            status = calibration.summary()
            if status.get("saved") and status.get("ready"):
                saved = json.loads(xy_calibration_path().read_text(encoding="utf-8"))
                if (saved.get("fit", {}).get("validated") and
                        len(saved.get("records", [])) == 5):
                    reference = {
                        "square_mm": saved.get("square_mm"),
                        "image_size": saved.get("image_size"),
                        "corners_px": saved.get("corners_px"),
                    }
        except (OSError, ValueError, TypeError, CalibrationError):
            reference = None
    try:
        return analyze_image(raw, square_mm=square_mm, reference=reference)
    except VisionError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/calibration", response_class=HTMLResponse)
def calibration_page():
    # Calibration UI changes rapidly during hardware bring-up. Never serve
    # a stale cached page with old onclick bindings or torque confirmations.
    return HTMLResponse(
        content=Path(__file__).with_name("calibration.html").read_text(encoding="utf-8"),
        headers={"Cache-Control": "no-store, max-age=0", "Pragma": "no-cache"},
    )


@app.get("/api/calibration/status")
def calibration_status():
    return calibration.summary()


@app.post("/api/calibration/start")
def calibration_start(req: CalibrationStart):
    try:
        return calibration.start(req.square_mm, req.image_size, req.corners_px)
    except CalibrationError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.post("/api/calibration/capture")
def calibration_capture(req: CalibrationCapture):
    try:
        return calibration.capture(req.session_id, req.point_id, req.col, req.row, service)
    except CalibrationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/calibration/save")
def calibration_save(req: CalibrationSave):
    try:
        return calibration.save(req.session_id)
    except CalibrationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/teach/torque")
def teach_torque(req: TeachTorque):
    return invoke(service.set_torque, req.enabled, req.declaration)


@app.get("/pick", response_class=HTMLResponse)
def pick_page():
    return HTMLResponse(
        content=Path(__file__).with_name("pick.html").read_text(encoding="utf-8"),
        headers={"Cache-Control": "no-store, max-age=0", "Pragma": "no-cache"},
    )


@app.get("/api/pick/profile")
def pick_profile_load():
    return load_pick_progress()


@app.post("/api/pick/draft")
def pick_draft_save(req: PickDraft):
    try:
        return save_pick_draft(req.values)
    except PickPlanningError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/pick/teach")
def pick_teach():
    try:
        return observe_tool_pose(service)
    except PickPlanningError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/pick/profile")
def pick_profile_save(req: PickProfile):
    try:
        return save_pick_profile(req.model_dump() if hasattr(req, "model_dump") else req.dict())
    except PickPlanningError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.post("/api/pick/preview")
def pick_plan_preview(req: PickPlan):
    try:
        return build_from_saved_files(req.ball_board_mm, image_size=req.image_size, corners_px=req.corners_px)
    except (PickPlanningError, CalibrationError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc


@app.get("/api/pick/execution")
def pick_execution_status():
    return executor.state(service)


@app.post("/api/pick/start")
def pick_execution_start(req: PickStart):
    return invoke(executor.start, req.ball_board_mm, req.image_size,
                  req.corners_px, service, req.declaration)


@app.post("/api/pick/step")
def pick_execution_step(req: PickStepRequest):
    return invoke(executor.step, req.session_id, req.step_index,
                  req.declaration, service)


@app.post("/api/pick/cancel")
def pick_execution_cancel():
    return executor.cancel(service)


@app.get("/api/ports")
def ports():
    return service.ports()


@app.get("/api/health")
def health():
    return service.health()


@app.post("/api/auto-connect")
def auto_connect():
    """Safe serial status connection, shared between the dashboard and calibration UI."""
    return service.auto_connect()


@app.post("/api/connect")
def connect(req: Connect):
    return {"status": invoke(service.connect, req.port)}


@app.post("/api/disconnect")
def disconnect():
    service.disconnect()
    return {"ok": True}


@app.get("/api/status")
def status():
    return invoke(service.status)


@app.post("/api/unlock-once")
def unlock(req: Unlock):
    return invoke(service.unlock_once, req.declaration)


@app.post("/api/disarm")
def disarm():
    return service.disarm()


@app.post("/api/recover")
def recover(req: Unlock):
    """Recover from a latched motion fault after manual safety check."""
    return invoke(service.recover_fault, req.declaration)


@app.post("/api/step")
def step(req: Step):
    return invoke(service.step, req.joint, req.delta)


@app.post("/api/gripper")
def gripper(req: GripperMove):
    return invoke(service.gripper_step, req.delta)


@app.post("/api/home/factory-reset")
def factory_reset():
    """Explicit official multi-joint init; requires preflight and UI confirmation."""
    return invoke(service.factory_reset)


@app.post("/api/home/save")
def home_save():
    return invoke(service.home_save)


@app.get("/api/home/preview")
def home_preview(kind: str = "saved"):
    return invoke(service.home_info, kind)


@app.post("/api/home/step")
def home_step(req: HomeMove):
    return invoke(service.home_step, req.kind, req.joint)


@app.on_event("shutdown")
def close_serial():
    service.disconnect()
